//! # Embedded Modbus RTU Transport
//!
//! An async Modbus RTU transport for embedded MCUs (RP2040, ESP32, STM32, …)
//! built on [`embedded-io-async`](https://docs.rs/embedded-io-async).
//!
//! ## Design choices
//!
//! * **No `std`** — works with `#![no_std]` + `extern crate alloc`.
//! * **`alloc` required** — `ModbusResponse` owns a `Vec<u8>` internally.
//!   Most embedded Rust projects using async I/O already provide a global
//!   allocator (e.g. `embedded-alloc`), so this is the pragmatic sweet spot.
//! * **`heapless::Vec<u8, 256>`** used for the outgoing frame buffer so the
//!   encode path is entirely stack-allocated.
//! * **CRC-16/Modbus** computed with the `crc` crate which is `no_std` native.
//!
//! ## Usage
//!
//! ```rust,no_run
//! // Cargo.toml:
//! // voltage_modbus = { version = "...", default-features = false, features = ["embedded"] }
//!
//! use voltage_modbus::embedded::EmbeddedRtuTransport;
//! use voltage_modbus::protocol::{ModbusFunction, ModbusRequest};
//!
//! // `port` is any type that impls embedded_io_async::{Read, Write}
//! // e.g. embassy_stm32::usart::UartRx/Tx or similar
//! async fn run(port: impl embedded_io_async::Read + embedded_io_async::Write) {
//!     let mut transport = EmbeddedRtuTransport::new(port);
//!
//!     let req = ModbusRequest::new_read(
//!         1,
//!         ModbusFunction::ReadHoldingRegisters,
//!         0,
//!         10,
//!     );
//!
//!     let response = transport.request(&req).await.unwrap();
//!     let registers = response.parse_registers().unwrap();
//!     // use registers …
//! }
//! ```

// Bring in alloc primitives when std is absent
#[cfg(not(feature = "std"))]
extern crate alloc;

#[cfg(not(feature = "std"))]
use alloc::{format, vec, vec::Vec};

#[cfg(feature = "std")]
use std::vec::Vec;

use crc::{Crc, CRC_16_MODBUS};
use embedded_io_async::{Read, Write};
use heapless::Vec as HVec;

use crate::error::{ModbusError, ModbusResult};
use crate::protocol::{ModbusFunction, ModbusRequest, ModbusResponse};

// Maximum RTU frame sizes (Modbus spec):
//   Request : slave(1) + FC(1) + addr(2) + qty(2) + [bc(1) + data≤246] + CRC(2) = 256 max
//   Response: slave(1) + FC(1) + bc(1)   + data≤252                     + CRC(2) = 256 max
const MAX_FRAME: usize = 256;

/// CRC calculator instance (zero-cost at runtime — uses const-initialised table)
static CRC_MODBUS: Crc<u16> = Crc::<u16>::new(&CRC_16_MODBUS);

// ============================================================================
// EmbeddedRtuTransport
// ============================================================================

/// Modbus RTU transport for embedded devices using `embedded-io-async`.
///
/// `RW` must implement both [`embedded_io_async::Read`] and
/// [`embedded_io_async::Write`].  A UART peripheral in full-duplex or
/// half-duplex mode (with separate DE/RE control) both work, as long as the
/// same object exposes both traits.
pub struct EmbeddedRtuTransport<RW> {
    io: RW,
}

impl<RW> EmbeddedRtuTransport<RW>
where
    RW: Read + Write,
{
    /// Wrap an existing I/O object.
    pub fn new(io: RW) -> Self {
        Self { io }
    }

    /// Destroy the transport and return the inner I/O object.
    pub fn into_inner(self) -> RW {
        self.io
    }

    // ------------------------------------------------------------------ //
    // Public API                                                           //
    // ------------------------------------------------------------------ //

    /// Send a Modbus RTU request and wait for the response.
    ///
    /// Encodes the request into an RTU frame (slave + PDU + CRC-16 LE),
    /// writes it to the I/O object, then reads bytes until the expected
    /// response length is reached.  The response CRC is verified before
    /// returning.
    pub async fn request(&mut self, request: &ModbusRequest) -> ModbusResult<ModbusResponse> {
        let frame = self.encode_request(request)?;
        self.write_frame(&frame).await?;
        let response_buf = self.read_response(request).await?;
        self.decode_response(response_buf)
    }

    // ------------------------------------------------------------------ //
    // Encoding                                                            //
    // ------------------------------------------------------------------ //

    /// Encode a `ModbusRequest` into a stack-allocated RTU frame.
    ///
    /// Returns a `heapless::Vec<u8, MAX_FRAME>` containing:
    /// `[slave_id, function_code, ...PDU body..., CRC_lo, CRC_hi]`
    pub fn encode_request(
        &self,
        request: &ModbusRequest,
    ) -> ModbusResult<HVec<u8, MAX_FRAME>> {
        let mut frame: HVec<u8, MAX_FRAME> = HVec::new();

        push(&mut frame, request.slave_id)?;
        push(&mut frame, request.function.to_u8())?;

        match request.function {
            ModbusFunction::ReadCoils
            | ModbusFunction::ReadDiscreteInputs
            | ModbusFunction::ReadHoldingRegisters
            | ModbusFunction::ReadInputRegisters => {
                extend(&mut frame, &request.address.to_be_bytes())?;
                extend(&mut frame, &request.quantity.to_be_bytes())?;
            }

            ModbusFunction::WriteSingleCoil => {
                extend(&mut frame, &request.address.to_be_bytes())?;
                let coil_value: u16 =
                    if !request.data.is_empty() && request.data[0] != 0 { 0xFF00 } else { 0x0000 };
                extend(&mut frame, &coil_value.to_be_bytes())?;
            }

            ModbusFunction::WriteSingleRegister => {
                extend(&mut frame, &request.address.to_be_bytes())?;
                if request.data.len() >= 2 {
                    extend(&mut frame, &request.data[0..2])?;
                } else {
                    extend(&mut frame, &[0u8, 0u8])?;
                }
            }

            ModbusFunction::WriteMultipleCoils | ModbusFunction::WriteMultipleRegisters => {
                extend(&mut frame, &request.address.to_be_bytes())?;
                extend(&mut frame, &request.quantity.to_be_bytes())?;
                let byte_count = u8::try_from(request.data.len())
                    .map_err(|_| ModbusError::invalid_data("data payload too large"))?;
                push(&mut frame, byte_count)?;
                extend(&mut frame, &request.data)?;
            }
        }

        let crc = CRC_MODBUS.checksum(&frame);
        extend(&mut frame, &crc.to_le_bytes())?; // CRC is little-endian in RTU

        Ok(frame)
    }

    // ------------------------------------------------------------------ //
    // Decoding                                                            //
    // ------------------------------------------------------------------ //

    /// Decode a raw RTU response frame.
    ///
    /// Verifies CRC, checks for exception responses, and returns a
    /// `ModbusResponse` owning the payload.
    pub fn decode_response(&self, frame: Vec<u8>) -> ModbusResult<ModbusResponse> {
        if frame.len() < 4 {
            return Err(ModbusError::frame("RTU frame too short"));
        }

        let pdu_len = frame.len() - 2; // everything except the two CRC bytes
        let received_crc = u16::from_le_bytes([frame[pdu_len], frame[pdu_len + 1]]);
        let calculated_crc = CRC_MODBUS.checksum(&frame[..pdu_len]);

        if received_crc != calculated_crc {
            return Err(ModbusError::frame(format!(
                "CRC mismatch: expected 0x{:04X}, got 0x{:04X}",
                calculated_crc, received_crc
            )));
        }

        let slave_id = frame[0];
        let function_code = frame[1];

        // Exception response: high bit set on function code
        if function_code & 0x80 != 0 {
            if frame.len() < 5 {
                return Err(ModbusError::frame("Invalid exception response"));
            }
            let original_fn = function_code & 0x7F;
            let exception_code = frame[2];
            return Ok(ModbusResponse::new_exception(
                slave_id,
                ModbusFunction::from_u8(original_fn)?,
                exception_code,
            ));
        }

        let function = ModbusFunction::from_u8(function_code)?;
        // RTU frame layout: [slave(1), FC(1), data..., CRC(2)]
        // data_start=2, data_len = pdu_len - 2  (skip slave_id and FC)
        let data_start = 2usize;
        let data_len = pdu_len.saturating_sub(2);

        Ok(ModbusResponse::new_from_frame(
            frame, slave_id, function, data_start, data_len,
        ))
    }

    // ------------------------------------------------------------------ //
    // I/O helpers                                                         //
    // ------------------------------------------------------------------ //

    async fn write_frame(&mut self, frame: &[u8]) -> ModbusResult<()> {
        self.io
            .write_all(frame)
            .await
            .map_err(|_| ModbusError::io("embedded write error"))
    }

    /// Read exactly the number of bytes expected for a response to `request`.
    async fn read_response(&mut self, request: &ModbusRequest) -> ModbusResult<Vec<u8>> {
        let expected = expected_response_len(request);
        let mut buf = vec![0u8; expected];
        self.io
            .read_exact(&mut buf)
            .await
            .map_err(|_| ModbusError::io("embedded read error"))?;
        Ok(buf)
    }
}

// ============================================================================
// Frame length prediction
// ============================================================================

/// Compute the expected byte length of the RTU *response* for a given request.
///
/// This lets us do a single exact `read_exact` instead of byte-by-byte polling.
/// Layout: `[slave(1), FC(1), ...payload..., CRC(2)]`
fn expected_response_len(request: &ModbusRequest) -> usize {
    match request.function {
        // Read functions: slave + FC + byte_count + data + CRC
        ModbusFunction::ReadCoils | ModbusFunction::ReadDiscreteInputs => {
            // quantity bits → ceil(quantity / 8) bytes of data
            let data_bytes = usize::from(request.quantity.div_ceil(8));
            1 + 1 + 1 + data_bytes + 2
        }
        ModbusFunction::ReadHoldingRegisters | ModbusFunction::ReadInputRegisters => {
            // quantity registers × 2 bytes each
            let data_bytes = usize::from(request.quantity) * 2;
            1 + 1 + 1 + data_bytes + 2
        }
        // Write-single: slave + FC + echo_addr(2) + echo_value(2) + CRC
        ModbusFunction::WriteSingleCoil | ModbusFunction::WriteSingleRegister => 1 + 1 + 2 + 2 + 2,
        // Write-multiple: slave + FC + echo_addr(2) + echo_qty(2) + CRC
        ModbusFunction::WriteMultipleCoils | ModbusFunction::WriteMultipleRegisters => {
            1 + 1 + 2 + 2 + 2
        }
    }
}

// ============================================================================
// Internal helpers — heapless push/extend with uniform error mapping
// ============================================================================

#[inline]
fn push(buf: &mut HVec<u8, MAX_FRAME>, byte: u8) -> ModbusResult<()> {
    buf.push(byte)
        .map_err(|_| ModbusError::frame("RTU frame buffer overflow"))
}

#[inline]
fn extend(buf: &mut HVec<u8, MAX_FRAME>, bytes: &[u8]) -> ModbusResult<()> {
    buf.extend_from_slice(bytes)
        .map_err(|_| ModbusError::frame("RTU frame buffer overflow"))
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::{ModbusFunction, ModbusRequest};

    // ------------------------------------------------------------------
    // Mock I/O backend
    // ------------------------------------------------------------------

    /// A mock I/O that replays a fixed byte sequence on `read` and
    /// captures everything written to it.
    struct MockIo {
        read_buf: Vec<u8>,
        read_pos: usize,
        pub written: Vec<u8>,
    }

    impl MockIo {
        fn new(read_data: Vec<u8>) -> Self {
            Self {
                read_buf: read_data,
                read_pos: 0,
                written: Vec::new(),
            }
        }
    }

    impl embedded_io_async::ErrorType for MockIo {
        type Error = MockError;
    }

    #[derive(Debug)]
    struct MockError;

    impl embedded_io_async::Error for MockError {
        fn kind(&self) -> embedded_io_async::ErrorKind {
            embedded_io_async::ErrorKind::Other
        }
    }

    impl Read for MockIo {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            let remaining = self.read_buf.len() - self.read_pos;
            if remaining == 0 {
                return Err(MockError);
            }
            let n = buf.len().min(remaining);
            buf[..n].copy_from_slice(&self.read_buf[self.read_pos..self.read_pos + n]);
            self.read_pos += n;
            Ok(n)
        }
    }

    impl Write for MockIo {
        async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            self.written.extend_from_slice(buf);
            Ok(buf.len())
        }
    }

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------

    /// Build a valid RTU response frame for a FC03 read of `n_regs` registers.
    fn make_fc03_response(slave_id: u8, regs: &[u16]) -> Vec<u8> {
        let mut frame: Vec<u8> = Vec::new();
        frame.push(slave_id);
        frame.push(0x03); // FC03
        frame.push((regs.len() * 2) as u8);
        for &r in regs {
            frame.extend_from_slice(&r.to_be_bytes());
        }
        let crc = CRC_MODBUS.checksum(&frame);
        frame.extend_from_slice(&crc.to_le_bytes());
        frame
    }

    /// Build a valid RTU exception response frame.
    fn make_exception_frame(slave_id: u8, fc: u8, exc_code: u8) -> Vec<u8> {
        let mut frame: Vec<u8> = vec![slave_id, fc | 0x80, exc_code];
        let crc = CRC_MODBUS.checksum(&frame);
        frame.extend_from_slice(&crc.to_le_bytes());
        frame
    }

    // ------------------------------------------------------------------
    // encode_request tests
    // ------------------------------------------------------------------

    #[test]
    fn test_encode_read_holding_registers() {
        let transport = EmbeddedRtuTransport::new(MockIo::new(vec![]));
        let req = ModbusRequest::new_read(1, ModbusFunction::ReadHoldingRegisters, 0x0000, 10);
        let frame = transport.encode_request(&req).unwrap();

        // Minimum: slave(1) FC(1) addr(2) qty(2) CRC(2) = 8 bytes
        assert_eq!(frame.len(), 8);
        assert_eq!(frame[0], 1);   // slave_id
        assert_eq!(frame[1], 0x03); // FC03
        assert_eq!(frame[2], 0x00); // address hi
        assert_eq!(frame[3], 0x00); // address lo
        assert_eq!(frame[4], 0x00); // qty hi
        assert_eq!(frame[5], 0x0A); // qty lo = 10

        // Verify CRC
        let pdu_len = frame.len() - 2;
        let expected_crc = CRC_MODBUS.checksum(&frame[..pdu_len]);
        let frame_crc = u16::from_le_bytes([frame[pdu_len], frame[pdu_len + 1]]);
        assert_eq!(expected_crc, frame_crc);
    }

    #[test]
    fn test_encode_write_single_register() {
        let transport = EmbeddedRtuTransport::new(MockIo::new(vec![]));
        let req = ModbusRequest::new_write(
            2,
            ModbusFunction::WriteSingleRegister,
            0x0064,  // address 100
            vec![0x12, 0x34],
        );
        let frame = transport.encode_request(&req).unwrap();

        assert_eq!(frame[0], 2);    // slave_id
        assert_eq!(frame[1], 0x06); // FC06
        assert_eq!(frame[2], 0x00);
        assert_eq!(frame[3], 0x64); // address 100
        assert_eq!(frame[4], 0x12);
        assert_eq!(frame[5], 0x34); // value

        let pdu_len = frame.len() - 2;
        let expected_crc = CRC_MODBUS.checksum(&frame[..pdu_len]);
        let frame_crc = u16::from_le_bytes([frame[pdu_len], frame[pdu_len + 1]]);
        assert_eq!(expected_crc, frame_crc);
    }

    #[test]
    fn test_encode_write_coil_on() {
        let transport = EmbeddedRtuTransport::new(MockIo::new(vec![]));
        let req = ModbusRequest::new_write(
            1,
            ModbusFunction::WriteSingleCoil,
            0,
            vec![1], // non-zero = ON
        );
        let frame = transport.encode_request(&req).unwrap();
        assert_eq!(frame[4], 0xFF);
        assert_eq!(frame[5], 0x00);
    }

    #[test]
    fn test_encode_write_coil_off() {
        let transport = EmbeddedRtuTransport::new(MockIo::new(vec![]));
        let req = ModbusRequest::new_write(1, ModbusFunction::WriteSingleCoil, 0, vec![0]);
        let frame = transport.encode_request(&req).unwrap();
        assert_eq!(frame[4], 0x00);
        assert_eq!(frame[5], 0x00);
    }

    // ------------------------------------------------------------------
    // decode_response tests
    // ------------------------------------------------------------------

    #[test]
    fn test_decode_fc03_response_roundtrip() {
        let transport = EmbeddedRtuTransport::new(MockIo::new(vec![]));
        let regs = [0x0001u16, 0x0002, 0x0003];
        let frame = make_fc03_response(1, &regs);

        let response = transport.decode_response(frame).unwrap();
        assert_eq!(response.slave_id, 1);
        assert_eq!(response.function, ModbusFunction::ReadHoldingRegisters);
        assert!(!response.is_exception());

        let parsed = response.parse_registers().unwrap();
        assert_eq!(parsed, regs);
    }

    #[test]
    fn test_decode_bad_crc_rejected() {
        let transport = EmbeddedRtuTransport::new(MockIo::new(vec![]));
        let mut frame = make_fc03_response(1, &[0xABCDu16]);
        // Corrupt the last CRC byte
        let last = frame.len() - 1;
        frame[last] ^= 0xFF;

        let result = transport.decode_response(frame);
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(matches!(err, ModbusError::Frame { .. }));
    }

    #[test]
    fn test_decode_exception_response() {
        let transport = EmbeddedRtuTransport::new(MockIo::new(vec![]));
        // FC03 exception, exception code 0x02 (illegal data address)
        let frame = make_exception_frame(1, 0x03, 0x02);

        let response = transport.decode_response(frame).unwrap();
        assert!(response.is_exception());
        assert_eq!(response.slave_id, 1);
        assert_eq!(response.function, ModbusFunction::ReadHoldingRegisters);
    }

    #[test]
    fn test_decode_frame_too_short() {
        let transport = EmbeddedRtuTransport::new(MockIo::new(vec![]));
        let short = vec![0x01, 0x03, 0x04]; // only 3 bytes
        assert!(transport.decode_response(short).is_err());
    }

    // ------------------------------------------------------------------
    // Full roundtrip: encode → (mock I/O) → decode
    // ------------------------------------------------------------------

    #[tokio::test]
    async fn test_request_roundtrip_fc03() {
        let regs = [0x1234u16, 0x5678];
        let response_frame = make_fc03_response(1, &regs);

        let mock = MockIo::new(response_frame.clone());
        let mut transport = EmbeddedRtuTransport::new(mock);

        let req = ModbusRequest::new_read(1, ModbusFunction::ReadHoldingRegisters, 0, 2);
        let response = transport.request(&req).await.unwrap();

        assert!(!response.is_exception());
        let parsed = response.parse_registers().unwrap();
        assert_eq!(parsed, regs);

        // Verify what was actually written to the mock
        let written = &transport.io.written;
        assert!(!written.is_empty());
        assert_eq!(written[0], 1);    // slave_id
        assert_eq!(written[1], 0x03); // FC03
    }

    #[tokio::test]
    async fn test_request_bad_crc_error() {
        let mut frame = make_fc03_response(1, &[0xBEEFu16]);
        let last = frame.len() - 1;
        frame[last] ^= 0xFF; // corrupt CRC

        let mock = MockIo::new(frame);
        let mut transport = EmbeddedRtuTransport::new(mock);

        let req = ModbusRequest::new_read(1, ModbusFunction::ReadHoldingRegisters, 0, 1);
        assert!(transport.request(&req).await.is_err());
    }
}
