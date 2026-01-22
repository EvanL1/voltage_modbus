//! # Modbus Protocol Implementation
//!
//! This module provides comprehensive Modbus protocol support including:
//! - Standard Modbus function codes (0x01-0x10)  
//! - Request and response message structures
//! - Data type conversions and validation
//! - Exception handling and error codes
//!
//! ## Supported Function Codes
//!
//! ### Read Functions
//! - **0x01**: Read Coils - Read 1 to 2000 contiguous coils
//! - **0x02**: Read Discrete Inputs - Read 1 to 2000 contiguous discrete inputs  
//! - **0x03**: Read Holding Registers - Read 1 to 125 contiguous holding registers
//! - **0x04**: Read Input Registers - Read 1 to 125 contiguous input registers
//!
//! ### Write Functions  
//! - **0x05**: Write Single Coil - Write a single coil ON or OFF
//! - **0x06**: Write Single Register - Write a single 16-bit register
//! - **0x0F**: Write Multiple Coils - Write multiple coils (1 to 1968)
//! - **0x10**: Write Multiple Registers - Write multiple registers (1 to 123)
//!
//! ## Usage Examples
//!
//! ### Creating Requests
//!
//! ```rust
//! use voltage_modbus::protocol::{ModbusRequest, ModbusFunction};
//!
//! // Read 10 holding registers starting at address 100
//! let read_request = ModbusRequest::new_read(
//!     1,                                    // slave_id
//!     ModbusFunction::ReadHoldingRegisters, // function
//!     100,                                  // start_address  
//!     10                                    // quantity
//! );
//!
//! // Write value 0x1234 to register 200  
//! let write_request = ModbusRequest::new_write(
//!     1,                                     // slave_id
//!     ModbusFunction::WriteSingleRegister,   // function
//!     200,                                   // address
//!     vec![0x12, 0x34]                       // data (big-endian)
//! );
//! ```
//!
//! ### Processing Responses
//!
//! ```rust
//! use voltage_modbus::protocol::{ModbusResponse, ModbusFunction};
//!
//! # fn process_response() -> Result<(), Box<dyn std::error::Error>> {
//! let response = ModbusResponse::new_success(
//!     1,                                    // slave_id
//!     ModbusFunction::ReadHoldingRegisters, // function
//!     vec![0x12, 0x34, 0x56, 0x78]         // data
//! );
//!
//! if !response.is_exception() {
//!     let registers = response.parse_registers()?;
//!     println!("Read registers: {:?}", registers); // [0x1234, 0x5678]
//! }
//! # Ok(())
//! # }
//! ```
//!
//! ### Data Conversion Utilities
//!
//! ```rust
//! use voltage_modbus::protocol::data_utils::*;
//!
//! // Convert 32-bit float to two 16-bit registers
//! let value = 123.456f32;
//! let registers = f32_to_registers(value);
//!
//! // Convert registers back to float
//! let restored = registers_to_f32(&registers)?;
//! assert_eq!(value, restored);
//!
//! // Pack boolean values into bytes
//! let bits = vec![true, false, true, true, false, true, false, false];
//! let packed = pack_bits(&bits);
//! let unpacked = unpack_bits(&packed, 8);
//! assert_eq!(bits, unpacked);
//! # Ok::<(), voltage_modbus::ModbusError>(())
//! ```

use crate::error::{ModbusError, ModbusResult};
/// Modbus protocol definitions and data structures
///
/// This module contains the core Modbus protocol definitions, including
/// function codes, data types, and request/response structures.
use std::fmt;

/// Modbus address type (0-65535)
pub type ModbusAddress = u16;

/// Modbus value type (16-bit register value)
pub type ModbusValue = u16;

/// Modbus slave/unit identifier (1-247)
pub type SlaveId = u8;

/// Modbus function codes
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum ModbusFunction {
    /// Read Coils (0x01)
    ReadCoils = 0x01,
    /// Read Discrete Inputs (0x02)
    ReadDiscreteInputs = 0x02,
    /// Read Holding Registers (0x03)
    ReadHoldingRegisters = 0x03,
    /// Read Input Registers (0x04)
    ReadInputRegisters = 0x04,
    /// Write Single Coil (0x05)
    WriteSingleCoil = 0x05,
    /// Write Single Register (0x06)
    WriteSingleRegister = 0x06,
    /// Write Multiple Coils (0x0F)
    WriteMultipleCoils = 0x0F,
    /// Write Multiple Registers (0x10)
    WriteMultipleRegisters = 0x10,
}

impl ModbusFunction {
    /// Convert from u8 to ModbusFunction
    pub fn from_u8(value: u8) -> ModbusResult<Self> {
        match value {
            0x01 => Ok(ModbusFunction::ReadCoils),
            0x02 => Ok(ModbusFunction::ReadDiscreteInputs),
            0x03 => Ok(ModbusFunction::ReadHoldingRegisters),
            0x04 => Ok(ModbusFunction::ReadInputRegisters),
            0x05 => Ok(ModbusFunction::WriteSingleCoil),
            0x06 => Ok(ModbusFunction::WriteSingleRegister),
            0x0F => Ok(ModbusFunction::WriteMultipleCoils),
            0x10 => Ok(ModbusFunction::WriteMultipleRegisters),
            _ => Err(ModbusError::invalid_function(value)),
        }
    }

    /// Convert to u8
    pub fn to_u8(self) -> u8 {
        self as u8
    }

    /// Check if this is a read function
    pub fn is_read_function(self) -> bool {
        matches!(
            self,
            ModbusFunction::ReadCoils
                | ModbusFunction::ReadDiscreteInputs
                | ModbusFunction::ReadHoldingRegisters
                | ModbusFunction::ReadInputRegisters
        )
    }

    /// Check if this is a write function
    pub fn is_write_function(self) -> bool {
        matches!(
            self,
            ModbusFunction::WriteSingleCoil
                | ModbusFunction::WriteSingleRegister
                | ModbusFunction::WriteMultipleCoils
                | ModbusFunction::WriteMultipleRegisters
        )
    }
}

impl fmt::Display for ModbusFunction {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let name = match self {
            ModbusFunction::ReadCoils => "Read Coils",
            ModbusFunction::ReadDiscreteInputs => "Read Discrete Inputs",
            ModbusFunction::ReadHoldingRegisters => "Read Holding Registers",
            ModbusFunction::ReadInputRegisters => "Read Input Registers",
            ModbusFunction::WriteSingleCoil => "Write Single Coil",
            ModbusFunction::WriteSingleRegister => "Write Single Register",
            ModbusFunction::WriteMultipleCoils => "Write Multiple Coils",
            ModbusFunction::WriteMultipleRegisters => "Write Multiple Registers",
        };
        write!(f, "{} (0x{:02X})", name, *self as u8)
    }
}

/// Modbus exception codes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ModbusException {
    IllegalFunction = 0x01,
    IllegalDataAddress = 0x02,
    IllegalDataValue = 0x03,
    ServerDeviceFailure = 0x04,
    Acknowledge = 0x05,
    ServerDeviceBusy = 0x06,
    MemoryParityError = 0x08,
    GatewayPathUnavailable = 0x0A,
    GatewayTargetDeviceFailedToRespond = 0x0B,
}

impl ModbusException {
    /// Convert from u8 to ModbusException
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0x01 => Some(ModbusException::IllegalFunction),
            0x02 => Some(ModbusException::IllegalDataAddress),
            0x03 => Some(ModbusException::IllegalDataValue),
            0x04 => Some(ModbusException::ServerDeviceFailure),
            0x05 => Some(ModbusException::Acknowledge),
            0x06 => Some(ModbusException::ServerDeviceBusy),
            0x08 => Some(ModbusException::MemoryParityError),
            0x0A => Some(ModbusException::GatewayPathUnavailable),
            0x0B => Some(ModbusException::GatewayTargetDeviceFailedToRespond),
            _ => None,
        }
    }

    /// Convert to u8
    pub fn to_u8(self) -> u8 {
        self as u8
    }

    /// Get human-readable description
    pub fn description(self) -> &'static str {
        match self {
            ModbusException::IllegalFunction => "The function code received in the query is not an allowable action for the server",
            ModbusException::IllegalDataAddress => "The data address received in the query is not an allowable address for the server",
            ModbusException::IllegalDataValue => "A value contained in the query data field is not an allowable value for server",
            ModbusException::ServerDeviceFailure => "An unrecoverable error occurred while the server was attempting to perform the requested action",
            ModbusException::Acknowledge => "The server has accepted the request and is processing it, but a long duration of time will be required to do so",
            ModbusException::ServerDeviceBusy => "The server is engaged in processing a long-duration program command",
            ModbusException::MemoryParityError => "The server attempted to read record file, but detected a parity error in the memory",
            ModbusException::GatewayPathUnavailable => "Gateway was unable to allocate an internal communication path",
            ModbusException::GatewayTargetDeviceFailedToRespond => "No response was obtained from the target device",
        }
    }
}

impl fmt::Display for ModbusException {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Modbus Exception 0x{:02X}: {}",
            self.to_u8(),
            self.description()
        )
    }
}

/// Modbus request structure
#[derive(Debug, Clone, PartialEq)]
pub struct ModbusRequest {
    pub slave_id: SlaveId,
    pub function: ModbusFunction,
    pub address: ModbusAddress,
    pub quantity: u16,
    pub data: Vec<u8>,
}

impl ModbusRequest {
    /// Create a new read request
    pub fn new_read(
        slave_id: SlaveId,
        function: ModbusFunction,
        address: ModbusAddress,
        quantity: u16,
    ) -> Self {
        Self {
            slave_id,
            function,
            address,
            quantity,
            data: Vec::new(),
        }
    }

    /// Create a new write request
    pub fn new_write(
        slave_id: SlaveId,
        function: ModbusFunction,
        address: ModbusAddress,
        data: Vec<u8>,
    ) -> Self {
        let quantity = match function {
            ModbusFunction::WriteSingleCoil | ModbusFunction::WriteSingleRegister => 1,
            ModbusFunction::WriteMultipleCoils => data.len() as u16 * 8,
            ModbusFunction::WriteMultipleRegisters => data.len() as u16 / 2,
            _ => 0,
        };

        Self {
            slave_id,
            function,
            address,
            quantity,
            data,
        }
    }

    /// Validate the request
    pub fn validate(&self) -> ModbusResult<()> {
        // Validate slave ID
        if self.slave_id == 0 || self.slave_id > 247 {
            return Err(ModbusError::invalid_data(format!(
                "Invalid slave ID: {}",
                self.slave_id
            )));
        }

        // Validate quantity for read operations
        if self.function.is_read_function() {
            if self.quantity == 0 {
                return Err(ModbusError::invalid_data(
                    "Quantity cannot be zero".to_string(),
                ));
            }

            match self.function {
                ModbusFunction::ReadCoils | ModbusFunction::ReadDiscreteInputs => {
                    if self.quantity > crate::MAX_READ_COILS as u16 {
                        return Err(ModbusError::invalid_data(format!(
                            "Too many coils requested: {}",
                            self.quantity
                        )));
                    }
                }
                ModbusFunction::ReadHoldingRegisters | ModbusFunction::ReadInputRegisters => {
                    if self.quantity > crate::MAX_READ_REGISTERS as u16 {
                        return Err(ModbusError::invalid_data(format!(
                            "Too many registers requested: {}",
                            self.quantity
                        )));
                    }
                }
                _ => {}
            }
        }

        Ok(())
    }
}

/// Modbus response structure
///
/// Uses internal buffer with offset/length tracking to enable zero-copy
/// parsing when receiving responses from transport layer.
#[derive(Debug, Clone, PartialEq)]
pub struct ModbusResponse {
    pub slave_id: SlaveId,
    pub function: ModbusFunction,
    /// Internal buffer storage (may be payload-only or complete frame)
    buffer: Vec<u8>,
    /// Offset where payload data starts within buffer
    data_offset: usize,
    /// Length of payload data
    data_len: usize,
    pub exception: Option<ModbusException>,
}

impl ModbusResponse {
    /// Create a successful response with payload data
    ///
    /// The data is stored directly with zero offset. For zero-copy parsing
    /// from TCP/RTU frames, use `new_from_tcp_frame` or `new_from_rtu_frame`.
    pub fn new_success(slave_id: SlaveId, function: ModbusFunction, data: Vec<u8>) -> Self {
        let data_len = data.len();
        Self {
            slave_id,
            function,
            buffer: data,
            data_offset: 0,
            data_len,
            exception: None,
        }
    }

    /// Create a response from a complete TCP frame (zero-copy)
    ///
    /// Takes ownership of the frame buffer and calculates payload offsets
    /// without copying the data portion.
    ///
    /// # Arguments
    /// * `frame` - Complete TCP frame including MBAP header
    /// * `slave_id` - Parsed slave ID
    /// * `function` - Parsed function code
    /// * `data_start` - Byte offset where payload data begins
    /// * `data_len` - Length of payload data
    #[inline]
    pub fn new_from_frame(
        frame: Vec<u8>,
        slave_id: SlaveId,
        function: ModbusFunction,
        data_start: usize,
        data_len: usize,
    ) -> Self {
        Self {
            slave_id,
            function,
            buffer: frame,
            data_offset: data_start,
            data_len,
            exception: None,
        }
    }

    /// Create an exception response
    pub fn new_exception(slave_id: SlaveId, function: ModbusFunction, exception_code: u8) -> Self {
        let exception = ModbusException::from_u8(exception_code);
        Self {
            slave_id,
            function,
            buffer: Vec::new(),
            data_offset: 0,
            data_len: 0,
            exception,
        }
    }

    /// Get payload data as a slice
    ///
    /// Returns the response payload without the function code or byte count prefix.
    /// For read responses, this includes the byte count byte followed by actual data.
    #[inline]
    pub fn data(&self) -> &[u8] {
        &self.buffer[self.data_offset..self.data_offset + self.data_len]
    }

    /// Get the length of payload data
    #[inline]
    pub fn data_len(&self) -> usize {
        self.data_len
    }

    /// Check if this is an exception response
    #[inline]
    pub fn is_exception(&self) -> bool {
        self.exception.is_some()
    }

    /// Get exception error if present
    pub fn get_exception(&self) -> Option<ModbusError> {
        self.exception
            .map(|exc| ModbusError::protocol(format!("Modbus exception: {}", exc)))
    }

    /// Parse response data as registers (u16 values)
    pub fn parse_registers(&self) -> ModbusResult<Vec<u16>> {
        if self.is_exception() {
            return Err(self.get_exception().unwrap());
        }

        let data = self.data();
        if data.is_empty() {
            return Err(ModbusError::frame("Empty response data"));
        }

        let byte_count = data[0] as usize;
        if data.len() < 1 + byte_count {
            return Err(ModbusError::frame("Incomplete register data"));
        }

        if byte_count % 2 != 0 {
            return Err(ModbusError::frame("Invalid register data length"));
        }

        let mut registers = Vec::with_capacity(byte_count / 2);
        for i in (1..1 + byte_count).step_by(2) {
            let value = u16::from_be_bytes([data[i], data[i + 1]]);
            registers.push(value);
        }

        Ok(registers)
    }

    /// Parse response data as bits (bool values)
    pub fn parse_bits(&self) -> ModbusResult<Vec<bool>> {
        if self.is_exception() {
            return Err(self.get_exception().unwrap());
        }

        let data = self.data();
        if data.is_empty() {
            return Err(ModbusError::frame("Empty response data"));
        }

        let byte_count = data[0] as usize;
        if data.len() < 1 + byte_count {
            return Err(ModbusError::frame("Incomplete bit data"));
        }

        let mut bits = Vec::with_capacity(byte_count * 8);
        for &byte_value in data.iter().skip(1).take(byte_count) {
            for bit_pos in 0..8 {
                bits.push((byte_value & (1 << bit_pos)) != 0);
            }
        }

        Ok(bits)
    }
}

/// Data conversion utilities
pub mod data_utils {
    use super::*;

    /// Convert register values to bytes (big-endian)
    pub fn registers_to_bytes(registers: &[u16]) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(registers.len() * 2);
        for &register in registers {
            bytes.extend_from_slice(&register.to_be_bytes());
        }
        bytes
    }

    /// Convert bytes to register values (big-endian)
    pub fn bytes_to_registers(bytes: &[u8]) -> ModbusResult<Vec<u16>> {
        if bytes.len() % 2 != 0 {
            return Err(ModbusError::invalid_data(
                "Byte array length must be even".to_string(),
            ));
        }

        let mut registers = Vec::new();
        for chunk in bytes.chunks(2) {
            let value = u16::from_be_bytes([chunk[0], chunk[1]]);
            registers.push(value);
        }
        Ok(registers)
    }

    /// Pack boolean values into bytes
    pub fn pack_bits(bits: &[bool]) -> Vec<u8> {
        let byte_count = (bits.len() + 7) / 8;
        let mut bytes = vec![0u8; byte_count];

        for (i, &bit) in bits.iter().enumerate() {
            if bit {
                let byte_index = i / 8;
                let bit_index = i % 8;
                bytes[byte_index] |= 1 << bit_index;
            }
        }

        bytes
    }

    /// Unpack bytes into boolean values
    pub fn unpack_bits(bytes: &[u8], bit_count: usize) -> Vec<bool> {
        let mut bits = Vec::with_capacity(bit_count);

        for i in 0..bit_count {
            let byte_index = i / 8;
            let bit_index = i % 8;

            if byte_index < bytes.len() {
                let bit_value = (bytes[byte_index] & (1 << bit_index)) != 0;
                bits.push(bit_value);
            } else {
                bits.push(false);
            }
        }

        bits
    }

    /// Convert u32 to two u16 registers (big-endian)
    pub fn u32_to_registers(value: u32) -> [u16; 2] {
        [(value >> 16) as u16, value as u16]
    }

    /// Convert two u16 registers to u32 (big-endian)
    pub fn registers_to_u32(registers: &[u16]) -> ModbusResult<u32> {
        if registers.len() < 2 {
            return Err(ModbusError::invalid_data(
                "Need at least 2 registers for u32".to_string(),
            ));
        }
        Ok(((registers[0] as u32) << 16) | (registers[1] as u32))
    }

    /// Convert f32 to two u16 registers (IEEE 754, big-endian)
    pub fn f32_to_registers(value: f32) -> [u16; 2] {
        u32_to_registers(value.to_bits())
    }

    /// Convert two u16 registers to f32 (IEEE 754, big-endian)
    pub fn registers_to_f32(registers: &[u16]) -> ModbusResult<f32> {
        let u32_value = registers_to_u32(registers)?;
        Ok(f32::from_bits(u32_value))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_function_conversion() {
        assert_eq!(
            ModbusFunction::from_u8(0x03).unwrap(),
            ModbusFunction::ReadHoldingRegisters
        );
        assert_eq!(ModbusFunction::ReadHoldingRegisters.to_u8(), 0x03);

        assert!(ModbusFunction::from_u8(0xFF).is_err());
    }

    #[test]
    fn test_exception_conversion() {
        assert_eq!(
            ModbusException::from_u8(0x02).unwrap(),
            ModbusException::IllegalDataAddress
        );
        assert_eq!(ModbusException::IllegalDataAddress.to_u8(), 0x02);
    }

    #[test]
    fn test_request_validation() {
        let valid_request =
            ModbusRequest::new_read(1, ModbusFunction::ReadHoldingRegisters, 100, 10);
        assert!(valid_request.validate().is_ok());

        let invalid_slave =
            ModbusRequest::new_read(0, ModbusFunction::ReadHoldingRegisters, 100, 10);
        assert!(invalid_slave.validate().is_err());

        let too_many_registers =
            ModbusRequest::new_read(1, ModbusFunction::ReadHoldingRegisters, 100, 200);
        assert!(too_many_registers.validate().is_err());
    }

    #[test]
    fn test_data_utils() {
        let registers = vec![0x1234, 0x5678];
        let bytes = data_utils::registers_to_bytes(&registers);
        assert_eq!(bytes, vec![0x12, 0x34, 0x56, 0x78]);

        let back_to_registers = data_utils::bytes_to_registers(&bytes).unwrap();
        assert_eq!(back_to_registers, registers);

        let bits = vec![true, false, true, true, false, false, false, false];
        let packed = data_utils::pack_bits(&bits);
        let unpacked = data_utils::unpack_bits(&packed, bits.len());
        assert_eq!(unpacked, bits);
    }

    #[test]
    fn test_response_parsing() {
        // Test register response
        let register_data = vec![4, 0x12, 0x34, 0x56, 0x78]; // byte_count + 2 registers
        let response =
            ModbusResponse::new_success(1, ModbusFunction::ReadHoldingRegisters, register_data);
        let registers = response.parse_registers().unwrap();
        assert_eq!(registers, vec![0x1234, 0x5678]);

        // Test bit response
        let bit_data = vec![1, 0b10101010]; // byte_count + 1 byte
        let response = ModbusResponse::new_success(1, ModbusFunction::ReadCoils, bit_data);
        let bits = response.parse_bits().unwrap();
        assert_eq!(bits[0], false); // LSB first
        assert_eq!(bits[1], true);
        assert_eq!(bits[2], false);
        assert_eq!(bits[3], true);
    }
}
