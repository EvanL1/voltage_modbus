//! # Modbus Protocol Implementation
//!
//! This module provides comprehensive Modbus protocol support including:
//! - Standard Modbus function codes (0x01-0x10, 0x16, 0x17, 0x2B)
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
//! ### Extended Functions
//! - **0x16**: Mask Write Register - Atomic read-modify-write via AND/OR masks
//! - **0x17**: Read/Write Multiple Registers - Write then read in one transaction
//! - **0x2B**: Read Device Identification (MEI 0x0E) - Vendor/product/revision objects
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

/// Modbus protocol definitions and data structures
///
/// This module contains the core Modbus protocol definitions, including
/// function codes, data types, and request/response structures.
///
/// ## no_std compatibility
///
/// This module is no_std compatible. It uses `alloc` for `Vec` and `format!`
/// (required for `ModbusRequest`/`ModbusResponse` which own heap-allocated data),
/// and `core::fmt` for `Display` implementations.
#[cfg(not(feature = "std"))]
use alloc::{format, string::ToString, vec, vec::Vec};

use core::fmt;

use crate::constants::MEI_READ_DEVICE_ID;
use crate::error::{ModbusError, ModbusResult};
use crate::pdu::ModbusPdu;

/// Modbus address type (0-65535)
pub type ModbusAddress = u16;

/// Modbus value type (16-bit register value)
pub type ModbusValue = u16;

/// Modbus slave/unit identifier (1-247)
pub type SlaveId = u8;

/// Modbus function codes
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
    /// Read Exception Status (0x07, serial line) — 8 device-defined status bits
    ReadExceptionStatus = 0x07,
    /// Diagnostics (0x08, serial line) — sub-function 0x0000 is the echo test
    Diagnostics = 0x08,
    /// Get Comm Event Counter (0x0B, serial line)
    GetCommEventCounter = 0x0B,
    /// Get Comm Event Log (0x0C, serial line)
    GetCommEventLog = 0x0C,
    /// Write Multiple Coils (0x0F)
    WriteMultipleCoils = 0x0F,
    /// Write Multiple Registers (0x10)
    WriteMultipleRegisters = 0x10,
    /// Report Server ID (0x11, serial line)
    ReportServerId = 0x11,
    /// Mask Write Register (0x16)
    MaskWriteRegister = 0x16,
    /// Read/Write Multiple Registers (0x17) — write is performed before read
    ReadWriteMultipleRegisters = 0x17,
    /// Read Device Identification (0x2B, MEI type 0x0E)
    ReadDeviceIdentification = 0x2B,
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
            0x07 => Ok(ModbusFunction::ReadExceptionStatus),
            0x08 => Ok(ModbusFunction::Diagnostics),
            0x0B => Ok(ModbusFunction::GetCommEventCounter),
            0x0C => Ok(ModbusFunction::GetCommEventLog),
            0x0F => Ok(ModbusFunction::WriteMultipleCoils),
            0x10 => Ok(ModbusFunction::WriteMultipleRegisters),
            0x11 => Ok(ModbusFunction::ReportServerId),
            0x16 => Ok(ModbusFunction::MaskWriteRegister),
            0x17 => Ok(ModbusFunction::ReadWriteMultipleRegisters),
            0x2B => Ok(ModbusFunction::ReadDeviceIdentification),
            _ => Err(ModbusError::invalid_function(value)),
        }
    }

    /// Convert to u8
    pub fn to_u8(self) -> u8 {
        self as u8
    }

    /// Check if this function reads data (its request carries a read
    /// address/quantity and its response returns data).
    ///
    /// Note: `ReadWriteMultipleRegisters` counts as a read — its primary
    /// address/quantity fields describe the read side.
    pub fn is_read_function(self) -> bool {
        matches!(
            self,
            ModbusFunction::ReadCoils
                | ModbusFunction::ReadDiscreteInputs
                | ModbusFunction::ReadHoldingRegisters
                | ModbusFunction::ReadInputRegisters
                | ModbusFunction::ReadWriteMultipleRegisters
        )
    }

    /// Check if this is a pure write function (no data returned beyond the
    /// echo). Only pure writes may be broadcast (slave_id = 0).
    pub fn is_write_function(self) -> bool {
        matches!(
            self,
            ModbusFunction::WriteSingleCoil
                | ModbusFunction::WriteSingleRegister
                | ModbusFunction::WriteMultipleCoils
                | ModbusFunction::WriteMultipleRegisters
                | ModbusFunction::MaskWriteRegister
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
            ModbusFunction::ReadExceptionStatus => "Read Exception Status",
            ModbusFunction::Diagnostics => "Diagnostics",
            ModbusFunction::GetCommEventCounter => "Get Comm Event Counter",
            ModbusFunction::GetCommEventLog => "Get Comm Event Log",
            ModbusFunction::WriteMultipleCoils => "Write Multiple Coils",
            ModbusFunction::WriteMultipleRegisters => "Write Multiple Registers",
            ModbusFunction::ReportServerId => "Report Server ID",
            ModbusFunction::MaskWriteRegister => "Mask Write Register",
            ModbusFunction::ReadWriteMultipleRegisters => "Read/Write Multiple Registers",
            ModbusFunction::ReadDeviceIdentification => "Read Device Identification",
        };
        write!(f, "{} (0x{:02X})", name, *self as u8)
    }
}

/// Modbus exception codes
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

    /// Create a new write request.
    ///
    /// For packed coil writes where the last byte is only partially used, use
    /// [`Self::new_write_multiple_coils`] to pass the exact coil quantity.
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

    /// Create a write-multiple-coils request with an explicit coil quantity.
    pub fn new_write_multiple_coils(
        slave_id: SlaveId,
        address: ModbusAddress,
        quantity: u16,
        data: Vec<u8>,
    ) -> Self {
        Self {
            slave_id,
            function: ModbusFunction::WriteMultipleCoils,
            address,
            quantity,
            data,
        }
    }

    /// Create a mask-write-register request (FC 0x16).
    ///
    /// The device computes `(current & and_mask) | (or_mask & !and_mask)`
    /// atomically, avoiding the read-modify-write race of FC03 + FC06.
    pub fn new_mask_write(
        slave_id: SlaveId,
        address: ModbusAddress,
        and_mask: u16,
        or_mask: u16,
    ) -> Self {
        let mut data = Vec::with_capacity(4);
        data.extend_from_slice(&and_mask.to_be_bytes());
        data.extend_from_slice(&or_mask.to_be_bytes());
        Self {
            slave_id,
            function: ModbusFunction::MaskWriteRegister,
            address,
            quantity: 1,
            data,
        }
    }

    /// Create a read/write-multiple-registers request (FC 0x17).
    ///
    /// The device performs the write **before** the read, in a single
    /// transaction. `address`/`quantity` fields hold the read side; the write
    /// side is encoded into `data` as `write_address(2) + write_quantity(2) +
    /// byte_count(1) + values`.
    pub fn new_read_write_multiple(
        slave_id: SlaveId,
        read_address: ModbusAddress,
        read_quantity: u16,
        write_address: ModbusAddress,
        values: &[u16],
    ) -> Self {
        let mut data = Vec::with_capacity(5 + values.len() * 2);
        data.extend_from_slice(&write_address.to_be_bytes());
        data.extend_from_slice(&(values.len() as u16).to_be_bytes());
        data.push((values.len() * 2) as u8);
        for &value in values {
            data.extend_from_slice(&value.to_be_bytes());
        }
        Self {
            slave_id,
            function: ModbusFunction::ReadWriteMultipleRegisters,
            address: read_address,
            quantity: read_quantity,
            data,
        }
    }

    /// Create a request that carries only the function code — used by the
    /// serial-line diagnostic functions FC 0x07 / 0x0B / 0x0C / 0x11.
    pub fn new_no_data(slave_id: SlaveId, function: ModbusFunction) -> Self {
        Self {
            slave_id,
            function,
            address: 0,
            quantity: 0,
            data: Vec::new(),
        }
    }

    /// Create a diagnostics request (FC 0x08).
    ///
    /// `sub_function` 0x0000 is the Return Query Data echo test; the device
    /// must echo `data` back unchanged. Other sub-functions return counters
    /// or control line diagnostics (device-dependent support).
    pub fn new_diagnostics(slave_id: SlaveId, sub_function: u16, data: u16) -> Self {
        let mut payload = Vec::with_capacity(4);
        payload.extend_from_slice(&sub_function.to_be_bytes());
        payload.extend_from_slice(&data.to_be_bytes());
        Self {
            slave_id,
            function: ModbusFunction::Diagnostics,
            address: 0,
            quantity: 0,
            data: payload,
        }
    }

    /// Create a read-device-identification request (FC 0x2B / MEI 0x0E).
    ///
    /// `read_code`: 1 = basic objects, 2 = regular, 3 = extended,
    /// 4 = one specific object. `object_id` is the object to start from
    /// (0x00 VendorName, 0x01 ProductCode, 0x02 MajorMinorRevision, ...).
    pub fn new_read_device_identification(slave_id: SlaveId, read_code: u8, object_id: u8) -> Self {
        Self {
            slave_id,
            function: ModbusFunction::ReadDeviceIdentification,
            address: 0,
            quantity: 0,
            data: vec![MEI_READ_DEVICE_ID, read_code, object_id],
        }
    }

    /// Encode this request's PDU — function code + payload, excluding all
    /// transport framing (no unit/slave id, MBAP header, CRC or LRC).
    ///
    /// Every transport (TCP, RTU, ASCII, RTU-over-TCP, embedded) shares this
    /// encoder, so adding a function code here enables it everywhere at once.
    pub fn encode_pdu(&self) -> ModbusResult<ModbusPdu> {
        let mut pdu = ModbusPdu::new();
        pdu.push(self.function.to_u8())?;

        match self.function {
            ModbusFunction::ReadCoils
            | ModbusFunction::ReadDiscreteInputs
            | ModbusFunction::ReadHoldingRegisters
            | ModbusFunction::ReadInputRegisters => {
                pdu.push_u16(self.address)?;
                pdu.push_u16(self.quantity)?;
            }

            ModbusFunction::WriteSingleCoil => {
                pdu.push_u16(self.address)?;
                let value: u16 = if !self.data.is_empty() && self.data[0] != 0 {
                    0xFF00
                } else {
                    0x0000
                };
                pdu.push_u16(value)?;
            }

            ModbusFunction::WriteSingleRegister => {
                pdu.push_u16(self.address)?;
                if self.data.len() >= 2 {
                    pdu.extend(&self.data[..2])?;
                } else {
                    pdu.extend(&[0, 0])?;
                }
            }

            ModbusFunction::WriteMultipleCoils | ModbusFunction::WriteMultipleRegisters => {
                pdu.push_u16(self.address)?;
                pdu.push_u16(self.quantity)?;
                let byte_count = u8::try_from(self.data.len()).map_err(|_| {
                    ModbusError::invalid_data("data payload too large for Modbus frame")
                })?;
                pdu.push(byte_count)?;
                pdu.extend(&self.data)?;
            }

            ModbusFunction::MaskWriteRegister => {
                // data = and_mask(2) + or_mask(2), validated in validate()
                pdu.push_u16(self.address)?;
                pdu.extend(&self.data)?;
            }

            ModbusFunction::ReadWriteMultipleRegisters => {
                // address/quantity = read side;
                // data = write_address(2) + write_quantity(2) + byte_count(1) + values
                pdu.push_u16(self.address)?;
                pdu.push_u16(self.quantity)?;
                pdu.extend(&self.data)?;
            }

            ModbusFunction::ReadDeviceIdentification => {
                // data = MEI type(1) + ReadDeviceId code(1) + object id(1)
                pdu.extend(&self.data)?;
            }

            ModbusFunction::ReadExceptionStatus
            | ModbusFunction::GetCommEventCounter
            | ModbusFunction::GetCommEventLog
            | ModbusFunction::ReportServerId => {
                // Function code only, no payload
            }

            ModbusFunction::Diagnostics => {
                // data = sub-function(2) + data field(2N)
                pdu.extend(&self.data)?;
            }
        }

        Ok(pdu)
    }

    /// Validate the request
    pub fn validate(&self) -> ModbusResult<()> {
        // Validate slave ID — 0 is the broadcast address (valid for write only), 1–247 are unicast
        if self.slave_id > 247 {
            return Err(ModbusError::invalid_data(format!(
                "Invalid slave ID: {} (must be 0-247)",
                self.slave_id
            )));
        }

        // Broadcast (slave_id = 0) is only valid for pure write operations per
        // the Modbus spec — any function that returns data has no way to reply.
        if self.slave_id == 0 && !self.function.is_write_function() {
            return Err(ModbusError::invalid_data(
                "Broadcast (slave_id=0) is only valid for write operations",
            ));
        }

        // Validate quantity for read operations
        if self.function.is_read_function() {
            validate_address_range(self.address, self.quantity)?;

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
                ModbusFunction::ReadWriteMultipleRegisters
                    if self.quantity > crate::constants::MAX_RW_READ_REGISTERS as u16 =>
                {
                    return Err(ModbusError::invalid_data(format!(
                        "Too many registers requested: {}",
                        self.quantity
                    )));
                }
                _ => {}
            }
        }

        match self.function {
            ModbusFunction::WriteSingleCoil => {
                validate_address_range(self.address, 1)?;
                match self.data.as_slice() {
                    [_] => {}
                    [hi, lo] if u16::from_be_bytes([*hi, *lo]) == 0x0000 => {}
                    [hi, lo] if u16::from_be_bytes([*hi, *lo]) == 0xFF00 => {}
                    _ => {
                        return Err(ModbusError::invalid_data(
                            "Invalid single coil payload; expected one boolean byte or 0x0000/0xFF00",
                        ));
                    }
                }
            }
            ModbusFunction::WriteSingleRegister => {
                validate_address_range(self.address, 1)?;
                if self.data.len() != 2 {
                    return Err(ModbusError::invalid_data(format!(
                        "Invalid single register payload length: expected 2, got {}",
                        self.data.len()
                    )));
                }
            }
            ModbusFunction::WriteMultipleCoils => {
                validate_address_range(self.address, self.quantity)?;
                if self.quantity > crate::MAX_WRITE_COILS as u16 {
                    return Err(ModbusError::invalid_data(format!(
                        "Too many coils to write: {}",
                        self.quantity
                    )));
                }
                let expected_bytes = usize::from(self.quantity.div_ceil(8));
                if self.data.len() != expected_bytes {
                    return Err(ModbusError::invalid_data(format!(
                        "Invalid coil payload length: expected {}, got {}",
                        expected_bytes,
                        self.data.len()
                    )));
                }
            }
            ModbusFunction::WriteMultipleRegisters => {
                validate_address_range(self.address, self.quantity)?;
                if self.quantity > crate::MAX_WRITE_REGISTERS as u16 {
                    return Err(ModbusError::invalid_data(format!(
                        "Too many registers to write: {}",
                        self.quantity
                    )));
                }
                let expected_bytes = usize::from(self.quantity) * 2;
                if self.data.len() != expected_bytes {
                    return Err(ModbusError::invalid_data(format!(
                        "Invalid register payload length: expected {}, got {}",
                        expected_bytes,
                        self.data.len()
                    )));
                }
            }
            ModbusFunction::MaskWriteRegister => {
                validate_address_range(self.address, 1)?;
                if self.data.len() != 4 {
                    return Err(ModbusError::invalid_data(format!(
                        "Invalid mask write payload length: expected 4 (and+or masks), got {}",
                        self.data.len()
                    )));
                }
            }
            ModbusFunction::ReadWriteMultipleRegisters => {
                // data = write_address(2) + write_quantity(2) + byte_count(1) + values
                if self.data.len() < 5 {
                    return Err(ModbusError::invalid_data(
                        "Read/write multiple payload too short",
                    ));
                }
                let write_address = u16::from_be_bytes([self.data[0], self.data[1]]);
                let write_quantity = u16::from_be_bytes([self.data[2], self.data[3]]);
                let byte_count = usize::from(self.data[4]);
                if write_quantity == 0
                    || write_quantity > crate::constants::MAX_RW_WRITE_REGISTERS as u16
                {
                    return Err(ModbusError::invalid_data(format!(
                        "Invalid read/write multiple write quantity: {}",
                        write_quantity
                    )));
                }
                validate_address_range(write_address, write_quantity)?;
                if byte_count != usize::from(write_quantity) * 2
                    || self.data.len() != 5 + byte_count
                {
                    return Err(ModbusError::invalid_data(
                        "Invalid read/write multiple payload length",
                    ));
                }
            }
            ModbusFunction::ReadDeviceIdentification => {
                // data = MEI type(1) + ReadDeviceId code(1) + object id(1)
                if self.data.len() != 3 || self.data[0] != MEI_READ_DEVICE_ID {
                    return Err(ModbusError::invalid_data(
                        "Invalid device identification payload (expect MEI 0x0E + code + object id)",
                    ));
                }
                if !(1..=4).contains(&self.data[1]) {
                    return Err(ModbusError::invalid_data(format!(
                        "Invalid ReadDeviceId code: {} (must be 1-4)",
                        self.data[1]
                    )));
                }
            }
            ModbusFunction::ReadExceptionStatus
            | ModbusFunction::GetCommEventCounter
            | ModbusFunction::GetCommEventLog
            | ModbusFunction::ReportServerId => {
                if !self.data.is_empty() {
                    return Err(ModbusError::invalid_data(
                        "This diagnostic function takes no request payload",
                    ));
                }
            }
            // data = sub-function(2) + data field(2N)
            ModbusFunction::Diagnostics if self.data.len() < 4 || self.data.len() % 2 != 0 => {
                return Err(ModbusError::invalid_data(
                    "Invalid diagnostics payload (expect sub-function + 16-bit data)",
                ));
            }
            _ => {}
        }

        Ok(())
    }
}

#[inline]
fn validate_address_range(address: ModbusAddress, quantity: u16) -> ModbusResult<()> {
    if quantity == 0 {
        return Err(ModbusError::invalid_address(address, quantity));
    }

    if address.checked_add(quantity - 1).is_none() {
        return Err(ModbusError::invalid_address(address, quantity));
    }

    Ok(())
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
    /// Decoded exception, `None` for success *and* for exception codes
    /// outside [`ModbusException`]; use [`Self::is_exception`] to test.
    pub exception: Option<ModbusException>,
    /// Raw exception code as received, kept so unknown codes still count as
    /// exceptions and errors carry the exact code.
    exception_code: Option<u8>,
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
            exception_code: None,
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
            exception_code: None,
        }
    }

    /// Create a synthetic acknowledgement for broadcast messages (slave_id = 0).
    ///
    /// Broadcast messages are sent to all slaves simultaneously and, per the Modbus
    /// specification, **no response is expected or returned**. This method produces a
    /// zero-data response that callers can use as a success indicator without waiting
    /// for a real reply.
    pub fn new_broadcast_ack(function: ModbusFunction) -> Self {
        Self {
            slave_id: 0,
            function,
            buffer: Vec::new(),
            data_offset: 0,
            data_len: 0,
            exception: None,
            exception_code: None,
        }
    }

    /// Create an exception response
    pub fn new_exception(slave_id: SlaveId, function: ModbusFunction, exception_code: u8) -> Self {
        Self {
            slave_id,
            function,
            buffer: Vec::new(),
            data_offset: 0,
            data_len: 0,
            exception: ModbusException::from_u8(exception_code),
            exception_code: Some(exception_code),
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
        self.exception_code.is_some()
    }

    /// Get exception error if present
    pub fn get_exception(&self) -> Option<ModbusError> {
        self.exception_code
            .map(|code| ModbusError::exception(self.function.to_u8(), code))
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

    /// Parse response data as a device identification block (FC 0x2B / MEI 0x0E)
    pub fn parse_device_identification(&self) -> ModbusResult<DeviceIdentification> {
        if self.is_exception() {
            return Err(self.get_exception().unwrap());
        }
        DeviceIdentification::parse(self.data())
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

/// Parsed get-comm-event-log response (FC 0x0C)
#[derive(Debug, Clone, PartialEq)]
pub struct CommEventLog {
    /// Device status word (0x0000 ready, 0xFFFF busy)
    pub status: u16,
    /// Event counter
    pub event_count: u16,
    /// Bus message counter
    pub message_count: u16,
    /// Raw event bytes, newest first (0-64 entries, device-defined encoding)
    pub events: Vec<u8>,
}

/// Parsed report-server-id response (FC 0x11)
#[derive(Debug, Clone, PartialEq)]
pub struct ServerIdReport {
    /// Device-specific server id bytes
    pub server_id: Vec<u8>,
    /// Run indicator: `Some(true)` = ON (0xFF), `Some(false)` = OFF (0x00),
    /// `None` if the device omits it or uses a non-standard trailer
    pub run_indicator_on: Option<bool>,
}

impl ServerIdReport {
    /// Parse from a response payload (bytes after the byte-count prefix)
    pub fn parse(payload: &[u8]) -> Self {
        match payload.split_last() {
            Some((&0xFF, id)) => Self {
                server_id: id.to_vec(),
                run_indicator_on: Some(true),
            },
            Some((&0x00, id)) => Self {
                server_id: id.to_vec(),
                run_indicator_on: Some(false),
            },
            _ => Self {
                server_id: payload.to_vec(),
                run_indicator_on: None,
            },
        }
    }
}

/// A single device-identification object (FC 0x2B / MEI 0x0E)
#[derive(Debug, Clone, PartialEq)]
pub struct DeviceIdObject {
    /// Object id: 0x00 VendorName, 0x01 ProductCode, 0x02 MajorMinorRevision,
    /// 0x03 VendorUrl, 0x04 ProductName, 0x05 ModelName, 0x06 UserApplicationName,
    /// 0x80+ device-specific
    pub id: u8,
    /// Raw object value (usually ASCII text)
    pub value: Vec<u8>,
}

impl DeviceIdObject {
    /// Object value as UTF-8 text, if valid
    pub fn as_str(&self) -> Option<&str> {
        core::str::from_utf8(&self.value).ok()
    }
}

/// Parsed read-device-identification response (FC 0x2B / MEI 0x0E)
#[derive(Debug, Clone, PartialEq)]
pub struct DeviceIdentification {
    /// Conformity level reported by the device (0x01-0x03, 0x81-0x83)
    pub conformity_level: u8,
    /// `true` when more objects remain than fit in this response; issue a
    /// follow-up request starting at [`Self::next_object_id`]
    pub more_follows: bool,
    /// Object id to continue from when `more_follows` is set
    pub next_object_id: u8,
    /// Objects returned in this response
    pub objects: Vec<DeviceIdObject>,
}

impl DeviceIdentification {
    /// Parse from a response PDU payload (the bytes after the 0x2B function code):
    /// `MEI type(1) + read code(1) + conformity(1) + more follows(1) +
    /// next object id(1) + object count(1) + [id(1) len(1) value(len)]...`
    pub fn parse(data: &[u8]) -> ModbusResult<Self> {
        if data.len() < 6 {
            return Err(ModbusError::frame(
                "Device identification response too short",
            ));
        }
        if data[0] != MEI_READ_DEVICE_ID {
            return Err(ModbusError::frame(format!(
                "Unexpected MEI type: 0x{:02X} (expected 0x0E)",
                data[0]
            )));
        }

        let conformity_level = data[2];
        let more_follows = data[3] == 0xFF;
        let next_object_id = data[4];
        let object_count = usize::from(data[5]);

        let mut objects = Vec::with_capacity(object_count);
        let mut pos = 6;
        for _ in 0..object_count {
            if pos + 2 > data.len() {
                return Err(ModbusError::frame("Truncated device identification object"));
            }
            let id = data[pos];
            let len = usize::from(data[pos + 1]);
            pos += 2;
            if pos + len > data.len() {
                return Err(ModbusError::frame("Truncated device identification value"));
            }
            objects.push(DeviceIdObject {
                id,
                value: data[pos..pos + len].to_vec(),
            });
            pos += len;
        }

        Ok(Self {
            conformity_level,
            more_follows,
            next_object_id,
            objects,
        })
    }

    /// Look up an object's raw value by id
    pub fn object(&self, id: u8) -> Option<&[u8]> {
        self.objects
            .iter()
            .find(|obj| obj.id == id)
            .map(|obj| obj.value.as_slice())
    }
}

/// Data conversion utilities
pub mod data_utils {
    use super::*;

    #[cfg(not(feature = "std"))]
    use alloc::vec;

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
        let byte_count = bits.len().div_ceil(8);
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
    #[cfg(not(feature = "std"))]
    use alloc::vec;

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

    /// Exception responses must surface as `ModbusError::Exception` carrying
    /// the raw code, so `is_recoverable()` can retry Acknowledge/Busy.
    #[test]
    fn test_exception_response_keeps_code() {
        let busy = ModbusResponse::new_exception(1, ModbusFunction::ReadHoldingRegisters, 0x06);
        match busy.get_exception() {
            Some(ModbusError::Exception { function, code, .. }) => {
                assert_eq!(function, 0x03);
                assert_eq!(code, 0x06);
            }
            other => panic!("expected Exception error, got {:?}", other),
        }
        assert!(busy.get_exception().unwrap().is_recoverable());
        assert!(busy.parse_registers().is_err());
    }

    /// Codes outside the known `ModbusException` list are still exceptions.
    #[test]
    fn test_unknown_exception_code_is_still_exception() {
        let resp = ModbusResponse::new_exception(1, ModbusFunction::ReadHoldingRegisters, 0x0C);
        assert!(resp.is_exception());
        assert!(matches!(
            resp.get_exception(),
            Some(ModbusError::Exception { code: 0x0C, .. })
        ));
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

        let address_overflow =
            ModbusRequest::new_read(1, ModbusFunction::ReadHoldingRegisters, u16::MAX, 2);
        assert!(address_overflow.validate().is_err());

        let valid_write_multiple = ModbusRequest {
            slave_id: 1,
            function: ModbusFunction::WriteMultipleRegisters,
            address: 10,
            quantity: 2,
            data: vec![0x12, 0x34, 0x56, 0x78],
        };
        assert!(valid_write_multiple.validate().is_ok());

        let invalid_write_payload = ModbusRequest {
            slave_id: 1,
            function: ModbusFunction::WriteMultipleRegisters,
            address: 10,
            quantity: 2,
            data: vec![0x12, 0x34],
        };
        assert!(invalid_write_payload.validate().is_err());

        let empty_single_register =
            ModbusRequest::new_write(1, ModbusFunction::WriteSingleRegister, 10, vec![]);
        assert!(empty_single_register.validate().is_err());

        let invalid_single_coil =
            ModbusRequest::new_write(1, ModbusFunction::WriteSingleCoil, 10, vec![0x00, 0x01]);
        assert!(invalid_single_coil.validate().is_err());
    }

    #[test]
    fn test_write_multiple_coils_preserves_explicit_quantity() {
        let req = ModbusRequest::new_write_multiple_coils(1, 10, 9, vec![0xFF, 0x01]);

        assert_eq!(req.quantity, 9);
        assert!(req.validate().is_ok());
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
        assert!(!bits[0]); // LSB first
        assert!(bits[1]);
        assert!(!bits[2]);
        assert!(bits[3]);
    }

    // -------------------------------------------------------------------------
    // Broadcast (slave_id = 0) tests
    // -------------------------------------------------------------------------

    #[test]
    fn test_broadcast_read_rejected() {
        // slave_id = 0 with any read function must be rejected
        for fc in [
            ModbusFunction::ReadCoils,
            ModbusFunction::ReadDiscreteInputs,
            ModbusFunction::ReadHoldingRegisters,
            ModbusFunction::ReadInputRegisters,
        ] {
            let req = ModbusRequest::new_read(0, fc, 0, 1);
            let err = req.validate().unwrap_err();
            assert!(
                err.to_string().contains("Broadcast"),
                "expected broadcast error for {fc:?}, got: {err}"
            );
        }
    }

    #[test]
    fn test_broadcast_write_validates_ok() {
        // slave_id = 0 with write functions must pass validation
        for fc in [
            ModbusFunction::WriteSingleCoil,
            ModbusFunction::WriteSingleRegister,
            ModbusFunction::WriteMultipleCoils,
            ModbusFunction::WriteMultipleRegisters,
        ] {
            let req = ModbusRequest::new_write(0, fc, 0, vec![0xFF, 0x00]);
            assert!(
                req.validate().is_ok(),
                "broadcast write should be valid for {fc:?}"
            );
        }
    }

    #[test]
    fn test_broadcast_ack_response() {
        let ack = ModbusResponse::new_broadcast_ack(ModbusFunction::WriteSingleRegister);
        assert_eq!(ack.slave_id, 0);
        assert_eq!(ack.function, ModbusFunction::WriteSingleRegister);
        assert!(!ack.is_exception());
        assert_eq!(ack.data_len(), 0);
        assert!(ack.data().is_empty());
    }

    #[test]
    fn test_invalid_slave_id_above_247() {
        let req = ModbusRequest::new_read(248, ModbusFunction::ReadHoldingRegisters, 0, 1);
        assert!(req.validate().is_err());
    }

    // -------------------------------------------------------------------------
    // Shared PDU encoder
    // -------------------------------------------------------------------------

    #[test]
    fn test_encode_pdu_known_bytes() {
        let req = ModbusRequest::new_read(1, ModbusFunction::ReadHoldingRegisters, 0x006B, 3);
        assert_eq!(
            req.encode_pdu().unwrap().as_slice(),
            &[0x03, 0x00, 0x6B, 0x00, 0x03]
        );

        // FC05 write coil ON normalizes to 0xFF00
        let req = ModbusRequest::new_write(1, ModbusFunction::WriteSingleCoil, 0x00AC, vec![0x01]);
        assert_eq!(
            req.encode_pdu().unwrap().as_slice(),
            &[0x05, 0x00, 0xAC, 0xFF, 0x00]
        );

        let req = ModbusRequest {
            slave_id: 1,
            function: ModbusFunction::WriteMultipleRegisters,
            address: 0x0001,
            quantity: 2,
            data: vec![0x00, 0x0A, 0x01, 0x02],
        };
        assert_eq!(
            req.encode_pdu().unwrap().as_slice(),
            &[0x10, 0x00, 0x01, 0x00, 0x02, 0x04, 0x00, 0x0A, 0x01, 0x02]
        );
    }

    #[test]
    fn test_encode_pdu_mask_write() {
        // Spec example: address 4, and 0x00F2, or 0x0025
        let req = ModbusRequest::new_mask_write(1, 0x0004, 0x00F2, 0x0025);
        assert!(req.validate().is_ok());
        assert_eq!(
            req.encode_pdu().unwrap().as_slice(),
            &[0x16, 0x00, 0x04, 0x00, 0xF2, 0x00, 0x25]
        );
    }

    #[test]
    fn test_encode_pdu_read_write_multiple() {
        // Spec example shape: read 6 regs from 3, write [0x00FF] at 14
        let req = ModbusRequest::new_read_write_multiple(1, 0x0003, 6, 0x000E, &[0x00FF]);
        assert!(req.validate().is_ok());
        assert_eq!(
            req.encode_pdu().unwrap().as_slice(),
            &[0x17, 0x00, 0x03, 0x00, 0x06, 0x00, 0x0E, 0x00, 0x01, 0x02, 0x00, 0xFF]
        );
    }

    #[test]
    fn test_encode_pdu_device_identification() {
        let req = ModbusRequest::new_read_device_identification(1, 1, 0);
        assert!(req.validate().is_ok());
        assert_eq!(
            req.encode_pdu().unwrap().as_slice(),
            &[0x2B, 0x0E, 0x01, 0x00]
        );
    }

    #[test]
    fn test_new_fc_broadcast_rules() {
        // Mask write is a pure write → broadcast allowed
        assert!(ModbusRequest::new_mask_write(0, 0, 0xFFFF, 0)
            .validate()
            .is_ok());
        // FC17 and FC2B return data → broadcast rejected
        assert!(ModbusRequest::new_read_write_multiple(0, 0, 1, 0, &[1])
            .validate()
            .is_err());
        assert!(ModbusRequest::new_read_device_identification(0, 1, 0)
            .validate()
            .is_err());
    }

    #[test]
    fn test_read_write_multiple_validation_limits() {
        // Read quantity above 125 rejected
        let req = ModbusRequest::new_read_write_multiple(1, 0, 126, 0, &[1]);
        assert!(req.validate().is_err());
        // Write quantity above 121 rejected
        let values = [0u16; 122];
        let req = ModbusRequest::new_read_write_multiple(1, 0, 1, 0, &values);
        assert!(req.validate().is_err());
    }

    #[test]
    fn test_encode_pdu_serial_diagnostics() {
        // No-payload functions encode as the bare function code
        for (function, code) in [
            (ModbusFunction::ReadExceptionStatus, 0x07),
            (ModbusFunction::GetCommEventCounter, 0x0B),
            (ModbusFunction::GetCommEventLog, 0x0C),
            (ModbusFunction::ReportServerId, 0x11),
        ] {
            let req = ModbusRequest::new_no_data(1, function);
            assert!(req.validate().is_ok());
            assert_eq!(req.encode_pdu().unwrap().as_slice(), &[code]);
        }

        // Diagnostics: sub-function + data
        let req = ModbusRequest::new_diagnostics(1, 0x0000, 0xA537);
        assert!(req.validate().is_ok());
        assert_eq!(
            req.encode_pdu().unwrap().as_slice(),
            &[0x08, 0x00, 0x00, 0xA5, 0x37]
        );
    }

    #[test]
    fn test_serial_diagnostics_broadcast_rejected() {
        // All diagnostic functions require a response → no broadcast
        for function in [
            ModbusFunction::ReadExceptionStatus,
            ModbusFunction::GetCommEventCounter,
            ModbusFunction::GetCommEventLog,
            ModbusFunction::ReportServerId,
        ] {
            assert!(ModbusRequest::new_no_data(0, function).validate().is_err());
        }
        assert!(ModbusRequest::new_diagnostics(0, 0, 0).validate().is_err());
    }

    #[test]
    fn test_server_id_report_parse() {
        let report = ServerIdReport::parse(&[b'P', b'M', b'1', 0xFF]);
        assert_eq!(report.server_id, b"PM1");
        assert_eq!(report.run_indicator_on, Some(true));

        let report = ServerIdReport::parse(&[0x42, 0x00]);
        assert_eq!(report.server_id, vec![0x42]);
        assert_eq!(report.run_indicator_on, Some(false));

        // Non-standard trailer: keep everything as id
        let report = ServerIdReport::parse(&[0x01, 0x02]);
        assert_eq!(report.server_id, vec![0x01, 0x02]);
        assert_eq!(report.run_indicator_on, None);
    }

    #[test]
    fn test_device_identification_parse() {
        let data = [
            0x0E, 0x01, 0x01, 0x00, 0x00,
            0x03, // header: mei, code, conformity, more, next, count
            0x00, 0x07, b'V', b'e', b'n', b'd', b'o', b'r', b'X', // VendorName
            0x01, 0x02, b'P', b'C', // ProductCode
            0x02, 0x04, b'V', b'2', b'.', b'1', // Revision
        ];
        let ident = DeviceIdentification::parse(&data).unwrap();
        assert_eq!(ident.conformity_level, 0x01);
        assert!(!ident.more_follows);
        assert_eq!(ident.objects.len(), 3);
        assert_eq!(ident.object(0x00), Some(&b"VendorX"[..]));
        assert_eq!(ident.objects[2].as_str(), Some("V2.1"));

        // Truncated object must error, not panic
        assert!(DeviceIdentification::parse(&data[..8]).is_err());
        assert!(DeviceIdentification::parse(&data[..3]).is_err());
    }
}
