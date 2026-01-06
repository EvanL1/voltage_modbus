//! # Voltage Modbus Error Handling
//!
//! This module provides comprehensive error handling for the Voltage Modbus library,
//! covering all aspects of Modbus communication including network transport, protocol
//! parsing, data validation, and device interaction errors.
//!
//! ## Overview
//!
//! The error system is designed to provide clear, actionable error information for
//! different failure scenarios in Modbus communication. All errors implement standard
//! Rust error traits and provide detailed context information to help with debugging
//! and error recovery.
//!
//! ## Error Categories
//!
//! ### Transport Errors
//! - **I/O Errors**: Network communication failures, serial port issues
//! - **Connection Errors**: Connection establishment and maintenance problems
//! - **Timeout Errors**: Operation timeouts with specific context
//!
//! ### Protocol Errors
//! - **Protocol Errors**: Modbus protocol specification violations
//! - **Frame Errors**: Message frame parsing and validation failures
//! - **CRC Errors**: Checksum validation failures for RTU communication
//! - **Exception Responses**: Standard Modbus exception codes from devices
//!
//! ### Data Errors
//! - **Invalid Function**: Unsupported or malformed function codes
//! - **Invalid Address**: Address range validation failures
//! - **Invalid Data**: Data format and validation errors
//!
//! ### System Errors
//! - **Configuration Errors**: Client/server configuration issues
//! - **Device Errors**: Device-specific communication problems
//! - **Internal Errors**: Library internal errors (should not occur in normal operation)
//!
//! ## Error Recovery
//!
//! Many errors provide information about recoverability:
//!
//! ```rust
//! use voltage_modbus::{ModbusError, ModbusResult};
//!
//! fn handle_error(result: ModbusResult<Vec<u16>>) {
//!     match result {
//!         Ok(data) => println!("Success: {:?}", data),
//!         Err(error) => {
//!             if error.is_recoverable() {
//!                 println!("Retryable error: {}", error);
//!                 // Implement retry logic
//!             } else {
//!                 println!("Fatal error: {}", error);
//!                 // Handle permanent failure
//!             }
//!         }
//!     }
//! }
//! ```
//!
//! ## Usage Examples
//!
//! ### Basic Error Handling
//!
//! ```rust
//! use voltage_modbus::{ModbusClient, ModbusError};
//!
//! async fn read_with_error_handling(client: &mut impl ModbusClient) {
//!     match client.read_03(1, 0, 10).await {
//!         Ok(registers) => {
//!             println!("Read {} registers", registers.len());
//!         },
//!         Err(ModbusError::Timeout { operation, timeout_ms }) => {
//!             println!("Timeout during {}: {}ms", operation, timeout_ms);
//!         },
//!         Err(ModbusError::Exception { function, code, message }) => {
//!             println!("Device exception: {} (function={:02X}, code={:02X})",
//!                      message, function, code);
//!         },
//!         Err(error) => {
//!             println!("Other error: {}", error);
//!         }
//!     }
//! }
//!
//! // Example showing function code naming
//! async fn example_with_function_codes(client: &mut impl ModbusClient) {
//!     // Read holding registers using function code naming
//!     match client.read_03(1, 0, 10).await {
//!         Ok(registers) => println!("Values: {:?}", registers),
//!         Err(ModbusError::Timeout { operation, timeout_ms }) => {
//!             println!("Request {} timed out after {}ms, retrying...", operation, timeout_ms);
//!             // Implement retry logic
//!         },
//!         Err(e) => println!("Error: {}", e),
//!     }
//! }
//! ```
//!
//! ### Error Classification
//!
//! ```rust
//! use voltage_modbus::ModbusError;
//!
//! fn classify_error(error: &ModbusError) {
//!     if error.is_transport_error() {
//!         println!("Network/transport issue: {}", error);
//!     } else if error.is_protocol_error() {
//!         println!("Modbus protocol issue: {}", error);
//!     } else {
//!         println!("Other issue: {}", error);
//!     }
//! }
//! ```
//!
//! ### Retry Logic
//!
//! ```rust
//! use voltage_modbus::{ModbusError, ModbusResult};
//! use tokio::time::{sleep, Duration};
//!
//! async fn read_with_retry<F, Fut>(operation: F, max_retries: usize) -> ModbusResult<Vec<u16>>
//! where
//!     F: Fn() -> Fut,
//!     Fut: std::future::Future<Output = ModbusResult<Vec<u16>>>,
//! {
//!     for attempt in 0..=max_retries {
//!         match operation().await {
//!             Ok(result) => return Ok(result),
//!             Err(error) if error.is_recoverable() && attempt < max_retries => {
//!                 println!("Attempt {} failed: {}", attempt + 1, error);
//!                 sleep(Duration::from_millis(100 * (attempt as u64 + 1))).await;
//!                 continue;
//!             },
//!             Err(error) => return Err(error),
//!         }
//!     }
//!     unreachable!()
//! }
//! ```

/// Modbus-specific error types
///
/// This module provides comprehensive error handling for all Modbus operations,
/// including network, protocol, and data validation errors.
use thiserror::Error;

/// Result type alias for Modbus operations
///
/// This is a convenience type alias that uses `ModbusError` as the error type
/// for all Modbus operations, providing consistent error handling throughout
/// the codebase.
pub type ModbusResult<T> = Result<T, ModbusError>;

/// Comprehensive Modbus error types
///
/// This enum covers all possible error conditions that can occur during Modbus
/// communication, from low-level I/O errors to high-level protocol violations.
/// Each variant provides detailed context about the specific failure, making it
/// easier to diagnose issues and implement appropriate recovery strategies.
#[derive(Error, Debug, Clone, PartialEq)]
pub enum ModbusError {
    /// I/O related errors (network, serial)
    ///
    /// Covers low-level I/O failures including network socket errors,
    /// serial port communication issues, and file system access problems.
    ///
    /// # Examples
    /// - TCP connection refused
    /// - Serial port access denied
    /// - Network interface down
    #[error("I/O error: {message}")]
    Io { message: String },

    /// Connection errors
    ///
    /// Specific to connection establishment and maintenance issues that
    /// are distinct from general I/O errors.
    ///
    /// # Examples
    /// - Connection refused by remote host
    /// - Connection lost during operation
    /// - Authentication failure
    #[error("Connection error: {message}")]
    Connection { message: String },

    /// Timeout errors
    ///
    /// Occurs when operations exceed their configured timeout limits.
    /// Includes context about which operation timed out and the timeout duration.
    ///
    /// # Examples
    /// - Read operation timeout
    /// - Write operation timeout
    /// - Connection establishment timeout
    #[error("Timeout after {timeout_ms}ms: {operation}")]
    Timeout { operation: String, timeout_ms: u64 },

    /// Protocol-level errors
    ///
    /// General Modbus protocol specification violations that don't fit
    /// into more specific categories.
    ///
    /// # Examples
    /// - Invalid message length
    /// - Unsupported protocol version
    /// - Protocol state machine violations
    #[error("Protocol error: {message}")]
    Protocol { message: String },

    /// Invalid function code
    ///
    /// Occurs when an unsupported or malformed Modbus function code is
    /// encountered, either in requests or responses.
    ///
    /// # Examples
    /// - Function code 0x99 (not in Modbus specification)
    /// - Function code that doesn't match expected response
    #[error("Invalid function code: {code}")]
    InvalidFunction { code: u8 },

    /// Invalid address range
    ///
    /// Address validation failures including out-of-range addresses,
    /// invalid quantity values, or address/quantity combinations that
    /// exceed protocol limits.
    ///
    /// # Examples
    /// - Reading 200 holding registers (max 125)
    /// - Starting address + quantity > 65535
    /// - Zero quantity in read request
    #[error("Invalid address: start={start}, count={count}")]
    InvalidAddress { start: u16, count: u16 },

    /// Invalid data value
    ///
    /// Data format and validation errors including values that don't
    /// conform to expected formats or ranges.
    ///
    /// # Examples
    /// - Coil value not 0x0000 or 0xFF00
    /// - Invalid string encoding in register data
    /// - Out-of-range numeric values
    #[error("Invalid data: {message}")]
    InvalidData { message: String },

    /// CRC validation failure
    ///
    /// Checksum validation failures in Modbus RTU communication.
    /// Provides both expected and actual CRC values for debugging.
    ///
    /// # Examples
    /// - Message corruption during transmission
    /// - Incorrect CRC calculation
    /// - Noise on communication line
    #[error("CRC validation failed: expected={expected:04X}, actual={actual:04X}")]
    CrcMismatch { expected: u16, actual: u16 },

    /// Modbus exception response
    ///
    /// Standard Modbus exception codes returned by devices to indicate
    /// various error conditions. Includes the original function code,
    /// exception code, and human-readable description.
    ///
    /// # Standard Exception Codes
    /// - 0x01: Illegal Function
    /// - 0x02: Illegal Data Address
    /// - 0x03: Illegal Data Value
    /// - 0x04: Slave Device Failure
    /// - 0x05: Acknowledge
    /// - 0x06: Slave Device Busy
    /// - 0x08: Memory Parity Error
    /// - 0x0A: Gateway Path Unavailable
    /// - 0x0B: Gateway Target Device Failed to Respond
    #[error("Modbus exception: function={function:02X}, code={code:02X} ({message})")]
    Exception {
        function: u8,
        code: u8,
        /// Uses `&'static str` since all exception messages are static strings
        message: &'static str,
    },

    /// Frame parsing errors
    ///
    /// Message frame format violations including invalid frame structure,
    /// incomplete frames, or framing protocol errors.
    ///
    /// # Examples
    /// - Incomplete TCP MBAP header
    /// - Invalid RTU frame structure
    /// - Message too short for claimed length
    #[error("Frame error: {message}")]
    Frame { message: String },

    /// Configuration errors
    ///
    /// Client or server configuration issues that prevent proper operation.
    ///
    /// # Examples
    /// - Invalid TCP port number
    /// - Malformed configuration file
    /// - Missing required configuration parameters
    #[error("Configuration error: {message}")]
    Configuration { message: String },

    /// Device not responding
    ///
    /// Specific error for devices that fail to respond to requests,
    /// distinct from timeout errors as it indicates a pattern of
    /// non-responsiveness.
    ///
    /// # Examples
    /// - Device powered off
    /// - Device address misconfigured
    /// - Communication medium failure
    #[error("Device {slave_id} not responding")]
    DeviceNotResponding { slave_id: u8 },

    /// Internal errors (should not occur in normal operation)
    ///
    /// Library internal errors that indicate bugs or unexpected
    /// conditions within the Modbus implementation itself.
    ///
    /// # Examples
    /// - Internal state corruption
    /// - Unexpected code path execution
    /// - Resource management failures
    #[error("Internal error: {message}")]
    Internal { message: String },

    // Legacy aliases for compatibility
    /// Legacy timeout error (use Timeout instead)
    #[error("Timeout")]
    #[deprecated(note = "Use Timeout with operation and timeout_ms fields")]
    TimeoutLegacy,

    /// Legacy invalid frame error (use Frame instead)
    #[error("Invalid frame")]
    #[deprecated(note = "Use Frame with message field")]
    InvalidFrame,

    /// Legacy invalid data value error (use InvalidData instead)
    #[error("Invalid data value")]
    #[deprecated(note = "Use InvalidData with message field")]
    InvalidDataValue,

    /// Legacy illegal function error (use InvalidFunction instead)
    #[error("Illegal function")]
    #[deprecated(note = "Use InvalidFunction with code field")]
    IllegalFunction,

    /// Legacy internal error (use Internal instead)
    #[error("Internal error")]
    #[deprecated(note = "Use Internal with message field")]
    InternalError,
}

impl ModbusError {
    /// Create a new I/O error
    ///
    /// # Arguments
    ///
    /// * `message` - Descriptive error message
    ///
    /// # Returns
    ///
    /// New `ModbusError::Io` variant
    pub fn io<S: Into<String>>(message: S) -> Self {
        Self::Io {
            message: message.into(),
        }
    }

    /// Create a new connection error
    ///
    /// # Arguments
    ///
    /// * `message` - Descriptive error message
    ///
    /// # Returns
    ///
    /// New `ModbusError::Connection` variant
    pub fn connection<S: Into<String>>(message: S) -> Self {
        Self::Connection {
            message: message.into(),
        }
    }

    /// Create a new timeout error
    ///
    /// # Arguments
    ///
    /// * `operation` - Description of the operation that timed out
    /// * `timeout_ms` - Timeout duration in milliseconds
    ///
    /// # Returns
    ///
    /// New `ModbusError::Timeout` variant
    pub fn timeout<S: Into<String>>(operation: S, timeout_ms: u64) -> Self {
        Self::Timeout {
            operation: operation.into(),
            timeout_ms,
        }
    }

    /// Create a new protocol error
    ///
    /// # Arguments
    ///
    /// * `message` - Descriptive error message
    ///
    /// # Returns
    ///
    /// New `ModbusError::Protocol` variant
    pub fn protocol<S: Into<String>>(message: S) -> Self {
        Self::Protocol {
            message: message.into(),
        }
    }

    /// Create an invalid function error
    ///
    /// # Arguments
    ///
    /// * `code` - The invalid function code
    ///
    /// # Returns
    ///
    /// New `ModbusError::InvalidFunction` variant
    pub fn invalid_function(code: u8) -> Self {
        Self::InvalidFunction { code }
    }

    /// Create an invalid address error
    ///
    /// # Arguments
    ///
    /// * `start` - Starting address
    /// * `count` - Number of registers/coils
    ///
    /// # Returns
    ///
    /// New `ModbusError::InvalidAddress` variant
    pub fn invalid_address(start: u16, count: u16) -> Self {
        Self::InvalidAddress { start, count }
    }

    /// Create an invalid data error
    ///
    /// # Arguments
    ///
    /// * `message` - Descriptive error message
    ///
    /// # Returns
    ///
    /// New `ModbusError::InvalidData` variant
    pub fn invalid_data<S: Into<String>>(message: S) -> Self {
        Self::InvalidData {
            message: message.into(),
        }
    }

    /// Create a CRC mismatch error
    ///
    /// # Arguments
    ///
    /// * `expected` - Expected CRC value
    /// * `actual` - Actual CRC value received
    ///
    /// # Returns
    ///
    /// New `ModbusError::CrcMismatch` variant
    pub fn crc_mismatch(expected: u16, actual: u16) -> Self {
        Self::CrcMismatch { expected, actual }
    }

    /// Create a Modbus exception error
    ///
    /// Automatically maps standard exception codes to human-readable messages.
    ///
    /// # Arguments
    ///
    /// * `function` - Original function code that caused the exception
    /// * `code` - Modbus exception code
    ///
    /// # Returns
    ///
    /// New `ModbusError::Exception` variant with appropriate message
    pub fn exception(function: u8, code: u8) -> Self {
        let message: &'static str = match code {
            0x01 => "Illegal Function",
            0x02 => "Illegal Data Address",
            0x03 => "Illegal Data Value",
            0x04 => "Slave Device Failure",
            0x05 => "Acknowledge",
            0x06 => "Slave Device Busy",
            0x08 => "Memory Parity Error",
            0x0A => "Gateway Path Unavailable",
            0x0B => "Gateway Target Device Failed to Respond",
            _ => "Unknown Exception",
        };

        Self::Exception {
            function,
            code,
            message,
        }
    }

    /// Create a frame error
    ///
    /// # Arguments
    ///
    /// * `message` - Descriptive error message
    ///
    /// # Returns
    ///
    /// New `ModbusError::Frame` variant
    pub fn frame<S: Into<String>>(message: S) -> Self {
        Self::Frame {
            message: message.into(),
        }
    }

    /// Create a configuration error
    ///
    /// # Arguments
    ///
    /// * `message` - Descriptive error message
    ///
    /// # Returns
    ///
    /// New `ModbusError::Configuration` variant
    pub fn configuration<S: Into<String>>(message: S) -> Self {
        Self::Configuration {
            message: message.into(),
        }
    }

    /// Create a device not responding error
    ///
    /// # Arguments
    ///
    /// * `slave_id` - ID of the non-responding device
    ///
    /// # Returns
    ///
    /// New `ModbusError::DeviceNotResponding` variant
    pub fn device_not_responding(slave_id: u8) -> Self {
        Self::DeviceNotResponding { slave_id }
    }

    /// Create an internal error
    ///
    /// # Arguments
    ///
    /// * `message` - Descriptive error message
    ///
    /// # Returns
    ///
    /// New `ModbusError::Internal` variant
    pub fn internal<S: Into<String>>(message: S) -> Self {
        Self::Internal {
            message: message.into(),
        }
    }

    /// Check if the error is recoverable (can retry)
    ///
    /// Determines whether an operation that failed with this error
    /// might succeed if retried, helping implement intelligent
    /// retry strategies.
    ///
    /// # Returns
    ///
    /// `true` if the error condition might be temporary and retrying
    /// the operation could succeed, `false` for permanent failures
    ///
    /// # Examples
    ///
    /// ```rust
    /// use voltage_modbus::ModbusError;
    ///
    /// let timeout_error = ModbusError::timeout("read operation", 5000);
    /// assert!(timeout_error.is_recoverable());
    ///
    /// let invalid_function = ModbusError::invalid_function(0x99);
    /// assert!(!invalid_function.is_recoverable());
    /// ```
    pub fn is_recoverable(&self) -> bool {
        match self {
            Self::Io { .. } => true,
            Self::Connection { .. } => true,
            Self::Timeout { .. } => true,
            Self::DeviceNotResponding { .. } => true,
            Self::Exception { code, .. } => {
                // Some exceptions are recoverable
                matches!(code, 0x05 | 0x06) // Acknowledge, Busy
            }
            _ => false,
        }
    }

    /// Check if the error is a network/transport issue
    ///
    /// Identifies errors that are related to the underlying transport
    /// mechanism (TCP, RTU) rather than Modbus protocol issues.
    ///
    /// # Returns
    ///
    /// `true` if the error is transport-related, `false` otherwise
    ///
    /// # Examples
    ///
    /// ```rust
    /// use voltage_modbus::ModbusError;
    ///
    /// let connection_error = ModbusError::connection("Connection refused");
    /// assert!(connection_error.is_transport_error());
    ///
    /// let exception_error = ModbusError::exception(0x03, 0x02);
    /// assert!(!exception_error.is_transport_error());
    /// ```
    pub fn is_transport_error(&self) -> bool {
        matches!(
            self,
            Self::Io { .. } | Self::Connection { .. } | Self::Timeout { .. }
        )
    }

    /// Check if the error is a protocol issue
    ///
    /// Identifies errors that are related to Modbus protocol violations
    /// or communication at the protocol level.
    ///
    /// # Returns
    ///
    /// `true` if the error is protocol-related, `false` otherwise
    ///
    /// # Examples
    ///
    /// ```rust
    /// use voltage_modbus::ModbusError;
    ///
    /// let exception_error = ModbusError::exception(0x03, 0x02);
    /// assert!(exception_error.is_protocol_error());
    ///
    /// let io_error = ModbusError::io("Network unreachable");
    /// assert!(!io_error.is_protocol_error());
    /// ```
    pub fn is_protocol_error(&self) -> bool {
        matches!(
            self,
            Self::Protocol { .. }
                | Self::InvalidFunction { .. }
                | Self::Exception { .. }
                | Self::Frame { .. }
                | Self::CrcMismatch { .. }
        )
    }
}

/// Convert from std::io::Error
///
/// Automatically converts standard I/O errors to `ModbusError::Io`,
/// preserving the original error message for debugging.
impl From<std::io::Error> for ModbusError {
    fn from(err: std::io::Error) -> Self {
        Self::io(err.to_string())
    }
}

/// Convert from tokio timeout errors
///
/// Converts Tokio's timeout errors to `ModbusError::Timeout` with
/// a generic timeout message (specific timeout duration should be
/// provided when creating timeout errors manually).
impl From<tokio::time::error::Elapsed> for ModbusError {
    fn from(_: tokio::time::error::Elapsed) -> Self {
        Self::timeout("Operation timeout", 0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_creation() {
        let err = ModbusError::timeout("read_registers", 5000);
        assert!(err.is_recoverable());
        assert!(err.is_transport_error());

        let err = ModbusError::exception(0x03, 0x02);
        assert!(!err.is_recoverable());
        assert!(err.is_protocol_error());
    }

    #[test]
    fn test_error_display() {
        let err = ModbusError::crc_mismatch(0x1234, 0x5678);
        let msg = format!("{}", err);
        assert!(msg.contains("CRC validation failed"));
        assert!(msg.contains("1234"));
        assert!(msg.contains("5678"));
    }
}
