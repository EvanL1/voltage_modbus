// Deprecated variants below are exposed for backward compatibility; derive
// macros (thiserror, defmt::Format) reference them and would otherwise trigger
// lints. Silencing at module scope keeps call-site warnings for users intact.
#![allow(deprecated)]

//! # Voltage Modbus Error Handling
//!
//! This module provides comprehensive error handling for the Voltage Modbus library,
//! covering all aspects of Modbus communication including network transport, protocol
//! parsing, data validation, and device interaction errors.
//!
//! ## no_std compatibility
//!
//! This module is no_std compatible when the `std` feature is disabled.
//! - With `std`: uses `thiserror` for derive macros, includes `From<std::io::Error>`.
//! - Without `std`: manually implements `Display` and `core::error::Error`; requires
//!   `alloc` for `String`-bearing variants.
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

#[cfg(not(feature = "std"))]
use alloc::string::String;

#[cfg(feature = "std")]
use thiserror::Error;

// core::fmt is used by the manual Display impl in no_std mode.
// In std mode thiserror generates all fmt code, so suppress the unused-import warning.
#[cfg(not(feature = "std"))]
use core::fmt;

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
///
/// In no_std builds all `String`-bearing variants still work — they use
/// `alloc::string::String` from the implicit `alloc` crate.
#[cfg_attr(feature = "std", derive(Error))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, PartialEq)]
pub enum ModbusError {
    /// I/O related errors (network, serial)
    #[cfg_attr(feature = "std", error("I/O error: {message}"))]
    Io { message: String },

    /// Connection errors
    #[cfg_attr(feature = "std", error("Connection error: {message}"))]
    Connection { message: String },

    /// Timeout errors
    #[cfg_attr(feature = "std", error("Timeout after {timeout_ms}ms: {operation}"))]
    Timeout { operation: String, timeout_ms: u64 },

    /// Protocol-level errors
    #[cfg_attr(feature = "std", error("Protocol error: {message}"))]
    Protocol { message: String },

    /// Invalid function code
    #[cfg_attr(feature = "std", error("Invalid function code: {code}"))]
    InvalidFunction { code: u8 },

    /// Invalid address range
    #[cfg_attr(
        feature = "std",
        error("Invalid address: start={start}, count={count}")
    )]
    InvalidAddress { start: u16, count: u16 },

    /// Invalid data value
    #[cfg_attr(feature = "std", error("Invalid data: {message}"))]
    InvalidData { message: String },

    /// CRC validation failure
    #[cfg_attr(
        feature = "std",
        error("CRC validation failed: expected={expected:04X}, actual={actual:04X}")
    )]
    CrcMismatch { expected: u16, actual: u16 },

    /// Modbus exception response
    #[cfg_attr(
        feature = "std",
        error("Modbus exception: function={function:02X}, code={code:02X} ({message})")
    )]
    Exception {
        function: u8,
        code: u8,
        /// Uses `&'static str` since all exception messages are static strings
        message: &'static str,
    },

    /// Frame parsing errors
    #[cfg_attr(feature = "std", error("Frame error: {message}"))]
    Frame { message: String },

    /// Configuration errors
    #[cfg_attr(feature = "std", error("Configuration error: {message}"))]
    Configuration { message: String },

    /// Device not responding
    #[cfg_attr(feature = "std", error("Device {slave_id} not responding"))]
    DeviceNotResponding { slave_id: u8 },

    /// Transaction ID mismatch in TCP response
    #[cfg_attr(
        feature = "std",
        error("Transaction ID mismatch: expected={expected:04X}, actual={actual:04X}")
    )]
    TransactionIdMismatch { expected: u16, actual: u16 },

    /// Internal errors (should not occur in normal operation)
    #[cfg_attr(feature = "std", error("Internal error: {message}"))]
    Internal { message: String },

    // Legacy aliases for compatibility
    /// Legacy timeout error (use Timeout instead)
    #[cfg_attr(feature = "std", error("Timeout"))]
    #[deprecated(note = "Use Timeout with operation and timeout_ms fields")]
    TimeoutLegacy,

    /// Legacy invalid frame error (use Frame instead)
    #[cfg_attr(feature = "std", error("Invalid frame"))]
    #[deprecated(note = "Use Frame with message field")]
    InvalidFrame,

    /// Legacy invalid data value error (use InvalidData instead)
    #[cfg_attr(feature = "std", error("Invalid data value"))]
    #[deprecated(note = "Use InvalidData with message field")]
    InvalidDataValue,

    /// Legacy illegal function error (use InvalidFunction instead)
    #[cfg_attr(feature = "std", error("Illegal function"))]
    #[deprecated(note = "Use InvalidFunction with code field")]
    IllegalFunction,

    /// Legacy internal error (use Internal instead)
    #[cfg_attr(feature = "std", error("Internal error"))]
    #[deprecated(note = "Use Internal with message field")]
    InternalError,
}

// In no_std mode we manually implement Display and core::error::Error,
// since thiserror is not available.
#[cfg(not(feature = "std"))]
impl fmt::Display for ModbusError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io { message } => write!(f, "I/O error: {}", message),
            Self::Connection { message } => write!(f, "Connection error: {}", message),
            Self::Timeout {
                operation,
                timeout_ms,
            } => write!(f, "Timeout after {}ms: {}", timeout_ms, operation),
            Self::Protocol { message } => write!(f, "Protocol error: {}", message),
            Self::InvalidFunction { code } => write!(f, "Invalid function code: {}", code),
            Self::InvalidAddress { start, count } => {
                write!(f, "Invalid address: start={}, count={}", start, count)
            }
            Self::InvalidData { message } => write!(f, "Invalid data: {}", message),
            Self::CrcMismatch { expected, actual } => write!(
                f,
                "CRC validation failed: expected={:04X}, actual={:04X}",
                expected, actual
            ),
            Self::Exception {
                function,
                code,
                message,
            } => write!(
                f,
                "Modbus exception: function={:02X}, code={:02X} ({})",
                function, code, message
            ),
            Self::Frame { message } => write!(f, "Frame error: {}", message),
            Self::Configuration { message } => write!(f, "Configuration error: {}", message),
            Self::DeviceNotResponding { slave_id } => {
                write!(f, "Device {} not responding", slave_id)
            }
            Self::TransactionIdMismatch { expected, actual } => write!(
                f,
                "Transaction ID mismatch: expected={:04X}, actual={:04X}",
                expected, actual
            ),
            Self::Internal { message } => write!(f, "Internal error: {}", message),
            #[allow(deprecated)]
            Self::TimeoutLegacy => write!(f, "Timeout"),
            #[allow(deprecated)]
            Self::InvalidFrame => write!(f, "Invalid frame"),
            #[allow(deprecated)]
            Self::InvalidDataValue => write!(f, "Invalid data value"),
            #[allow(deprecated)]
            Self::IllegalFunction => write!(f, "Illegal function"),
            #[allow(deprecated)]
            Self::InternalError => write!(f, "Internal error"),
        }
    }
}

// In std mode, thiserror already derives Display; we still need the core::error::Error
// impl for no_std builds (core::error::Error was stabilised in Rust 1.81).
#[cfg(not(feature = "std"))]
impl core::error::Error for ModbusError {}

impl ModbusError {
    /// Create a new I/O error
    pub fn io<S: Into<String>>(message: S) -> Self {
        Self::Io {
            message: message.into(),
        }
    }

    /// Create a new connection error
    pub fn connection<S: Into<String>>(message: S) -> Self {
        Self::Connection {
            message: message.into(),
        }
    }

    /// Create a new timeout error
    pub fn timeout<S: Into<String>>(operation: S, timeout_ms: u64) -> Self {
        Self::Timeout {
            operation: operation.into(),
            timeout_ms,
        }
    }

    /// Create a new protocol error
    pub fn protocol<S: Into<String>>(message: S) -> Self {
        Self::Protocol {
            message: message.into(),
        }
    }

    /// Create an invalid function error
    pub fn invalid_function(code: u8) -> Self {
        Self::InvalidFunction { code }
    }

    /// Create an invalid address error
    pub fn invalid_address(start: u16, count: u16) -> Self {
        Self::InvalidAddress { start, count }
    }

    /// Create an invalid data error
    pub fn invalid_data<S: Into<String>>(message: S) -> Self {
        Self::InvalidData {
            message: message.into(),
        }
    }

    /// Create a CRC mismatch error
    pub fn crc_mismatch(expected: u16, actual: u16) -> Self {
        Self::CrcMismatch { expected, actual }
    }

    /// Create a Modbus exception error
    ///
    /// Automatically maps standard exception codes to human-readable messages.
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
    pub fn frame<S: Into<String>>(message: S) -> Self {
        Self::Frame {
            message: message.into(),
        }
    }

    /// Create a configuration error
    pub fn configuration<S: Into<String>>(message: S) -> Self {
        Self::Configuration {
            message: message.into(),
        }
    }

    /// Create a device not responding error
    pub fn device_not_responding(slave_id: u8) -> Self {
        Self::DeviceNotResponding { slave_id }
    }

    /// Create a transaction ID mismatch error
    pub fn transaction_id_mismatch(expected: u16, actual: u16) -> Self {
        Self::TransactionIdMismatch { expected, actual }
    }

    /// Create an internal error
    pub fn internal<S: Into<String>>(message: S) -> Self {
        Self::Internal {
            message: message.into(),
        }
    }

    /// Check if the error is recoverable (can retry)
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
            Self::TransactionIdMismatch { .. } => true,
            Self::Exception { code, .. } => {
                matches!(code, 0x05 | 0x06) // Acknowledge, Busy
            }
            _ => false,
        }
    }

    /// Check if the error is a network/transport issue
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
                | Self::TransactionIdMismatch { .. }
        )
    }
}

/// Convert from std::io::Error — only available with the `std` feature
#[cfg(feature = "std")]
impl From<std::io::Error> for ModbusError {
    fn from(err: std::io::Error) -> Self {
        Self::io(err.to_string())
    }
}

/// Convert from tokio timeout errors — only available with the `std` feature
#[cfg(feature = "std")]
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
