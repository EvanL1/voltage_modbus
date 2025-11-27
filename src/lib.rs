//! # Voltage Modbus - High-Performance Industrial Modbus Library
//!
//! **Author:** Evan Liu <liuyifanz.1996@gmail.com>
//! **Version:** 0.4.0
//! **License:** MIT
//!
//! A comprehensive, high-performance Modbus TCP/RTU implementation in pure Rust
//! designed for industrial automation, IoT applications, and smart grid systems.
//!
//! ## Features
//!
//! - **High Performance**: Async/await support with Tokio, stack-allocated PDU
//! - **Complete Protocol Support**: Modbus TCP and RTU protocols
//! - **Memory Safe**: Pure Rust implementation with zero unsafe code
//! - **Zero-Copy Operations**: Optimized for minimal memory allocations
//! - **Industrial Features**: Command batching, read merging, device limits
//! - **Built-in Monitoring**: Comprehensive statistics and metrics
//!
//! ## Supported Function Codes
//!
//! | Code | Function | Client |
//! |------|----------|--------|
//! | 0x01 | Read Coils | ✅ |
//! | 0x02 | Read Discrete Inputs | ✅ |
//! | 0x03 | Read Holding Registers | ✅ |
//! | 0x04 | Read Input Registers | ✅ |
//! | 0x05 | Write Single Coil | ✅ |
//! | 0x06 | Write Single Register | ✅ |
//! | 0x0F | Write Multiple Coils | ✅ |
//! | 0x10 | Write Multiple Registers | ✅ |
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use voltage_modbus::{ModbusTcpClient, ModbusClient, ModbusResult};
//! use std::time::Duration;
//!
//! #[tokio::main]
//! async fn main() -> ModbusResult<()> {
//!     // Connect to Modbus TCP server
//!     let mut client = ModbusTcpClient::from_address("127.0.0.1:502", Duration::from_secs(5)).await?;
//!
//!     // Read holding registers
//!     let values = client.read_03(1, 0, 10).await?;
//!     println!("Read registers: {:?}", values);
//!
//!     // Write single register
//!     client.write_06(1, 100, 0x1234).await?;
//!
//!     client.close().await?;
//!     Ok(())
//! }
//! ```

// ============================================================================
// Core modules
// ============================================================================

/// Core error types and result handling
pub mod error;

/// Modbus protocol constants based on official specification
pub mod constants;

/// High-performance PDU with stack-allocated fixed array
pub mod pdu;

/// Modbus protocol definitions and message handling
pub mod protocol;

/// Network transport layer for TCP and RTU communication
pub mod transport;

/// Modbus client implementations
pub mod client;

/// Utility functions and performance monitoring
pub mod utils;

/// Logging system for the library
pub mod logging;

// ============================================================================
// Industrial enhancement modules (Phase 3)
// ============================================================================

/// Industrial data value types for Modbus
pub mod value;

/// Byte order handling for multi-register data types
pub mod bytes;

/// Encoding and decoding of Modbus data with byte order support
pub mod codec;

/// Command batching for optimized write operations
pub mod batcher;

/// Device-specific protocol limits configuration
pub mod device_limits;

// ============================================================================
// Re-exports for convenience
// ============================================================================

pub use client::{GenericModbusClient, ModbusClient, ModbusTcpClient};
pub use constants::*;
pub use error::{ModbusError, ModbusResult};
pub use logging::{CallbackLogger, LogCallback, LogLevel, LoggingMode};
pub use pdu::{ModbusPdu, PduBuilder};
pub use protocol::{ModbusFunction, ModbusRequest, ModbusResponse};
pub use transport::{ModbusTransport, TcpTransport, TransportStats};
pub use utils::{OperationTimer, PerformanceMetrics};

// Industrial enhancement re-exports
pub use batcher::{BatchCommand, CommandBatcher, DEFAULT_BATCH_WINDOW_MS, DEFAULT_MAX_BATCH_SIZE};
pub use bytes::{
    regs_to_bytes_4, regs_to_bytes_8, regs_to_f32, regs_to_f64, regs_to_i32, regs_to_u32, ByteOrder,
};
pub use codec::{
    clamp_to_data_type, decode_register_value, encode_f64_as_type, encode_value,
    parse_read_response, registers_for_type, ModbusCodec,
};
pub use device_limits::{
    DeviceLimits, DEFAULT_INTER_REQUEST_DELAY_MS, DEFAULT_MAX_READ_COILS,
    DEFAULT_MAX_READ_REGISTERS, DEFAULT_MAX_WRITE_COILS, DEFAULT_MAX_WRITE_REGISTERS,
};
pub use value::ModbusValue;

#[cfg(feature = "rtu")]
pub use client::ModbusRtuClient;

#[cfg(feature = "rtu")]
pub use transport::RtuTransport;

/// Default timeout for operations (5 seconds)
pub const DEFAULT_TIMEOUT_MS: u64 = 5000;

/// Modbus TCP default port
pub const DEFAULT_TCP_PORT: u16 = 502;

/// Library version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Get library information
pub fn info() -> String {
    format!(
        "Voltage Modbus v{} - High-performance industrial Modbus library by Evan Liu",
        VERSION
    )
}
