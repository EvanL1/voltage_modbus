//! # Voltage Modbus - High-Performance Industrial Modbus Library
//!
//! **Author:** Evan Liu <liuyifanz.1996@gmail.com>
//! **Version:** 0.6.1
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
//! - **no_std Core**: PDU/protocol layer usable on embedded MCUs (disable default features)
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
//! ## Quick Start (std)
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
//!
//! ## no_std Usage (embedded)
//!
//! Add to `Cargo.toml`:
//! ```toml
//! voltage_modbus = { version = "...", default-features = false }
//! ```
//!
//! Then use the core PDU/protocol modules without any std dependency:
//!
//! ```rust,no_run
//! use voltage_modbus::pdu::PduBuilder;
//! use voltage_modbus::constants::MAX_PDU_SIZE;
//!
//! // Build a read-holding-registers request PDU
//! let pdu = PduBuilder::build_read_request(0x03, 100, 10).unwrap();
//! let raw_bytes = pdu.as_slice(); // &[u8] — send over your transport
//! ```

// ============================================================================
// no_std support
// ============================================================================
#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(docsrs, feature(doc_auto_cfg))]

#[cfg(not(feature = "std"))]
extern crate alloc;

// ============================================================================
// Core modules — always available (no_std compatible)
// ============================================================================

/// Modbus protocol constants based on official specification
pub mod constants;

/// Core error types and result handling
pub mod error;

/// High-performance PDU with stack-allocated fixed array
pub mod pdu;

/// Modbus protocol definitions and message handling
pub mod protocol;

// ============================================================================
// std-only modules — require async runtime, heap collections, or OS APIs
// ============================================================================

/// Network transport layer for TCP and RTU communication
#[cfg(feature = "std")]
pub mod transport;

/// Modbus client implementations
#[cfg(feature = "std")]
pub mod client;

/// Utility functions and performance monitoring
#[cfg(feature = "std")]
pub mod utils;

/// Logging system for the library
#[cfg(feature = "std")]
pub mod logging;

// ============================================================================
// Industrial enhancement modules (std-only)
// ============================================================================

/// Industrial data value types for Modbus
#[cfg(feature = "std")]
pub mod value;

/// Byte order handling for multi-register data types
#[cfg(feature = "std")]
pub mod bytes;

/// Encoding and decoding of Modbus data with byte order support
#[cfg(feature = "std")]
pub mod codec;

/// Command batching for optimized write operations
#[cfg(feature = "std")]
pub mod batcher;

/// Read coalescing for merging adjacent/overlapping register read requests
#[cfg(feature = "std")]
pub mod coalescer;

/// Shared scheduler trait over batcher + coalescer request types
#[cfg(feature = "std")]
pub mod scheduler;

/// Device-specific protocol limits configuration
#[cfg(feature = "std")]
pub mod device_limits;

/// Modbus server implementation (TCP slave mode)
#[cfg(feature = "std")]
pub mod server;

/// Embedded async RTU transport via `embedded-io-async` (no_std + alloc)
#[cfg(feature = "embedded")]
pub mod embedded;

/// In-memory register bank for server implementations
#[cfg(feature = "std")]
pub mod register_bank;

// ============================================================================
// Re-exports for convenience
// ============================================================================

// === Core protocol — always available (no_std compatible) ===
pub use constants::{
    MAX_PDU_SIZE, MAX_READ_COILS, MAX_READ_REGISTERS, MAX_WRITE_COILS, MAX_WRITE_REGISTERS,
};
pub use error::{ModbusError, ModbusResult};
pub use pdu::{ModbusPdu, PduBuilder};
pub use protocol::{ModbusFunction, ModbusRequest, ModbusResponse, SlaveId};

// === std-only re-exports ===

#[cfg(feature = "std")]
pub use tokio;

#[cfg(feature = "std")]
pub use client::{GenericModbusClient, ModbusClient, ModbusTcpClient};

#[cfg(feature = "std")]
pub use bytes::ByteOrder;

#[cfg(feature = "std")]
pub use value::ModbusValue;

#[cfg(feature = "std")]
pub use batcher::{BatchCommand, CommandBatcher};

#[cfg(feature = "std")]
pub use coalescer::{CoalescedRead, ReadCoalescer, ReadRequest};

#[cfg(feature = "std")]
pub use scheduler::ScheduledRequest;

#[cfg(feature = "std")]
pub use codec::ModbusCodec;

#[cfg(feature = "std")]
pub use device_limits::DeviceLimits;

#[cfg(feature = "std")]
pub use client::ModbusRtuOverTcpClient;

#[cfg(feature = "std")]
pub use transport::{ModbusTransport, RtuOverTcpTransport, TcpTransport, TransportStats};

#[cfg(feature = "std")]
pub use transport::{PacketCallback, PacketDirection};

#[cfg(feature = "std")]
pub use utils::PerformanceMetrics;

#[cfg(feature = "std")]
pub use logging::{CallbackLogger, LogCallback, LogLevel, LoggingMode};

#[cfg(feature = "std")]
pub use register_bank::{ModbusRegisterBank, RegisterBankStats};

#[cfg(feature = "std")]
pub use server::{ModbusServer, ModbusTcpServer, ModbusTcpServerConfig, ServerStats};

// === Hidden but preserved (backward compatibility, std-only) ===
#[cfg(feature = "std")]
#[doc(hidden)]
pub use batcher::{DEFAULT_BATCH_WINDOW_MS, DEFAULT_MAX_BATCH_SIZE};

#[cfg(feature = "std")]
#[doc(hidden)]
pub use bytes::{
    regs_to_bytes_4, regs_to_bytes_8, regs_to_f32, regs_to_f64, regs_to_i32, regs_to_u32,
};

#[cfg(feature = "std")]
#[doc(hidden)]
pub use codec::{
    clamp_to_data_type, decode_register_value, encode_f64_as_type, encode_value,
    parse_read_response, registers_for_type,
};

#[cfg(feature = "std")]
#[doc(hidden)]
pub use device_limits::{
    DEFAULT_INTER_REQUEST_DELAY_MS, DEFAULT_MAX_READ_COILS, DEFAULT_MAX_READ_REGISTERS,
    DEFAULT_MAX_WRITE_COILS, DEFAULT_MAX_WRITE_REGISTERS,
};

#[cfg(feature = "std")]
#[doc(hidden)]
pub use utils::OperationTimer;

#[cfg(feature = "rtu")]
pub use client::{ModbusAsciiClient, ModbusRtuClient};

#[cfg(feature = "rtu")]
pub use server::{ModbusRtuServer, ModbusRtuServerConfig};

#[cfg(feature = "rtu")]
pub use transport::{AsciiTransport, RtuTransport};

#[cfg(feature = "embedded")]
pub use embedded::EmbeddedRtuTransport;

/// Default timeout for operations (5 seconds)
pub const DEFAULT_TIMEOUT_MS: u64 = 5000;

/// Modbus TCP default port
pub const DEFAULT_TCP_PORT: u16 = 502;

/// Library version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Get library information (std only — requires String allocation)
#[cfg(feature = "std")]
pub fn info() -> String {
    format!(
        "Voltage Modbus v{} - High-performance industrial Modbus library by Evan Liu",
        VERSION
    )
}
