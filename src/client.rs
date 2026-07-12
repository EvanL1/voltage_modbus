//! High-level Modbus client implementations
//!
//! This module provides user-friendly client interfaces for Modbus communication,
//! abstracting away the low-level protocol details.
//!
//! # Architecture
//!
//! The key insight is that Modbus TCP and RTU share the same application layer (PDU),
//! differing only in transport layer encapsulation:
//! - **TCP**: MBAP Header + PDU
//! - **RTU**: Slave ID + PDU + CRC
//!
//! This allows us to implement the application logic once and reuse it for both transports
//! through the [`GenericModbusClient`] abstraction.
//!
//! # API Naming Convention
//!
//! This library provides a **dual-track API**:
//!
//! | Function Code | Primary Name | Semantic Alias |
//! |---------------|--------------|----------------|
//! | 0x01 | `read_01()` | `read_coils()` |
//! | 0x02 | `read_02()` | `read_discrete_inputs()` |
//! | 0x03 | `read_03()` | `read_holding_registers()` |
//! | 0x04 | `read_04()` | `read_input_registers()` |
//! | 0x05 | `write_05()` | `write_single_coil()` |
//! | 0x06 | `write_06()` | `write_single_register()` |
//! | 0x0F | `write_0f()` | `write_multiple_coils()` |
//! | 0x10 | `write_10()` | `write_multiple_registers()` |
//!
//! # Quick Start
//!
//! ```rust,no_run
//! use voltage_modbus::{ModbusTcpClient, ModbusClient, ModbusResult};
//! use std::time::Duration;
//!
//! #[tokio::main]
//! async fn main() -> ModbusResult<()> {
//!     // Create TCP client
//!     let mut client = ModbusTcpClient::from_address(
//!         "127.0.0.1:502",
//!         Duration::from_secs(5)
//!     ).await?;
//!
//!     // Read 10 holding registers from slave 1, starting at address 0
//!     let registers = client.read_03(1, 0, 10).await?;
//!     println!("Registers: {:?}", registers);
//!
//!     // Write a value to register 100
//!     client.write_06(1, 100, 0x1234).await?;
//!
//!     client.close().await?;
//!     Ok(())
//! }
//! ```
use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;

use crate::coalescer::ReadCoalescer;
use crate::device_limits::DeviceLimits;
use crate::error::{ModbusError, ModbusResult};
use crate::logging::CallbackLogger;
use crate::protocol::{ModbusFunction, ModbusRequest, ModbusResponse, SlaveId};
use crate::transport::{ModbusTransport, TcpTransport, TransportStats};

#[cfg(feature = "rtu")]
use crate::transport::RtuTransport;

/// Retry policy for recoverable request failures.
///
/// Applied by [`GenericModbusClient`] around every request: transient errors
/// (timeouts, connection loss, device-busy exceptions — see
/// [`ModbusError::is_recoverable`]) are retried with exponential backoff,
/// while permanent errors (illegal address, CRC mismatch, response validation
/// failures) fail immediately. The default policy performs no retries, so
/// existing behavior is unchanged unless a policy is installed.
///
/// All standard write functions are idempotent (they set absolute values),
/// so retrying a write after a timeout is safe.
///
/// # Example
///
/// ```rust,no_run
/// use voltage_modbus::{ModbusTcpClient, RetryPolicy};
/// use std::time::Duration;
///
/// # async fn example() -> voltage_modbus::ModbusResult<()> {
/// let mut client = ModbusTcpClient::from_address("127.0.0.1:502", Duration::from_secs(5)).await?;
/// client.set_retry_policy(RetryPolicy::new(3)); // up to 3 retries, 100ms..2s backoff
/// # Ok(())
/// # }
/// ```
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RetryPolicy {
    /// Extra attempts after the first failure (0 = no retries)
    pub max_retries: u32,
    /// Backoff before the first retry; doubles on each subsequent retry
    pub initial_backoff: Duration,
    /// Upper bound on the backoff between attempts
    pub max_backoff: Duration,
}

impl RetryPolicy {
    /// No retries — every error is returned immediately (the default)
    pub const fn none() -> Self {
        Self {
            max_retries: 0,
            initial_backoff: Duration::ZERO,
            max_backoff: Duration::ZERO,
        }
    }

    /// Retry up to `max_retries` times, starting at 100 ms backoff and
    /// doubling up to 2 s
    pub const fn new(max_retries: u32) -> Self {
        Self {
            max_retries,
            initial_backoff: Duration::from_millis(100),
            max_backoff: Duration::from_secs(2),
        }
    }

    /// Fully custom policy
    pub const fn with_backoff(
        max_retries: u32,
        initial_backoff: Duration,
        max_backoff: Duration,
    ) -> Self {
        Self {
            max_retries,
            initial_backoff,
            max_backoff,
        }
    }
}

impl Default for RetryPolicy {
    fn default() -> Self {
        Self::none()
    }
}

/// Trait defining the interface for Modbus client operations.
///
/// This trait provides async methods for all standard Modbus functions,
/// with clear function code references for better understanding.
///
/// # Implemented By
///
/// - [`ModbusTcpClient`] - Modbus TCP client
/// - [`ModbusRtuClient`] - Modbus RTU client (requires `rtu` feature)
/// - [`GenericModbusClient`] - Generic client for custom transports
///
/// # Protocol Limits
///
/// The Modbus specification defines these limits:
///
/// | Operation | Limit |
/// |-----------|-------|
/// | Read Coils (0x01) | 2000 coils |
/// | Read Discrete Inputs (0x02) | 2000 bits |
/// | Read Holding Registers (0x03) | 125 registers |
/// | Read Input Registers (0x04) | 125 registers |
/// | Write Multiple Coils (0x0F) | 1968 coils |
/// | Write Multiple Registers (0x10) | 123 registers |
pub trait ModbusClient: Send + Sync {
    /// Read coils (function code 0x01).
    ///
    /// Reads the ON/OFF status of discrete coils in a remote device.
    ///
    /// # Arguments
    ///
    /// * `slave_id` - The Modbus slave/unit ID (1-247)
    /// * `address` - Starting coil address (0-65535)
    /// * `quantity` - Number of coils to read (1-2000)
    ///
    /// # Returns
    ///
    /// A vector of boolean values representing coil states.
    fn read_01(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> impl std::future::Future<Output = ModbusResult<Vec<bool>>> + Send;

    /// Read discrete inputs (function code 0x02).
    ///
    /// Reads the ON/OFF status of discrete inputs in a remote device.
    ///
    /// # Arguments
    ///
    /// * `slave_id` - The Modbus slave/unit ID (1-247)
    /// * `address` - Starting input address (0-65535)
    /// * `quantity` - Number of inputs to read (1-2000)
    fn read_02(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> impl std::future::Future<Output = ModbusResult<Vec<bool>>> + Send;

    /// Read holding registers (function code 0x03).
    ///
    /// Reads the contents of a contiguous block of holding registers.
    /// This is the most commonly used function for reading process data.
    ///
    /// # Arguments
    ///
    /// * `slave_id` - The Modbus slave/unit ID (1-247)
    /// * `address` - Starting register address (0-65535)
    /// * `quantity` - Number of registers to read (1-125)
    ///
    /// # Returns
    ///
    /// A vector of 16-bit register values.
    fn read_03(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> impl std::future::Future<Output = ModbusResult<Vec<u16>>> + Send;

    /// Read input registers (function code 0x04).
    ///
    /// Reads the contents of a contiguous block of input registers.
    /// Input registers are typically read-only analog inputs.
    ///
    /// # Arguments
    ///
    /// * `slave_id` - The Modbus slave/unit ID (1-247)
    /// * `address` - Starting register address (0-65535)
    /// * `quantity` - Number of registers to read (1-125)
    fn read_04(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> impl std::future::Future<Output = ModbusResult<Vec<u16>>> + Send;

    /// Write single coil (function code 0x05).
    ///
    /// Writes a single coil to either ON or OFF in a remote device.
    ///
    /// # Arguments
    ///
    /// * `slave_id` - The Modbus slave/unit ID (1-247)
    /// * `address` - Coil address (0-65535)
    /// * `value` - `true` for ON (0xFF00), `false` for OFF (0x0000)
    fn write_05(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        value: bool,
    ) -> impl std::future::Future<Output = ModbusResult<()>> + Send;

    /// Write single register (function code 0x06).
    ///
    /// Writes a single holding register in a remote device.
    ///
    /// # Arguments
    ///
    /// * `slave_id` - The Modbus slave/unit ID (1-247)
    /// * `address` - Register address (0-65535)
    /// * `value` - 16-bit value to write
    fn write_06(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        value: u16,
    ) -> impl std::future::Future<Output = ModbusResult<()>> + Send;

    /// Write multiple coils (function code 0x0F).
    ///
    /// Writes a sequence of coils to either ON or OFF in a remote device.
    ///
    /// # Arguments
    ///
    /// * `slave_id` - The Modbus slave/unit ID (1-247)
    /// * `address` - Starting coil address (0-65535)
    /// * `values` - Slice of boolean values (1-1968 coils)
    fn write_0f(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        values: &[bool],
    ) -> impl std::future::Future<Output = ModbusResult<()>> + Send;

    /// Write multiple registers (function code 0x10).
    ///
    /// Writes a block of contiguous registers in a remote device.
    ///
    /// # Arguments
    ///
    /// * `slave_id` - The Modbus slave/unit ID (1-247)
    /// * `address` - Starting register address (0-65535)
    /// * `values` - Slice of 16-bit values to write (1-123 registers)
    fn write_10(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        values: &[u16],
    ) -> impl std::future::Future<Output = ModbusResult<()>> + Send;

    // ===== Batch read operations =====

    /// Batch read coils (function code 0x01) with automatic chunking.
    ///
    /// Reads a large range of coils by automatically splitting the request
    /// into smaller chunks according to device limits.
    ///
    /// # Arguments
    ///
    /// * `slave_id` - The Modbus slave/unit ID (1-247)
    /// * `address` - Starting coil address (0-65535)
    /// * `quantity` - Total number of coils to read (can exceed 2000)
    /// * `limits` - Device-specific limits configuration
    ///
    /// # Returns
    ///
    /// A vector of boolean values representing all coil states.
    ///
    /// # Example
    ///
    /// ```rust,no_run
    /// use voltage_modbus::{ModbusTcpClient, ModbusClient, DeviceLimits};
    /// use std::time::Duration;
    ///
    /// # async fn example() -> voltage_modbus::ModbusResult<()> {
    /// let mut client = ModbusTcpClient::from_address("127.0.0.1:502", Duration::from_secs(5)).await?;
    /// let limits = DeviceLimits::new();
    ///
    /// // Read 5000 coils (automatically split into 3 requests)
    /// let coils = client.read_01_batch(1, 0, 5000, &limits).await?;
    /// # Ok(())
    /// # }
    /// ```
    fn read_01_batch(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
        limits: &DeviceLimits,
    ) -> impl std::future::Future<Output = ModbusResult<Vec<bool>>> + Send
    where
        Self: Sized,
    {
        let max_read_coils = limits.max_read_coils;
        let inter_request_delay_ms = limits.inter_request_delay_ms;
        async move {
            if quantity == 0 {
                return Ok(Vec::new());
            }

            let mut result = Vec::with_capacity(quantity as usize);
            let mut current_address = address;
            let mut remaining = quantity;

            while remaining > 0 {
                let count = remaining.min(max_read_coils);
                let chunk = self.read_01(slave_id, current_address, count).await?;
                result.extend_from_slice(&chunk);

                current_address = current_address.saturating_add(count);
                remaining -= count;

                if inter_request_delay_ms > 0 && remaining > 0 {
                    tokio::time::sleep(Duration::from_millis(inter_request_delay_ms)).await;
                }
            }

            Ok(result)
        }
    }

    /// Batch read discrete inputs (function code 0x02) with automatic chunking.
    ///
    /// Reads a large range of discrete inputs by automatically splitting the request
    /// into smaller chunks according to device limits.
    ///
    /// # Arguments
    ///
    /// * `slave_id` - The Modbus slave/unit ID (1-247)
    /// * `address` - Starting input address (0-65535)
    /// * `quantity` - Total number of inputs to read (can exceed 2000)
    /// * `limits` - Device-specific limits configuration
    fn read_02_batch(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
        limits: &DeviceLimits,
    ) -> impl std::future::Future<Output = ModbusResult<Vec<bool>>> + Send
    where
        Self: Sized,
    {
        let max_read_coils = limits.max_read_coils;
        let inter_request_delay_ms = limits.inter_request_delay_ms;
        async move {
            if quantity == 0 {
                return Ok(Vec::new());
            }

            let mut result = Vec::with_capacity(quantity as usize);
            let mut current_address = address;
            let mut remaining = quantity;

            while remaining > 0 {
                let count = remaining.min(max_read_coils);
                let chunk = self.read_02(slave_id, current_address, count).await?;
                result.extend_from_slice(&chunk);

                current_address = current_address.saturating_add(count);
                remaining -= count;

                if inter_request_delay_ms > 0 && remaining > 0 {
                    tokio::time::sleep(Duration::from_millis(inter_request_delay_ms)).await;
                }
            }

            Ok(result)
        }
    }

    /// Batch read holding registers (function code 0x03) with automatic chunking.
    ///
    /// Reads a large range of holding registers by automatically splitting the request
    /// into smaller chunks according to device limits.
    ///
    /// # Arguments
    ///
    /// * `slave_id` - The Modbus slave/unit ID (1-247)
    /// * `address` - Starting register address (0-65535)
    /// * `quantity` - Total number of registers to read (can exceed 125)
    /// * `limits` - Device-specific limits configuration
    ///
    /// # Returns
    ///
    /// A vector of 16-bit register values.
    ///
    /// # Example
    ///
    /// ```rust,no_run
    /// use voltage_modbus::{ModbusTcpClient, ModbusClient, DeviceLimits};
    /// use std::time::Duration;
    ///
    /// # async fn example() -> voltage_modbus::ModbusResult<()> {
    /// let mut client = ModbusTcpClient::from_address("127.0.0.1:502", Duration::from_secs(5)).await?;
    /// let limits = DeviceLimits::new();
    ///
    /// // Read 500 registers (automatically split into 4 requests of 125 each)
    /// let registers = client.read_03_batch(1, 0, 500, &limits).await?;
    /// # Ok(())
    /// # }
    /// ```
    fn read_03_batch(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
        limits: &DeviceLimits,
    ) -> impl std::future::Future<Output = ModbusResult<Vec<u16>>> + Send
    where
        Self: Sized,
    {
        let max_read_registers = limits.max_read_registers;
        let inter_request_delay_ms = limits.inter_request_delay_ms;
        async move {
            if quantity == 0 {
                return Ok(Vec::new());
            }

            let mut result = Vec::with_capacity(quantity as usize);
            let mut current_address = address;
            let mut remaining = quantity;

            while remaining > 0 {
                let count = remaining.min(max_read_registers);
                let chunk = self.read_03(slave_id, current_address, count).await?;
                result.extend_from_slice(&chunk);

                current_address = current_address.saturating_add(count);
                remaining -= count;

                if inter_request_delay_ms > 0 && remaining > 0 {
                    tokio::time::sleep(Duration::from_millis(inter_request_delay_ms)).await;
                }
            }

            Ok(result)
        }
    }

    /// Batch read input registers (function code 0x04) with automatic chunking.
    ///
    /// Reads a large range of input registers by automatically splitting the request
    /// into smaller chunks according to device limits.
    ///
    /// # Arguments
    ///
    /// * `slave_id` - The Modbus slave/unit ID (1-247)
    /// * `address` - Starting register address (0-65535)
    /// * `quantity` - Total number of registers to read (can exceed 125)
    /// * `limits` - Device-specific limits configuration
    fn read_04_batch(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
        limits: &DeviceLimits,
    ) -> impl std::future::Future<Output = ModbusResult<Vec<u16>>> + Send
    where
        Self: Sized,
    {
        let max_read_registers = limits.max_read_registers;
        let inter_request_delay_ms = limits.inter_request_delay_ms;
        async move {
            if quantity == 0 {
                return Ok(Vec::new());
            }

            let mut result = Vec::with_capacity(quantity as usize);
            let mut current_address = address;
            let mut remaining = quantity;

            while remaining > 0 {
                let count = remaining.min(max_read_registers);
                let chunk = self.read_04(slave_id, current_address, count).await?;
                result.extend_from_slice(&chunk);

                current_address = current_address.saturating_add(count);
                remaining -= count;

                if inter_request_delay_ms > 0 && remaining > 0 {
                    tokio::time::sleep(Duration::from_millis(inter_request_delay_ms)).await;
                }
            }

            Ok(result)
        }
    }

    /// Check if the client is connected.
    ///
    /// Returns `true` if the underlying transport is connected and ready.
    fn is_connected(&self) -> bool;

    /// Close the client connection.
    ///
    /// Gracefully closes the underlying transport connection.
    fn close(&mut self) -> impl std::future::Future<Output = ModbusResult<()>> + Send;

    /// Get transport statistics.
    ///
    /// Returns statistics about requests sent and responses received.
    fn get_stats(&self) -> TransportStats;

    // ===== Semantic name aliases (for readability) =====

    /// Alias for `read_01` - Read coils
    #[inline]
    fn read_coils(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> impl std::future::Future<Output = ModbusResult<Vec<bool>>> + Send {
        self.read_01(slave_id, address, quantity)
    }

    /// Alias for `read_02` - Read discrete inputs
    #[inline]
    fn read_discrete_inputs(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> impl std::future::Future<Output = ModbusResult<Vec<bool>>> + Send {
        self.read_02(slave_id, address, quantity)
    }

    /// Alias for `read_03` - Read holding registers
    #[inline]
    fn read_holding_registers(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> impl std::future::Future<Output = ModbusResult<Vec<u16>>> + Send {
        self.read_03(slave_id, address, quantity)
    }

    /// Alias for `read_04` - Read input registers
    #[inline]
    fn read_input_registers(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> impl std::future::Future<Output = ModbusResult<Vec<u16>>> + Send {
        self.read_04(slave_id, address, quantity)
    }

    /// Alias for `write_05` - Write single coil
    #[inline]
    fn write_single_coil(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        value: bool,
    ) -> impl std::future::Future<Output = ModbusResult<()>> + Send {
        self.write_05(slave_id, address, value)
    }

    /// Alias for `write_06` - Write single register
    #[inline]
    fn write_single_register(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        value: u16,
    ) -> impl std::future::Future<Output = ModbusResult<()>> + Send {
        self.write_06(slave_id, address, value)
    }

    /// Alias for `write_0f` - Write multiple coils
    #[inline]
    fn write_multiple_coils(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        values: &[bool],
    ) -> impl std::future::Future<Output = ModbusResult<()>> + Send {
        self.write_0f(slave_id, address, values)
    }

    /// Alias for `write_10` - Write multiple registers
    #[inline]
    fn write_multiple_registers(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        values: &[u16],
    ) -> impl std::future::Future<Output = ModbusResult<()>> + Send {
        self.write_10(slave_id, address, values)
    }

    // ===== Batch read semantic aliases =====

    /// Alias for `read_01_batch` - Batch read coils with automatic chunking
    #[inline]
    fn read_coils_batch(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
        limits: &DeviceLimits,
    ) -> impl std::future::Future<Output = ModbusResult<Vec<bool>>> + Send
    where
        Self: Sized,
    {
        self.read_01_batch(slave_id, address, quantity, limits)
    }

    /// Alias for `read_02_batch` - Batch read discrete inputs with automatic chunking
    #[inline]
    fn read_discrete_inputs_batch(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
        limits: &DeviceLimits,
    ) -> impl std::future::Future<Output = ModbusResult<Vec<bool>>> + Send
    where
        Self: Sized,
    {
        self.read_02_batch(slave_id, address, quantity, limits)
    }

    /// Alias for `read_03_batch` - Batch read holding registers with automatic chunking
    #[inline]
    fn read_holding_registers_batch(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
        limits: &DeviceLimits,
    ) -> impl std::future::Future<Output = ModbusResult<Vec<u16>>> + Send
    where
        Self: Sized,
    {
        self.read_03_batch(slave_id, address, quantity, limits)
    }

    /// Alias for `read_04_batch` - Batch read input registers with automatic chunking
    #[inline]
    fn read_input_registers_batch(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
        limits: &DeviceLimits,
    ) -> impl std::future::Future<Output = ModbusResult<Vec<u16>>> + Send
    where
        Self: Sized,
    {
        self.read_04_batch(slave_id, address, quantity, limits)
    }
}

/// Generic Modbus client that works with any transport
///
/// This client implements the common application layer logic (PDU construction and parsing)
/// while delegating transport-specific concerns to the underlying transport implementation.
/// This eliminates code duplication between TCP and RTU clients since the PDU is identical.
pub struct GenericModbusClient<T: ModbusTransport> {
    transport: T,
    logger: Option<CallbackLogger>,
    retry_policy: RetryPolicy,
}

impl<T: ModbusTransport> GenericModbusClient<T> {
    /// Create a new generic client with the specified transport
    pub fn new(transport: T) -> Self {
        Self {
            transport,
            logger: None,
            retry_policy: RetryPolicy::none(),
        }
    }

    /// Create a new generic client with logging
    pub fn with_logger(transport: T, logger: CallbackLogger) -> Self {
        Self {
            transport,
            logger: Some(logger),
            retry_policy: RetryPolicy::none(),
        }
    }

    /// Get a reference to the underlying transport
    pub fn transport(&self) -> &T {
        &self.transport
    }

    /// Get a mutable reference to the underlying transport
    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    /// Set the retry policy applied to every request
    pub fn set_retry_policy(&mut self, policy: RetryPolicy) {
        self.retry_policy = policy;
    }

    /// Builder-style variant of [`Self::set_retry_policy`]
    pub fn with_retry_policy(mut self, policy: RetryPolicy) -> Self {
        self.retry_policy = policy;
        self
    }

    /// Execute a raw request, applying the configured [`RetryPolicy`] to
    /// recoverable failures (timeouts, connection loss, device-busy).
    pub async fn execute_request(
        &mut self,
        request: ModbusRequest,
    ) -> ModbusResult<ModbusResponse> {
        // Reject broadcast for non-write functions early — no response would ever arrive.
        if request.slave_id == 0 && !request.function.is_write_function() {
            return Err(ModbusError::invalid_data(
                "Broadcast (slave_id=0) is only valid for write operations",
            ));
        }
        request.validate()?;

        let mut attempt: u32 = 0;
        let mut backoff = self.retry_policy.initial_backoff;
        loop {
            match self.try_request_once(&request).await {
                Ok(response) => return Ok(response),
                Err(error) if error.is_recoverable() && attempt < self.retry_policy.max_retries => {
                    attempt += 1;
                    tracing::debug!(
                        attempt = attempt,
                        max_retries = self.retry_policy.max_retries,
                        error = %error,
                        "modbus.request.retry"
                    );
                    if backoff > Duration::ZERO {
                        tokio::time::sleep(backoff).await;
                        backoff = (backoff * 2).min(self.retry_policy.max_backoff);
                    }
                }
                Err(error) => return Err(error),
            }
        }
    }

    /// One request/response attempt including response validation and logging
    async fn try_request_once(&mut self, request: &ModbusRequest) -> ModbusResult<ModbusResponse> {
        // Log request if logger is available
        // Note: For accurate packet logging with real TID, use transport.set_packet_callback()
        if let Some(ref logger) = self.logger {
            logger.log_request(
                None, // TID is embedded in real packet via packet_callback
                request.slave_id,
                request.function.to_u8(),
                request.address,
                request.quantity,
                &request.data,
            );
        }

        // For broadcast writes (slave_id = 0) the transport layer returns a synthetic
        // ack immediately without waiting for a response (Modbus spec: no reply expected).
        // Regular unicast requests wait for the real device response.
        let response = self.transport.request(request).await?;
        validate_response_matches_request(request, &response)?;

        // Log response if logger is available
        if let Some(ref logger) = self.logger {
            logger.log_response(
                None,
                response.slave_id,
                response.function.to_u8(),
                response.data(),
            );
        }

        Ok(response)
    }
}

fn validate_response_matches_request(
    request: &ModbusRequest,
    response: &ModbusResponse,
) -> ModbusResult<()> {
    if let Some(error) = response.get_exception() {
        return Err(error);
    }

    if response.slave_id != request.slave_id {
        return Err(ModbusError::protocol(format!(
            "Response slave ID mismatch: expected {}, got {}",
            request.slave_id, response.slave_id
        )));
    }

    if response.function != request.function {
        return Err(ModbusError::protocol(format!(
            "Response function mismatch: expected 0x{:02X}, got 0x{:02X}",
            request.function.to_u8(),
            response.function.to_u8()
        )));
    }

    if request.slave_id == 0 {
        return Ok(());
    }

    match request.function {
        ModbusFunction::ReadCoils | ModbusFunction::ReadDiscreteInputs => {
            validate_read_byte_count(request, response, usize::from(request.quantity.div_ceil(8)))
        }
        ModbusFunction::ReadHoldingRegisters | ModbusFunction::ReadInputRegisters => {
            validate_read_byte_count(request, response, usize::from(request.quantity) * 2)
        }
        ModbusFunction::WriteSingleCoil => validate_write_echo(
            response,
            request.address,
            expected_single_coil_value(request),
        ),
        ModbusFunction::WriteSingleRegister => {
            let data = request.data.as_slice();
            if data.len() != 2 {
                return Err(ModbusError::invalid_data(
                    "Invalid single register payload length",
                ));
            }
            validate_write_echo(
                response,
                request.address,
                u16::from_be_bytes([data[0], data[1]]),
            )
        }
        ModbusFunction::WriteMultipleCoils | ModbusFunction::WriteMultipleRegisters => {
            validate_write_echo(response, request.address, request.quantity)
        }
        ModbusFunction::MaskWriteRegister => {
            // Response echoes the request: address(2) + and_mask(2) + or_mask(2)
            let data = response.data();
            if data.len() != 6 {
                return Err(ModbusError::frame(format!(
                    "Invalid mask write response length: expected 6, got {}",
                    data.len()
                )));
            }
            if data[0..2] != request.address.to_be_bytes() || data[2..6] != request.data[..] {
                return Err(ModbusError::protocol(
                    "Mask write echo mismatch: device returned different address or masks",
                ));
            }
            Ok(())
        }
        ModbusFunction::ReadWriteMultipleRegisters => {
            // Response carries the read side: byte_count + registers
            validate_read_byte_count(request, response, usize::from(request.quantity) * 2)
        }
        ModbusFunction::ReadDeviceIdentification => {
            let data = response.data();
            if data.len() < 6 || data[0] != crate::constants::MEI_READ_DEVICE_ID {
                return Err(ModbusError::frame(
                    "Invalid device identification response header",
                ));
            }
            Ok(())
        }
        ModbusFunction::ReadExceptionStatus => {
            if response.data().len() != 1 {
                return Err(ModbusError::frame(
                    "Invalid exception status response length",
                ));
            }
            Ok(())
        }
        ModbusFunction::Diagnostics => {
            // Response echoes the sub-function, followed by echo/counter data
            let data = response.data();
            if data.len() < 4 || data[0..2] != request.data[0..2] {
                return Err(ModbusError::frame("Invalid diagnostics response"));
            }
            Ok(())
        }
        ModbusFunction::GetCommEventCounter => {
            if response.data().len() != 4 {
                return Err(ModbusError::frame(
                    "Invalid comm event counter response length",
                ));
            }
            Ok(())
        }
        ModbusFunction::GetCommEventLog => {
            // byte_count + status(2) + event count(2) + message count(2) + events
            let data = response.data();
            if data.len() < 7 || usize::from(data[0]) != data.len() - 1 {
                return Err(ModbusError::frame("Invalid comm event log response"));
            }
            Ok(())
        }
        ModbusFunction::ReportServerId => {
            // byte_count + device-specific payload
            let data = response.data();
            if data.len() < 2 || usize::from(data[0]) != data.len() - 1 {
                return Err(ModbusError::frame("Invalid report server id response"));
            }
            Ok(())
        }
    }
}

fn validate_read_byte_count(
    request: &ModbusRequest,
    response: &ModbusResponse,
    expected_byte_count: usize,
) -> ModbusResult<()> {
    let data = response.data();
    if data.len() != 1 + expected_byte_count {
        return Err(ModbusError::frame(format!(
            "Invalid read response length for 0x{:02X}: expected {}, got {}",
            request.function.to_u8(),
            1 + expected_byte_count,
            data.len()
        )));
    }
    if usize::from(data[0]) != expected_byte_count {
        return Err(ModbusError::frame(format!(
            "Invalid read response byte count for 0x{:02X}: expected {}, got {}",
            request.function.to_u8(),
            expected_byte_count,
            data[0]
        )));
    }
    Ok(())
}

fn validate_write_echo(
    response: &ModbusResponse,
    expected_address: u16,
    expected_value_or_quantity: u16,
) -> ModbusResult<()> {
    let data = response.data();
    if data.len() != 4 {
        return Err(ModbusError::frame(format!(
            "Invalid write response length: expected 4, got {}",
            data.len()
        )));
    }

    let actual_address = u16::from_be_bytes([data[0], data[1]]);
    let actual_value_or_quantity = u16::from_be_bytes([data[2], data[3]]);
    if actual_address != expected_address || actual_value_or_quantity != expected_value_or_quantity
    {
        return Err(ModbusError::protocol(format!(
            "Write echo mismatch: expected address={} value={}, got address={} value={}",
            expected_address, expected_value_or_quantity, actual_address, actual_value_or_quantity
        )));
    }

    Ok(())
}

fn expected_single_coil_value(request: &ModbusRequest) -> u16 {
    if !request.data.is_empty() && request.data[0] != 0 {
        0xFF00
    } else {
        0x0000
    }
}

impl<T: ModbusTransport + Send + Sync> ModbusClient for GenericModbusClient<T> {
    async fn read_01(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<bool>> {
        if quantity == 0 || quantity > 2000 {
            return Err(ModbusError::invalid_data("Invalid quantity"));
        }

        let request = ModbusRequest {
            slave_id,
            function: ModbusFunction::ReadCoils,
            address,
            quantity,
            data: vec![],
        };

        let response = self.execute_request(request).await?;
        // Use parse_bits() which correctly skips byte_count prefix
        let mut bits = response.parse_bits()?;
        bits.truncate(quantity as usize);
        Ok(bits)
    }

    async fn read_02(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<bool>> {
        if quantity == 0 || quantity > 2000 {
            return Err(ModbusError::invalid_data("Invalid quantity"));
        }

        let request = ModbusRequest {
            slave_id,
            function: ModbusFunction::ReadDiscreteInputs,
            address,
            quantity,
            data: vec![],
        };

        let response = self.execute_request(request).await?;
        // Use parse_bits() which correctly skips byte_count prefix
        let mut bits = response.parse_bits()?;
        bits.truncate(quantity as usize);
        Ok(bits)
    }

    async fn read_03(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<u16>> {
        if quantity == 0 || quantity > 125 {
            return Err(ModbusError::invalid_data("Invalid quantity"));
        }

        let request = ModbusRequest {
            slave_id,
            function: ModbusFunction::ReadHoldingRegisters,
            address,
            quantity,
            data: vec![],
        };

        let response = self.execute_request(request).await?;
        // Use parse_registers() which correctly skips byte_count prefix
        response.parse_registers()
    }

    async fn read_04(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<u16>> {
        if quantity == 0 || quantity > 125 {
            return Err(ModbusError::invalid_data("Invalid quantity"));
        }

        let request = ModbusRequest {
            slave_id,
            function: ModbusFunction::ReadInputRegisters,
            address,
            quantity,
            data: vec![],
        };

        let response = self.execute_request(request).await?;
        // Use parse_registers() which correctly skips byte_count prefix
        response.parse_registers()
    }

    async fn write_05(&mut self, slave_id: SlaveId, address: u16, value: bool) -> ModbusResult<()> {
        let request = ModbusRequest {
            slave_id,
            function: ModbusFunction::WriteSingleCoil,
            address,
            quantity: 1,
            data: if value {
                vec![0xFF, 0x00]
            } else {
                vec![0x00, 0x00]
            },
        };

        self.execute_request(request).await?;
        Ok(())
    }

    async fn write_06(&mut self, slave_id: SlaveId, address: u16, value: u16) -> ModbusResult<()> {
        let [hi, lo] = value.to_be_bytes();
        let request = ModbusRequest {
            slave_id,
            function: ModbusFunction::WriteSingleRegister,
            address,
            quantity: 1,
            data: vec![hi, lo],
        };

        self.execute_request(request).await?;
        Ok(())
    }

    async fn write_0f(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        values: &[bool],
    ) -> ModbusResult<()> {
        if values.is_empty() || values.len() > 1968 {
            return Err(ModbusError::invalid_data("Invalid quantity"));
        }

        let byte_count = values.len().div_ceil(8);
        // Note: byte_count is added by transport layer, we only send the coil data
        let mut data = Vec::with_capacity(byte_count);

        for chunk in values.chunks(8) {
            let mut byte = 0u8;
            for (i, &coil) in chunk.iter().enumerate() {
                if coil {
                    byte |= 1 << i;
                }
            }
            data.push(byte);
        }

        let request = ModbusRequest {
            slave_id,
            function: ModbusFunction::WriteMultipleCoils,
            address,
            quantity: values.len() as u16,
            data,
        };

        self.execute_request(request).await?;
        Ok(())
    }

    async fn write_10(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        values: &[u16],
    ) -> ModbusResult<()> {
        if values.is_empty() || values.len() > 123 {
            return Err(ModbusError::invalid_data("Invalid quantity"));
        }

        // Note: byte_count is added by transport layer, we only send the register data
        let mut data = Vec::with_capacity(values.len() * 2);
        for &value in values {
            data.extend_from_slice(&value.to_be_bytes());
        }

        let request = ModbusRequest {
            slave_id,
            function: ModbusFunction::WriteMultipleRegisters,
            address,
            quantity: values.len() as u16,
            data,
        };

        self.execute_request(request).await?;
        Ok(())
    }

    fn is_connected(&self) -> bool {
        self.transport.is_connected()
    }

    async fn close(&mut self) -> ModbusResult<()> {
        self.transport.close().await
    }

    fn get_stats(&self) -> TransportStats {
        self.transport.get_stats()
    }
}

/// Coalesced read methods available on any `GenericModbusClient<T>`
impl<T: ModbusTransport + Send + Sync> GenericModbusClient<T> {
    /// 批量读取多个 Holding Register 区域，自动合并相邻请求（FC03）
    ///
    /// 将多个 `(address, quantity)` 区域按 [`ReadCoalescer`] 的规则合并，
    /// 用更少的网络请求完成读取，然后按原始输入顺序返回各区域的数据。
    ///
    /// # Arguments
    ///
    /// * `slave_id` - 从站 ID（1-247）
    /// * `regions` - 待读取区域列表，每个元素为 `(address, quantity)`
    ///
    /// # Returns
    ///
    /// 按输入顺序返回每个区域的寄存器数据。
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// use voltage_modbus::{ModbusTcpClient, ModbusResult};
    /// use std::time::Duration;
    ///
    /// # async fn example() -> ModbusResult<()> {
    /// let mut client = ModbusTcpClient::from_address("127.0.0.1:502", Duration::from_secs(5)).await?;
    ///
    /// // 读取温度(0-1)、压力(2-3)、流量(10-11)，三个区域合并为一次请求
    /// let results = client.read_holding_registers_coalesced(1, &[(0, 2), (2, 2), (10, 2)]).await?;
    /// let temperature = &results[0]; // [reg0, reg1]
    /// let pressure    = &results[1]; // [reg2, reg3]
    /// let flow        = &results[2]; // [reg10, reg11]
    /// # Ok(())
    /// # }
    /// ```
    pub async fn read_holding_registers_coalesced(
        &mut self,
        slave_id: u8,
        regions: &[(u16, u16)],
    ) -> ModbusResult<Vec<Vec<u16>>> {
        self.inner_read_coalesced(slave_id, 0x03, regions).await
    }

    /// 批量读取多个 Input Register 区域，自动合并相邻请求（FC04）
    ///
    /// 与 [`read_holding_registers_coalesced`](Self::read_holding_registers_coalesced) 相同，
    /// 使用 FC04（Input Registers）。
    pub async fn read_input_registers_coalesced(
        &mut self,
        slave_id: u8,
        regions: &[(u16, u16)],
    ) -> ModbusResult<Vec<Vec<u16>>> {
        self.inner_read_coalesced(slave_id, 0x04, regions).await
    }

    /// 内部实现：对给定 function code 执行读合并
    async fn inner_read_coalesced(
        &mut self,
        slave_id: u8,
        function: u8,
        regions: &[(u16, u16)],
    ) -> ModbusResult<Vec<Vec<u16>>> {
        if regions.is_empty() {
            return Ok(Vec::new());
        }

        // 构建 ReadRequest 列表
        let requests: Vec<crate::coalescer::ReadRequest> = regions
            .iter()
            .map(|&(address, quantity)| {
                crate::coalescer::ReadRequest::new(slave_id, function, address, quantity)
            })
            .collect();

        let coalescer = ReadCoalescer::new();
        let coalesced_list = coalescer.coalesce(&requests);

        // 按合并后的顺序执行读请求，收集 (original_index → data) 映射
        let mut results: Vec<Vec<u16>> = vec![Vec::new(); regions.len()];

        for coalesced in &coalesced_list {
            // 执行合并后的读请求
            let data = match function {
                0x03 => {
                    self.read_03(slave_id, coalesced.address, coalesced.quantity)
                        .await?
                }
                0x04 => {
                    self.read_04(slave_id, coalesced.address, coalesced.quantity)
                        .await?
                }
                _ => return Err(ModbusError::invalid_function(function)),
            };

            // 从合并响应中提取各原始区域的数据
            let extracted = coalescer.extract_results(coalesced, &data);
            for (i, &(orig_idx, _, _)) in coalesced.mappings.iter().enumerate() {
                results[orig_idx] = extracted[i].clone();
            }
        }

        Ok(results)
    }
}

/// Extended function codes (FC 0x16 / 0x17 / 0x2B), available on any transport
impl<T: ModbusTransport + Send + Sync> GenericModbusClient<T> {
    /// Mask write register (function code 0x16).
    ///
    /// The device atomically computes
    /// `(current & and_mask) | (or_mask & !and_mask)`, avoiding the
    /// read-modify-write race of a separate FC03 + FC06 sequence.
    pub async fn write_16(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        and_mask: u16,
        or_mask: u16,
    ) -> ModbusResult<()> {
        let request = ModbusRequest::new_mask_write(slave_id, address, and_mask, or_mask);
        self.execute_request(request).await?;
        Ok(())
    }

    /// Alias for [`Self::write_16`] — mask write register
    #[inline]
    pub async fn mask_write_register(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        and_mask: u16,
        or_mask: u16,
    ) -> ModbusResult<()> {
        self.write_16(slave_id, address, and_mask, or_mask).await
    }

    /// Read/write multiple registers (function code 0x17).
    ///
    /// Writes `values` at `write_address` and reads `read_quantity` registers
    /// from `read_address` in one transaction; per spec, the device performs
    /// the write **before** the read.
    pub async fn read_write_17(
        &mut self,
        slave_id: SlaveId,
        read_address: u16,
        read_quantity: u16,
        write_address: u16,
        values: &[u16],
    ) -> ModbusResult<Vec<u16>> {
        if read_quantity == 0
            || usize::from(read_quantity) > crate::constants::MAX_RW_READ_REGISTERS
        {
            return Err(ModbusError::invalid_data("Invalid read quantity"));
        }
        if values.is_empty() || values.len() > crate::constants::MAX_RW_WRITE_REGISTERS {
            return Err(ModbusError::invalid_data("Invalid write quantity"));
        }

        let request = ModbusRequest::new_read_write_multiple(
            slave_id,
            read_address,
            read_quantity,
            write_address,
            values,
        );
        let response = self.execute_request(request).await?;
        response.parse_registers()
    }

    /// Alias for [`Self::read_write_17`] — read/write multiple registers
    #[inline]
    pub async fn read_write_multiple_registers(
        &mut self,
        slave_id: SlaveId,
        read_address: u16,
        read_quantity: u16,
        write_address: u16,
        values: &[u16],
    ) -> ModbusResult<Vec<u16>> {
        self.read_write_17(slave_id, read_address, read_quantity, write_address, values)
            .await
    }

    /// Read device identification (function code 0x2B / MEI type 0x0E).
    ///
    /// `read_code`: 1 = basic objects (VendorName / ProductCode / Revision),
    /// 2 = regular, 3 = extended, 4 = one specific object. When the returned
    /// block has `more_follows` set, issue a follow-up call starting at its
    /// `next_object_id`.
    pub async fn read_device_identification(
        &mut self,
        slave_id: SlaveId,
        read_code: u8,
        object_id: u8,
    ) -> ModbusResult<crate::protocol::DeviceIdentification> {
        let request = ModbusRequest::new_read_device_identification(slave_id, read_code, object_id);
        let response = self.execute_request(request).await?;
        response.parse_device_identification()
    }
}

/// Serial-line diagnostic functions (FC 0x07 / 0x08 / 0x0B / 0x0C / 0x11)
impl<T: ModbusTransport + Send + Sync> GenericModbusClient<T> {
    /// Read exception status (FC 0x07) — 8 device-defined status bits.
    pub async fn read_exception_status(&mut self, slave_id: SlaveId) -> ModbusResult<u8> {
        let request = ModbusRequest::new_no_data(slave_id, ModbusFunction::ReadExceptionStatus);
        let response = self.execute_request(request).await?;
        Ok(response.data()[0]) // length validated to 1 by response validation
    }

    /// Diagnostics (FC 0x08).
    ///
    /// Sub-function `0x0000` is the Return Query Data echo test: the device
    /// must return `data` unchanged, making this the standard link health
    /// check. Returns the 16-bit data field from the response (echo or
    /// counter value depending on the sub-function).
    pub async fn diagnostics(
        &mut self,
        slave_id: SlaveId,
        sub_function: u16,
        data: u16,
    ) -> ModbusResult<u16> {
        let request = ModbusRequest::new_diagnostics(slave_id, sub_function, data);
        let response = self.execute_request(request).await?;
        let payload = response.data(); // length >= 4 validated
        Ok(u16::from_be_bytes([payload[2], payload[3]]))
    }

    /// Get comm event counter (FC 0x0B) — returns `(status, event_count)`.
    /// Status 0xFFFF means the device is busy with a long-running command.
    pub async fn get_comm_event_counter(&mut self, slave_id: SlaveId) -> ModbusResult<(u16, u16)> {
        let request = ModbusRequest::new_no_data(slave_id, ModbusFunction::GetCommEventCounter);
        let response = self.execute_request(request).await?;
        let payload = response.data(); // length validated to 4
        Ok((
            u16::from_be_bytes([payload[0], payload[1]]),
            u16::from_be_bytes([payload[2], payload[3]]),
        ))
    }

    /// Get comm event log (FC 0x0C) — status, counters and the raw event bytes.
    pub async fn get_comm_event_log(
        &mut self,
        slave_id: SlaveId,
    ) -> ModbusResult<crate::protocol::CommEventLog> {
        let request = ModbusRequest::new_no_data(slave_id, ModbusFunction::GetCommEventLog);
        let response = self.execute_request(request).await?;
        let payload = response.data(); // byte count + >= 6 bytes validated
        Ok(crate::protocol::CommEventLog {
            status: u16::from_be_bytes([payload[1], payload[2]]),
            event_count: u16::from_be_bytes([payload[3], payload[4]]),
            message_count: u16::from_be_bytes([payload[5], payload[6]]),
            events: payload[7..].to_vec(),
        })
    }

    /// Report server id (FC 0x11) — device-specific id bytes plus the run
    /// indicator when present.
    pub async fn report_server_id(
        &mut self,
        slave_id: SlaveId,
    ) -> ModbusResult<crate::protocol::ServerIdReport> {
        let request = ModbusRequest::new_no_data(slave_id, ModbusFunction::ReportServerId);
        let response = self.execute_request(request).await?;
        Ok(crate::protocol::ServerIdReport::parse(
            &response.data()[1..],
        ))
    }
}

/// Modbus TCP client implementation using the generic client
pub struct ModbusTcpClient {
    inner: GenericModbusClient<TcpTransport>,
}

impl ModbusTcpClient {
    /// Create a new TCP client
    pub async fn new(addr: SocketAddr, timeout: Duration) -> ModbusResult<Self> {
        let transport = TcpTransport::new(addr, timeout).await?;
        Ok(Self {
            inner: GenericModbusClient::new(transport),
        })
    }

    /// Create a new TCP client with logging
    pub async fn with_logging(
        addr: &str,
        timeout: Duration,
        logger: Option<CallbackLogger>,
    ) -> ModbusResult<Self> {
        let addr: SocketAddr = addr
            .parse()
            .map_err(|e| ModbusError::configuration(format!("Invalid address: {}", e)))?;
        let transport = TcpTransport::new(addr, timeout).await?;
        let logger = logger.unwrap_or_default();
        Ok(Self {
            inner: GenericModbusClient::with_logger(transport, logger),
        })
    }

    /// Create a new TCP client from address string
    pub async fn from_address(addr: &str, timeout: Duration) -> ModbusResult<Self> {
        let addr: SocketAddr = addr
            .parse()
            .map_err(|e| ModbusError::configuration(format!("Invalid address: {}", e)))?;
        Self::new(addr, timeout).await
    }

    /// Create a new TCP client from transport
    pub fn from_transport(transport: TcpTransport) -> Self {
        Self {
            inner: GenericModbusClient::new(transport),
        }
    }

    /// Get the server address
    pub fn server_address(&self) -> SocketAddr {
        self.inner.transport().address
    }

    /// Enable or disable packet logging on existing client
    pub fn set_packet_logging(&mut self, enabled: bool) {
        self.inner.transport_mut().set_packet_logging(enabled);
    }

    /// Set the retry policy for recoverable failures — see [`RetryPolicy`]
    pub fn set_retry_policy(&mut self, policy: RetryPolicy) {
        self.inner.set_retry_policy(policy);
    }

    /// Access the underlying generic client — exposes the full method set
    /// (serial diagnostics FC 0x07/0x08/0x0B/0x0C/0x11, coalesced reads, ...)
    pub fn generic_mut(&mut self) -> &mut GenericModbusClient<TcpTransport> {
        &mut self.inner
    }

    /// Convert into a cloneable, task-shareable handle — see [`SharedModbusClient`]
    pub fn into_shared(self) -> SharedModbusClient<TcpTransport> {
        SharedModbusClient::new(self.inner)
    }

    /// Execute a raw request
    pub async fn execute_request(
        &mut self,
        request: ModbusRequest,
    ) -> ModbusResult<ModbusResponse> {
        self.inner.execute_request(request).await
    }

    /// Execute multiple requests in a pipeline (concurrent send, batch receive).
    ///
    /// Sends all requests over the TCP connection with a single `write_all`, then
    /// receives all responses and reorders them to match the original request order.
    ///
    /// Modbus TCP's MBAP Transaction ID field makes this safe: each response carries
    /// the TID of its request, so responses can arrive in any order.
    ///
    /// # Arguments
    ///
    /// * `requests` - List of requests to send (each must have a valid slave ID)
    /// * `pipeline_timeout` - Total timeout for the entire pipeline operation
    ///
    /// # Returns
    ///
    /// A `Vec<ModbusResult<ModbusResponse>>` in the **same order** as `requests`.
    /// Individual entries may be `Err` if that particular request failed, while the
    /// others remain `Ok`.
    ///
    /// Returns `Err` only for fatal errors (send failure, connection loss) that
    /// prevent *any* response from being received.
    ///
    /// # Example
    ///
    /// ```rust,no_run
    /// use voltage_modbus::{ModbusTcpClient, ModbusResult};
    /// use voltage_modbus::protocol::{ModbusRequest, ModbusFunction};
    /// use std::time::Duration;
    ///
    /// # async fn example() -> ModbusResult<()> {
    /// let mut client = ModbusTcpClient::from_address("127.0.0.1:502", Duration::from_secs(5)).await?;
    ///
    /// let requests = vec![
    ///     ModbusRequest::new_read(1, ModbusFunction::ReadHoldingRegisters, 0, 10),
    ///     ModbusRequest::new_read(1, ModbusFunction::ReadHoldingRegisters, 100, 5),
    ///     ModbusRequest::new_read(1, ModbusFunction::ReadInputRegisters, 0, 3),
    /// ];
    ///
    /// let results = client.pipeline(requests, Duration::from_secs(5)).await?;
    /// for (i, result) in results.iter().enumerate() {
    ///     match result {
    ///         Ok(response) => println!("Request {}: {} bytes", i, response.data_len()),
    ///         Err(e) => println!("Request {}: failed - {}", i, e),
    ///     }
    /// }
    /// # Ok(())
    /// # }
    /// ```
    pub async fn pipeline(
        &mut self,
        requests: Vec<ModbusRequest>,
        pipeline_timeout: Duration,
    ) -> ModbusResult<Vec<ModbusResult<ModbusResponse>>> {
        if requests.is_empty() {
            return Ok(Vec::new());
        }

        let count = requests.len();
        let transport = self.inner.transport_mut();

        // Send all frames; returns the TID assigned to each request (same order)
        let tids = transport.send_pipeline_requests(&requests).await?;

        // Receive all responses indexed by TID
        let mut response_map = transport
            .receive_pipeline_responses(count, pipeline_timeout)
            .await?;

        // Reorder by original request order using tids
        let results = tids
            .into_iter()
            .map(|tid| {
                response_map.remove(&tid).unwrap_or_else(|| {
                    Err(ModbusError::timeout(
                        "pipeline response missing",
                        pipeline_timeout.as_millis() as u64,
                    ))
                })
            })
            .collect();

        Ok(results)
    }

    /// Convenience method: pipeline multiple FC03 (read holding registers) requests.
    ///
    /// Each entry in `reads` is `(address, quantity)`.  Results are returned in the
    /// same order; each entry is `Ok(Vec<u16>)` on success or `Err` on failure.
    ///
    /// # Arguments
    ///
    /// * `slave_id` - Modbus slave ID (1-247)
    /// * `reads` - Slice of `(start_address, quantity)` pairs
    /// * `pipeline_timeout` - Total timeout for the pipeline operation
    ///
    /// # Example
    ///
    /// ```rust,no_run
    /// use voltage_modbus::{ModbusTcpClient, ModbusResult};
    /// use std::time::Duration;
    ///
    /// # async fn example() -> ModbusResult<()> {
    /// let mut client = ModbusTcpClient::from_address("127.0.0.1:502", Duration::from_secs(5)).await?;
    ///
    /// let results = client.pipeline_reads(1, &[(0, 10), (100, 5), (200, 3)], Duration::from_secs(5)).await?;
    /// for (i, result) in results.iter().enumerate() {
    ///     match result {
    ///         Ok(regs) => println!("Segment {}: {:?}", i, regs),
    ///         Err(e) => println!("Segment {}: error - {}", i, e),
    ///     }
    /// }
    /// # Ok(())
    /// # }
    /// ```
    pub async fn pipeline_reads(
        &mut self,
        slave_id: SlaveId,
        reads: &[(u16, u16)], // (address, quantity)
        pipeline_timeout: Duration,
    ) -> ModbusResult<Vec<ModbusResult<Vec<u16>>>> {
        let requests: Vec<ModbusRequest> = reads
            .iter()
            .map(|&(address, quantity)| {
                ModbusRequest::new_read(
                    slave_id,
                    ModbusFunction::ReadHoldingRegisters,
                    address,
                    quantity,
                )
            })
            .collect();

        let raw_results = self.pipeline(requests, pipeline_timeout).await?;

        let results = raw_results
            .into_iter()
            .map(|r| r.and_then(|resp| resp.parse_registers()))
            .collect();

        Ok(results)
    }

    /// Mask write register (FC 0x16) — see [`GenericModbusClient::write_16`]
    pub async fn write_16(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        and_mask: u16,
        or_mask: u16,
    ) -> ModbusResult<()> {
        self.inner
            .write_16(slave_id, address, and_mask, or_mask)
            .await
    }

    /// Read/write multiple registers (FC 0x17) — see [`GenericModbusClient::read_write_17`]
    pub async fn read_write_17(
        &mut self,
        slave_id: SlaveId,
        read_address: u16,
        read_quantity: u16,
        write_address: u16,
        values: &[u16],
    ) -> ModbusResult<Vec<u16>> {
        self.inner
            .read_write_17(slave_id, read_address, read_quantity, write_address, values)
            .await
    }

    /// Read device identification (FC 0x2B / MEI 0x0E) —
    /// see [`GenericModbusClient::read_device_identification`]
    pub async fn read_device_identification(
        &mut self,
        slave_id: SlaveId,
        read_code: u8,
        object_id: u8,
    ) -> ModbusResult<crate::protocol::DeviceIdentification> {
        self.inner
            .read_device_identification(slave_id, read_code, object_id)
            .await
    }
}

impl ModbusClient for ModbusTcpClient {
    async fn read_01(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<bool>> {
        self.inner.read_01(slave_id, address, quantity).await
    }

    async fn read_02(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<bool>> {
        self.inner.read_02(slave_id, address, quantity).await
    }

    async fn read_03(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<u16>> {
        self.inner.read_03(slave_id, address, quantity).await
    }

    async fn read_04(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<u16>> {
        self.inner.read_04(slave_id, address, quantity).await
    }

    async fn write_05(&mut self, slave_id: SlaveId, address: u16, value: bool) -> ModbusResult<()> {
        self.inner.write_05(slave_id, address, value).await
    }

    async fn write_06(&mut self, slave_id: SlaveId, address: u16, value: u16) -> ModbusResult<()> {
        self.inner.write_06(slave_id, address, value).await
    }

    async fn write_0f(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        values: &[bool],
    ) -> ModbusResult<()> {
        self.inner.write_0f(slave_id, address, values).await
    }

    async fn write_10(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        values: &[u16],
    ) -> ModbusResult<()> {
        self.inner.write_10(slave_id, address, values).await
    }

    fn is_connected(&self) -> bool {
        self.inner.is_connected()
    }

    async fn close(&mut self) -> ModbusResult<()> {
        self.inner.close().await
    }

    fn get_stats(&self) -> TransportStats {
        self.inner.get_stats()
    }
}

/// Modbus RTU client implementation using the generic client
#[cfg(feature = "rtu")]
pub struct ModbusRtuClient {
    inner: GenericModbusClient<RtuTransport>,
}

#[cfg(feature = "rtu")]
impl ModbusRtuClient {
    /// Create a new RTU client with default settings
    pub fn new(port: &str, baud_rate: u32) -> ModbusResult<Self> {
        let transport = RtuTransport::new(port, baud_rate)?;
        Ok(Self {
            inner: GenericModbusClient::new(transport),
        })
    }

    /// Create a new RTU client with logging
    pub fn with_logging(
        port: &str,
        baud_rate: u32,
        logger: Option<CallbackLogger>,
    ) -> ModbusResult<Self> {
        let transport = RtuTransport::new(port, baud_rate)?;
        let logger = logger.unwrap_or_default();
        Ok(Self {
            inner: GenericModbusClient::with_logger(transport, logger),
        })
    }

    /// Create a new RTU client with custom configuration and logging
    pub fn with_config_and_logging(
        port: &str,
        baud_rate: u32,
        data_bits: tokio_serial::DataBits,
        stop_bits: tokio_serial::StopBits,
        parity: tokio_serial::Parity,
        timeout: Duration,
        logger: Option<CallbackLogger>,
    ) -> ModbusResult<Self> {
        let transport =
            RtuTransport::new_with_config(port, baud_rate, data_bits, stop_bits, parity, timeout)?;
        let logger = logger.unwrap_or_default();
        Ok(Self {
            inner: GenericModbusClient::with_logger(transport, logger),
        })
    }

    /// Create from existing RtuTransport
    pub fn from_transport(transport: RtuTransport) -> Self {
        Self {
            inner: GenericModbusClient::new(transport),
        }
    }

    /// Get the transport reference
    pub fn transport(&self) -> &RtuTransport {
        self.inner.transport()
    }

    /// Enable or disable packet logging on existing client
    pub fn set_packet_logging(&mut self, enabled: bool) {
        self.inner.transport_mut().set_packet_logging(enabled);
    }

    /// Set the retry policy for recoverable failures — see [`RetryPolicy`]
    pub fn set_retry_policy(&mut self, policy: RetryPolicy) {
        self.inner.set_retry_policy(policy);
    }

    /// Access the underlying generic client — exposes the full method set
    /// (serial diagnostics FC 0x07/0x08/0x0B/0x0C/0x11, coalesced reads, ...)
    pub fn generic_mut(&mut self) -> &mut GenericModbusClient<RtuTransport> {
        &mut self.inner
    }

    /// Convert into a cloneable, task-shareable handle — see [`SharedModbusClient`]
    pub fn into_shared(self) -> SharedModbusClient<RtuTransport> {
        SharedModbusClient::new(self.inner)
    }

    /// Execute a raw request
    pub async fn execute_request(
        &mut self,
        request: ModbusRequest,
    ) -> ModbusResult<ModbusResponse> {
        self.inner.execute_request(request).await
    }

    /// Mask write register (FC 0x16) — see [`GenericModbusClient::write_16`]
    pub async fn write_16(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        and_mask: u16,
        or_mask: u16,
    ) -> ModbusResult<()> {
        self.inner
            .write_16(slave_id, address, and_mask, or_mask)
            .await
    }

    /// Read/write multiple registers (FC 0x17) — see [`GenericModbusClient::read_write_17`]
    pub async fn read_write_17(
        &mut self,
        slave_id: SlaveId,
        read_address: u16,
        read_quantity: u16,
        write_address: u16,
        values: &[u16],
    ) -> ModbusResult<Vec<u16>> {
        self.inner
            .read_write_17(slave_id, read_address, read_quantity, write_address, values)
            .await
    }

    /// Read device identification (FC 0x2B / MEI 0x0E) —
    /// see [`GenericModbusClient::read_device_identification`]
    pub async fn read_device_identification(
        &mut self,
        slave_id: SlaveId,
        read_code: u8,
        object_id: u8,
    ) -> ModbusResult<crate::protocol::DeviceIdentification> {
        self.inner
            .read_device_identification(slave_id, read_code, object_id)
            .await
    }
}

/// Modbus RTU-over-TCP client.
///
/// Uses RTU framing (slave + PDU + CRC-16) over a raw TCP stream. Common on
/// industrial gateways that bridge serial Modbus onto Ethernet without
/// translating to proper Modbus TCP. Does not require serial dependencies.
pub struct ModbusRtuOverTcpClient {
    inner: GenericModbusClient<crate::transport::RtuOverTcpTransport>,
}

impl ModbusRtuOverTcpClient {
    /// Connect to an RTU-over-TCP gateway.
    pub async fn new(address: std::net::SocketAddr, timeout: Duration) -> ModbusResult<Self> {
        let transport = crate::transport::RtuOverTcpTransport::new(address, timeout).await?;
        Ok(Self {
            inner: GenericModbusClient::new(transport),
        })
    }

    /// Parse address string and connect (e.g. `"192.168.1.10:502"`).
    pub async fn from_address(address: &str, timeout: Duration) -> ModbusResult<Self> {
        let transport =
            crate::transport::RtuOverTcpTransport::from_address(address, timeout).await?;
        Ok(Self {
            inner: GenericModbusClient::new(transport),
        })
    }

    /// Set the retry policy for recoverable failures — see [`RetryPolicy`]
    pub fn set_retry_policy(&mut self, policy: RetryPolicy) {
        self.inner.set_retry_policy(policy);
    }

    /// Access the underlying generic client — exposes the full method set
    /// (serial diagnostics FC 0x07/0x08/0x0B/0x0C/0x11, coalesced reads, ...)
    pub fn generic_mut(
        &mut self,
    ) -> &mut GenericModbusClient<crate::transport::RtuOverTcpTransport> {
        &mut self.inner
    }

    /// Convert into a cloneable, task-shareable handle — see [`SharedModbusClient`]
    pub fn into_shared(self) -> SharedModbusClient<crate::transport::RtuOverTcpTransport> {
        SharedModbusClient::new(self.inner)
    }

    /// Execute a raw request.
    pub async fn execute_request(
        &mut self,
        request: ModbusRequest,
    ) -> ModbusResult<ModbusResponse> {
        self.inner.execute_request(request).await
    }

    /// Mask write register (FC 0x16) — see [`GenericModbusClient::write_16`]
    pub async fn write_16(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        and_mask: u16,
        or_mask: u16,
    ) -> ModbusResult<()> {
        self.inner
            .write_16(slave_id, address, and_mask, or_mask)
            .await
    }

    /// Read/write multiple registers (FC 0x17) — see [`GenericModbusClient::read_write_17`]
    pub async fn read_write_17(
        &mut self,
        slave_id: SlaveId,
        read_address: u16,
        read_quantity: u16,
        write_address: u16,
        values: &[u16],
    ) -> ModbusResult<Vec<u16>> {
        self.inner
            .read_write_17(slave_id, read_address, read_quantity, write_address, values)
            .await
    }

    /// Read device identification (FC 0x2B / MEI 0x0E) —
    /// see [`GenericModbusClient::read_device_identification`]
    pub async fn read_device_identification(
        &mut self,
        slave_id: SlaveId,
        read_code: u8,
        object_id: u8,
    ) -> ModbusResult<crate::protocol::DeviceIdentification> {
        self.inner
            .read_device_identification(slave_id, read_code, object_id)
            .await
    }
}

impl ModbusClient for ModbusRtuOverTcpClient {
    async fn read_01(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<bool>> {
        self.inner.read_01(slave_id, address, quantity).await
    }
    async fn read_02(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<bool>> {
        self.inner.read_02(slave_id, address, quantity).await
    }
    async fn read_03(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<u16>> {
        self.inner.read_03(slave_id, address, quantity).await
    }
    async fn read_04(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<u16>> {
        self.inner.read_04(slave_id, address, quantity).await
    }
    async fn write_05(&mut self, slave_id: SlaveId, address: u16, value: bool) -> ModbusResult<()> {
        self.inner.write_05(slave_id, address, value).await
    }
    async fn write_06(&mut self, slave_id: SlaveId, address: u16, value: u16) -> ModbusResult<()> {
        self.inner.write_06(slave_id, address, value).await
    }
    async fn write_0f(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        values: &[bool],
    ) -> ModbusResult<()> {
        self.inner.write_0f(slave_id, address, values).await
    }
    async fn write_10(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        values: &[u16],
    ) -> ModbusResult<()> {
        self.inner.write_10(slave_id, address, values).await
    }
    fn is_connected(&self) -> bool {
        self.inner.is_connected()
    }
    async fn close(&mut self) -> ModbusResult<()> {
        self.inner.close().await
    }
    fn get_stats(&self) -> TransportStats {
        self.inner.get_stats()
    }
}

/// Modbus ASCII client implementation using the generic client.
///
/// Thin wrapper over [`GenericModbusClient`]`<`[`AsciiTransport`](crate::transport::AsciiTransport)`>` — all
/// protocol logic is shared with TCP and RTU; only the framing differs.
#[cfg(feature = "rtu")]
pub struct ModbusAsciiClient {
    inner: GenericModbusClient<crate::transport::AsciiTransport>,
}

#[cfg(feature = "rtu")]
impl ModbusAsciiClient {
    /// Create a new ASCII client with default settings (7E1, 1s timeouts).
    pub fn new(port: &str, baud_rate: u32) -> ModbusResult<Self> {
        let transport = crate::transport::AsciiTransport::new(port, baud_rate)?;
        Ok(Self {
            inner: GenericModbusClient::new(transport),
        })
    }

    /// Create from an existing [`AsciiTransport`](crate::transport::AsciiTransport).
    pub fn from_transport(transport: crate::transport::AsciiTransport) -> Self {
        Self {
            inner: GenericModbusClient::new(transport),
        }
    }

    /// Borrow the underlying transport.
    pub fn transport(&self) -> &crate::transport::AsciiTransport {
        self.inner.transport()
    }

    /// Set the retry policy for recoverable failures — see [`RetryPolicy`]
    pub fn set_retry_policy(&mut self, policy: RetryPolicy) {
        self.inner.set_retry_policy(policy);
    }

    /// Access the underlying generic client — exposes the full method set
    /// (serial diagnostics FC 0x07/0x08/0x0B/0x0C/0x11, coalesced reads, ...)
    pub fn generic_mut(&mut self) -> &mut GenericModbusClient<crate::transport::AsciiTransport> {
        &mut self.inner
    }

    /// Convert into a cloneable, task-shareable handle — see [`SharedModbusClient`]
    pub fn into_shared(self) -> SharedModbusClient<crate::transport::AsciiTransport> {
        SharedModbusClient::new(self.inner)
    }

    /// Execute a raw request.
    pub async fn execute_request(
        &mut self,
        request: ModbusRequest,
    ) -> ModbusResult<ModbusResponse> {
        self.inner.execute_request(request).await
    }

    /// Mask write register (FC 0x16) — see [`GenericModbusClient::write_16`]
    pub async fn write_16(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        and_mask: u16,
        or_mask: u16,
    ) -> ModbusResult<()> {
        self.inner
            .write_16(slave_id, address, and_mask, or_mask)
            .await
    }

    /// Read/write multiple registers (FC 0x17) — see [`GenericModbusClient::read_write_17`]
    pub async fn read_write_17(
        &mut self,
        slave_id: SlaveId,
        read_address: u16,
        read_quantity: u16,
        write_address: u16,
        values: &[u16],
    ) -> ModbusResult<Vec<u16>> {
        self.inner
            .read_write_17(slave_id, read_address, read_quantity, write_address, values)
            .await
    }

    /// Read device identification (FC 0x2B / MEI 0x0E) —
    /// see [`GenericModbusClient::read_device_identification`]
    pub async fn read_device_identification(
        &mut self,
        slave_id: SlaveId,
        read_code: u8,
        object_id: u8,
    ) -> ModbusResult<crate::protocol::DeviceIdentification> {
        self.inner
            .read_device_identification(slave_id, read_code, object_id)
            .await
    }
}

#[cfg(feature = "rtu")]
impl ModbusClient for ModbusAsciiClient {
    async fn read_01(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<bool>> {
        self.inner.read_01(slave_id, address, quantity).await
    }
    async fn read_02(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<bool>> {
        self.inner.read_02(slave_id, address, quantity).await
    }
    async fn read_03(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<u16>> {
        self.inner.read_03(slave_id, address, quantity).await
    }
    async fn read_04(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<u16>> {
        self.inner.read_04(slave_id, address, quantity).await
    }
    async fn write_05(&mut self, slave_id: SlaveId, address: u16, value: bool) -> ModbusResult<()> {
        self.inner.write_05(slave_id, address, value).await
    }
    async fn write_06(&mut self, slave_id: SlaveId, address: u16, value: u16) -> ModbusResult<()> {
        self.inner.write_06(slave_id, address, value).await
    }
    async fn write_0f(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        values: &[bool],
    ) -> ModbusResult<()> {
        self.inner.write_0f(slave_id, address, values).await
    }
    async fn write_10(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        values: &[u16],
    ) -> ModbusResult<()> {
        self.inner.write_10(slave_id, address, values).await
    }
    fn is_connected(&self) -> bool {
        self.inner.is_connected()
    }
    async fn close(&mut self) -> ModbusResult<()> {
        self.inner.close().await
    }
    fn get_stats(&self) -> TransportStats {
        self.inner.get_stats()
    }
}

#[cfg(feature = "rtu")]
impl ModbusClient for ModbusRtuClient {
    async fn read_01(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<bool>> {
        self.inner.read_01(slave_id, address, quantity).await
    }

    async fn read_02(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<bool>> {
        self.inner.read_02(slave_id, address, quantity).await
    }

    async fn read_03(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<u16>> {
        self.inner.read_03(slave_id, address, quantity).await
    }

    async fn read_04(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<u16>> {
        self.inner.read_04(slave_id, address, quantity).await
    }

    async fn write_05(&mut self, slave_id: SlaveId, address: u16, value: bool) -> ModbusResult<()> {
        self.inner.write_05(slave_id, address, value).await
    }

    async fn write_06(&mut self, slave_id: SlaveId, address: u16, value: u16) -> ModbusResult<()> {
        self.inner.write_06(slave_id, address, value).await
    }

    async fn write_0f(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        values: &[bool],
    ) -> ModbusResult<()> {
        self.inner.write_0f(slave_id, address, values).await
    }

    async fn write_10(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        values: &[u16],
    ) -> ModbusResult<()> {
        self.inner.write_10(slave_id, address, values).await
    }

    fn is_connected(&self) -> bool {
        self.inner.is_connected()
    }

    async fn close(&mut self) -> ModbusResult<()> {
        self.inner.close().await
    }

    fn get_stats(&self) -> TransportStats {
        self.inner.get_stats()
    }
}

/// Modbus/TCP Security client — Modbus TCP over TLS (IANA port 802).
///
/// Thin wrapper over [`GenericModbusClient`]`<`[`TlsTransport`](crate::transport::TlsTransport)`>`;
/// all protocol logic is shared with the other transports. Build a
/// `rustls::ClientConfig` with your CA roots (and client certificate — the
/// Modbus Security spec mandates mutual TLS in production) and pass it in.
#[cfg(feature = "tls")]
pub struct ModbusTlsClient {
    inner: GenericModbusClient<crate::transport::TlsTransport>,
}

#[cfg(feature = "tls")]
impl ModbusTlsClient {
    /// Connect and complete the TLS handshake.
    pub async fn new(
        addr: SocketAddr,
        server_name: &str,
        config: Arc<tokio_rustls::rustls::ClientConfig>,
        timeout: Duration,
    ) -> ModbusResult<Self> {
        let transport =
            crate::transport::TlsTransport::new(addr, server_name, config, timeout).await?;
        Ok(Self {
            inner: GenericModbusClient::new(transport),
        })
    }

    /// Parse an address string and connect (e.g. `"192.168.1.10:802"`).
    pub async fn from_address(
        address: &str,
        server_name: &str,
        config: Arc<tokio_rustls::rustls::ClientConfig>,
        timeout: Duration,
    ) -> ModbusResult<Self> {
        let transport =
            crate::transport::TlsTransport::from_address(address, server_name, config, timeout)
                .await?;
        Ok(Self {
            inner: GenericModbusClient::new(transport),
        })
    }

    /// Set the retry policy for recoverable failures — see [`RetryPolicy`]
    pub fn set_retry_policy(&mut self, policy: RetryPolicy) {
        self.inner.set_retry_policy(policy);
    }

    /// Access the underlying generic client — exposes the full method set
    /// (extended FCs, serial diagnostics, coalesced reads, ...)
    pub fn generic_mut(&mut self) -> &mut GenericModbusClient<crate::transport::TlsTransport> {
        &mut self.inner
    }

    /// Convert into a cloneable, task-shareable handle — see [`SharedModbusClient`]
    pub fn into_shared(self) -> SharedModbusClient<crate::transport::TlsTransport> {
        SharedModbusClient::new(self.inner)
    }

    /// Execute a raw request.
    pub async fn execute_request(
        &mut self,
        request: ModbusRequest,
    ) -> ModbusResult<ModbusResponse> {
        self.inner.execute_request(request).await
    }
}

#[cfg(feature = "tls")]
impl ModbusClient for ModbusTlsClient {
    async fn read_01(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<bool>> {
        self.inner.read_01(slave_id, address, quantity).await
    }
    async fn read_02(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<bool>> {
        self.inner.read_02(slave_id, address, quantity).await
    }
    async fn read_03(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<u16>> {
        self.inner.read_03(slave_id, address, quantity).await
    }
    async fn read_04(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<u16>> {
        self.inner.read_04(slave_id, address, quantity).await
    }
    async fn write_05(&mut self, slave_id: SlaveId, address: u16, value: bool) -> ModbusResult<()> {
        self.inner.write_05(slave_id, address, value).await
    }
    async fn write_06(&mut self, slave_id: SlaveId, address: u16, value: u16) -> ModbusResult<()> {
        self.inner.write_06(slave_id, address, value).await
    }
    async fn write_0f(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        values: &[bool],
    ) -> ModbusResult<()> {
        self.inner.write_0f(slave_id, address, values).await
    }
    async fn write_10(
        &mut self,
        slave_id: SlaveId,
        address: u16,
        values: &[u16],
    ) -> ModbusResult<()> {
        self.inner.write_10(slave_id, address, values).await
    }
    fn is_connected(&self) -> bool {
        self.inner.is_connected()
    }
    async fn close(&mut self) -> ModbusResult<()> {
        self.inner.close().await
    }
    fn get_stats(&self) -> TransportStats {
        self.inner.get_stats()
    }
}

/// Cloneable, task-shareable Modbus client handle.
///
/// Wraps a [`GenericModbusClient`] in an async mutex so multiple tokio tasks
/// can issue requests over one connection through `&self` methods — no
/// external locking or `&mut` juggling required. Requests are serialized,
/// which is what Modbus mandates anyway: one outstanding transaction per
/// serial bus, and per-connection ordering on TCP.
///
/// # Example
///
/// ```rust,no_run
/// use voltage_modbus::{ModbusTcpClient, ModbusResult};
/// use std::time::Duration;
///
/// # async fn example() -> ModbusResult<()> {
/// let client = ModbusTcpClient::from_address("127.0.0.1:502", Duration::from_secs(5)).await?;
/// let shared = client.into_shared();
///
/// let handle = shared.clone();
/// let task = tokio::spawn(async move { handle.read_03(1, 0, 10).await });
///
/// let regs = shared.read_03(1, 100, 5).await?;
/// let other = task.await.unwrap()?;
/// # let _ = (regs, other);
/// # Ok(())
/// # }
/// ```
pub struct SharedModbusClient<T: ModbusTransport> {
    inner: Arc<tokio::sync::Mutex<GenericModbusClient<T>>>,
}

impl<T: ModbusTransport> Clone for SharedModbusClient<T> {
    fn clone(&self) -> Self {
        Self {
            inner: self.inner.clone(),
        }
    }
}

impl<T: ModbusTransport + Send + Sync> SharedModbusClient<T> {
    /// Wrap a generic client for shared use
    pub fn new(client: GenericModbusClient<T>) -> Self {
        Self {
            inner: Arc::new(tokio::sync::Mutex::new(client)),
        }
    }

    /// Read coils (FC 0x01)
    pub async fn read_01(
        &self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<bool>> {
        self.inner
            .lock()
            .await
            .read_01(slave_id, address, quantity)
            .await
    }

    /// Read discrete inputs (FC 0x02)
    pub async fn read_02(
        &self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<bool>> {
        self.inner
            .lock()
            .await
            .read_02(slave_id, address, quantity)
            .await
    }

    /// Read holding registers (FC 0x03)
    pub async fn read_03(
        &self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<u16>> {
        self.inner
            .lock()
            .await
            .read_03(slave_id, address, quantity)
            .await
    }

    /// Read input registers (FC 0x04)
    pub async fn read_04(
        &self,
        slave_id: SlaveId,
        address: u16,
        quantity: u16,
    ) -> ModbusResult<Vec<u16>> {
        self.inner
            .lock()
            .await
            .read_04(slave_id, address, quantity)
            .await
    }

    /// Write single coil (FC 0x05)
    pub async fn write_05(&self, slave_id: SlaveId, address: u16, value: bool) -> ModbusResult<()> {
        self.inner
            .lock()
            .await
            .write_05(slave_id, address, value)
            .await
    }

    /// Write single register (FC 0x06)
    pub async fn write_06(&self, slave_id: SlaveId, address: u16, value: u16) -> ModbusResult<()> {
        self.inner
            .lock()
            .await
            .write_06(slave_id, address, value)
            .await
    }

    /// Write multiple coils (FC 0x0F)
    pub async fn write_0f(
        &self,
        slave_id: SlaveId,
        address: u16,
        values: &[bool],
    ) -> ModbusResult<()> {
        self.inner
            .lock()
            .await
            .write_0f(slave_id, address, values)
            .await
    }

    /// Write multiple registers (FC 0x10)
    pub async fn write_10(
        &self,
        slave_id: SlaveId,
        address: u16,
        values: &[u16],
    ) -> ModbusResult<()> {
        self.inner
            .lock()
            .await
            .write_10(slave_id, address, values)
            .await
    }

    /// Mask write register (FC 0x16)
    pub async fn write_16(
        &self,
        slave_id: SlaveId,
        address: u16,
        and_mask: u16,
        or_mask: u16,
    ) -> ModbusResult<()> {
        self.inner
            .lock()
            .await
            .write_16(slave_id, address, and_mask, or_mask)
            .await
    }

    /// Read/write multiple registers (FC 0x17)
    pub async fn read_write_17(
        &self,
        slave_id: SlaveId,
        read_address: u16,
        read_quantity: u16,
        write_address: u16,
        values: &[u16],
    ) -> ModbusResult<Vec<u16>> {
        self.inner
            .lock()
            .await
            .read_write_17(slave_id, read_address, read_quantity, write_address, values)
            .await
    }

    /// Read device identification (FC 0x2B / MEI 0x0E)
    pub async fn read_device_identification(
        &self,
        slave_id: SlaveId,
        read_code: u8,
        object_id: u8,
    ) -> ModbusResult<crate::protocol::DeviceIdentification> {
        self.inner
            .lock()
            .await
            .read_device_identification(slave_id, read_code, object_id)
            .await
    }

    /// Execute a raw request
    pub async fn execute_request(&self, request: ModbusRequest) -> ModbusResult<ModbusResponse> {
        self.inner.lock().await.execute_request(request).await
    }

    /// Set the retry policy — see [`RetryPolicy`]
    pub async fn set_retry_policy(&self, policy: RetryPolicy) {
        self.inner.lock().await.set_retry_policy(policy);
    }

    /// Get transport statistics
    pub async fn get_stats(&self) -> TransportStats {
        self.inner.lock().await.get_stats()
    }

    /// Close the underlying connection
    pub async fn close(&self) -> ModbusResult<()> {
        self.inner.lock().await.close().await
    }
}

/// High-level utility functions for common operations
pub mod utils {
    use super::*;

    /// Read multiple register types in a single operation
    pub async fn read_mixed_registers<T: ModbusClient>(
        client: &mut T,
        slave_id: SlaveId,
        operations: &[(ModbusFunction, u16, u16)], // (function, address, quantity)
    ) -> ModbusResult<Vec<Vec<u16>>> {
        let mut results = Vec::new();

        for &(function, address, quantity) in operations {
            let values = match function {
                ModbusFunction::ReadHoldingRegisters => {
                    client.read_03(slave_id, address, quantity).await?
                }
                ModbusFunction::ReadInputRegisters => {
                    client.read_04(slave_id, address, quantity).await?
                }
                _ => return Err(ModbusError::invalid_function(function.to_u8())),
            };
            results.push(values);
        }

        Ok(results)
    }

    /// Batch write multiple registers
    pub async fn batch_write_registers<T: ModbusClient>(
        client: &mut T,
        slave_id: SlaveId,
        writes: &[(u16, Vec<u16>)], // (address, values)
    ) -> ModbusResult<()> {
        for (address, values) in writes {
            if values.len() == 1 {
                client.write_06(slave_id, *address, values[0]).await?;
            } else {
                client.write_10(slave_id, *address, values).await?;
            }
        }
        Ok(())
    }

    /// Convert register values to different data types
    pub fn registers_to_u32_be(registers: &[u16]) -> Vec<u32> {
        registers
            .chunks(2)
            .filter_map(|chunk| {
                if chunk.len() == 2 {
                    Some(((chunk[0] as u32) << 16) | (chunk[1] as u32))
                } else {
                    None
                }
            })
            .collect()
    }

    /// Convert register values to i32 (big-endian)
    pub fn registers_to_i32_be(registers: &[u16]) -> Vec<i32> {
        registers_to_u32_be(registers)
            .into_iter()
            .map(|v| v as i32)
            .collect()
    }

    /// Convert register values to f32 (IEEE 754, big-endian)
    pub fn registers_to_f32_be(registers: &[u16]) -> Vec<f32> {
        registers_to_u32_be(registers)
            .into_iter()
            .map(f32::from_bits)
            .collect()
    }

    /// Convert u32 values to register pairs (big-endian)
    pub fn u32_to_registers_be(values: &[u32]) -> Vec<u16> {
        values
            .iter()
            .flat_map(|&v| [(v >> 16) as u16, v as u16])
            .collect()
    }

    /// Convert f32 values to register pairs (IEEE 754, big-endian)
    pub fn f32_to_registers_be(values: &[f32]) -> Vec<u16> {
        let u32_values: Vec<u32> = values.iter().map(|&v| v.to_bits()).collect();
        u32_to_registers_be(&u32_values)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_register_conversion() {
        let registers = vec![0x1234, 0x5678, 0xABCD, 0xEF01];
        let u32_values = utils::registers_to_u32_be(&registers);
        assert_eq!(u32_values, vec![0x12345678, 0xABCDEF01]);

        let back_to_registers = utils::u32_to_registers_be(&u32_values);
        assert_eq!(back_to_registers, registers);
    }

    #[test]
    fn test_float_conversion() {
        let float_values = vec![1.5f32, -2.75f32];
        let registers = utils::f32_to_registers_be(&float_values);
        let back_to_floats = utils::registers_to_f32_be(&registers);

        for (original, converted) in float_values.iter().zip(back_to_floats.iter()) {
            assert!((original - converted).abs() < f32::EPSILON);
        }
    }

    #[tokio::test]
    async fn test_tcp_client_creation() {
        use std::time::Duration;

        // Test with valid but non-existent address
        let result = ModbusTcpClient::from_address("127.0.0.1:9999", Duration::from_secs(1)).await;
        // This might fail due to connection refused, which is expected
        println!("TCP client creation result: {:?}", result.is_ok());
    }

    // =========================================================================
    // MockTransport for batch read tests
    // =========================================================================

    use std::collections::VecDeque;
    use std::sync::Mutex;

    /// Mock transport for testing batch read methods
    struct MockTransport {
        /// Records all requests received
        requests: Mutex<Vec<ModbusRequest>>,
        /// Pre-configured responses (FIFO queue)
        responses: Mutex<VecDeque<ModbusResult<ModbusResponse>>>,
        /// Connection state
        connected: Mutex<bool>,
    }

    impl MockTransport {
        fn new() -> Self {
            Self {
                requests: Mutex::new(Vec::new()),
                responses: Mutex::new(VecDeque::new()),
                connected: Mutex::new(true),
            }
        }

        /// Add a response to the queue
        fn add_response(&self, response: ModbusResult<ModbusResponse>) {
            self.responses.lock().unwrap().push_back(response);
        }

        /// Get recorded requests for verification
        fn get_requests(&self) -> Vec<ModbusRequest> {
            self.requests.lock().unwrap().clone()
        }
    }

    impl ModbusTransport for MockTransport {
        fn request(
            &mut self,
            request: &ModbusRequest,
        ) -> impl std::future::Future<Output = ModbusResult<ModbusResponse>> + Send {
            // Record the request
            self.requests.lock().unwrap().push(request.clone());

            // Broadcast writes (slave_id = 0): mirror what real transports do —
            // return a synthetic ack without consuming a pre-configured response.
            let result = if request.slave_id == 0 {
                Ok(ModbusResponse::new_broadcast_ack(request.function))
            } else {
                // Get the next response from queue
                self.responses
                    .lock()
                    .unwrap()
                    .pop_front()
                    .unwrap_or_else(|| Err(ModbusError::connection("No response prepared in mock")))
            };

            async move { result }
        }

        fn is_connected(&self) -> bool {
            *self.connected.lock().unwrap()
        }

        fn close(&mut self) -> impl std::future::Future<Output = ModbusResult<()>> + Send {
            *self.connected.lock().unwrap() = false;
            async { Ok(()) }
        }

        fn get_stats(&self) -> TransportStats {
            TransportStats::default()
        }
    }

    // =========================================================================
    // Helper functions for creating mock responses
    // =========================================================================

    /// Create a FC03/FC04 (read registers) response with byte_count prefix
    fn create_register_response(slave_id: SlaveId, values: &[u16]) -> ModbusResponse {
        let byte_count = (values.len() * 2) as u8;
        let mut data = Vec::with_capacity(1 + values.len() * 2);
        data.push(byte_count);
        for &val in values {
            data.extend_from_slice(&val.to_be_bytes());
        }
        ModbusResponse::new_success(slave_id, ModbusFunction::ReadHoldingRegisters, data)
    }

    /// Create a FC01/FC02 (read coils/discrete inputs) response with byte_count prefix
    fn create_coil_response(slave_id: SlaveId, coils: &[bool]) -> ModbusResponse {
        let byte_count = coils.len().div_ceil(8) as u8;
        let mut data = Vec::with_capacity(1 + byte_count as usize);
        data.push(byte_count);

        // Pack bits into bytes (LSB first within each byte)
        let mut byte = 0u8;
        for (i, &coil) in coils.iter().enumerate() {
            if coil {
                byte |= 1 << (i % 8);
            }
            if (i + 1) % 8 == 0 || i == coils.len() - 1 {
                data.push(byte);
                byte = 0;
            }
        }
        ModbusResponse::new_success(slave_id, ModbusFunction::ReadCoils, data)
    }

    fn create_write_response(
        slave_id: SlaveId,
        function: ModbusFunction,
        address: u16,
        value_or_quantity: u16,
    ) -> ModbusResponse {
        let mut data = Vec::with_capacity(4);
        data.extend_from_slice(&address.to_be_bytes());
        data.extend_from_slice(&value_or_quantity.to_be_bytes());
        ModbusResponse::new_success(slave_id, function, data)
    }

    #[tokio::test]
    async fn test_read_rejects_wrong_function_response() {
        let mock = MockTransport::new();
        let mut data = Vec::new();
        data.push(2);
        data.extend_from_slice(&0x1234u16.to_be_bytes());
        mock.add_response(Ok(ModbusResponse::new_success(
            1,
            ModbusFunction::ReadInputRegisters,
            data,
        )));

        let mut client = GenericModbusClient::new(mock);
        let err = client.read_03(1, 0, 1).await.unwrap_err();
        assert!(err.to_string().contains("function mismatch"));
    }

    #[tokio::test]
    async fn test_read_rejects_wrong_byte_count() {
        let mock = MockTransport::new();
        mock.add_response(Ok(ModbusResponse::new_success(
            1,
            ModbusFunction::ReadHoldingRegisters,
            vec![4, 0x12, 0x34],
        )));

        let mut client = GenericModbusClient::new(mock);
        let err = client.read_03(1, 0, 1).await.unwrap_err();
        assert!(err.to_string().contains("read response"));
    }

    #[tokio::test]
    async fn test_write_single_register_rejects_wrong_echo_value() {
        let mock = MockTransport::new();
        mock.add_response(Ok(create_write_response(
            1,
            ModbusFunction::WriteSingleRegister,
            100,
            0x2222,
        )));

        let mut client = GenericModbusClient::new(mock);
        let err = client.write_06(1, 100, 0x1111).await.unwrap_err();
        assert!(err.to_string().contains("Write echo mismatch"));
    }

    #[tokio::test]
    async fn test_write_multiple_registers_rejects_wrong_echo_quantity() {
        let mock = MockTransport::new();
        mock.add_response(Ok(create_write_response(
            1,
            ModbusFunction::WriteMultipleRegisters,
            10,
            1,
        )));

        let mut client = GenericModbusClient::new(mock);
        let err = client.write_10(1, 10, &[0x1111, 0x2222]).await.unwrap_err();
        assert!(err.to_string().contains("Write echo mismatch"));
    }

    // =========================================================================
    // Batch read tests
    // =========================================================================

    #[tokio::test]
    async fn test_read_03_batch_single_chunk() {
        // When quantity <= max_read_registers, only one request should be made
        let mock = MockTransport::new();

        // Prepare response for 10 registers
        let values: Vec<u16> = (1..=10).collect();
        mock.add_response(Ok(create_register_response(1, &values)));

        let mut client = GenericModbusClient::new(mock);
        let limits = DeviceLimits::new().with_max_read_registers(50);

        let result = client.read_03_batch(1, 0, 10, &limits).await.unwrap();

        assert_eq!(result, values);
        assert_eq!(client.transport().get_requests().len(), 1);

        let req = &client.transport().get_requests()[0];
        assert_eq!(req.address, 0);
        assert_eq!(req.quantity, 10);
    }

    #[tokio::test]
    async fn test_read_03_batch_multiple_chunks() {
        // When quantity > max_read_registers, multiple requests should be made
        let mock = MockTransport::new();

        // Prepare responses for 3 chunks: 50 + 50 + 20 = 120 registers
        let chunk1: Vec<u16> = (1..=50).collect();
        let chunk2: Vec<u16> = (51..=100).collect();
        let chunk3: Vec<u16> = (101..=120).collect();

        mock.add_response(Ok(create_register_response(1, &chunk1)));
        mock.add_response(Ok(create_register_response(1, &chunk2)));
        mock.add_response(Ok(create_register_response(1, &chunk3)));

        let mut client = GenericModbusClient::new(mock);
        let limits = DeviceLimits::new().with_max_read_registers(50);

        let result = client.read_03_batch(1, 0, 120, &limits).await.unwrap();

        // Verify result contains all values
        let expected: Vec<u16> = (1..=120).collect();
        assert_eq!(result, expected);

        // Verify 3 requests were made
        let requests = client.transport().get_requests();
        assert_eq!(requests.len(), 3);

        // Verify addresses and quantities
        assert_eq!(requests[0].address, 0);
        assert_eq!(requests[0].quantity, 50);
        assert_eq!(requests[1].address, 50);
        assert_eq!(requests[1].quantity, 50);
        assert_eq!(requests[2].address, 100);
        assert_eq!(requests[2].quantity, 20);
    }

    #[tokio::test]
    async fn test_read_03_batch_exact_boundary() {
        // When quantity == max_read_registers, only one request
        let mock = MockTransport::new();

        let values: Vec<u16> = (1..=50).collect();
        mock.add_response(Ok(create_register_response(1, &values)));

        let mut client = GenericModbusClient::new(mock);
        let limits = DeviceLimits::new().with_max_read_registers(50);

        let result = client.read_03_batch(1, 100, 50, &limits).await.unwrap();

        assert_eq!(result, values);
        assert_eq!(client.transport().get_requests().len(), 1);

        let req = &client.transport().get_requests()[0];
        assert_eq!(req.address, 100);
        assert_eq!(req.quantity, 50);
    }

    #[tokio::test]
    async fn test_read_03_batch_empty() {
        // When quantity == 0, return empty Vec immediately without any requests
        let mock = MockTransport::new();
        let mut client = GenericModbusClient::new(mock);
        let limits = DeviceLimits::new();

        let result = client.read_03_batch(1, 0, 0, &limits).await.unwrap();

        assert!(result.is_empty());
        assert_eq!(client.transport().get_requests().len(), 0);
    }

    #[tokio::test]
    async fn test_read_03_batch_error_propagation() {
        // When a request fails mid-batch, error should be propagated
        let mock = MockTransport::new();

        // First chunk succeeds
        let chunk1: Vec<u16> = (1..=50).collect();
        mock.add_response(Ok(create_register_response(1, &chunk1)));

        // Second chunk fails
        mock.add_response(Err(ModbusError::timeout("Simulated timeout", 1000)));

        let mut client = GenericModbusClient::new(mock);
        let limits = DeviceLimits::new().with_max_read_registers(50);

        let result = client.read_03_batch(1, 0, 100, &limits).await;

        assert!(result.is_err());
        // Only 2 requests should have been made (second one failed)
        assert_eq!(client.transport().get_requests().len(), 2);
    }

    #[tokio::test]
    async fn test_read_01_batch_coils() {
        // Test batch reading coils
        let mock = MockTransport::new();

        // Prepare responses for 2 chunks: 500 + 100 = 600 coils
        let chunk1: Vec<bool> = (0..500).map(|i| i % 2 == 0).collect();
        let chunk2: Vec<bool> = (0..100).map(|i| i % 3 == 0).collect();

        mock.add_response(Ok(create_coil_response(1, &chunk1)));
        mock.add_response(Ok(create_coil_response(1, &chunk2)));

        let mut client = GenericModbusClient::new(mock);
        let limits = DeviceLimits::new().with_max_read_coils(500);

        let result = client.read_01_batch(1, 0, 600, &limits).await.unwrap();

        // Verify total count
        assert_eq!(result.len(), 600);

        // Verify requests
        let requests = client.transport().get_requests();
        assert_eq!(requests.len(), 2);
        assert_eq!(requests[0].quantity, 500);
        assert_eq!(requests[1].quantity, 100);
    }

    // =========================================================================
    // Extended function codes (FC 0x16 / 0x17 / 0x2B)
    // =========================================================================

    #[tokio::test]
    async fn test_write_16_mask_write_echo() {
        let mock = MockTransport::new();
        // Correct echo: addr + and_mask + or_mask
        mock.add_response(Ok(ModbusResponse::new_success(
            1,
            ModbusFunction::MaskWriteRegister,
            vec![0x00, 0x04, 0x00, 0xF2, 0x00, 0x25],
        )));
        let mut client = GenericModbusClient::new(mock);
        client.write_16(1, 4, 0x00F2, 0x0025).await.unwrap();
    }

    #[tokio::test]
    async fn test_write_16_rejects_wrong_echo() {
        let mock = MockTransport::new();
        mock.add_response(Ok(ModbusResponse::new_success(
            1,
            ModbusFunction::MaskWriteRegister,
            vec![0x00, 0x04, 0x00, 0x00, 0x00, 0x25], // and_mask differs
        )));
        let mut client = GenericModbusClient::new(mock);
        assert!(client.write_16(1, 4, 0x00F2, 0x0025).await.is_err());
    }

    #[tokio::test]
    async fn test_read_write_17_returns_read_registers() {
        let mock = MockTransport::new();
        mock.add_response(Ok(ModbusResponse::new_success(
            1,
            ModbusFunction::ReadWriteMultipleRegisters,
            vec![4, 0x12, 0x34, 0x56, 0x78],
        )));
        let mut client = GenericModbusClient::new(mock);
        let regs = client.read_write_17(1, 0, 2, 100, &[0xAAAA]).await.unwrap();
        assert_eq!(regs, vec![0x1234, 0x5678]);
    }

    #[tokio::test]
    async fn test_read_write_17_rejects_wrong_byte_count() {
        let mock = MockTransport::new();
        mock.add_response(Ok(ModbusResponse::new_success(
            1,
            ModbusFunction::ReadWriteMultipleRegisters,
            vec![2, 0x12, 0x34], // 1 register, but 2 were requested
        )));
        let mut client = GenericModbusClient::new(mock);
        assert!(client.read_write_17(1, 0, 2, 100, &[0xAAAA]).await.is_err());
    }

    #[tokio::test]
    async fn test_read_device_identification_parses_objects() {
        let mock = MockTransport::new();
        let mut data = vec![0x0E, 0x01, 0x01, 0x00, 0x00, 0x01];
        data.extend_from_slice(&[0x00, 0x04]);
        data.extend_from_slice(b"ACME");
        mock.add_response(Ok(ModbusResponse::new_success(
            1,
            ModbusFunction::ReadDeviceIdentification,
            data,
        )));
        let mut client = GenericModbusClient::new(mock);
        let ident = client.read_device_identification(1, 1, 0).await.unwrap();
        assert_eq!(ident.objects.len(), 1);
        assert_eq!(ident.object(0x00), Some(&b"ACME"[..]));
        assert!(!ident.more_follows);
    }

    // =========================================================================
    // Serial-line diagnostic functions (FC 0x07 / 0x08 / 0x0B / 0x0C / 0x11)
    // =========================================================================

    #[tokio::test]
    async fn test_read_exception_status() {
        let mock = MockTransport::new();
        mock.add_response(Ok(ModbusResponse::new_success(
            1,
            ModbusFunction::ReadExceptionStatus,
            vec![0b0110_0001],
        )));
        let mut client = GenericModbusClient::new(mock);
        assert_eq!(client.read_exception_status(1).await.unwrap(), 0b0110_0001);
    }

    #[tokio::test]
    async fn test_diagnostics_echo() {
        let mock = MockTransport::new();
        mock.add_response(Ok(ModbusResponse::new_success(
            1,
            ModbusFunction::Diagnostics,
            vec![0x00, 0x00, 0xA5, 0x37],
        )));
        let mut client = GenericModbusClient::new(mock);
        assert_eq!(client.diagnostics(1, 0x0000, 0xA537).await.unwrap(), 0xA537);
    }

    #[tokio::test]
    async fn test_diagnostics_rejects_wrong_subfunction_echo() {
        let mock = MockTransport::new();
        mock.add_response(Ok(ModbusResponse::new_success(
            1,
            ModbusFunction::Diagnostics,
            vec![0x00, 0x01, 0xA5, 0x37], // echoes sub-function 1, we sent 0
        )));
        let mut client = GenericModbusClient::new(mock);
        assert!(client.diagnostics(1, 0x0000, 0xA537).await.is_err());
    }

    #[tokio::test]
    async fn test_get_comm_event_counter() {
        let mock = MockTransport::new();
        mock.add_response(Ok(ModbusResponse::new_success(
            1,
            ModbusFunction::GetCommEventCounter,
            vec![0x00, 0x00, 0x01, 0x08],
        )));
        let mut client = GenericModbusClient::new(mock);
        assert_eq!(
            client.get_comm_event_counter(1).await.unwrap(),
            (0x0000, 0x0108)
        );
    }

    #[tokio::test]
    async fn test_get_comm_event_log() {
        let mock = MockTransport::new();
        // byte_count=8: status(2) + event(2) + message(2) + 2 event bytes
        mock.add_response(Ok(ModbusResponse::new_success(
            1,
            ModbusFunction::GetCommEventLog,
            vec![8, 0x00, 0x00, 0x01, 0x08, 0x01, 0x21, 0x20, 0x00],
        )));
        let mut client = GenericModbusClient::new(mock);
        let log = client.get_comm_event_log(1).await.unwrap();
        assert_eq!(log.status, 0x0000);
        assert_eq!(log.event_count, 0x0108);
        assert_eq!(log.message_count, 0x0121);
        assert_eq!(log.events, vec![0x20, 0x00]);
    }

    #[tokio::test]
    async fn test_report_server_id() {
        let mock = MockTransport::new();
        // byte_count=4: id "PM1" + run indicator ON
        mock.add_response(Ok(ModbusResponse::new_success(
            1,
            ModbusFunction::ReportServerId,
            vec![4, b'P', b'M', b'1', 0xFF],
        )));
        let mut client = GenericModbusClient::new(mock);
        let report = client.report_server_id(1).await.unwrap();
        assert_eq!(report.server_id, b"PM1");
        assert_eq!(report.run_indicator_on, Some(true));
    }

    // =========================================================================
    // SharedModbusClient tests
    // =========================================================================

    #[tokio::test]
    async fn test_shared_client_concurrent_reads() {
        let mock = MockTransport::new();
        for _ in 0..4 {
            mock.add_response(Ok(create_register_response(1, &[0x1234])));
        }
        let shared = SharedModbusClient::new(GenericModbusClient::new(mock));

        let tasks: Vec<_> = (0..4)
            .map(|_| {
                let handle = shared.clone();
                tokio::spawn(async move { handle.read_03(1, 0, 1).await })
            })
            .collect();

        for task in tasks {
            assert_eq!(task.await.unwrap().unwrap(), vec![0x1234]);
        }
        assert_eq!(shared.get_stats().await.requests_sent, 0); // mock stats stay default
    }

    // =========================================================================
    // RetryPolicy tests
    // =========================================================================

    #[tokio::test]
    async fn test_retry_recovers_after_transient_timeout() {
        let mock = MockTransport::new();
        mock.add_response(Err(ModbusError::timeout("simulated", 100)));
        mock.add_response(Ok(create_register_response(1, &[0x1234])));

        let mut client = GenericModbusClient::new(mock)
            .with_retry_policy(RetryPolicy::with_backoff(2, Duration::ZERO, Duration::ZERO));

        let regs = client.read_03(1, 0, 1).await.unwrap();
        assert_eq!(regs, vec![0x1234]);
        assert_eq!(client.transport().get_requests().len(), 2);
    }

    #[tokio::test]
    async fn test_no_retry_by_default() {
        let mock = MockTransport::new();
        mock.add_response(Err(ModbusError::timeout("simulated", 100)));
        mock.add_response(Ok(create_register_response(1, &[0x1234])));

        let mut client = GenericModbusClient::new(mock);
        assert!(client.read_03(1, 0, 1).await.is_err());
        assert_eq!(client.transport().get_requests().len(), 1);
    }

    #[tokio::test]
    async fn test_no_retry_for_permanent_errors() {
        let mock = MockTransport::new();
        // Illegal data address exception — not recoverable, must not be retried
        mock.add_response(Err(ModbusError::exception(0x03, 0x02)));
        mock.add_response(Ok(create_register_response(1, &[0x1234])));

        let mut client = GenericModbusClient::new(mock)
            .with_retry_policy(RetryPolicy::with_backoff(3, Duration::ZERO, Duration::ZERO));

        assert!(client.read_03(1, 0, 1).await.is_err());
        assert_eq!(client.transport().get_requests().len(), 1);
    }

    #[tokio::test]
    async fn test_retry_gives_up_after_max_retries() {
        let mock = MockTransport::new();
        for _ in 0..5 {
            mock.add_response(Err(ModbusError::timeout("simulated", 100)));
        }

        let mut client = GenericModbusClient::new(mock)
            .with_retry_policy(RetryPolicy::with_backoff(2, Duration::ZERO, Duration::ZERO));

        assert!(client.read_03(1, 0, 1).await.is_err());
        // 1 initial attempt + 2 retries
        assert_eq!(client.transport().get_requests().len(), 3);
    }

    // =========================================================================
    // Broadcast (slave_id = 0) tests
    // =========================================================================

    /// Broadcast write coil (FC05) must succeed without waiting for a response.
    #[tokio::test]
    async fn test_broadcast_write_coil() {
        let mock = MockTransport::new();
        let mut client = GenericModbusClient::new(mock);

        // slave_id = 0, write single coil ON at address 1
        let result = client.write_05(0, 1, true).await;
        assert!(
            result.is_ok(),
            "broadcast write_05 should succeed: {result:?}"
        );

        // The request must have been forwarded to the transport
        let reqs = client.transport().get_requests();
        assert_eq!(reqs.len(), 1);
        assert_eq!(reqs[0].slave_id, 0);
        assert_eq!(reqs[0].function, ModbusFunction::WriteSingleCoil);
    }

    /// Broadcast write register (FC06) must succeed.
    #[tokio::test]
    async fn test_broadcast_write_register() {
        let mock = MockTransport::new();
        let mut client = GenericModbusClient::new(mock);

        let result = client.write_06(0, 100, 0xABCD).await;
        assert!(
            result.is_ok(),
            "broadcast write_06 should succeed: {result:?}"
        );

        let reqs = client.transport().get_requests();
        assert_eq!(reqs.len(), 1);
        assert_eq!(reqs[0].slave_id, 0);
        assert_eq!(reqs[0].function, ModbusFunction::WriteSingleRegister);
    }

    /// Broadcast write multiple registers (FC16) must succeed.
    #[tokio::test]
    async fn test_broadcast_write_multiple() {
        let mock = MockTransport::new();
        let mut client = GenericModbusClient::new(mock);

        let result = client.write_10(0, 0, &[0x0001, 0x0002, 0x0003]).await;
        assert!(
            result.is_ok(),
            "broadcast write_10 should succeed: {result:?}"
        );

        let reqs = client.transport().get_requests();
        assert_eq!(reqs.len(), 1);
        assert_eq!(reqs[0].slave_id, 0);
        assert_eq!(reqs[0].function, ModbusFunction::WriteMultipleRegisters);
    }

    /// Broadcast read (any FC) must be rejected with an error.
    #[tokio::test]
    async fn test_broadcast_read_rejected() {
        let mock = MockTransport::new();
        let mut client = GenericModbusClient::new(mock);

        let err = client.read_03(0, 0, 1).await.unwrap_err();
        assert!(
            err.to_string().contains("Broadcast"),
            "expected broadcast error, got: {err}"
        );

        // No request should have been sent to the transport
        assert!(client.transport().get_requests().is_empty());
    }

    /// The synthetic broadcast ack has no data and no exception.
    #[tokio::test]
    async fn test_broadcast_response_is_ack() {
        let mock = MockTransport::new();
        let mut client = GenericModbusClient::new(mock);

        // Use execute_request directly to inspect the returned ModbusResponse
        let request =
            ModbusRequest::new_write(0, ModbusFunction::WriteSingleRegister, 10, vec![0x00, 0x01]);
        let response = client.execute_request(request).await.unwrap();

        assert_eq!(response.slave_id, 0);
        assert_eq!(response.function, ModbusFunction::WriteSingleRegister);
        assert!(!response.is_exception());
        assert!(response.data().is_empty());
    }

    // =========================================================================
    // Pipeline tests (using a real in-process TCP server)
    // =========================================================================

    use tokio::io::{AsyncReadExt as _, AsyncWriteExt as _};

    /// Build a minimal Modbus TCP response frame for a FC03 (read holding registers) reply.
    ///
    /// `tid` must match the TID in the request so the client accepts it.
    fn build_fc03_response_frame(tid: u16, slave_id: u8, values: &[u16]) -> Vec<u8> {
        let byte_count = (values.len() * 2) as u8;
        // PDU: unit_id(1) + func(1) + byte_count(1) + data(n*2)
        let pdu_len = (2 + 1 + values.len() * 2) as u16;
        let mut frame = Vec::new();
        frame.extend_from_slice(&tid.to_be_bytes()); // transaction id
        frame.extend_from_slice(&0u16.to_be_bytes()); // protocol id
        frame.extend_from_slice(&pdu_len.to_be_bytes()); // length
        frame.push(slave_id); // unit id
        frame.push(0x03); // function code
        frame.push(byte_count);
        for &v in values {
            frame.extend_from_slice(&v.to_be_bytes());
        }
        frame
    }

    /// Build a minimal Modbus TCP response frame for a FC06 (write single register) reply.
    fn build_fc06_response_frame(tid: u16, slave_id: u8, address: u16, value: u16) -> Vec<u8> {
        let pdu_len: u16 = 6; // unit_id(1) + func(1) + addr(2) + value(2)
        let mut frame = Vec::new();
        frame.extend_from_slice(&tid.to_be_bytes());
        frame.extend_from_slice(&0u16.to_be_bytes());
        frame.extend_from_slice(&pdu_len.to_be_bytes());
        frame.push(slave_id);
        frame.push(0x06);
        frame.extend_from_slice(&address.to_be_bytes());
        frame.extend_from_slice(&value.to_be_bytes());
        frame
    }

    /// Spawn a minimal single-use TCP server that reads exactly `request_count` Modbus TCP
    /// frames, then calls `handler` with the list of (tid, function_code) pairs, and returns
    /// whatever bytes `handler` produces.
    async fn spawn_mock_server<H, Fut>(
        request_count: usize,
        handler: H,
    ) -> (std::net::SocketAddr, tokio::task::JoinHandle<()>)
    where
        H: FnOnce(Vec<(u16, u8, u8)>) -> Fut + Send + 'static,
        Fut: std::future::Future<Output = Vec<u8>> + Send,
    {
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();

        let handle = tokio::spawn(async move {
            let (mut socket, _) = listener.accept().await.unwrap();
            let mut requests_meta: Vec<(u16, u8, u8)> = Vec::new(); // (tid, slave_id, func)

            for _ in 0..request_count {
                // Read MBAP header (6 bytes)
                let mut mbap = [0u8; 6];
                socket.read_exact(&mut mbap).await.unwrap();
                let tid = u16::from_be_bytes([mbap[0], mbap[1]]);
                let length = u16::from_be_bytes([mbap[4], mbap[5]]) as usize;

                // Read PDU
                let mut pdu = vec![0u8; length];
                socket.read_exact(&mut pdu).await.unwrap();
                let slave_id = pdu[0];
                let func = pdu[1];
                requests_meta.push((tid, slave_id, func));
            }

            let response_bytes = handler(requests_meta).await;
            socket.write_all(&response_bytes).await.unwrap();
        });

        (addr, handle)
    }

    #[tokio::test]
    async fn test_pipeline_empty() {
        // Empty request list should return empty result immediately (no network needed)
        let (server_addr, _handle) = spawn_mock_server(0, |_| async { vec![] }).await;

        let mut client = ModbusTcpClient::new(server_addr, Duration::from_secs(5))
            .await
            .unwrap();

        let results = client
            .pipeline(vec![], Duration::from_secs(5))
            .await
            .unwrap();

        assert!(results.is_empty());
    }

    #[tokio::test]
    async fn test_pipeline_single() {
        // Single pipeline request should behave identically to a regular read_03 call.
        let (server_addr, server_handle) = spawn_mock_server(1, |meta| async move {
            let (tid, slave_id, _func) = meta[0];
            let values: Vec<u16> = vec![1, 2, 3, 4, 5];
            build_fc03_response_frame(tid, slave_id, &values)
        })
        .await;

        let mut client = ModbusTcpClient::new(server_addr, Duration::from_secs(5))
            .await
            .unwrap();

        let requests = vec![ModbusRequest::new_read(
            1,
            ModbusFunction::ReadHoldingRegisters,
            0,
            5,
        )];

        let results = client
            .pipeline(requests, Duration::from_secs(5))
            .await
            .unwrap();

        assert_eq!(results.len(), 1);
        let registers = results[0].as_ref().unwrap().parse_registers().unwrap();
        assert_eq!(registers, vec![1, 2, 3, 4, 5]);

        server_handle.await.unwrap();
    }

    #[tokio::test]
    async fn test_pipeline_basic() {
        // 3 read requests pipelined — server replies in same order but could be any order.
        // We reply in order here; test verifies result ordering is correct.
        let (server_addr, server_handle) = spawn_mock_server(3, |meta| async move {
            let mut out = Vec::new();
            let expected_values: Vec<Vec<u16>> = vec![
                vec![10, 11, 12],     // response for req 0
                vec![20, 21],         // response for req 1
                vec![30, 31, 32, 33], // response for req 2
            ];
            for (i, (tid, slave_id, _func)) in meta.iter().enumerate() {
                out.extend_from_slice(&build_fc03_response_frame(
                    *tid,
                    *slave_id,
                    &expected_values[i],
                ));
            }
            out
        })
        .await;

        let mut client = ModbusTcpClient::new(server_addr, Duration::from_secs(5))
            .await
            .unwrap();

        let requests = vec![
            ModbusRequest::new_read(1, ModbusFunction::ReadHoldingRegisters, 0, 3),
            ModbusRequest::new_read(1, ModbusFunction::ReadHoldingRegisters, 100, 2),
            ModbusRequest::new_read(1, ModbusFunction::ReadHoldingRegisters, 200, 4),
        ];

        let results = client
            .pipeline(requests, Duration::from_secs(5))
            .await
            .unwrap();

        assert_eq!(results.len(), 3);
        assert_eq!(
            results[0].as_ref().unwrap().parse_registers().unwrap(),
            vec![10, 11, 12]
        );
        assert_eq!(
            results[1].as_ref().unwrap().parse_registers().unwrap(),
            vec![20, 21]
        );
        assert_eq!(
            results[2].as_ref().unwrap().parse_registers().unwrap(),
            vec![30, 31, 32, 33]
        );

        server_handle.await.unwrap();
    }

    #[tokio::test]
    async fn test_pipeline_mixed() {
        // Mix of read (FC03) and write (FC06) requests
        let (server_addr, server_handle) = spawn_mock_server(2, |meta| async move {
            let mut out = Vec::new();
            // First request: FC03 read
            let (tid0, slave0, _) = meta[0];
            out.extend_from_slice(&build_fc03_response_frame(tid0, slave0, &[42, 43]));
            // Second request: FC06 write — echo back address + value
            let (tid1, slave1, _) = meta[1];
            out.extend_from_slice(&build_fc06_response_frame(tid1, slave1, 200, 0x1234));
            out
        })
        .await;

        let mut client = ModbusTcpClient::new(server_addr, Duration::from_secs(5))
            .await
            .unwrap();

        let requests = vec![
            ModbusRequest::new_read(1, ModbusFunction::ReadHoldingRegisters, 0, 2),
            ModbusRequest::new_write(
                1,
                ModbusFunction::WriteSingleRegister,
                200,
                vec![0x12, 0x34],
            ),
        ];

        let results = client
            .pipeline(requests, Duration::from_secs(5))
            .await
            .unwrap();

        assert_eq!(results.len(), 2);
        // FC03 response
        assert_eq!(
            results[0].as_ref().unwrap().parse_registers().unwrap(),
            vec![42, 43]
        );
        // FC06 response succeeds
        assert!(results[1].is_ok());

        server_handle.await.unwrap();
    }

    #[tokio::test]
    async fn test_pipeline_reads_convenience() {
        // Test the pipeline_reads convenience method
        let (server_addr, server_handle) = spawn_mock_server(2, |meta| async move {
            let mut out = Vec::new();
            let data = [vec![1u16, 2, 3], vec![4u16, 5]];
            for (i, (tid, slave_id, _)) in meta.iter().enumerate() {
                out.extend_from_slice(&build_fc03_response_frame(*tid, *slave_id, &data[i]));
            }
            out
        })
        .await;

        let mut client = ModbusTcpClient::new(server_addr, Duration::from_secs(5))
            .await
            .unwrap();

        let results = client
            .pipeline_reads(1, &[(0, 3), (100, 2)], Duration::from_secs(5))
            .await
            .unwrap();

        assert_eq!(results.len(), 2);
        assert_eq!(results[0].as_ref().unwrap(), &[1, 2, 3]);
        assert_eq!(results[1].as_ref().unwrap(), &[4, 5]);

        server_handle.await.unwrap();
    }

    #[tokio::test]
    async fn test_pipeline_out_of_order_responses() {
        // Server sends responses in REVERSE order (TID2 first, then TID1).
        // Client must return results in ORIGINAL request order.
        let (server_addr, server_handle) = spawn_mock_server(2, |meta| async move {
            let mut out = Vec::new();
            // Send response for second request first (reverse order)
            let (tid1, slave1, _) = meta[1];
            out.extend_from_slice(&build_fc03_response_frame(tid1, slave1, &[200u16, 201]));
            // Then send response for first request
            let (tid0, slave0, _) = meta[0];
            out.extend_from_slice(&build_fc03_response_frame(tid0, slave0, &[100u16, 101]));
            out
        })
        .await;

        let mut client = ModbusTcpClient::new(server_addr, Duration::from_secs(5))
            .await
            .unwrap();

        let requests = vec![
            ModbusRequest::new_read(1, ModbusFunction::ReadHoldingRegisters, 0, 2),
            ModbusRequest::new_read(1, ModbusFunction::ReadHoldingRegisters, 10, 2),
        ];

        let results = client
            .pipeline(requests, Duration::from_secs(5))
            .await
            .unwrap();

        assert_eq!(results.len(), 2);
        // Results must be in original request order despite out-of-order server responses
        assert_eq!(
            results[0].as_ref().unwrap().parse_registers().unwrap(),
            vec![100u16, 101]
        );
        assert_eq!(
            results[1].as_ref().unwrap().parse_registers().unwrap(),
            vec![200u16, 201]
        );

        server_handle.await.unwrap();
    }
}

#[cfg(all(test, feature = "rtu"))]
mod rtu_tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn test_rtu_client_creation() {
        // Test RTU client creation (will fail if no serial port available)
        let result = ModbusRtuClient::new("/dev/ttyUSB0", 9600);
        println!("RTU client creation result: {:?}", result.is_ok());

        // Test with custom configuration
        let result = ModbusRtuClient::with_config_and_logging(
            "/dev/ttyUSB0",
            9600,
            tokio_serial::DataBits::Eight,
            tokio_serial::StopBits::One,
            tokio_serial::Parity::None,
            Duration::from_secs(1),
            None,
        );
        println!(
            "RTU client with config creation result: {:?}",
            result.is_ok()
        );
    }

    #[tokio::test]
    async fn test_rtu_client_operations() {
        // This test will only pass if a serial port is available
        // In a real environment, you would have a Modbus RTU device connected

        // Try to create RTU client - this might fail if no port is available
        let client_result = ModbusRtuClient::new("/dev/ttyUSB0", 9600);

        if let Ok(mut client) = client_result {
            // Test connection status
            println!("RTU client connected: {}", client.is_connected());

            // Test reading coils (this will likely timeout without a real device)
            let read_result =
                tokio::time::timeout(Duration::from_millis(100), client.read_01(1, 0, 8)).await;

            match read_result {
                Ok(Ok(coils)) => {
                    println!("Successfully read {} coils", coils.len());
                }
                Ok(Err(e)) => {
                    println!("Read operation failed (expected without device): {}", e);
                }
                Err(_) => {
                    println!("Read operation timed out (expected without device)");
                }
            }

            // Close the client
            let _ = client.close().await;
        } else {
            println!("RTU client creation failed (expected without serial port)");
        }
    }

    #[test]
    fn test_rtu_client_configuration() {
        // Test different configurations
        let configs = vec![
            ("/dev/ttyUSB0", 9600),
            ("/dev/ttyUSB1", 19200),
            ("/dev/ttyS0", 38400),
            ("COM1", 115200),
        ];

        for (port, baud) in configs {
            let result = ModbusRtuClient::new(port, baud);
            // We expect these to fail without actual hardware, but they should not panic
            println!(
                "RTU client creation for {} at {} baud: {}",
                port,
                baud,
                result.is_ok()
            );
        }
    }
}
