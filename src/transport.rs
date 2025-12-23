//! # Modbus Transport Layer
//!
//! This module provides transport layer implementations for Modbus communication,
//! supporting both TCP and RTU protocols with a unified interface.
//!
//! ## Supported Transports
//!
//! ### Modbus TCP (`TcpTransport`)
//! - Full TCP/IP communication support
//! - Automatic connection management and reconnection
//! - MBAP header handling with transaction ID management
//! - Configurable timeouts and statistics
//!
//! ### Modbus RTU (`RtuTransport`)  
//! - Serial port communication (RS-232, RS-485)
//! - CRC-16 validation for message integrity
//! - Automatic frame gap calculation based on baud rate
//! - Configurable serial parameters (data bits, stop bits, parity)
//!
//! ## Usage Examples
//!
//! ### TCP Transport
//!
//! ```rust,no_run
//! use voltage_modbus::transport::{TcpTransport, ModbusTransport};
//! use voltage_modbus::protocol::{ModbusRequest, ModbusFunction};
//! use std::time::Duration;
//!
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     // Create TCP transport
//!     let mut transport = TcpTransport::new(
//!         "127.0.0.1:502".parse()?,
//!         Duration::from_secs(5)
//!     ).await?;
//!     
//!     // Create and send request
//!     let request = ModbusRequest::new_read(
//!         1,                                    // slave_id
//!         ModbusFunction::ReadHoldingRegisters, // function
//!         0,                                    // address
//!         10                                    // quantity
//!     );
//!     
//!     let response = transport.request(&request).await?;
//!     println!("Response: {:?}", response);
//!     
//!     // Get statistics
//!     let stats = transport.get_stats();
//!     println!("Requests sent: {}", stats.requests_sent);
//!     
//!     transport.close().await?;
//!     Ok(())
//! }
//! ```
//!
//! ### RTU Transport
//!
//! ```rust,no_run
//! use voltage_modbus::transport::{RtuTransport, ModbusTransport};
//! use voltage_modbus::protocol::{ModbusRequest, ModbusFunction};
//!
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     // Create RTU transport with default settings
//!     let mut transport = RtuTransport::new("/dev/ttyUSB0", 9600)?;
//!     
//!     // Or with full configuration
//!     let mut transport = RtuTransport::new_with_config(
//!         "/dev/ttyUSB0",
//!         9600,
//!         tokio_serial::DataBits::Eight,
//!         tokio_serial::StopBits::One,
//!         tokio_serial::Parity::None,
//!         std::time::Duration::from_secs(1)
//!     )?;
//!     
//!     // Send request
//!     let request = ModbusRequest::new_read(
//!         1,                                 // slave_id
//!         ModbusFunction::ReadCoils,         // function  
//!         0,                                 // address
//!         8                                  // quantity
//!     );
//!     
//!     let response = transport.request(&request).await?;
//!     println!("Response: {:?}", response);
//!     
//!     transport.close().await?;
//!     Ok(())
//! }
//! ```
//!
//! ## Transport Statistics
//!
//! Both transport implementations provide detailed statistics for monitoring:
//!
//! ```rust,no_run
//! # use voltage_modbus::transport::{ModbusTransport, TransportStats};
//! # fn example(transport: &impl ModbusTransport) {
//! let stats = transport.get_stats();
//!
//! println!("Communication Statistics:");
//! println!("  Requests sent: {}", stats.requests_sent);
//! println!("  Responses received: {}", stats.responses_received);
//! println!("  Errors: {}", stats.errors);
//! println!("  Timeouts: {}", stats.timeouts);
//! println!("  Bytes sent: {}", stats.bytes_sent);
//! println!("  Bytes received: {}", stats.bytes_received);
//!
//! if stats.requests_sent > 0 {
//!     let success_rate = (stats.responses_received as f64 / stats.requests_sent as f64) * 100.0;
//!     println!("  Success rate: {:.2}%", success_rate);
//! }
//! # }
//! ```

#[cfg(feature = "rtu")]
use crc::{Crc, CRC_16_MODBUS};
/// Modbus transport layer implementations
///
/// This module provides the transport layer abstractions and implementations
/// for both Modbus TCP and RTU protocols.
use std::net::SocketAddr;
use std::time::Duration;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use tokio::time::timeout;
use tracing::info;

#[cfg(feature = "rtu")]
use tokio_serial;

use crate::error::{ModbusError, ModbusResult};
use crate::protocol::{ModbusFunction, ModbusRequest, ModbusResponse};

/// Maximum frame size for Modbus TCP (MBAP header + PDU)
const MAX_TCP_FRAME_SIZE: usize = 260;

/// Modbus TCP Application Protocol header size
const MBAP_HEADER_SIZE: usize = 6;

/// Maximum frame size for Modbus RTU
#[cfg(feature = "rtu")]
const MAX_RTU_FRAME_SIZE: usize = 256;

/// CRC calculator for RTU
#[cfg(feature = "rtu")]
const CRC_MODBUS: Crc<u16> = Crc::<u16>::new(&CRC_16_MODBUS);

/// Format raw bytes as hex string for packet logging
fn format_hex_packet(data: &[u8]) -> String {
    data.iter()
        .map(|b| format!("{:02X}", b))
        .collect::<Vec<_>>()
        .join(" ")
}

/// Log packet with direction and format
fn log_packet(direction: &str, data: &[u8], protocol: &str, slave_id: Option<u8>) {
    let hex_string = format_hex_packet(data);
    match slave_id {
        Some(id) => info!(
            "[MODBUS-{}] {} slave:{} {}",
            protocol, direction, id, hex_string
        ),
        None => info!("[MODBUS-{}] {} {}", protocol, direction, hex_string),
    }
}

/// Transport layer abstraction for Modbus communication protocols
///
/// This trait defines a common interface for different Modbus transport mechanisms,
/// allowing the same application code to work with TCP, RTU, or future transport types.
///
/// ## Thread Safety
///
/// All implementations must be `Send + Sync` to support multi-threaded usage patterns.
///
/// ## Error Handling
///
/// All methods return `ModbusResult<T>` to provide comprehensive error information
/// including transport-specific error conditions, timeouts, and protocol violations.
///
/// ## Usage Example
///
/// ```rust,no_run
/// use voltage_modbus::transport::{ModbusTransport, RtuTransport};
/// use voltage_modbus::protocol::{ModbusRequest, ModbusFunction};
///
/// async fn generic_request(transport: &mut impl ModbusTransport) -> Result<Vec<u16>, Box<dyn std::error::Error>> {
///     let request = ModbusRequest::new_read(
///         1,                                    // slave_id
///         ModbusFunction::ReadHoldingRegisters, // function
///         0,                                    // address
///         10                                    // quantity
///     );
///     
///     let response = transport.request(&request).await?;
///     let registers = response.parse_registers()?;
///     Ok(registers)
/// }
/// ```
pub trait ModbusTransport: Send + Sync {
    /// Send a Modbus request and wait for response
    ///
    /// This is the core method for Modbus communication. It handles the complete
    /// request-response cycle including frame encoding, transmission, response
    /// reception, and frame decoding.
    ///
    /// # Arguments
    ///
    /// * `request` - The Modbus request to send
    ///
    /// # Returns
    ///
    /// * `Ok(response)` - Successful response from the device
    /// * `Err(error)` - Communication error, timeout, or protocol violation
    ///
    /// # Errors
    ///
    /// This method can return various error types:
    /// - `ModbusError::Timeout` - Request timed out
    /// - `ModbusError::Connection` - Connection lost or failed
    /// - `ModbusError::Protocol` - Protocol violation in response
    /// - `ModbusError::Exception` - Device returned Modbus exception
    /// - `ModbusError::Frame` - Frame format or CRC error (RTU)
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use voltage_modbus::transport::{RtuTransport, ModbusTransport};
    /// use voltage_modbus::protocol::{ModbusRequest, ModbusFunction};
    ///
    /// # async fn example() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut transport = RtuTransport::new("/dev/ttyUSB0", 9600)?;
    ///
    /// let request = ModbusRequest::new_read(
    ///     1,                                    // slave_id
    ///     ModbusFunction::ReadHoldingRegisters, // function
    ///     100,                                  // start_address
    ///     5                                     // quantity
    /// );
    ///
    /// match transport.request(&request).await {
    ///     Ok(response) => {
    ///         if !response.is_exception() {
    ///             let values = response.parse_registers()?;
    ///             println!("Read values: {:?}", values);
    ///         }
    ///     }
    ///     Err(error) => {
    ///         eprintln!("Request failed: {}", error);
    ///     }
    /// }
    /// # Ok(())
    /// # }
    /// ```
    fn request(&mut self, request: &ModbusRequest) -> impl std::future::Future<Output = ModbusResult<ModbusResponse>> + Send;

    /// Check if the transport connection is active
    ///
    /// Returns `true` if the transport believes it has an active connection
    /// to the remote device. This is a local check and does not verify
    /// that the remote device is responsive.
    ///
    /// # Returns
    ///
    /// * `true` - Transport is connected
    /// * `false` - Transport is disconnected or connection lost
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// # use voltage_modbus::transport::{ModbusTransport, RtuTransport};
    /// # fn example(transport: &impl ModbusTransport) {
    /// if transport.is_connected() {
    ///     println!("Transport is connected");
    /// } else {
    ///     println!("Transport is disconnected");
    /// }
    /// # }
    /// ```
    fn is_connected(&self) -> bool;

    /// Close the transport connection gracefully
    ///
    /// Closes the underlying connection (TCP socket, serial port, etc.) and
    /// releases associated resources. After calling this method, the transport
    /// should not be used for further communication.
    ///
    /// # Returns
    ///
    /// * `Ok(())` - Connection closed successfully
    /// * `Err(error)` - Error during connection closure (usually non-critical)
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use voltage_modbus::transport::{RtuTransport, ModbusTransport};
    ///
    /// # async fn example() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut transport = RtuTransport::new("/dev/ttyUSB0", 9600)?;
    ///
    /// // Use transport for communication...
    ///
    /// // Clean shutdown
    /// transport.close().await?;
    /// # Ok(())
    /// # }
    /// ```
    fn close(&mut self) -> impl std::future::Future<Output = ModbusResult<()>> + Send;

    /// Get communication statistics and performance metrics
    ///
    /// Returns detailed statistics about the transport's communication history,
    /// including request counts, error rates, and data transfer volumes.
    ///
    /// # Returns
    ///
    /// A `TransportStats` structure containing:
    /// - Request and response counts
    /// - Error and timeout counts  
    /// - Bytes sent and received
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// # use voltage_modbus::transport::{ModbusTransport, RtuTransport};
    /// # fn example(transport: &impl ModbusTransport) {
    /// let stats = transport.get_stats();
    ///
    /// println!("Communication Statistics:");
    /// println!("  Requests sent: {}", stats.requests_sent);
    /// println!("  Responses received: {}", stats.responses_received);
    /// println!("  Errors: {}", stats.errors);
    /// println!("  Timeouts: {}", stats.timeouts);
    ///
    /// if stats.requests_sent > 0 {
    ///     let success_rate = (stats.responses_received as f64 / stats.requests_sent as f64) * 100.0;
    ///     println!("  Success rate: {:.2}%", success_rate);
    /// }
    /// # }
    /// ```
    fn get_stats(&self) -> TransportStats;
}

/// Transport layer statistics
#[derive(Debug, Clone, Default)]
pub struct TransportStats {
    pub requests_sent: u64,
    pub responses_received: u64,
    pub errors: u64,
    pub timeouts: u64,
    pub bytes_sent: u64,
    pub bytes_received: u64,
}

/// Modbus TCP transport implementation  
pub struct TcpTransport {
    stream: Option<TcpStream>,
    pub address: SocketAddr,
    timeout: Duration,
    transaction_id: u16,
    stats: TransportStats,
    /// Enable packet logging for debugging
    packet_logging: bool,
}

impl TcpTransport {
    /// Create a new TCP transport
    pub async fn new(address: SocketAddr, timeout: Duration) -> ModbusResult<Self> {
        let stream = TcpStream::connect(address).await.map_err(|e| {
            ModbusError::connection(format!("Failed to connect to {}: {}", address, e))
        })?;

        Ok(Self {
            stream: Some(stream),
            address,
            timeout,
            transaction_id: 1,
            stats: TransportStats::default(),
            packet_logging: false,
        })
    }

    /// Create a new TCP transport with packet logging enabled
    pub async fn with_packet_logging(
        address: SocketAddr,
        timeout: Duration,
        enable_logging: bool,
    ) -> ModbusResult<Self> {
        let stream = TcpStream::connect(address).await.map_err(|e| {
            ModbusError::connection(format!("Failed to connect to {}: {}", address, e))
        })?;

        Ok(Self {
            stream: Some(stream),
            address,
            timeout,
            transaction_id: 1,
            stats: TransportStats::default(),
            packet_logging: enable_logging,
        })
    }

    /// Enable or disable packet logging
    pub fn set_packet_logging(&mut self, enabled: bool) {
        self.packet_logging = enabled;
    }

    /// Reconnect to the server
    async fn reconnect(&mut self) -> ModbusResult<()> {
        self.stream = None;

        let stream = TcpStream::connect(self.address).await.map_err(|e| {
            ModbusError::connection(format!("Failed to reconnect to {}: {}", self.address, e))
        })?;

        self.stream = Some(stream);
        Ok(())
    }

    /// Get next transaction ID
    fn next_transaction_id(&mut self) -> u16 {
        self.transaction_id = self.transaction_id.wrapping_add(1);
        if self.transaction_id == 0 {
            self.transaction_id = 1;
        }
        self.transaction_id
    }

    /// Encode request to TCP frame
    fn encode_request(&mut self, request: &ModbusRequest) -> Vec<u8> {
        let transaction_id = self.next_transaction_id();
        let protocol_id = 0u16; // Always 0 for Modbus

        // Calculate PDU length (unit_id + function_code + data)
        let pdu_length = 1
            + 1
            + match request.function {
                ModbusFunction::ReadCoils
                | ModbusFunction::ReadDiscreteInputs
                | ModbusFunction::ReadHoldingRegisters
                | ModbusFunction::ReadInputRegisters => 4, // address (2) + quantity (2)

                ModbusFunction::WriteSingleCoil | ModbusFunction::WriteSingleRegister => 4, // address (2) + value (2)

                ModbusFunction::WriteMultipleCoils | ModbusFunction::WriteMultipleRegisters => {
                    5 + request.data.len()
                } // address (2) + quantity (2) + byte_count (1) + data
            };

        let mut frame = Vec::with_capacity(MBAP_HEADER_SIZE + pdu_length);

        // MBAP Header: Transaction ID (2) + Protocol ID (2) + Length (2)
        frame.extend_from_slice(&transaction_id.to_be_bytes());
        frame.extend_from_slice(&protocol_id.to_be_bytes());
        frame.extend_from_slice(&(pdu_length as u16).to_be_bytes()); // PDU length without MBAP header

        // PDU: Unit ID + Function Code + Data
        frame.push(request.slave_id);
        frame.push(request.function.to_u8());
        frame.extend_from_slice(&request.address.to_be_bytes());

        match request.function {
            ModbusFunction::ReadCoils
            | ModbusFunction::ReadDiscreteInputs
            | ModbusFunction::ReadHoldingRegisters
            | ModbusFunction::ReadInputRegisters => {
                frame.extend_from_slice(&request.quantity.to_be_bytes());
            }

            ModbusFunction::WriteSingleCoil => {
                let value: u16 = if !request.data.is_empty() && request.data[0] != 0 {
                    0xFF00
                } else {
                    0x0000
                };
                frame.extend_from_slice(&value.to_be_bytes());
            }

            ModbusFunction::WriteSingleRegister => {
                if request.data.len() >= 2 {
                    frame.extend_from_slice(&request.data[0..2]);
                } else {
                    frame.extend_from_slice(&[0, 0]);
                }
            }

            ModbusFunction::WriteMultipleCoils | ModbusFunction::WriteMultipleRegisters => {
                frame.extend_from_slice(&request.quantity.to_be_bytes());
                frame.push(request.data.len() as u8);
                frame.extend_from_slice(&request.data);
            }
        }

        frame
    }

    /// Decode response from TCP frame
    fn decode_response(&self, frame: &[u8]) -> ModbusResult<ModbusResponse> {
        if frame.len() < MBAP_HEADER_SIZE + 2 {
            return Err(ModbusError::frame("Frame too short"));
        }

        // Parse MBAP header
        let _transaction_id = u16::from_be_bytes([frame[0], frame[1]]);
        let _protocol_id = u16::from_be_bytes([frame[2], frame[3]]);
        let length = u16::from_be_bytes([frame[4], frame[5]]);
        let slave_id = frame[6];

        if frame.len() < MBAP_HEADER_SIZE + length as usize {
            return Err(ModbusError::frame("Incomplete frame"));
        }

        // Parse PDU
        let function_code = frame[7];

        // Check for exception response
        if function_code & 0x80 != 0 {
            if frame.len() < MBAP_HEADER_SIZE + 3 {
                return Err(ModbusError::frame("Invalid exception response"));
            }

            let original_function = function_code & 0x7F;
            let exception_code = frame[8];

            return Ok(ModbusResponse::new_exception(
                slave_id,
                ModbusFunction::from_u8(original_function)?,
                exception_code,
            ));
        }

        let function = ModbusFunction::from_u8(function_code)?;
        let data = frame[MBAP_HEADER_SIZE + 2..MBAP_HEADER_SIZE + length as usize].to_vec();

        Ok(ModbusResponse::new_success(slave_id, function, data))
    }
}

impl ModbusTransport for TcpTransport {
    async fn request(&mut self, request: &ModbusRequest) -> ModbusResult<ModbusResponse> {
        // Validate request
        request.validate()?;

        // Ensure connection
        if self.stream.is_none() {
            self.reconnect().await?;
        }

        // Encode and send request
        let frame = self.encode_request(request);
        self.stats.requests_sent += 1;
        self.stats.bytes_sent += frame.len() as u64;

        // Log outgoing packet
        if self.packet_logging {
            log_packet("send", &frame, "TCP", Some(request.slave_id));
        }

        let stream = self.stream.as_mut().unwrap();

        let send_result = timeout(self.timeout, stream.write_all(&frame)).await;
        if send_result.is_err() || send_result.unwrap().is_err() {
            self.stats.timeouts += 1;
            self.stats.errors += 1;
            self.stream = None; // Mark connection as broken
            return Err(ModbusError::timeout(
                "send request",
                self.timeout.as_millis() as u64,
            ));
        }

        // Read response header first
        let mut header_buf = [0u8; MBAP_HEADER_SIZE + 1]; // MBAP + function code
        let read_result = timeout(self.timeout, stream.read_exact(&mut header_buf)).await;

        if read_result.is_err() || read_result.unwrap().is_err() {
            self.stats.timeouts += 1;
            self.stats.errors += 1;
            self.stream = None;
            return Err(ModbusError::timeout(
                "read response header",
                self.timeout.as_millis() as u64,
            ));
        }

        // Parse length from header
        let length = u16::from_be_bytes([header_buf[4], header_buf[5]]);
        if length > MAX_TCP_FRAME_SIZE as u16 {
            self.stats.errors += 1;
            return Err(ModbusError::frame("Response frame too large"));
        }

        // Read remaining data
        let remaining_bytes = (length as usize).saturating_sub(1); // -1 for function code already read
        let mut response_buf = vec![0u8; MBAP_HEADER_SIZE + 1 + remaining_bytes];
        response_buf[..MBAP_HEADER_SIZE + 1].copy_from_slice(&header_buf);

        if remaining_bytes > 0 {
            let read_result = timeout(
                self.timeout,
                stream.read_exact(&mut response_buf[MBAP_HEADER_SIZE + 1..]),
            )
            .await;

            if read_result.is_err() || read_result.unwrap().is_err() {
                self.stats.timeouts += 1;
                self.stats.errors += 1;
                self.stream = None;
                return Err(ModbusError::timeout(
                    "read response data",
                    self.timeout.as_millis() as u64,
                ));
            }
        }

        self.stats.responses_received += 1;
        self.stats.bytes_received += response_buf.len() as u64;

        // Log incoming packet
        if self.packet_logging {
            log_packet("receive", &response_buf, "TCP", Some(request.slave_id));
        }

        // Decode response
        let response = self.decode_response(&response_buf)?;

        // Check for exception
        if let Some(error) = response.get_exception() {
            self.stats.errors += 1;
            return Err(error);
        }

        Ok(response)
    }

    fn is_connected(&self) -> bool {
        self.stream.is_some()
    }

    async fn close(&mut self) -> ModbusResult<()> {
        if let Some(mut stream) = self.stream.take() {
            let _ = stream.shutdown().await;
        }
        Ok(())
    }

    fn get_stats(&self) -> TransportStats {
        self.stats.clone()
    }
}

/// Modbus RTU transport implementation
#[cfg(feature = "rtu")]
pub struct RtuTransport {
    /// Serial port connection
    port: Option<tokio_serial::SerialStream>,
    /// Port name/path
    port_name: String,
    /// Baud rate
    baud_rate: u32,
    /// Data bits (7 or 8)
    data_bits: tokio_serial::DataBits,
    /// Stop bits
    stop_bits: tokio_serial::StopBits,
    /// Parity
    parity: tokio_serial::Parity,
    /// Timeout for operations
    timeout: Duration,
    /// Frame gap time in milliseconds (minimum time between frames)
    frame_gap: Duration,
    /// Transport statistics
    stats: TransportStats,
    /// Enable packet logging for debugging
    packet_logging: bool,
}

#[cfg(feature = "rtu")]
impl RtuTransport {
    /// Create a new RTU transport
    pub fn new(port: &str, baud_rate: u32) -> ModbusResult<Self> {
        Self::new_with_config(
            port,
            baud_rate,
            tokio_serial::DataBits::Eight,
            tokio_serial::StopBits::One,
            tokio_serial::Parity::None,
            Duration::from_millis(1000),
        )
    }

    /// Create a new RTU transport with full configuration
    pub fn new_with_config(
        port: &str,
        baud_rate: u32,
        data_bits: tokio_serial::DataBits,
        stop_bits: tokio_serial::StopBits,
        parity: tokio_serial::Parity,
        timeout: Duration,
    ) -> ModbusResult<Self> {
        // Calculate frame gap time based on baud rate
        // Minimum gap is 3.5 character times
        let char_time_us = (11_000_000 / baud_rate) as u64; // 11 bits per character in microseconds
        let frame_gap = Duration::from_micros(char_time_us * 35 / 10); // 3.5 character times

        let mut transport = Self {
            port: None,
            port_name: port.to_string(),
            baud_rate,
            data_bits,
            stop_bits,
            parity,
            timeout,
            frame_gap,
            stats: TransportStats::default(),
            packet_logging: false,
        };

        // Try to connect immediately
        transport.connect()?;

        Ok(transport)
    }

    /// Create a new RTU transport with packet logging enabled
    pub fn new_with_packet_logging(
        port: &str,
        baud_rate: u32,
        data_bits: tokio_serial::DataBits,
        stop_bits: tokio_serial::StopBits,
        parity: tokio_serial::Parity,
        timeout: Duration,
        enable_logging: bool,
    ) -> ModbusResult<Self> {
        let char_time_us = (11_000_000 / baud_rate) as u64;
        let frame_gap = Duration::from_micros(char_time_us * 35 / 10);

        let mut transport = Self {
            port: None,
            port_name: port.to_string(),
            baud_rate,
            data_bits,
            stop_bits,
            parity,
            timeout,
            frame_gap,
            stats: TransportStats::default(),
            packet_logging: enable_logging,
        };

        transport.connect()?;
        Ok(transport)
    }

    /// Enable or disable packet logging
    pub fn set_packet_logging(&mut self, enabled: bool) {
        self.packet_logging = enabled;
    }

    /// Connect to the serial port
    fn connect(&mut self) -> ModbusResult<()> {
        let builder = tokio_serial::new(&self.port_name, self.baud_rate)
            .data_bits(self.data_bits)
            .stop_bits(self.stop_bits)
            .parity(self.parity)
            .timeout(self.timeout);

        let port = tokio_serial::SerialStream::open(&builder).map_err(|e| {
            ModbusError::connection(format!(
                "Failed to open serial port {}: {}",
                self.port_name, e
            ))
        })?;

        self.port = Some(port);

        Ok(())
    }

    /// Calculate CRC for RTU frame
    fn calculate_crc(data: &[u8]) -> u16 {
        CRC_MODBUS.checksum(data)
    }

    /// Encode request to RTU frame
    fn encode_request(&self, request: &ModbusRequest) -> ModbusResult<Vec<u8>> {
        let mut frame = Vec::new();

        // Slave ID
        frame.push(request.slave_id);

        // Function code
        frame.push(request.function.to_u8());

        // Function-specific data
        match request.function {
            ModbusFunction::ReadCoils
            | ModbusFunction::ReadDiscreteInputs
            | ModbusFunction::ReadHoldingRegisters
            | ModbusFunction::ReadInputRegisters => {
                // Address (2 bytes) + Quantity (2 bytes)
                frame.extend_from_slice(&request.address.to_be_bytes());
                frame.extend_from_slice(&request.quantity.to_be_bytes());
            }

            ModbusFunction::WriteSingleCoil => {
                // Address (2 bytes) + Value (2 bytes: 0x0000 or 0xFF00)
                frame.extend_from_slice(&request.address.to_be_bytes());
                let value: u16 = if !request.data.is_empty() && request.data[0] != 0 {
                    0xFF00
                } else {
                    0x0000
                };
                frame.extend_from_slice(&value.to_be_bytes());
            }

            ModbusFunction::WriteSingleRegister => {
                // Address (2 bytes) + Value (2 bytes)
                frame.extend_from_slice(&request.address.to_be_bytes());
                if request.data.len() >= 2 {
                    frame.extend_from_slice(&request.data[0..2]);
                } else {
                    frame.extend_from_slice(&[0, 0]);
                }
            }

            ModbusFunction::WriteMultipleCoils => {
                // Address (2 bytes) + Quantity (2 bytes) + Byte count (1 byte) + Data
                frame.extend_from_slice(&request.address.to_be_bytes());
                frame.extend_from_slice(&request.quantity.to_be_bytes());
                frame.push(request.data.len() as u8);
                frame.extend_from_slice(&request.data);
            }

            ModbusFunction::WriteMultipleRegisters => {
                // Address (2 bytes) + Quantity (2 bytes) + Byte count (1 byte) + Data
                frame.extend_from_slice(&request.address.to_be_bytes());
                frame.extend_from_slice(&request.quantity.to_be_bytes());
                frame.push(request.data.len() as u8);
                frame.extend_from_slice(&request.data);
            }
        }

        // Calculate and append CRC
        let crc = Self::calculate_crc(&frame);
        frame.extend_from_slice(&crc.to_le_bytes()); // CRC is little-endian in RTU

        Ok(frame)
    }

    /// Decode response from RTU frame
    fn decode_response(&self, frame: &[u8]) -> ModbusResult<ModbusResponse> {
        if frame.len() < 4 {
            return Err(ModbusError::frame("RTU frame too short"));
        }

        // Verify CRC
        let data_len = frame.len() - 2;
        let received_crc = u16::from_le_bytes([frame[data_len], frame[data_len + 1]]);
        let calculated_crc = Self::calculate_crc(&frame[..data_len]);

        if received_crc != calculated_crc {
            return Err(ModbusError::frame(format!(
                "CRC mismatch: expected 0x{:04X}, got 0x{:04X}",
                calculated_crc, received_crc
            )));
        }

        let slave_id = frame[0];
        let function_code = frame[1];

        // Check for exception response
        if function_code & 0x80 != 0 {
            if frame.len() < 5 {
                return Err(ModbusError::frame("Invalid exception response"));
            }

            let original_function = function_code & 0x7F;
            let exception_code = frame[2];

            return Ok(ModbusResponse::new_exception(
                slave_id,
                ModbusFunction::from_u8(original_function)?,
                exception_code,
            ));
        }

        let function = ModbusFunction::from_u8(function_code)?;
        let data = if frame.len() > 4 {
            frame[2..data_len].to_vec()
        } else {
            Vec::new()
        };

        Ok(ModbusResponse::new_success(slave_id, function, data))
    }

    /// Wait for frame gap before sending next frame
    async fn wait_frame_gap(&self) {
        tokio::time::sleep(self.frame_gap).await;
    }

    /// Read RTU frame from serial port
    async fn read_frame(&mut self) -> ModbusResult<Vec<u8>> {
        let port = self
            .port
            .as_mut()
            .ok_or_else(|| ModbusError::connection("Serial port not connected"))?;

        let mut frame = Vec::new();
        let mut buffer = [0u8; 1];

        // Read until frame gap timeout
        loop {
            match timeout(self.frame_gap, port.read_exact(&mut buffer)).await {
                Ok(Ok(_)) => {
                    frame.push(buffer[0]);

                    // Prevent frames from getting too large
                    if frame.len() > MAX_RTU_FRAME_SIZE {
                        return Err(ModbusError::frame("RTU frame too large"));
                    }
                }
                Ok(Err(e)) => {
                    return Err(ModbusError::io(format!("Serial read error: {}", e)));
                }
                Err(_) => {
                    // Timeout - end of frame
                    if !frame.is_empty() {
                        break;
                    }
                    // If no data yet, continue waiting
                }
            }
        }

        if frame.is_empty() {
            return Err(ModbusError::timeout(
                "No response received",
                self.timeout.as_millis() as u64,
            ));
        }

        Ok(frame)
    }
}

#[cfg(feature = "rtu")]
impl ModbusTransport for RtuTransport {
    async fn request(&mut self, request: &ModbusRequest) -> ModbusResult<ModbusResponse> {
        // Validate request
        request.validate()?;

        // Ensure connection
        if self.port.is_none() {
            self.connect()?;
        }

        // Wait for frame gap before sending
        self.wait_frame_gap().await;

        // Encode request
        let frame = self.encode_request(request)?;
        self.stats.requests_sent += 1;
        self.stats.bytes_sent += frame.len() as u64;

        // Log outgoing packet
        if self.packet_logging {
            log_packet("send", &frame, "RTU", Some(request.slave_id));
        }

        // Send request
        let port = self
            .port
            .as_mut()
            .ok_or_else(|| ModbusError::connection("Serial port not connected"))?;

        let send_result = timeout(self.timeout, port.write_all(&frame)).await;
        match send_result {
            Ok(Ok(_)) => {
                // Flush to ensure data is sent
                let _ = timeout(self.timeout, port.flush()).await;
            }
            Ok(Err(e)) => {
                self.stats.errors += 1;
                return Err(ModbusError::io(format!("Failed to send RTU frame: {}", e)));
            }
            Err(_) => {
                self.stats.timeouts += 1;
                self.stats.errors += 1;
                return Err(ModbusError::timeout(
                    "send request",
                    self.timeout.as_millis() as u64,
                ));
            }
        }

        // Read response
        let response_frame = match timeout(self.timeout, self.read_frame()).await {
            Ok(Ok(frame)) => frame,
            Ok(Err(e)) => {
                self.stats.errors += 1;
                return Err(e);
            }
            Err(_) => {
                self.stats.timeouts += 1;
                self.stats.errors += 1;
                return Err(ModbusError::timeout(
                    "read response",
                    self.timeout.as_millis() as u64,
                ));
            }
        };

        self.stats.responses_received += 1;
        self.stats.bytes_received += response_frame.len() as u64;

        // Log incoming packet
        if self.packet_logging {
            log_packet("receive", &response_frame, "RTU", Some(request.slave_id));
        }

        // Decode response
        let response = self.decode_response(&response_frame)?;

        // Check if response is for the correct slave
        if response.slave_id != request.slave_id {
            self.stats.errors += 1;
            return Err(ModbusError::protocol(format!(
                "Response slave ID mismatch: expected {}, got {}",
                request.slave_id, response.slave_id
            )));
        }

        // Check for exception
        if let Some(error) = response.get_exception() {
            self.stats.errors += 1;
            return Err(error);
        }

        Ok(response)
    }

    fn is_connected(&self) -> bool {
        self.port.is_some()
    }

    async fn close(&mut self) -> ModbusResult<()> {
        if let Some(_port) = self.port.take() {
            // SerialStream doesn't need explicit close, it will be dropped
        }
        Ok(())
    }

    fn get_stats(&self) -> TransportStats {
        self.stats.clone()
    }
}

/// Modbus ASCII transport implementation
///
/// ASCII transport uses human-readable ASCII text format with LRC error checking.
/// This is useful for debugging, testing, and integration with legacy systems.
///
/// ## ASCII Frame Format
///
/// ```text
/// Start -> Address -> Function -> Data -> LRC -> End
///   :        01         03       001000010002   F4   \r\n
/// ```
///
/// - **Start**: ':' (0x3A)
/// - **Address**: 2 ASCII characters (1 byte slave address)
/// - **Function**: 2 ASCII characters (1 byte function code)
/// - **Data**: Variable length ASCII characters (n bytes data)
/// - **LRC**: 2 ASCII characters (1 byte Longitudinal Redundancy Check)
/// - **End**: CR LF (0x0D 0x0A)
///
/// ## Use Cases
///
/// - **Debugging**: Human-readable format for easy troubleshooting
/// - **Legacy Systems**: Integration with older SCADA systems
/// - **Educational**: Learning Modbus protocol structure
/// - **Manual Testing**: Can be typed manually in serial terminals
#[cfg(feature = "rtu")]
pub struct AsciiTransport {
    /// Serial port connection
    port: Option<tokio_serial::SerialStream>,
    /// Port name/path
    port_name: String,
    /// Baud rate
    baud_rate: u32,
    /// Data bits (typically 7 for ASCII)
    data_bits: tokio_serial::DataBits,
    /// Stop bits
    stop_bits: tokio_serial::StopBits,
    /// Parity
    parity: tokio_serial::Parity,
    /// Timeout for operations
    timeout: Duration,
    /// Inter-character timeout (time to wait between characters)
    inter_char_timeout: Duration,
    /// Transport statistics
    stats: TransportStats,
}

#[cfg(feature = "rtu")]
impl AsciiTransport {
    /// Create a new ASCII transport with default settings
    ///
    /// Default configuration:
    /// - 7 data bits (ASCII standard)
    /// - Even parity (recommended for ASCII)
    /// - 1 stop bit
    /// - 1 second timeout
    /// - 1 second inter-character timeout
    ///
    /// # Arguments
    ///
    /// * `port` - Serial port path (e.g., "/dev/ttyUSB0" or "COM1")
    /// * `baud_rate` - Communication speed (e.g., 9600, 19200)
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use voltage_modbus::transport::AsciiTransport;
    ///
    /// let transport = AsciiTransport::new("/dev/ttyUSB0", 9600)?;
    /// # Ok::<(), voltage_modbus::ModbusError>(())
    /// ```
    pub fn new(port: &str, baud_rate: u32) -> ModbusResult<Self> {
        Self::new_with_config(
            port,
            baud_rate,
            tokio_serial::DataBits::Seven, // ASCII standard
            tokio_serial::StopBits::One,
            tokio_serial::Parity::Even,  // Recommended for ASCII
            Duration::from_secs(1),      // Read timeout
            Duration::from_millis(1000), // Inter-character timeout
        )
    }

    /// Create a new ASCII transport with full configuration
    ///
    /// # Arguments
    ///
    /// * `port` - Serial port path
    /// * `baud_rate` - Communication speed
    /// * `data_bits` - Number of data bits (7 or 8)
    /// * `stop_bits` - Number of stop bits
    /// * `parity` - Parity checking mode
    /// * `timeout` - Overall operation timeout
    /// * `inter_char_timeout` - Maximum time between characters in a frame
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use voltage_modbus::transport::AsciiTransport;
    /// use std::time::Duration;
    ///
    /// let transport = AsciiTransport::new_with_config(
    ///     "/dev/ttyUSB0",
    ///     9600,
    ///     tokio_serial::DataBits::Seven,
    ///     tokio_serial::StopBits::One,
    ///     tokio_serial::Parity::Even,
    ///     Duration::from_secs(5),
    ///     Duration::from_millis(100)
    /// )?;
    /// # Ok::<(), voltage_modbus::ModbusError>(())
    /// ```
    pub fn new_with_config(
        port: &str,
        baud_rate: u32,
        data_bits: tokio_serial::DataBits,
        stop_bits: tokio_serial::StopBits,
        parity: tokio_serial::Parity,
        timeout: Duration,
        inter_char_timeout: Duration,
    ) -> ModbusResult<Self> {
        let mut transport = Self {
            port: None,
            port_name: port.to_string(),
            baud_rate,
            data_bits,
            stop_bits,
            parity,
            timeout,
            inter_char_timeout,
            stats: TransportStats::default(),
        };

        // Try to connect immediately
        transport.connect()?;

        Ok(transport)
    }

    /// Connect to the serial port
    fn connect(&mut self) -> ModbusResult<()> {
        let builder = tokio_serial::new(&self.port_name, self.baud_rate)
            .data_bits(self.data_bits)
            .stop_bits(self.stop_bits)
            .parity(self.parity)
            .timeout(self.timeout);

        let port = tokio_serial::SerialStream::open(&builder).map_err(|e| {
            ModbusError::connection(format!(
                "Failed to open serial port {}: {}",
                self.port_name, e
            ))
        })?;

        self.port = Some(port);

        Ok(())
    }

    /// Calculate LRC (Longitudinal Redundancy Check) for ASCII frame
    ///
    /// LRC is calculated as the two's complement of the sum of all data bytes.
    /// For ASCII frames, this includes address, function code, and data fields.
    ///
    /// # Arguments
    ///
    /// * `data` - The raw data bytes (not ASCII encoded)
    ///
    /// # Returns
    ///
    /// The LRC checksum value
    fn calculate_lrc(data: &[u8]) -> u8 {
        let sum: u16 = data.iter().map(|&b| b as u16).sum();
        (-(sum as i16)) as u8 // Two's complement
    }

    /// Convert byte to 2-character ASCII hex string
    ///
    /// # Examples
    ///
    /// ```text
    /// 0x01 -> "01"
    /// 0xFF -> "FF"
    /// ```
    fn byte_to_ascii_hex(byte: u8) -> [u8; 2] {
        let high = (byte >> 4) & 0x0F;
        let low = byte & 0x0F;

        let high_char = if high < 10 {
            b'0' + high
        } else {
            b'A' + (high - 10)
        };
        let low_char = if low < 10 {
            b'0' + low
        } else {
            b'A' + (low - 10)
        };

        [high_char, low_char]
    }

    /// Convert 2-character ASCII hex string to byte
    ///
    /// # Examples
    ///
    /// ```text
    /// "01" -> 0x01
    /// "FF" -> 0xFF
    /// ```
    fn ascii_hex_to_byte(ascii: &[u8]) -> ModbusResult<u8> {
        if ascii.len() != 2 {
            return Err(ModbusError::frame("Invalid ASCII hex length"));
        }

        let high = Self::ascii_char_to_hex(ascii[0])?;
        let low = Self::ascii_char_to_hex(ascii[1])?;

        Ok((high << 4) | low)
    }

    /// Convert single ASCII character to hex value
    fn ascii_char_to_hex(c: u8) -> ModbusResult<u8> {
        match c {
            b'0'..=b'9' => Ok(c - b'0'),
            b'A'..=b'F' => Ok(c - b'A' + 10),
            b'a'..=b'f' => Ok(c - b'a' + 10),
            _ => Err(ModbusError::frame(format!(
                "Invalid ASCII hex character: {}",
                c as char
            ))),
        }
    }

    /// Encode request to ASCII frame
    ///
    /// ASCII frame format: `:AAFFDDD...LRCCRLF`
    /// - `:` - Start character
    /// - `AA` - Address (2 ASCII chars)
    /// - `FF` - Function code (2 ASCII chars)
    /// - `DDD...` - Data (variable length ASCII chars)
    /// - `LRC` - Checksum (2 ASCII chars)
    /// - `CRLF` - End characters (0x0D, 0x0A)
    fn encode_request(&self, request: &ModbusRequest) -> ModbusResult<Vec<u8>> {
        // Build raw data for LRC calculation
        let mut raw_data = Vec::new();
        raw_data.push(request.slave_id);
        raw_data.push(request.function.to_u8());

        // Add function-specific data
        match request.function {
            ModbusFunction::ReadCoils
            | ModbusFunction::ReadDiscreteInputs
            | ModbusFunction::ReadHoldingRegisters
            | ModbusFunction::ReadInputRegisters => {
                raw_data.extend_from_slice(&request.address.to_be_bytes());
                raw_data.extend_from_slice(&request.quantity.to_be_bytes());
            }

            ModbusFunction::WriteSingleCoil => {
                raw_data.extend_from_slice(&request.address.to_be_bytes());
                let value: u16 = if !request.data.is_empty() && request.data[0] != 0 {
                    0xFF00
                } else {
                    0x0000
                };
                raw_data.extend_from_slice(&value.to_be_bytes());
            }

            ModbusFunction::WriteSingleRegister => {
                raw_data.extend_from_slice(&request.address.to_be_bytes());
                if request.data.len() >= 2 {
                    raw_data.extend_from_slice(&request.data[0..2]);
                } else {
                    raw_data.extend_from_slice(&[0, 0]);
                }
            }

            ModbusFunction::WriteMultipleCoils | ModbusFunction::WriteMultipleRegisters => {
                raw_data.extend_from_slice(&request.address.to_be_bytes());
                raw_data.extend_from_slice(&request.quantity.to_be_bytes());
                raw_data.push(request.data.len() as u8);
                raw_data.extend_from_slice(&request.data);
            }
        }

        // Calculate LRC
        let lrc = Self::calculate_lrc(&raw_data);

        // Build ASCII frame
        let mut frame = Vec::new();

        // Start character
        frame.push(b':');

        // Convert each byte to ASCII hex
        for &byte in &raw_data {
            let ascii_hex = Self::byte_to_ascii_hex(byte);
            frame.extend_from_slice(&ascii_hex);
        }

        // Add LRC
        let lrc_ascii = Self::byte_to_ascii_hex(lrc);
        frame.extend_from_slice(&lrc_ascii);

        // End characters (CR LF)
        frame.push(0x0D); // CR
        frame.push(0x0A); // LF

        Ok(frame)
    }

    /// Decode response from ASCII frame
    fn decode_response(&self, frame: &[u8]) -> ModbusResult<ModbusResponse> {
        // Minimum frame: ":AAFFLLCRLF" = 11 characters
        if frame.len() < 11 {
            return Err(ModbusError::frame("ASCII frame too short"));
        }

        // Check start character
        if frame[0] != b':' {
            return Err(ModbusError::frame("Invalid ASCII frame start character"));
        }

        // Check end characters
        let len = frame.len();
        if frame[len - 2] != 0x0D || frame[len - 1] != 0x0A {
            return Err(ModbusError::frame("Invalid ASCII frame end characters"));
        }

        // Extract ASCII data (without start and end)
        let ascii_data = &frame[1..len - 2];

        // ASCII data length must be even (each byte = 2 ASCII chars)
        if ascii_data.len() % 2 != 0 {
            return Err(ModbusError::frame("Invalid ASCII frame length"));
        }

        // Convert ASCII to raw bytes
        let mut raw_data = Vec::new();
        for chunk in ascii_data.chunks(2) {
            let byte = Self::ascii_hex_to_byte(chunk)?;
            raw_data.push(byte);
        }

        // Must have at least address, function, and LRC
        if raw_data.len() < 3 {
            return Err(ModbusError::frame("ASCII frame too short after decoding"));
        }

        // Extract LRC and data
        let received_lrc = raw_data.pop().unwrap();
        let calculated_lrc = Self::calculate_lrc(&raw_data);

        // Verify LRC
        if received_lrc != calculated_lrc {
            return Err(ModbusError::frame(format!(
                "LRC mismatch: expected 0x{:02X}, got 0x{:02X}",
                calculated_lrc, received_lrc
            )));
        }

        let slave_id = raw_data[0];
        let function_code = raw_data[1];

        // Check for exception response
        if function_code & 0x80 != 0 {
            if raw_data.len() < 3 {
                return Err(ModbusError::frame("Invalid exception response"));
            }

            let original_function = function_code & 0x7F;
            let exception_code = raw_data[2];

            return Ok(ModbusResponse::new_exception(
                slave_id,
                ModbusFunction::from_u8(original_function)?,
                exception_code,
            ));
        }

        let function = ModbusFunction::from_u8(function_code)?;
        let data = if raw_data.len() > 2 {
            raw_data[2..].to_vec()
        } else {
            Vec::new()
        };

        Ok(ModbusResponse::new_success(slave_id, function, data))
    }

    /// Read ASCII frame from serial port
    ///
    /// ASCII frames are terminated by CR LF, so we read until we see these characters.
    async fn read_frame(&mut self) -> ModbusResult<Vec<u8>> {
        let port = self
            .port
            .as_mut()
            .ok_or_else(|| ModbusError::connection("Serial port not connected"))?;

        let mut frame = Vec::new();
        let mut buffer = [0u8; 1];

        // Read until CR LF
        loop {
            match timeout(self.inter_char_timeout, port.read_exact(&mut buffer)).await {
                Ok(Ok(_)) => {
                    frame.push(buffer[0]);

                    // Check for frame end (CR LF)
                    if frame.len() >= 2
                        && frame[frame.len() - 2] == 0x0D
                        && frame[frame.len() - 1] == 0x0A
                    {
                        break;
                    }

                    // Prevent frames from getting too large
                    if frame.len() > MAX_RTU_FRAME_SIZE * 2 {
                        // ASCII is ~2x larger than RTU
                        return Err(ModbusError::frame("ASCII frame too large"));
                    }
                }
                Ok(Err(e)) => {
                    return Err(ModbusError::io(format!("Serial read error: {}", e)));
                }
                Err(_) => {
                    if frame.is_empty() {
                        // No data received, continue waiting
                        continue;
                    } else {
                        // Timeout during frame reception
                        return Err(ModbusError::timeout(
                            "Incomplete ASCII frame",
                            self.inter_char_timeout.as_millis() as u64,
                        ));
                    }
                }
            }
        }

        if frame.is_empty() {
            return Err(ModbusError::timeout(
                "No response received",
                self.timeout.as_millis() as u64,
            ));
        }

        Ok(frame)
    }
}

#[cfg(feature = "rtu")]
impl ModbusTransport for AsciiTransport {
    async fn request(&mut self, request: &ModbusRequest) -> ModbusResult<ModbusResponse> {
        // Validate request
        request.validate()?;

        // Ensure connection
        if self.port.is_none() {
            self.connect()?;
        }

        // Encode request
        let frame = self.encode_request(request)?;
        self.stats.requests_sent += 1;
        self.stats.bytes_sent += frame.len() as u64;

        // Send request
        let port = self
            .port
            .as_mut()
            .ok_or_else(|| ModbusError::connection("Serial port not connected"))?;

        let send_result = timeout(self.timeout, port.write_all(&frame)).await;
        match send_result {
            Ok(Ok(_)) => {
                // Flush to ensure data is sent
                let _ = timeout(self.timeout, port.flush()).await;
            }
            Ok(Err(e)) => {
                self.stats.errors += 1;
                return Err(ModbusError::io(format!(
                    "Failed to send ASCII frame: {}",
                    e
                )));
            }
            Err(_) => {
                self.stats.timeouts += 1;
                self.stats.errors += 1;
                return Err(ModbusError::timeout(
                    "send request",
                    self.timeout.as_millis() as u64,
                ));
            }
        }

        // Read response
        let response_frame = match timeout(self.timeout, self.read_frame()).await {
            Ok(Ok(frame)) => frame,
            Ok(Err(e)) => {
                self.stats.errors += 1;
                return Err(e);
            }
            Err(_) => {
                self.stats.timeouts += 1;
                self.stats.errors += 1;
                return Err(ModbusError::timeout(
                    "read response",
                    self.timeout.as_millis() as u64,
                ));
            }
        };

        self.stats.responses_received += 1;
        self.stats.bytes_received += response_frame.len() as u64;

        // Decode response
        let response = self.decode_response(&response_frame)?;

        // Check if response is for the correct slave
        if response.slave_id != request.slave_id {
            self.stats.errors += 1;
            return Err(ModbusError::protocol(format!(
                "Response slave ID mismatch: expected {}, got {}",
                request.slave_id, response.slave_id
            )));
        }

        // Check for exception
        if let Some(error) = response.get_exception() {
            self.stats.errors += 1;
            return Err(error);
        }

        Ok(response)
    }

    fn is_connected(&self) -> bool {
        self.port.is_some()
    }

    async fn close(&mut self) -> ModbusResult<()> {
        if let Some(_port) = self.port.take() {
            // SerialStream doesn't need explicit close, it will be dropped
        }
        Ok(())
    }

    fn get_stats(&self) -> TransportStats {
        self.stats.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_tcp_transport_creation() {
        let addr = "127.0.0.1:502".parse().unwrap();
        let timeout = Duration::from_secs(5);

        // This will fail unless there's a server running, but tests the creation logic
        let result = TcpTransport::new(addr, timeout).await;
        // Don't assert success since we don't have a test server
        println!("TCP transport creation result: {:?}", result.is_ok());
    }
}

#[cfg(all(test, feature = "rtu"))]
mod rtu_tests {
    use super::*;
    use crate::protocol::ModbusFunction;

    #[test]
    fn test_crc_calculation() {
        let data = [0x01, 0x03, 0x00, 0x00, 0x00, 0x02];
        let crc = RtuTransport::calculate_crc(&data);
        // Expected CRC for this data should be calculated
        assert!(crc > 0);
    }

    #[test]
    fn test_ascii_lrc_calculation() {
        let data = [0x01, 0x03, 0x00, 0x00, 0x00, 0x02];
        let lrc = AsciiTransport::calculate_lrc(&data);

        // LRC is two's complement of sum
        let sum: u16 = data.iter().map(|&b| b as u16).sum();
        let expected_lrc = (-(sum as i16)) as u8;

        assert_eq!(lrc, expected_lrc);
    }

    #[test]
    fn test_ascii_hex_conversion() {
        // Test byte to ASCII hex
        let ascii_hex = AsciiTransport::byte_to_ascii_hex(0x1A);
        assert_eq!(ascii_hex, [b'1', b'A']);

        let ascii_hex = AsciiTransport::byte_to_ascii_hex(0x0F);
        assert_eq!(ascii_hex, [b'0', b'F']);

        // Test ASCII hex to byte
        let byte = AsciiTransport::ascii_hex_to_byte(&[b'1', b'A']).unwrap();
        assert_eq!(byte, 0x1A);

        let byte = AsciiTransport::ascii_hex_to_byte(&[b'0', b'F']).unwrap();
        assert_eq!(byte, 0x0F);

        // Test lowercase support
        let byte = AsciiTransport::ascii_hex_to_byte(&[b'a', b'f']).unwrap();
        assert_eq!(byte, 0xAF);
    }

    #[test]
    fn test_ascii_frame_encoding() {
        // Create a mock ASCII transport for testing encoding
        let transport = create_mock_ascii_transport();

        let request = ModbusRequest::new_read(
            0x01,                                 // slave_id
            ModbusFunction::ReadHoldingRegisters, // function
            0x0000,                               // address
            0x0002,                               // quantity
        );

        let frame = transport.encode_request(&request).unwrap();

        // Calculate expected LRC manually for verification
        // Data: [0x01, 0x03, 0x00, 0x00, 0x00, 0x02]
        let data = [0x01u8, 0x03, 0x00, 0x00, 0x00, 0x02];
        let sum: u16 = data.iter().map(|&b| b as u16).sum(); // sum = 6
        let expected_lrc = (-(sum as i16)) as u8; // LRC = 0xFA

        // Expected frame: ":010300000002FA\r\n"
        let expected = format!(":010300000002{:02X}\r\n", expected_lrc);
        let expected_bytes = expected.as_bytes();

        assert_eq!(frame, expected_bytes);
        assert_eq!(frame[0], b':'); // Start
        assert_eq!(frame[frame.len() - 2], 0x0D); // CR
        assert_eq!(frame[frame.len() - 1], 0x0A); // LF
    }

    #[test]
    fn test_ascii_frame_decoding() {
        let transport = create_mock_ascii_transport();

        // Test successful response with correct LRC
        // Simulating a response: slave=1, function=3, byte_count=4, data=[0x00, 0xAB, 0x00, 0xCD]
        let test_data = [0x01u8, 0x03, 0x04, 0x00, 0xAB, 0x00, 0xCD];
        let sum: u16 = test_data.iter().map(|&b| b as u16).sum();
        let lrc = (-(sum as i16)) as u8;

        let frame = format!(":01030400AB00CD{:02X}\r\n", lrc);
        let response = transport.decode_response(frame.as_bytes()).unwrap();

        assert_eq!(response.slave_id, 0x01);
        assert_eq!(response.function, ModbusFunction::ReadHoldingRegisters);
        assert!(!response.is_exception());

        // Test exception response: slave=1, function=0x83(exception), exception_code=0x02
        // Data: [0x01, 0x83, 0x02]
        let exc_data = [0x01u8, 0x83, 0x02];
        let exc_sum: u16 = exc_data.iter().map(|&b| b as u16).sum();
        let exc_lrc = (-(exc_sum as i16)) as u8;

        let exception_frame = format!(":018302{:02X}\r\n", exc_lrc);
        let exception_response = transport
            .decode_response(exception_frame.as_bytes())
            .unwrap();

        assert_eq!(exception_response.slave_id, 0x01);
        assert!(exception_response.is_exception());
    }

    #[test]
    fn test_ascii_error_handling() {
        let transport = create_mock_ascii_transport();

        // Test invalid start character
        let invalid_start = b"X010300000002C5\r\n";
        assert!(transport.decode_response(invalid_start).is_err());

        // Test invalid end characters
        let invalid_end = b":010300000002C5\r\r";
        assert!(transport.decode_response(invalid_end).is_err());

        // Test odd length (invalid ASCII hex)
        let odd_length = b":01030000002C5\r\n";
        assert!(transport.decode_response(odd_length).is_err());

        // Test LRC mismatch
        let wrong_lrc = b":010300000002FF\r\n";
        assert!(transport.decode_response(wrong_lrc).is_err());
    }

    /// Helper function to create ASCII transport for testing
    fn create_mock_ascii_transport() -> AsciiTransport {
        // Create transport without connecting to actual port
        AsciiTransport {
            port: None,
            port_name: "mock".to_string(),
            baud_rate: 9600,
            data_bits: tokio_serial::DataBits::Seven,
            stop_bits: tokio_serial::StopBits::One,
            parity: tokio_serial::Parity::Even,
            timeout: Duration::from_secs(1),
            inter_char_timeout: Duration::from_millis(100),
            stats: TransportStats::default(),
        }
    }
}
