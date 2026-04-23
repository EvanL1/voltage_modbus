//! Modbus server implementations
//!
//! This module provides complete server-side implementations for both TCP and RTU protocols.

use std::net::SocketAddr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::{broadcast, Semaphore};
use tokio::time::timeout;

#[cfg(feature = "rtu")]
use tokio_serial;
use tracing::{debug, error, info, warn};

use crate::constants::{MAX_READ_COILS, MAX_READ_REGISTERS, MAX_WRITE_COILS, MAX_WRITE_REGISTERS};
use crate::error::{ModbusError, ModbusResult};

use crate::register_bank::{ModbusRegisterBank, RegisterBankStats};

/// Maximum frame size for Modbus TCP
const MAX_TCP_FRAME_SIZE: usize = 260;

/// MBAP header size
const MBAP_HEADER_SIZE: usize = 6;

/// Modbus server trait
pub trait ModbusServer: Send + Sync {
    /// Start the server
    fn start(&mut self) -> impl std::future::Future<Output = ModbusResult<()>> + Send;

    /// Stop the server
    fn stop(&mut self) -> impl std::future::Future<Output = ModbusResult<()>> + Send;

    /// Check if server is running
    fn is_running(&self) -> bool;

    /// Get server statistics
    fn get_stats(&self) -> ServerStats;

    /// Get register bank reference
    fn get_register_bank(&self) -> Option<Arc<ModbusRegisterBank>>;
}

/// Server statistics
#[derive(Debug, Clone, Default)]
pub struct ServerStats {
    pub connections_count: u64,
    pub total_requests: u64,
    pub successful_requests: u64,
    pub failed_requests: u64,
    pub bytes_received: u64,
    pub bytes_sent: u64,
    pub uptime_seconds: u64,
    pub register_bank_stats: Option<RegisterBankStats>,
}

/// Modbus TCP server configuration
#[derive(Debug, Clone)]
pub struct ModbusTcpServerConfig {
    pub bind_address: SocketAddr,
    pub max_connections: usize,
    pub request_timeout: Duration,
    pub register_bank: Option<Arc<ModbusRegisterBank>>,
}

impl Default for ModbusTcpServerConfig {
    fn default() -> Self {
        Self {
            bind_address: "127.0.0.1:502".parse().unwrap(),
            max_connections: 100,
            request_timeout: Duration::from_secs(30),
            register_bank: None,
        }
    }
}

/// Modbus TCP server implementation
pub struct ModbusTcpServer {
    config: ModbusTcpServerConfig,
    register_bank: Arc<ModbusRegisterBank>,
    stats: Arc<Mutex<ServerStats>>,
    shutdown_tx: Option<broadcast::Sender<()>>,
    is_running: Arc<AtomicBool>,
    start_time: Option<std::time::Instant>,
}

impl ModbusTcpServer {
    /// Create a new TCP server with default configuration
    pub fn new(bind_address: &str) -> ModbusResult<Self> {
        let addr = bind_address
            .parse()
            .map_err(|e| ModbusError::invalid_data(format!("Invalid bind address: {}", e)))?;

        let config = ModbusTcpServerConfig {
            bind_address: addr,
            ..Default::default()
        };

        Self::with_config(config)
    }

    /// Create a new TCP server with custom configuration
    pub fn with_config(config: ModbusTcpServerConfig) -> ModbusResult<Self> {
        let register_bank = config
            .register_bank
            .clone()
            .unwrap_or_else(|| Arc::new(ModbusRegisterBank::new()));

        Ok(Self {
            config,
            register_bank,
            stats: Arc::new(Mutex::new(ServerStats::default())),
            shutdown_tx: None,
            is_running: Arc::new(AtomicBool::new(false)),
            start_time: None,
        })
    }

    /// Set custom register bank
    pub fn set_register_bank(&mut self, register_bank: Arc<ModbusRegisterBank>) {
        self.register_bank = register_bank;
    }

    /// Handle client connection
    async fn handle_client(
        mut stream: TcpStream,
        register_bank: Arc<ModbusRegisterBank>,
        stats: Arc<Mutex<ServerStats>>,
        mut shutdown_rx: broadcast::Receiver<()>,
        request_timeout: Duration,
    ) {
        let peer_addr = stream
            .peer_addr()
            .unwrap_or_else(|_| "unknown".parse().unwrap());
        info!("📡 New client connected: {}", peer_addr);

        // Update connection count
        if let Ok(mut stats) = stats.lock() {
            stats.connections_count += 1;
        }

        loop {
            tokio::select! {
                // Handle shutdown signal
                _ = shutdown_rx.recv() => {
                    debug!("Shutdown signal received for client {}", peer_addr);
                    break;
                }

                // Handle client request
                result = timeout(request_timeout, Self::read_tcp_frame(&mut stream)) => {
                    match result {
                        Ok(Ok(frame)) => {
                            // Update stats
                            if let Ok(mut stats) = stats.lock() {
                                stats.total_requests += 1;
                                stats.bytes_received += frame.len() as u64;
                            }

                            // Process request
                            match Self::handle_request(&frame, &register_bank).await {
                                Ok(response_data) => {
                                    if let Err(e) = stream.write_all(&response_data).await {
                                        error!("Failed to send response to {}: {}", peer_addr, e);
                                        break;
                                    } else {
                                        // Update success stats
                                        if let Ok(mut stats) = stats.lock() {
                                            stats.successful_requests += 1;
                                            stats.bytes_sent += response_data.len() as u64;
                                        }
                                    }
                                }
                                Err(e) => {
                                    error!("Error processing request from {}: {}", peer_addr, e);

                                    // Send error response if possible
                                    let exception_code = Self::exception_code_for_error(&e);
                                    if let Ok(error_response) =
                                        Self::create_error_response(&frame, exception_code)
                                    {
                                        let _ = stream.write_all(&error_response).await;
                                        if let Ok(mut stats) = stats.lock() {
                                            stats.bytes_sent += error_response.len() as u64;
                                        }
                                    }

                                    // Update error stats
                                    if let Ok(mut stats) = stats.lock() {
                                        stats.failed_requests += 1;
                                    }
                                }
                            }
                        }
                        Ok(Err(e)) => {
                            error!("Read error from {}: {}", peer_addr, e);
                            break;
                        }
                        Err(_) => {
                            warn!("Read timeout from {}", peer_addr);
                            break;
                        }
                    }
                }
            }
        }

        info!("🔌 Client {} disconnected", peer_addr);
    }

    async fn read_tcp_frame(stream: &mut TcpStream) -> ModbusResult<Vec<u8>> {
        let mut header = [0u8; MBAP_HEADER_SIZE];
        stream.read_exact(&mut header).await?;

        let protocol_id = u16::from_be_bytes([header[2], header[3]]);
        if protocol_id != 0 {
            return Err(ModbusError::frame(format!(
                "Invalid protocol ID: {:04X}",
                protocol_id
            )));
        }

        let length = u16::from_be_bytes([header[4], header[5]]);
        if !(2..=254).contains(&length) {
            return Err(ModbusError::frame(format!(
                "Invalid MBAP length: {} (must be 2-254)",
                length
            )));
        }

        let total_len = MBAP_HEADER_SIZE + usize::from(length);
        if total_len > MAX_TCP_FRAME_SIZE {
            return Err(ModbusError::frame("TCP frame too large"));
        }

        let mut frame = vec![0u8; total_len];
        frame[..MBAP_HEADER_SIZE].copy_from_slice(&header);
        stream.read_exact(&mut frame[MBAP_HEADER_SIZE..]).await?;
        Ok(frame)
    }

    /// Process Modbus request
    async fn handle_request(
        data: &[u8],
        register_bank: &Arc<ModbusRegisterBank>,
    ) -> ModbusResult<Vec<u8>> {
        if data.len() < MBAP_HEADER_SIZE + 2 {
            return Err(ModbusError::frame("Invalid TCP frame length"));
        }

        let transaction_id = u16::from_be_bytes([data[0], data[1]]);
        let protocol_id = u16::from_be_bytes([data[2], data[3]]);
        if protocol_id != 0 {
            return Err(ModbusError::frame(format!(
                "Invalid protocol ID: {:04X}",
                protocol_id
            )));
        }

        let length = u16::from_be_bytes([data[4], data[5]]);
        let expected_len = MBAP_HEADER_SIZE + usize::from(length);
        if !(2..=254).contains(&length) || data.len() != expected_len {
            return Err(ModbusError::frame("Invalid TCP frame length"));
        }

        let unit_id = data[6];
        let function_code = data[7];
        let pdu_data = &data[8..];

        debug!("Processing function code: 0x{:02X}", function_code);

        let pdu_response = match function_code {
            0x01 => Self::handle_read_01(pdu_data, register_bank).await,
            0x02 => Self::handle_read_02(pdu_data, register_bank).await,
            0x03 => Self::handle_read_03(pdu_data, register_bank).await,
            0x04 => Self::handle_read_04(pdu_data, register_bank).await,
            0x05 => Self::handle_write_05(pdu_data, register_bank).await,
            0x06 => Self::handle_write_06(pdu_data, register_bank).await,
            0x0F => Self::handle_write_0f(pdu_data, register_bank).await,
            0x10 => Self::handle_write_10(pdu_data, register_bank).await,
            _ => {
                warn!("Unsupported function code: 0x{:02X}", function_code);
                Err(ModbusError::invalid_function(function_code))
            }
        }?;

        Self::create_success_response(transaction_id, unit_id, &pdu_response)
    }

    fn create_success_response(
        transaction_id: u16,
        unit_id: u8,
        pdu_response: &[u8],
    ) -> ModbusResult<Vec<u8>> {
        let length = u16::try_from(1 + pdu_response.len())
            .map_err(|_| ModbusError::frame("TCP response too large"))?;
        if length > 254 {
            return Err(ModbusError::frame("TCP response too large"));
        }

        let mut response = Vec::with_capacity(MBAP_HEADER_SIZE + usize::from(length));
        response.extend_from_slice(&transaction_id.to_be_bytes());
        response.extend_from_slice(&0u16.to_be_bytes());
        response.extend_from_slice(&length.to_be_bytes());
        response.push(unit_id);
        response.extend_from_slice(pdu_response);
        Ok(response)
    }

    fn exception_code_for_error(error: &ModbusError) -> u8 {
        match error {
            ModbusError::InvalidFunction { .. } => 0x01,
            ModbusError::InvalidAddress { .. } => 0x02,
            ModbusError::InvalidData { .. } | ModbusError::Frame { .. } => 0x03,
            _ => 0x04,
        }
    }

    /// Handle read coils (0x01)
    async fn handle_read_01(
        data: &[u8],
        register_bank: &Arc<ModbusRegisterBank>,
    ) -> ModbusResult<Vec<u8>> {
        if data.len() < 4 {
            return Err(ModbusError::frame("Invalid read coils request"));
        }

        let address = u16::from_be_bytes([data[0], data[1]]);
        let quantity = u16::from_be_bytes([data[2], data[3]]);
        if quantity == 0 || usize::from(quantity) > MAX_READ_COILS {
            return Err(ModbusError::invalid_data("Invalid read coils quantity"));
        }

        let coils = register_bank.read_01(address, quantity)?;

        // Pack coils into bytes
        let byte_count = (quantity as usize).div_ceil(8);
        // Pre-allocate: function_code(1) + byte_count(1) + data(byte_count)
        let mut response = Vec::with_capacity(2 + byte_count);
        response.push(0x01);
        response.push(
            u8::try_from(byte_count)
                .map_err(|_| ModbusError::invalid_data("Read coils response too large"))?,
        );

        for chunk in coils.chunks(8) {
            let mut byte = 0u8;
            for (i, &coil) in chunk.iter().enumerate() {
                if coil {
                    byte |= 1 << i;
                }
            }
            response.push(byte);
        }

        Ok(response)
    }

    /// Handle read discrete inputs (0x02)
    async fn handle_read_02(
        data: &[u8],
        register_bank: &Arc<ModbusRegisterBank>,
    ) -> ModbusResult<Vec<u8>> {
        if data.len() < 4 {
            return Err(ModbusError::frame("Invalid read discrete inputs request"));
        }

        let address = u16::from_be_bytes([data[0], data[1]]);
        let quantity = u16::from_be_bytes([data[2], data[3]]);
        if quantity == 0 || usize::from(quantity) > MAX_READ_COILS {
            return Err(ModbusError::invalid_data(
                "Invalid read discrete inputs quantity",
            ));
        }

        let inputs = register_bank.read_02(address, quantity)?;

        // Pack inputs into bytes
        let byte_count = (quantity as usize).div_ceil(8);
        // Pre-allocate: function_code(1) + byte_count(1) + data(byte_count)
        let mut response = Vec::with_capacity(2 + byte_count);
        response.push(0x02);
        response.push(
            u8::try_from(byte_count).map_err(|_| {
                ModbusError::invalid_data("Read discrete inputs response too large")
            })?,
        );

        for chunk in inputs.chunks(8) {
            let mut byte = 0u8;
            for (i, &input) in chunk.iter().enumerate() {
                if input {
                    byte |= 1 << i;
                }
            }
            response.push(byte);
        }

        Ok(response)
    }

    /// Handle read holding registers (0x03)
    async fn handle_read_03(
        data: &[u8],
        register_bank: &Arc<ModbusRegisterBank>,
    ) -> ModbusResult<Vec<u8>> {
        if data.len() < 4 {
            return Err(ModbusError::frame("invalid frame"));
        }

        let address = u16::from_be_bytes([data[0], data[1]]);
        let quantity = u16::from_be_bytes([data[2], data[3]]);
        if quantity == 0 || usize::from(quantity) > MAX_READ_REGISTERS {
            return Err(ModbusError::invalid_data(
                "Invalid read holding registers quantity",
            ));
        }

        let registers = register_bank.read_03(address, quantity)?;

        // Pre-allocate: function_code(1) + byte_count(1) + data(quantity*2)
        let data_len = (quantity as usize) * 2;
        let mut response = Vec::with_capacity(2 + data_len);
        response.push(0x03);
        response.push(
            u8::try_from(data_len).map_err(|_| {
                ModbusError::invalid_data("Read holding registers response too large")
            })?,
        );
        for &register in &registers {
            response.extend_from_slice(&register.to_be_bytes());
        }

        Ok(response)
    }

    /// Handle read input registers (0x04)
    async fn handle_read_04(
        data: &[u8],
        register_bank: &Arc<ModbusRegisterBank>,
    ) -> ModbusResult<Vec<u8>> {
        if data.len() < 4 {
            return Err(ModbusError::frame("invalid frame"));
        }

        let address = u16::from_be_bytes([data[0], data[1]]);
        let quantity = u16::from_be_bytes([data[2], data[3]]);
        if quantity == 0 || usize::from(quantity) > MAX_READ_REGISTERS {
            return Err(ModbusError::invalid_data(
                "Invalid read input registers quantity",
            ));
        }

        let registers = register_bank.read_04(address, quantity)?;

        // Pre-allocate: function_code(1) + byte_count(1) + data(quantity*2)
        let data_len = (quantity as usize) * 2;
        let mut response = Vec::with_capacity(2 + data_len);
        response.push(0x04);
        response.push(
            u8::try_from(data_len).map_err(|_| {
                ModbusError::invalid_data("Read input registers response too large")
            })?,
        );
        for &register in &registers {
            response.extend_from_slice(&register.to_be_bytes());
        }

        Ok(response)
    }

    /// Handle write single coil (0x05)
    async fn handle_write_05(
        data: &[u8],
        register_bank: &Arc<ModbusRegisterBank>,
    ) -> ModbusResult<Vec<u8>> {
        if data.len() < 4 {
            return Err(ModbusError::frame("invalid frame"));
        }

        let address = u16::from_be_bytes([data[0], data[1]]);
        let value_bytes = u16::from_be_bytes([data[2], data[3]]);
        if value_bytes != 0xFF00 && value_bytes != 0x0000 {
            return Err(ModbusError::invalid_data("Invalid coil write value"));
        }
        let coil_value = value_bytes == 0xFF00;

        register_bank.write_05(address, coil_value)?;

        let mut response = vec![0x05];
        response.extend_from_slice(&address.to_be_bytes());
        response.extend_from_slice(&value_bytes.to_be_bytes());
        Ok(response)
    }

    /// Handle write single register (0x06)
    async fn handle_write_06(
        data: &[u8],
        register_bank: &Arc<ModbusRegisterBank>,
    ) -> ModbusResult<Vec<u8>> {
        if data.len() < 4 {
            return Err(ModbusError::frame("invalid frame"));
        }

        let address = u16::from_be_bytes([data[0], data[1]]);
        let value = u16::from_be_bytes([data[2], data[3]]);

        register_bank.write_06(address, value)?;

        let mut response = vec![0x06];
        response.extend_from_slice(&address.to_be_bytes());
        response.extend_from_slice(&value.to_be_bytes());
        Ok(response)
    }

    /// Handle write multiple coils (0x0F)
    async fn handle_write_0f(
        data: &[u8],
        register_bank: &Arc<ModbusRegisterBank>,
    ) -> ModbusResult<Vec<u8>> {
        if data.len() < 5 {
            return Err(ModbusError::frame("invalid frame"));
        }

        let address = u16::from_be_bytes([data[0], data[1]]);
        let quantity = u16::from_be_bytes([data[2], data[3]]);
        let byte_count = data[4] as usize;
        if quantity == 0 || usize::from(quantity) > MAX_WRITE_COILS {
            return Err(ModbusError::invalid_data(
                "Invalid write multiple coils quantity",
            ));
        }
        if byte_count != usize::from(quantity).div_ceil(8) {
            return Err(ModbusError::frame("invalid frame"));
        }

        if data.len() < 5 + byte_count {
            return Err(ModbusError::frame("invalid frame"));
        }

        // Pre-allocate coils vector
        let mut coils = Vec::with_capacity(quantity as usize);
        for i in 0..quantity {
            let byte_index = 5 + (i / 8) as usize;
            let bit_index = i % 8;
            let bit_value = (data[byte_index] & (1 << bit_index)) != 0;
            coils.push(bit_value);
        }

        register_bank.write_0f(address, &coils)?;

        // Pre-allocate response: function_code(1) + address(2) + quantity(2) = 5
        let mut response = Vec::with_capacity(5);
        response.push(0x0F);
        response.extend_from_slice(&address.to_be_bytes());
        response.extend_from_slice(&quantity.to_be_bytes());
        Ok(response)
    }

    /// Handle write multiple registers (0x10)
    async fn handle_write_10(
        data: &[u8],
        register_bank: &Arc<ModbusRegisterBank>,
    ) -> ModbusResult<Vec<u8>> {
        if data.len() < 5 {
            return Err(ModbusError::frame("invalid frame"));
        }

        let address = u16::from_be_bytes([data[0], data[1]]);
        let quantity = u16::from_be_bytes([data[2], data[3]]);
        let byte_count = data[4] as usize;
        if quantity == 0 || usize::from(quantity) > MAX_WRITE_REGISTERS {
            return Err(ModbusError::invalid_data(
                "Invalid write multiple registers quantity",
            ));
        }

        if data.len() < 5 + byte_count || byte_count != (quantity as usize * 2) {
            return Err(ModbusError::frame("invalid frame"));
        }

        // Pre-allocate registers vector
        let mut registers = Vec::with_capacity(quantity as usize);
        for i in 0..quantity {
            let byte_offset = 5 + (i as usize * 2);
            let value = u16::from_be_bytes([data[byte_offset], data[byte_offset + 1]]);
            registers.push(value);
        }

        register_bank.write_10(address, &registers)?;

        // Pre-allocate response: function_code(1) + address(2) + quantity(2) = 5
        let mut response = Vec::with_capacity(5);
        response.push(0x10);
        response.extend_from_slice(&address.to_be_bytes());
        response.extend_from_slice(&quantity.to_be_bytes());
        Ok(response)
    }

    /// Create error response
    fn create_error_response(request: &[u8], exception_code: u8) -> ModbusResult<Vec<u8>> {
        if request.len() < MBAP_HEADER_SIZE + 2 {
            return Err(ModbusError::frame("Request too short for error response"));
        }

        let transaction_id = u16::from_be_bytes([request[0], request[1]]);
        let protocol_id = 0u16;
        let length = 3u16; // unit_id + function_code + exception_code
        let unit_id = request[6];
        let function_code = request[7] | 0x80; // Set exception bit

        let mut response = Vec::with_capacity(MBAP_HEADER_SIZE + 3);

        // MBAP header
        response.extend_from_slice(&transaction_id.to_be_bytes());
        response.extend_from_slice(&protocol_id.to_be_bytes());
        response.extend_from_slice(&length.to_be_bytes());

        // Exception PDU
        response.push(unit_id);
        response.push(function_code);
        response.push(exception_code);

        Ok(response)
    }
}

impl ModbusServer for ModbusTcpServer {
    async fn start(&mut self) -> ModbusResult<()> {
        if self.is_running.load(Ordering::Relaxed) {
            return Err(ModbusError::protocol("Server is already running"));
        }

        info!(
            "🚀 Starting Modbus TCP server on {}",
            self.config.bind_address
        );

        let listener = TcpListener::bind(self.config.bind_address)
            .await
            .map_err(|e| {
                ModbusError::connection(format!(
                    "Failed to bind to {}: {}",
                    self.config.bind_address, e
                ))
            })?;

        let (shutdown_tx, _) = broadcast::channel(1);
        self.shutdown_tx = Some(shutdown_tx.clone());
        self.start_time = Some(std::time::Instant::now());

        info!("✅ Modbus TCP server started successfully");
        info!("📊 Server configuration:");
        info!("   - Bind address: {}", self.config.bind_address);
        info!("   - Max connections: {}", self.config.max_connections);
        info!("   - Request timeout: {:?}", self.config.request_timeout);

        let register_bank = self.register_bank.clone();
        let stats = self.stats.clone();
        let request_timeout = self.config.request_timeout;
        let connection_limit = Arc::new(Semaphore::new(self.config.max_connections));
        let is_running_flag = self.is_running.clone();
        let mut shutdown_rx = shutdown_tx.subscribe();

        // Set is_running to true only after successfully binding and before starting the listen loop
        self.is_running.store(true, Ordering::Relaxed);

        tokio::spawn(async move {
            loop {
                tokio::select! {
                    result = listener.accept() => {
                        match result {
                            Ok((stream, addr)) => {
                                debug!("Accepted connection from {}", addr);
                                let permit = match connection_limit.clone().try_acquire_owned() {
                                    Ok(permit) => permit,
                                    Err(_) => {
                                        warn!("Rejecting {}: max connections reached", addr);
                                        continue;
                                    }
                                };

                                let register_bank = register_bank.clone();
                                let stats = stats.clone();
                                let shutdown_rx = shutdown_tx.subscribe();

                                tokio::spawn(async move {
                                    let _permit = permit;
                                    Self::handle_client(stream, register_bank, stats, shutdown_rx, request_timeout).await;
                                });
                            }
                            Err(e) => {
                                error!("Failed to accept connection: {}", e);
                            }
                        }
                    }
                    _ = shutdown_rx.recv() => {
                        info!("Shutdown signal received, stopping server");
                        break;
                    }
                }
            }

            is_running_flag.store(false, Ordering::Relaxed);
        });

        Ok(())
    }

    async fn stop(&mut self) -> ModbusResult<()> {
        if let Some(shutdown_tx) = &self.shutdown_tx {
            let _ = shutdown_tx.send(());
        }

        self.is_running.store(false, Ordering::Relaxed);

        info!("⏹️  Modbus TCP server stopped");
        Ok(())
    }

    fn is_running(&self) -> bool {
        self.is_running.load(Ordering::Relaxed)
    }

    fn get_stats(&self) -> ServerStats {
        let mut stats = self
            .stats
            .lock()
            .map(|stats| stats.clone())
            .unwrap_or_default();

        if let Some(start_time) = self.start_time {
            stats.uptime_seconds = start_time.elapsed().as_secs();
        }

        stats.register_bank_stats = Some(self.register_bank.get_stats());
        stats
    }

    fn get_register_bank(&self) -> Option<Arc<ModbusRegisterBank>> {
        Some(self.register_bank.clone())
    }
}

/// Modbus RTU server configuration
#[cfg(feature = "rtu")]
#[derive(Debug, Clone)]
pub struct ModbusRtuServerConfig {
    pub port: String,
    pub baud_rate: u32,
    pub data_bits: tokio_serial::DataBits,
    pub stop_bits: tokio_serial::StopBits,
    pub parity: tokio_serial::Parity,
    pub timeout: Duration,
    pub frame_gap: Duration,
    pub register_bank: Option<Arc<ModbusRegisterBank>>,
}

#[cfg(feature = "rtu")]
impl Default for ModbusRtuServerConfig {
    fn default() -> Self {
        Self {
            port: "/dev/ttyUSB0".to_string(),
            baud_rate: 9600,
            data_bits: tokio_serial::DataBits::Eight,
            stop_bits: tokio_serial::StopBits::One,
            parity: tokio_serial::Parity::None,
            timeout: Duration::from_secs(1),
            frame_gap: Duration::from_millis(4), // Default 3.5 char time at 9600 baud
            register_bank: None,
        }
    }
}

/// Modbus RTU server implementation
#[cfg(feature = "rtu")]
pub struct ModbusRtuServer {
    config: ModbusRtuServerConfig,
    register_bank: Arc<ModbusRegisterBank>,
    stats: Arc<Mutex<ServerStats>>,
    shutdown_tx: Option<broadcast::Sender<()>>,
    is_running: Arc<AtomicBool>,
    start_time: Option<std::time::Instant>,
}

#[cfg(feature = "rtu")]
impl ModbusRtuServer {
    /// Create a new RTU server with default configuration
    pub fn new(port: &str, baud_rate: u32) -> ModbusResult<Self> {
        let config = ModbusRtuServerConfig {
            port: port.to_string(),
            baud_rate,
            ..Default::default()
        };

        Self::with_config(config)
    }

    /// Create a new RTU server with custom configuration
    pub fn with_config(config: ModbusRtuServerConfig) -> ModbusResult<Self> {
        let register_bank = config
            .register_bank
            .clone()
            .unwrap_or_else(|| Arc::new(ModbusRegisterBank::new()));

        Ok(Self {
            config,
            register_bank,
            stats: Arc::new(Mutex::new(ServerStats::default())),
            shutdown_tx: None,
            is_running: Arc::new(AtomicBool::new(false)),
            start_time: None,
        })
    }

    /// Set custom register bank
    pub fn set_register_bank(&mut self, register_bank: Arc<ModbusRegisterBank>) {
        self.register_bank = register_bank;
    }

    /// Calculate CRC for RTU frames
    fn calculate_crc(data: &[u8]) -> u16 {
        use crc::{Crc, CRC_16_MODBUS};
        const CRC_MODBUS: Crc<u16> = Crc::<u16>::new(&CRC_16_MODBUS);
        CRC_MODBUS.checksum(data)
    }

    /// Handle RTU request
    async fn handle_request(&mut self, data: &[u8]) -> ModbusResult<Vec<u8>> {
        if data.len() < 3 {
            return Err(ModbusError::frame("Invalid RTU frame length"));
        }

        let slave_id = data[0];
        let function_code = data[1];
        let pdu_data = &data[2..data.len() - 2]; // Remove CRC

        // Verify slave ID matches (or is broadcast)
        if slave_id != 0 && slave_id != 1 {
            return Err(ModbusError::device_not_responding(slave_id));
        }

        let response_pdu = match function_code {
            0x01 => Self::handle_read_01(pdu_data, &self.register_bank).await?,
            0x02 => Self::handle_read_02(pdu_data, &self.register_bank).await?,
            0x03 => Self::handle_read_03(pdu_data, &self.register_bank).await?,
            0x04 => Self::handle_read_04(pdu_data, &self.register_bank).await?,
            0x05 => Self::handle_write_05(pdu_data, &self.register_bank).await?,
            0x06 => Self::handle_write_06(pdu_data, &self.register_bank).await?,
            0x0F => Self::handle_write_0f(pdu_data, &self.register_bank).await?,
            0x10 => Self::handle_write_10(pdu_data, &self.register_bank).await?,
            _ => {
                return Err(ModbusError::invalid_function(function_code));
            }
        };

        // Build RTU response: slave_id + function_code + response_data
        let mut response = vec![slave_id, function_code];
        response.extend_from_slice(&response_pdu[1..]); // Skip function code from PDU

        Ok(response)
    }

    /// Create RTU error response
    #[allow(dead_code)]
    fn create_rtu_error_response(
        slave_id: u8,
        function_code: u8,
        exception_code: u8,
    ) -> ModbusResult<Vec<u8>> {
        let mut response = Vec::new();
        response.push(slave_id);
        response.push(function_code | 0x80); // Set exception bit
        response.push(exception_code);

        let crc = Self::calculate_crc(&response);
        response.extend_from_slice(&crc.to_le_bytes());

        Ok(response)
    }

    /// Handle RTU communication loop
    async fn handle_rtu_communication(
        mut port: tokio_serial::SerialStream,
        register_bank: Arc<ModbusRegisterBank>,
        stats: Arc<Mutex<ServerStats>>,
        mut shutdown_rx: broadcast::Receiver<()>,
        frame_gap: Duration,
    ) {
        info!("🔌 RTU server communication started");

        let mut buffer = vec![0u8; 256];
        let mut frame_buffer = Vec::new();
        let mut last_activity = std::time::Instant::now();

        loop {
            tokio::select! {
                _ = shutdown_rx.recv() => {
                    debug!("Shutdown signal received for RTU server");
                    break;
                }

                result = tokio::time::timeout(Duration::from_millis(100), port.read(&mut buffer)) => {
                    match result {
                        Ok(Ok(bytes_read)) if bytes_read > 0 => {
                            let now = std::time::Instant::now();

                            // Check for frame gap
                            if now.duration_since(last_activity) > frame_gap && !frame_buffer.is_empty() {
                                // Process accumulated frame
                                Self::process_accumulated_frame(
                                    &frame_buffer,
                                    &mut port,
                                    &register_bank,
                                    &stats
                                ).await;
                                frame_buffer.clear();
                            }

                            // Accumulate data
                            frame_buffer.extend_from_slice(&buffer[..bytes_read]);
                            last_activity = now;

                            // Update stats
                            if let Ok(mut stats) = stats.lock() {
                                stats.bytes_received += bytes_read as u64;
                            }
                        }
                        Ok(Ok(_)) => {
                            // No data read, but successful read operation
                        }
                        Ok(Err(e)) => {
                            error!("RTU read error: {}", e);
                            break;
                        }
                        Err(_) => {
                            // Timeout - check if we have a complete frame
                            let now = std::time::Instant::now();
                            if !frame_buffer.is_empty() && now.duration_since(last_activity) > frame_gap {
                                Self::process_accumulated_frame(
                                    &frame_buffer,
                                    &mut port,
                                    &register_bank,
                                    &stats
                                ).await;
                                frame_buffer.clear();
                            }
                        }
                    }
                }
            }
        }

        info!("🔌 RTU server communication stopped");
    }

    /// Process accumulated frame data
    async fn process_accumulated_frame(
        frame: &[u8],
        port: &mut tokio_serial::SerialStream,
        register_bank: &Arc<ModbusRegisterBank>,
        stats: &Arc<Mutex<ServerStats>>,
    ) {
        // Update request stats
        if let Ok(mut stats) = stats.lock() {
            stats.total_requests += 1;
        }

        // Create a temporary server instance for processing
        let mut temp_server = ModbusRtuServer {
            config: ModbusRtuServerConfig::default(),
            register_bank: register_bank.clone(),
            stats: stats.clone(),
            shutdown_tx: None,
            is_running: Arc::new(AtomicBool::new(false)),
            start_time: None,
        };

        match temp_server.handle_request(frame).await {
            Ok(response) => {
                // Calculate CRC for response
                let mut response_with_crc = response;
                let crc = Self::calculate_crc(&response_with_crc);
                response_with_crc.extend_from_slice(&crc.to_le_bytes());

                if let Err(e) = port.write_all(&response_with_crc).await {
                    error!("Failed to write response: {}", e);
                    if let Ok(mut stats) = stats.lock() {
                        stats.failed_requests += 1;
                    }
                } else if let Ok(mut stats) = stats.lock() {
                    stats.successful_requests += 1;
                    stats.bytes_sent += response_with_crc.len() as u64;
                }
            }
            Err(e) => {
                error!("Error processing request: {}", e);
                if let Ok(mut stats) = stats.lock() {
                    stats.failed_requests += 1;
                }
                // Send exception response if needed
            }
        }
    }

    // Reuse the same handler methods from TCP server
    async fn handle_read_01(
        data: &[u8],
        register_bank: &Arc<ModbusRegisterBank>,
    ) -> ModbusResult<Vec<u8>> {
        ModbusTcpServer::handle_read_01(data, register_bank).await
    }

    async fn handle_read_02(
        data: &[u8],
        register_bank: &Arc<ModbusRegisterBank>,
    ) -> ModbusResult<Vec<u8>> {
        ModbusTcpServer::handle_read_02(data, register_bank).await
    }

    async fn handle_read_03(
        data: &[u8],
        register_bank: &Arc<ModbusRegisterBank>,
    ) -> ModbusResult<Vec<u8>> {
        ModbusTcpServer::handle_read_03(data, register_bank).await
    }

    async fn handle_read_04(
        data: &[u8],
        register_bank: &Arc<ModbusRegisterBank>,
    ) -> ModbusResult<Vec<u8>> {
        ModbusTcpServer::handle_read_04(data, register_bank).await
    }

    async fn handle_write_05(
        data: &[u8],
        register_bank: &Arc<ModbusRegisterBank>,
    ) -> ModbusResult<Vec<u8>> {
        ModbusTcpServer::handle_write_05(data, register_bank).await
    }

    async fn handle_write_06(
        data: &[u8],
        register_bank: &Arc<ModbusRegisterBank>,
    ) -> ModbusResult<Vec<u8>> {
        ModbusTcpServer::handle_write_06(data, register_bank).await
    }

    async fn handle_write_0f(
        data: &[u8],
        register_bank: &Arc<ModbusRegisterBank>,
    ) -> ModbusResult<Vec<u8>> {
        ModbusTcpServer::handle_write_0f(data, register_bank).await
    }

    async fn handle_write_10(
        data: &[u8],
        register_bank: &Arc<ModbusRegisterBank>,
    ) -> ModbusResult<Vec<u8>> {
        ModbusTcpServer::handle_write_10(data, register_bank).await
    }
}

/// Modbus RTU server implementation
///
/// Note: This is a placeholder for future implementation
#[cfg(feature = "rtu")]
impl ModbusServer for ModbusRtuServer {
    async fn start(&mut self) -> ModbusResult<()> {
        if self.is_running.load(Ordering::Relaxed) {
            return Err(ModbusError::protocol("RTU Server is already running"));
        }

        info!("🚀 Starting Modbus RTU server on {}", self.config.port);

        // Create serial port connection
        let port = tokio_serial::SerialStream::open(
            &tokio_serial::new(&self.config.port, self.config.baud_rate)
                .data_bits(self.config.data_bits)
                .stop_bits(self.config.stop_bits)
                .parity(self.config.parity)
                .timeout(self.config.timeout),
        )
        .map_err(|e| {
            ModbusError::connection(format!(
                "Failed to open serial port {}: {}",
                self.config.port, e
            ))
        })?;

        let (shutdown_tx, _) = broadcast::channel(1);
        self.shutdown_tx = Some(shutdown_tx.clone());
        self.start_time = Some(std::time::Instant::now());

        self.is_running.store(true, Ordering::Relaxed);

        info!("✅ Modbus RTU server started successfully");
        info!("📊 Server configuration:");
        info!("   - Port: {}", self.config.port);
        info!("   - Baud rate: {}", self.config.baud_rate);
        info!("   - Data bits: {:?}", self.config.data_bits);
        info!("   - Stop bits: {:?}", self.config.stop_bits);
        info!("   - Parity: {:?}", self.config.parity);
        info!("   - Timeout: {:?}", self.config.timeout);

        let register_bank = self.register_bank.clone();
        let stats = self.stats.clone();
        let frame_gap = self.config.frame_gap;
        let is_running_flag = self.is_running.clone();
        let shutdown_rx = shutdown_tx.subscribe();

        tokio::spawn(async move {
            Self::handle_rtu_communication(port, register_bank, stats, shutdown_rx, frame_gap)
                .await;

            is_running_flag.store(false, Ordering::Relaxed);
        });

        Ok(())
    }

    async fn stop(&mut self) -> ModbusResult<()> {
        if let Some(shutdown_tx) = &self.shutdown_tx {
            let _ = shutdown_tx.send(());
        }

        self.is_running.store(false, Ordering::Relaxed);

        info!("⏹️  Modbus RTU server stopped");
        Ok(())
    }

    fn is_running(&self) -> bool {
        self.is_running.load(Ordering::Relaxed)
    }

    fn get_stats(&self) -> ServerStats {
        let mut stats = self
            .stats
            .lock()
            .map(|stats| stats.clone())
            .unwrap_or_default();

        if let Some(start_time) = self.start_time {
            stats.uptime_seconds = start_time.elapsed().as_secs();
        }

        stats.register_bank_stats = Some(self.register_bank.get_stats());
        stats
    }

    fn get_register_bank(&self) -> Option<Arc<ModbusRegisterBank>> {
        Some(self.register_bank.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(feature = "rtu")]
    use std::time::Duration;

    #[test]
    fn test_tcp_server_creation() {
        // Test TCP server creation
        let result = ModbusTcpServer::new("127.0.0.1:5020");
        assert!(result.is_ok());

        let server = result.unwrap();
        assert!(!server.is_running());
        assert!(server.get_register_bank().is_some());
    }

    #[tokio::test]
    async fn test_tcp_handle_request_returns_complete_mbap_frame() {
        let register_bank = Arc::new(ModbusRegisterBank::new());
        register_bank.write_06(0, 0x1234).unwrap();

        let request = [
            0x12, 0x34, // transaction id
            0x00, 0x00, // protocol id
            0x00, 0x06, // length: unit + fc + address + quantity
            0x01, // unit id
            0x03, // read holding registers
            0x00, 0x00, // address
            0x00, 0x01, // quantity
        ];

        let response = ModbusTcpServer::handle_request(&request, &register_bank)
            .await
            .unwrap();

        assert_eq!(
            response,
            vec![
                0x12, 0x34, // transaction id is preserved
                0x00, 0x00, // protocol id
                0x00, 0x05, // length: unit + fc + byte_count + two data bytes
                0x01, // unit id
                0x03, // function
                0x02, // byte count
                0x12, 0x34,
            ]
        );
    }

    #[cfg(feature = "rtu")]
    #[test]
    fn test_rtu_server_creation() {
        // Test RTU server creation
        let result = ModbusRtuServer::new("/dev/ttyUSB0", 9600);
        assert!(result.is_ok());

        let server = result.unwrap();
        assert!(!server.is_running());
        assert!(server.get_register_bank().is_some());
    }

    #[cfg(feature = "rtu")]
    #[test]
    fn test_rtu_server_configuration() {
        // Test RTU server with custom configuration
        let config = ModbusRtuServerConfig {
            port: "/dev/ttyUSB0".to_string(),
            baud_rate: 19200,
            data_bits: tokio_serial::DataBits::Eight,
            stop_bits: tokio_serial::StopBits::Two,
            parity: tokio_serial::Parity::Even,
            timeout: Duration::from_secs(2),
            frame_gap: Duration::from_millis(5),
            register_bank: None,
        };

        let result = ModbusRtuServer::with_config(config);
        assert!(result.is_ok());

        let server = result.unwrap();
        assert!(!server.is_running());
    }

    #[cfg(feature = "rtu")]
    #[tokio::test]
    async fn test_rtu_server_lifecycle() {
        // Test RTU server start/stop lifecycle
        let mut server = ModbusRtuServer::new("/dev/ttyUSB0", 9600).unwrap();

        // Server should not be running initially
        assert!(!server.is_running());

        // Try to start server (will fail without actual serial port)
        let start_result = server.start().await;

        if start_result.is_ok() {
            // If start succeeded (unlikely without hardware), test stop
            tokio::time::sleep(Duration::from_millis(10)).await;
            let stop_result = server.stop().await;
            assert!(stop_result.is_ok());
        } else {
            // Expected to fail without actual hardware
            println!(
                "RTU server start failed (expected without serial port): {:?}",
                start_result.err()
            );
        }
    }

    #[cfg(feature = "rtu")]
    #[test]
    fn test_crc_calculation() {
        // Test CRC calculation function
        let test_data = vec![0x01, 0x03, 0x00, 0x00, 0x00, 0x02];
        let crc = ModbusRtuServer::calculate_crc(&test_data);

        // CRC should be consistent
        assert_eq!(crc, ModbusRtuServer::calculate_crc(&test_data));

        // Different data should give different CRC
        let test_data2 = vec![0x01, 0x04, 0x00, 0x00, 0x00, 0x01];
        let crc2 = ModbusRtuServer::calculate_crc(&test_data2);
        assert_ne!(crc, crc2);
    }

    #[cfg(feature = "rtu")]
    #[test]
    fn test_rtu_error_response() {
        // Test RTU error response creation
        let result = ModbusRtuServer::create_rtu_error_response(0x01, 0x03, 0x01);
        assert!(result.is_ok());

        let response = result.unwrap();
        assert_eq!(response[0], 0x01); // Slave ID
        assert_eq!(response[1], 0x83); // Function code with error bit
        assert_eq!(response[2], 0x01); // Exception code
        assert_eq!(response.len(), 5); // Slave + Function + Exception + CRC (2 bytes)
    }

    #[tokio::test]
    async fn test_server_stats() {
        // Test server statistics
        let server = ModbusTcpServer::new("127.0.0.1:5021").unwrap();
        let stats = server.get_stats();

        // Initial stats should be zero
        assert_eq!(stats.connections_count, 0);
        assert_eq!(stats.total_requests, 0);
        assert_eq!(stats.successful_requests, 0);
        assert_eq!(stats.failed_requests, 0);
    }

    #[test]
    fn test_register_bank_integration() {
        // Test server with custom register bank
        let register_bank = Arc::new(ModbusRegisterBank::new());

        // Set some test values
        register_bank.write_05(0, true).unwrap();
        register_bank.write_06(0, 0x1234).unwrap();

        let mut server = ModbusTcpServer::new("127.0.0.1:5022").unwrap();
        server.set_register_bank(register_bank.clone());

        // Verify register bank is set
        let server_bank = server.get_register_bank().unwrap();
        let coils = server_bank.read_coils(0, 1).unwrap();
        let registers = server_bank.read_holding_registers(0, 1).unwrap();

        assert!(coils[0]);
        assert_eq!(registers[0], 0x1234);
    }

    #[tokio::test]
    async fn test_register_operations() {
        let register_bank = Arc::new(ModbusRegisterBank::new());

        // Test write operations
        register_bank.write_05(0, true).unwrap();
        register_bank.write_06(0, 0x1234).unwrap();

        // Test read operations
        let coils = register_bank.read_coils(0, 1).unwrap();
        assert_eq!(coils, vec![true]);

        let registers = register_bank.read_holding_registers(0, 1).unwrap();
        assert_eq!(registers, vec![0x1234]);
    }
}
