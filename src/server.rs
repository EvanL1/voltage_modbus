//! Modbus server implementations
//!
//! This module provides complete server-side implementations for both TCP and RTU protocols.

use std::future::Future;
use std::net::SocketAddr;
use std::pin::Pin;
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

use crate::constants::{
    MAX_READ_COILS, MAX_READ_REGISTERS, MAX_RW_READ_REGISTERS, MAX_RW_WRITE_REGISTERS,
    MAX_WRITE_COILS, MAX_WRITE_REGISTERS,
};
use crate::error::{ModbusError, ModbusResult};

use crate::register_bank::{ModbusRegisterBank, RegisterBankStats};

/// Maximum frame size for Modbus TCP
const MAX_TCP_FRAME_SIZE: usize = 260;

/// MBAP header size
const MBAP_HEADER_SIZE: usize = 6;

/// Pause after a failed `accept()` so persistent errors (e.g. EMFILE when out
/// of file descriptors) don't spin the accept loop at 100% CPU.
const ACCEPT_ERROR_BACKOFF: Duration = Duration::from_millis(100);

mod sealed {
    pub trait Sealed {}
    impl Sealed for super::ModbusTcpServer {}
    #[cfg(feature = "rtu")]
    impl Sealed for super::ModbusRtuServer {}
}

/// Modbus server trait
///
/// Sealed: implemented only by this crate's servers, so methods can be added
/// without a breaking change. Customize behavior through [`ModbusService`].
pub trait ModbusServer: sealed::Sealed + Send + Sync {
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

/// Boxed future returned by [`ModbusService::handle_pdu`]
pub type ServiceFuture<'a> = Pin<Box<dyn Future<Output = ModbusResult<Vec<u8>>> + Send + 'a>>;

/// Server-side PDU handler.
///
/// The TCP and RTU servers strip the transport framing (MBAP header, or
/// slave id + CRC) off each request and delegate the raw PDU here. Implement
/// this trait to back a server with your own data source — live sensor
/// values, a downstream gateway, a simulation — instead of static storage.
/// [`ModbusRegisterBank`] provides the default in-memory implementation.
///
/// Return the response PDU starting with the function code (no unit id, MBAP
/// header or CRC — the server adds the framing). Returning `Err` makes the
/// server reply with the matching Modbus exception, e.g.
/// `ModbusError::InvalidAddress` → exception 0x02, `InvalidFunction` → 0x01.
///
/// # Example
///
/// ```rust,no_run
/// use std::sync::Arc;
/// use voltage_modbus::server::{ModbusService, ModbusTcpServer, ServiceFuture};
/// use voltage_modbus::ModbusError;
///
/// struct LiveSensors;
///
/// impl ModbusService for LiveSensors {
///     fn handle_pdu<'a>(&'a self, function_code: u8, data: &'a [u8]) -> ServiceFuture<'a> {
///         Box::pin(async move {
///             match function_code {
///                 // FC04: answer every input-register read with one value
///                 0x04 => Ok(vec![0x04, 0x02, 0x12, 0x34]),
///                 other => Err(ModbusError::invalid_function(other)),
///             }
///         })
///     }
/// }
///
/// # fn example() -> voltage_modbus::ModbusResult<()> {
/// let mut server = ModbusTcpServer::new("0.0.0.0:502")?;
/// server.set_service(Arc::new(LiveSensors));
/// # Ok(())
/// # }
/// ```
pub trait ModbusService: Send + Sync {
    /// Handle one request PDU (`function_code` + payload `data`)
    fn handle_pdu<'a>(&'a self, function_code: u8, data: &'a [u8]) -> ServiceFuture<'a>;
}

/// Default service: dispatch onto the in-memory register bank
impl ModbusService for ModbusRegisterBank {
    fn handle_pdu<'a>(&'a self, function_code: u8, data: &'a [u8]) -> ServiceFuture<'a> {
        Box::pin(async move {
            match function_code {
                0x01 => ModbusTcpServer::handle_read_01(data, self).await,
                0x02 => ModbusTcpServer::handle_read_02(data, self).await,
                0x03 => ModbusTcpServer::handle_read_03(data, self).await,
                0x04 => ModbusTcpServer::handle_read_04(data, self).await,
                0x05 => ModbusTcpServer::handle_write_05(data, self).await,
                0x06 => ModbusTcpServer::handle_write_06(data, self).await,
                0x0F => ModbusTcpServer::handle_write_0f(data, self).await,
                0x10 => ModbusTcpServer::handle_write_10(data, self).await,
                0x16 => ModbusTcpServer::handle_write_16(data, self).await,
                0x17 => ModbusTcpServer::handle_read_write_17(data, self).await,
                // Diagnostics: sub-function 0x0000 (Return Query Data) echo test
                0x08 if data.len() >= 4 && data[0] == 0 && data[1] == 0 => {
                    let mut response = Vec::with_capacity(1 + data.len());
                    response.push(0x08);
                    response.extend_from_slice(data);
                    Ok(response)
                }
                _ => {
                    warn!("Unsupported function code: 0x{:02X}", function_code);
                    Err(ModbusError::invalid_function(function_code))
                }
            }
        })
    }
}

/// Device identification objects served for FC 0x2B / MEI 0x0E requests.
///
/// Install on a server with [`ModbusTcpServer::set_device_identity`] /
/// `ModbusRtuServer::set_device_identity` (`rtu` feature); FC 0x2B requests are then
/// answered from these objects while all other function codes continue to
/// flow to the active [`ModbusService`].
#[derive(Debug, Clone, Default)]
pub struct DeviceIdentity {
    /// `(object id, value)` pairs, kept sorted by id
    objects: Vec<(u8, Vec<u8>)>,
}

impl DeviceIdentity {
    /// Basic identity from the three mandatory objects:
    /// VendorName (0x00), ProductCode (0x01), MajorMinorRevision (0x02)
    pub fn basic(vendor_name: &str, product_code: &str, revision: &str) -> Self {
        Self::default()
            .with_object(0x00, vendor_name.as_bytes())
            .with_object(0x01, product_code.as_bytes())
            .with_object(0x02, revision.as_bytes())
    }

    /// Add or replace an object (values longer than 255 bytes are truncated,
    /// as object lengths are encoded in one byte)
    pub fn with_object(mut self, id: u8, value: &[u8]) -> Self {
        let value = value[..value.len().min(255)].to_vec();
        match self
            .objects
            .binary_search_by_key(&id, |(obj_id, _)| *obj_id)
        {
            Ok(pos) => self.objects[pos].1 = value,
            Err(pos) => self.objects.insert(pos, (id, value)),
        }
        self
    }

    /// Conformity level: basic/regular/extended stream, plus the individual
    /// access bit (0x80) — code 4 single-object reads are always supported
    fn conformity_level(&self) -> u8 {
        let max_id = self.objects.last().map(|(id, _)| *id).unwrap_or(0);
        let stream_level = if max_id <= 0x02 {
            0x01
        } else if max_id <= 0x06 {
            0x02
        } else {
            0x03
        };
        stream_level | 0x80
    }

    /// Thin public wrapper around [`Self::handle_request`] for fuzz testing.
    ///
    /// Only compiled under `cfg(fuzzing)` (set by cargo-fuzz) and for tests.
    #[cfg(any(fuzzing, test))]
    #[doc(hidden)]
    pub fn handle_request_fuzz(&self, data: &[u8]) -> ModbusResult<Vec<u8>> {
        self.handle_request(data)
    }

    /// Handle an FC 0x2B request payload (bytes after the function code).
    fn handle_request(&self, data: &[u8]) -> ModbusResult<Vec<u8>> {
        use crate::constants::{MAX_PDU_SIZE, MEI_READ_DEVICE_ID};

        if data.len() != 3 || data[0] != MEI_READ_DEVICE_ID {
            return Err(ModbusError::invalid_data(
                "Invalid device identification request",
            ));
        }
        let read_code = data[1];
        let object_id = data[2];

        let selected: Vec<&(u8, Vec<u8>)> = match read_code {
            1..=3 => {
                let max_id = match read_code {
                    1 => 0x02,
                    2 => 0x06,
                    _ => 0xFF,
                };
                // Stream read starts at object_id; per spec, restart from the
                // first object when the requested id is not present
                let start = if self.objects.iter().any(|(id, _)| *id == object_id) {
                    object_id
                } else {
                    0
                };
                self.objects
                    .iter()
                    .filter(|(id, _)| *id >= start && *id <= max_id)
                    .collect()
            }
            4 => match self.objects.iter().find(|(id, _)| *id == object_id) {
                Some(object) => vec![object],
                // Unknown object id → exception 0x02 (Illegal Data Address)
                None => return Err(ModbusError::invalid_address(u16::from(object_id), 1)),
            },
            _ => {
                return Err(ModbusError::invalid_data(
                    "Invalid ReadDeviceId code (must be 1-4)",
                ))
            }
        };

        // Header: fc, mei, read code, conformity, more, next id, count
        let mut response = vec![
            0x2B,
            MEI_READ_DEVICE_ID,
            read_code,
            self.conformity_level(),
            0x00, // more follows — patched below
            0x00, // next object id — patched below
            0x00, // object count — patched below
        ];

        let mut count: u8 = 0;
        for (id, value) in selected {
            if response.len() + 2 + value.len() > MAX_PDU_SIZE {
                response[4] = 0xFF; // more follows
                response[5] = *id; // continue from this object
                break;
            }
            response.push(*id);
            response.push(value.len() as u8);
            response.extend_from_slice(value);
            count += 1;
        }
        response[6] = count;

        Ok(response)
    }
}

/// Service wrapper installed by `set_device_identity`: answers FC 0x2B from
/// the configured identity, forwards everything else to the inner service.
struct WithDeviceIdentity {
    inner: Arc<dyn ModbusService>,
    identity: Arc<DeviceIdentity>,
}

impl ModbusService for WithDeviceIdentity {
    fn handle_pdu<'a>(&'a self, function_code: u8, data: &'a [u8]) -> ServiceFuture<'a> {
        if function_code == 0x2B {
            let result = self.identity.handle_request(data);
            Box::pin(async move { result })
        } else {
            self.inner.handle_pdu(function_code, data)
        }
    }
}

/// Compose the service handed to the accept/serve loop: wrap with the device
/// identity responder when one is configured.
fn effective_service(
    service: &Arc<dyn ModbusService>,
    identity: &Option<Arc<DeviceIdentity>>,
) -> Arc<dyn ModbusService> {
    match identity {
        Some(identity) => Arc::new(WithDeviceIdentity {
            inner: service.clone(),
            identity: identity.clone(),
        }),
        None => service.clone(),
    }
}

/// Server statistics
#[derive(Debug, Clone, Default)]
#[non_exhaustive]
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
#[non_exhaustive]
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
    /// Request handler; defaults to the register bank itself
    service: Arc<dyn ModbusService>,
    /// FC 0x2B responder, when configured
    device_identity: Option<Arc<DeviceIdentity>>,
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
            service: register_bank.clone(),
            register_bank,
            device_identity: None,
            stats: Arc::new(Mutex::new(ServerStats::default())),
            shutdown_tx: None,
            is_running: Arc::new(AtomicBool::new(false)),
            start_time: None,
        })
    }

    /// Set custom register bank (also makes it the active request handler)
    pub fn set_register_bank(&mut self, register_bank: Arc<ModbusRegisterBank>) {
        self.service = register_bank.clone();
        self.register_bank = register_bank;
    }

    /// Back this server with a custom [`ModbusService`] instead of the
    /// default in-memory register bank.
    ///
    /// Takes effect for connections accepted after the next [`Self::start`].
    /// [`Self::get_register_bank`] keeps returning the default bank, which is
    /// no longer consulted while a custom service is installed.
    pub fn set_service(&mut self, service: Arc<dyn ModbusService>) {
        self.service = service;
    }

    /// Serve FC 0x2B (Read Device Identification) from these objects.
    /// Takes effect from the next [`Self::start`].
    pub fn set_device_identity(&mut self, identity: DeviceIdentity) {
        self.device_identity = Some(Arc::new(identity));
    }

    /// Handle client connection
    async fn handle_client(
        mut stream: TcpStream,
        service: Arc<dyn ModbusService>,
        stats: Arc<Mutex<ServerStats>>,
        mut shutdown_rx: broadcast::Receiver<()>,
        request_timeout: Duration,
    ) {
        let peer_addr = stream
            .peer_addr()
            .map(|addr| addr.to_string())
            .unwrap_or_else(|_| "unknown".to_string());
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
                            match Self::handle_request(&frame, service.as_ref()).await {
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
    async fn handle_request(data: &[u8], service: &dyn ModbusService) -> ModbusResult<Vec<u8>> {
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

        let pdu_response = service.handle_pdu(function_code, pdu_data).await?;

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
        register_bank: &ModbusRegisterBank,
    ) -> ModbusResult<Vec<u8>> {
        if data.len() != 4 {
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
        register_bank: &ModbusRegisterBank,
    ) -> ModbusResult<Vec<u8>> {
        if data.len() != 4 {
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
        register_bank: &ModbusRegisterBank,
    ) -> ModbusResult<Vec<u8>> {
        if data.len() != 4 {
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
        register_bank: &ModbusRegisterBank,
    ) -> ModbusResult<Vec<u8>> {
        if data.len() != 4 {
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
        register_bank: &ModbusRegisterBank,
    ) -> ModbusResult<Vec<u8>> {
        if data.len() != 4 {
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
        register_bank: &ModbusRegisterBank,
    ) -> ModbusResult<Vec<u8>> {
        if data.len() != 4 {
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
        register_bank: &ModbusRegisterBank,
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

        if data.len() != 5 + byte_count {
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
        register_bank: &ModbusRegisterBank,
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

        if data.len() != 5 + byte_count || byte_count != (quantity as usize * 2) {
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

    /// Handle mask write register (0x16)
    ///
    /// `result = (current & and_mask) | (or_mask & !and_mask)`
    async fn handle_write_16(
        data: &[u8],
        register_bank: &ModbusRegisterBank,
    ) -> ModbusResult<Vec<u8>> {
        if data.len() != 6 {
            return Err(ModbusError::frame("invalid frame"));
        }

        let address = u16::from_be_bytes([data[0], data[1]]);
        let and_mask = u16::from_be_bytes([data[2], data[3]]);
        let or_mask = u16::from_be_bytes([data[4], data[5]]);

        // Single lock for the read-modify-write (atomic vs. other connections)
        register_bank.mask_write_register(address, and_mask, or_mask)?;

        // Response echoes the request
        let mut response = Vec::with_capacity(7);
        response.push(0x16);
        response.extend_from_slice(data);
        Ok(response)
    }

    /// Handle read/write multiple registers (0x17)
    ///
    /// Per spec the write is performed before the read.
    async fn handle_read_write_17(
        data: &[u8],
        register_bank: &ModbusRegisterBank,
    ) -> ModbusResult<Vec<u8>> {
        if data.len() < 9 {
            return Err(ModbusError::frame("invalid frame"));
        }

        let read_address = u16::from_be_bytes([data[0], data[1]]);
        let read_quantity = u16::from_be_bytes([data[2], data[3]]);
        let write_address = u16::from_be_bytes([data[4], data[5]]);
        let write_quantity = u16::from_be_bytes([data[6], data[7]]);
        let byte_count = usize::from(data[8]);

        if read_quantity == 0 || usize::from(read_quantity) > MAX_RW_READ_REGISTERS {
            return Err(ModbusError::invalid_data(
                "Invalid read/write multiple read quantity",
            ));
        }
        if write_quantity == 0 || usize::from(write_quantity) > MAX_RW_WRITE_REGISTERS {
            return Err(ModbusError::invalid_data(
                "Invalid read/write multiple write quantity",
            ));
        }
        if byte_count != usize::from(write_quantity) * 2 || data.len() != 9 + byte_count {
            return Err(ModbusError::frame("invalid frame"));
        }

        let values: Vec<u16> = data[9..]
            .chunks_exact(2)
            .map(|chunk| u16::from_be_bytes([chunk[0], chunk[1]]))
            .collect();

        // Write first, then read (spec-mandated ordering), under one lock
        let registers = register_bank.write_read_registers(
            write_address,
            &values,
            read_address,
            read_quantity,
        )?;

        let data_len = registers.len() * 2;
        let mut response = Vec::with_capacity(2 + data_len);
        response.push(0x17);
        response.push(
            u8::try_from(data_len)
                .map_err(|_| ModbusError::invalid_data("Read/write response too large"))?,
        );
        for &register in &registers {
            response.extend_from_slice(&register.to_be_bytes());
        }
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

        let service = effective_service(&self.service, &self.device_identity);
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

                                let service = service.clone();
                                let stats = stats.clone();
                                let shutdown_rx = shutdown_tx.subscribe();

                                tokio::spawn(async move {
                                    let _permit = permit;
                                    Self::handle_client(stream, service, stats, shutdown_rx, request_timeout).await;
                                });
                            }
                            Err(e) => {
                                error!("Failed to accept connection: {}", e);
                                // accept() errors such as EMFILE/ENFILE persist until
                                // resources free up; back off instead of busy-looping.
                                tokio::time::sleep(ACCEPT_ERROR_BACKOFF).await;
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
#[non_exhaustive]
pub struct ModbusRtuServerConfig {
    pub port: String,
    pub baud_rate: u32,
    /// Slave address this server answers to (1-247).
    ///
    /// Frames addressed to other slaves are ignored; broadcast frames
    /// (address 0) are executed but never answered, per the Modbus spec.
    pub slave_id: u8,
    pub data_bits: tokio_serial::DataBits,
    pub stop_bits: tokio_serial::StopBits,
    pub parity: tokio_serial::Parity,
    pub timeout: Duration,
    /// Inter-frame silence (t3.5) that ends a frame. A request to this server
    /// that has only partly arrived (e.g. split by a USB-RS485 adapter's
    /// chunking) waits up to 50 ms after its last byte for the rest.
    pub frame_gap: Duration,
    pub register_bank: Option<Arc<ModbusRegisterBank>>,
}

#[cfg(feature = "rtu")]
impl Default for ModbusRtuServerConfig {
    fn default() -> Self {
        Self {
            port: "/dev/ttyUSB0".to_string(),
            baud_rate: 9600,
            slave_id: 1,
            data_bits: tokio_serial::DataBits::Eight,
            stop_bits: tokio_serial::StopBits::One,
            parity: tokio_serial::Parity::None,
            timeout: Duration::from_secs(1),
            frame_gap: Duration::from_millis(4), // Default 3.5 char time at 9600 baud
            register_bank: None,
        }
    }
}

/// Maximum Modbus RTU ADU size (slave + 253-byte PDU + CRC).
#[cfg(feature = "rtu")]
const MAX_RTU_ADU_SIZE: usize = 256;

/// How long a partly-arrived request to this server may wait, after its last
/// byte, for the rest. Covers USB-RS485 adapters that deliver a frame in
/// chunks (FTDI's default latency timer is 16 ms).
#[cfg(feature = "rtu")]
const PARTIAL_REQUEST_GRACE: Duration = Duration::from_millis(50);

/// Accumulates received bytes into one RTU frame until a t3.5 idle gap.
///
/// At the gap, a buffer that is the beginning of a request to this server
/// (see [`crate::rtu_frame::is_partial_request`]) keeps waiting, up to
/// [`PARTIAL_REQUEST_GRACE`] after its last byte, instead of ending there.
///
/// The buffer is capped at [`MAX_RTU_ADU_SIZE`]: an over-long burst (line
/// noise, garbage) is discarded entirely, including any bytes received after
/// the overflow, until the next idle gap ends it.
#[cfg(feature = "rtu")]
#[derive(Debug, Default)]
struct RtuFrameAccumulator {
    own_slave_id: u8,
    buffer: Vec<u8>,
    /// Buffer lengths at each t3.5 gap waited through — where 1.0.1 would
    /// have ended a frame
    gaps: Vec<usize>,
    overflowed: bool,
    /// When the last byte arrived (tokio clock, so paused-time tests work)
    last_byte: Option<tokio::time::Instant>,
}

#[cfg(feature = "rtu")]
impl RtuFrameAccumulator {
    fn new(own_slave_id: u8) -> Self {
        Self {
            own_slave_id,
            ..Self::default()
        }
    }

    /// Append received bytes; on overflow drop the frame being accumulated.
    fn push(&mut self, bytes: &[u8]) {
        self.last_byte = Some(tokio::time::Instant::now());
        if self.overflowed {
            return;
        }
        // Segments before the last gap waited through failed their CRC (that
        // is why we waited); 1.0.1 would have dropped them already
        if self.buffer.len() + bytes.len() > MAX_RTU_ADU_SIZE {
            if let Some(&last_gap) = self.gaps.last() {
                self.buffer.drain(..last_gap);
                self.gaps.clear();
            }
        }
        if self.buffer.len() + bytes.len() > MAX_RTU_ADU_SIZE {
            warn!(
                "Discarding RTU frame longer than {} bytes (line noise?)",
                MAX_RTU_ADU_SIZE
            );
            self.buffer.clear();
            self.overflowed = true;
            return;
        }
        self.buffer.extend_from_slice(bytes);
    }

    /// True when no frame is in progress (no idle timer needed).
    fn is_idle(&self) -> bool {
        self.buffer.is_empty() && !self.overflowed
    }

    #[cfg(test)]
    fn buffered_len(&self) -> usize {
        self.buffer.len()
    }

    /// Remaining wait for the rest of a partly-arrived request to us, if the
    /// buffer is one and the grace has not run out.
    fn partial_request_wait(&self) -> Option<Duration> {
        if self.overflowed || !crate::rtu_frame::is_partial_request(&self.buffer, self.own_slave_id)
        {
            return None;
        }
        let deadline = self.last_byte? + PARTIAL_REQUEST_GRACE;
        let left = deadline.saturating_duration_since(tokio::time::Instant::now());
        (!left.is_zero()).then_some(left)
    }

    /// How long the next read may block before the buffer is looked at:
    /// t3.5 after fresh bytes (so every gap 1.0.1 would have ended a frame at
    /// is observed and recorded), then the rest of a partial request's grace.
    fn idle_limit(&self, frame_gap: Duration) -> Duration {
        match self.partial_request_wait() {
            Some(wait) if self.gaps.last() == Some(&self.buffer.len()) => wait.max(frame_gap),
            _ => frame_gap,
        }
    }

    /// At a t3.5 gap: keep waiting (and record the gap) only while the buffer
    /// is a partly-arrived request to us AND the segment since the previous
    /// gap fails its CRC — i.e. only through bytes 1.0.1 would have dropped.
    /// A CRC-valid segment is a frame 1.0.1 would process right now, so the
    /// wait ends and it is processed on time.
    fn wait_through_gap(&mut self) -> bool {
        if self.partial_request_wait().is_none() {
            return false;
        }
        let segment_start = self.gaps.last().copied().unwrap_or(0);
        if crate::rtu_frame::crc_ok(&self.buffer[segment_start..]) {
            return false;
        }
        self.mark_gap();
        true
    }

    /// Record a t3.5 gap that was waited through instead of ending the frame.
    fn mark_gap(&mut self) {
        if self.gaps.last() != Some(&self.buffer.len()) {
            self.gaps.push(self.buffer.len());
        }
    }

    /// Called on a t3.5 idle gap: returns the completed frame(s) and resets
    /// for the next frame.
    ///
    /// A wait that ended in a CRC-valid frame yields that reassembled frame.
    /// Otherwise the buffer is split at every gap waited through, yielding
    /// exactly the frames 1.0.1 would have processed — so a glitch byte or a
    /// truncated request cannot swallow the request that follows it.
    fn end_frame(&mut self) -> Vec<Vec<u8>> {
        let overflowed = std::mem::take(&mut self.overflowed);
        let buffer = std::mem::take(&mut self.buffer);
        let gaps = std::mem::take(&mut self.gaps);
        if overflowed || buffer.is_empty() {
            return Vec::new();
        }
        if gaps.is_empty() {
            return vec![buffer];
        }
        let mut segments = Vec::with_capacity(gaps.len() + 1);
        let mut start = 0;
        for end in gaps.into_iter().chain([buffer.len()]) {
            if end > start {
                segments.push(buffer[start..end].to_vec());
                start = end;
            }
        }
        // Reassembled only if no segment is a frame on its own (CRC-16 has no
        // output XOR: a valid frame plus trailing 0x00 bytes still passes)
        let reassembled = segments.iter().all(|s| !crate::rtu_frame::crc_ok(s))
            && crate::rtu_frame::is_complete_request(&buffer, self.own_slave_id)
            && crate::rtu_frame::crc_ok(&buffer);
        if reassembled {
            vec![buffer]
        } else {
            segments
        }
    }
}

/// Modbus RTU server implementation
#[cfg(feature = "rtu")]
pub struct ModbusRtuServer {
    config: ModbusRtuServerConfig,
    register_bank: Arc<ModbusRegisterBank>,
    /// Request handler; defaults to the register bank itself
    service: Arc<dyn ModbusService>,
    /// FC 0x2B responder, when configured
    device_identity: Option<Arc<DeviceIdentity>>,
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
            frame_gap: crate::transport::rtu_frame_gap(baud_rate),
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
            service: register_bank.clone(),
            register_bank,
            device_identity: None,
            stats: Arc::new(Mutex::new(ServerStats::default())),
            shutdown_tx: None,
            is_running: Arc::new(AtomicBool::new(false)),
            start_time: None,
        })
    }

    /// Set custom register bank (also makes it the active request handler)
    pub fn set_register_bank(&mut self, register_bank: Arc<ModbusRegisterBank>) {
        self.service = register_bank.clone();
        self.register_bank = register_bank;
    }

    /// Back this server with a custom [`ModbusService`] instead of the
    /// default in-memory register bank — see [`ModbusTcpServer::set_service`].
    pub fn set_service(&mut self, service: Arc<dyn ModbusService>) {
        self.service = service;
    }

    /// Serve FC 0x2B (Read Device Identification) from these objects.
    /// Takes effect from the next [`Self::start`].
    pub fn set_device_identity(&mut self, identity: DeviceIdentity) {
        self.device_identity = Some(Arc::new(identity));
    }

    /// Calculate CRC for RTU frames
    fn calculate_crc(data: &[u8]) -> u16 {
        use crc::{Crc, CRC_16_MODBUS};
        const CRC_MODBUS: Crc<u16> = Crc::<u16>::new(&CRC_16_MODBUS);
        CRC_MODBUS.checksum(data)
    }

    /// Process a single received RTU frame.
    ///
    /// Returns `Some(frame)` (CRC already appended) when a reply must be sent.
    /// Returns `None` when the frame must stay unanswered per the Modbus spec:
    /// noise/CRC-corrupted frames, frames addressed to another slave, and
    /// broadcasts (executed but never acknowledged — a reply would collide
    /// with other slaves on the bus).
    async fn process_frame(
        frame: &[u8],
        own_slave_id: u8,
        service: &dyn ModbusService,
    ) -> Option<Vec<u8>> {
        // Minimum RTU frame: slave(1) + function(1) + CRC(2)
        if frame.len() < 4 {
            return None;
        }

        let crc_split = frame.len() - 2;
        let received_crc = u16::from_le_bytes([frame[crc_split], frame[crc_split + 1]]);
        let calculated_crc = Self::calculate_crc(&frame[..crc_split]);
        if received_crc != calculated_crc {
            warn!(
                "Ignoring RTU frame with bad CRC: expected {:04X}, got {:04X}",
                calculated_crc, received_crc
            );
            return None;
        }

        let slave_id = frame[0];
        if slave_id != 0 && slave_id != own_slave_id {
            return None;
        }

        let function_code = frame[1];
        let pdu_data = &frame[2..crc_split];

        let result = service.handle_pdu(function_code, pdu_data).await;

        // Broadcast: executed above, but never answered
        if slave_id == 0 {
            return None;
        }

        match result {
            Ok(pdu) => {
                let mut response = Vec::with_capacity(1 + pdu.len() + 2);
                response.push(slave_id);
                response.extend_from_slice(&pdu);
                let crc = Self::calculate_crc(&response);
                response.extend_from_slice(&crc.to_le_bytes());
                Some(response)
            }
            Err(e) => {
                let exception_code = ModbusTcpServer::exception_code_for_error(&e);
                Self::create_rtu_error_response(slave_id, function_code, exception_code).ok()
            }
        }
    }

    /// Thin public wrapper around [`Self::process_frame`] for fuzz testing.
    ///
    /// Dispatches into a real [`ModbusRegisterBank`] (the default service) so
    /// fuzzed frames exercise the full CRC-check → slave-filter → FC-dispatch
    /// → register-bank path, including the register read/write handlers.
    ///
    /// Only compiled under `cfg(fuzzing)` (set by cargo-fuzz) and for tests.
    #[cfg(any(fuzzing, test))]
    #[doc(hidden)]
    pub async fn process_frame_fuzz(frame: &[u8], own_slave_id: u8) -> Option<Vec<u8>> {
        let bank = ModbusRegisterBank::new();
        Self::process_frame(frame, own_slave_id, &bank).await
    }

    /// Create RTU error response
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
    async fn handle_rtu_communication<S>(
        mut port: S,
        own_slave_id: u8,
        service: Arc<dyn ModbusService>,
        stats: Arc<Mutex<ServerStats>>,
        mut shutdown_rx: broadcast::Receiver<()>,
        frame_gap: Duration,
    ) where
        S: tokio::io::AsyncRead + tokio::io::AsyncWrite + Unpin,
    {
        info!("🔌 RTU server communication started");

        let mut buffer = [0u8; MAX_RTU_ADU_SIZE];
        let mut frame = RtuFrameAccumulator::new(own_slave_id);

        loop {
            // While a frame is in progress, t3.5 of silence ends it (or, for a
            // partly-arrived request to us, the rest of its grace); when idle,
            // wait for the next byte with no timeout.
            let idle_limit = frame.idle_limit(frame_gap);
            let read = async {
                if frame.is_idle() {
                    Ok(port.read(&mut buffer).await)
                } else {
                    tokio::time::timeout(idle_limit, port.read(&mut buffer)).await
                }
            };

            let result = tokio::select! {
                _ = shutdown_rx.recv() => {
                    debug!("Shutdown signal received for RTU server");
                    break;
                }
                result = read => result,
            };

            match result {
                Ok(Ok(bytes_read)) if bytes_read > 0 => {
                    frame.push(&buffer[..bytes_read]);
                    if let Ok(mut stats) = stats.lock() {
                        stats.bytes_received += bytes_read as u64;
                    }
                }
                Ok(Ok(_)) => {
                    // EOF: the port is gone; looping would spin at 100% CPU
                    info!("RTU port closed (EOF)");
                    break;
                }
                Ok(Err(e)) => {
                    error!("RTU read error: {}", e);
                    break;
                }
                Err(_) => {
                    // A request to us that is still arriving: keep waiting
                    if frame.wait_through_gap() {
                        continue;
                    }
                    // t3.5 idle gap elapsed: the frame is complete
                    for complete in frame.end_frame() {
                        Self::process_accumulated_frame(
                            &complete,
                            &mut port,
                            own_slave_id,
                            service.as_ref(),
                            &stats,
                        )
                        .await;
                    }
                }
            }
        }

        info!("🔌 RTU server communication stopped");
    }

    /// Process accumulated frame data
    async fn process_accumulated_frame<S>(
        frame: &[u8],
        port: &mut S,
        own_slave_id: u8,
        service: &dyn ModbusService,
        stats: &Arc<Mutex<ServerStats>>,
    ) where
        S: tokio::io::AsyncWrite + Unpin,
    {
        // Update request stats
        if let Ok(mut stats) = stats.lock() {
            stats.total_requests += 1;
        }

        // None = frame must stay unanswered (noise, bad CRC, other slave, broadcast)
        let Some(response) = Self::process_frame(frame, own_slave_id, service).await else {
            return;
        };

        if let Err(e) = port.write_all(&response).await {
            error!("Failed to write response: {}", e);
            if let Ok(mut stats) = stats.lock() {
                stats.failed_requests += 1;
            }
        } else if let Ok(mut stats) = stats.lock() {
            stats.successful_requests += 1;
            stats.bytes_sent += response.len() as u64;
        }
    }
}

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
        info!("   - Slave ID: {}", self.config.slave_id);
        info!("   - Data bits: {:?}", self.config.data_bits);
        info!("   - Stop bits: {:?}", self.config.stop_bits);
        info!("   - Parity: {:?}", self.config.parity);
        info!("   - Timeout: {:?}", self.config.timeout);

        let service = effective_service(&self.service, &self.device_identity);
        let stats = self.stats.clone();
        let frame_gap = self.config.frame_gap;
        let own_slave_id = self.config.slave_id;
        let is_running_flag = self.is_running.clone();
        let shutdown_rx = shutdown_tx.subscribe();

        tokio::spawn(async move {
            Self::handle_rtu_communication(
                port,
                own_slave_id,
                service,
                stats,
                shutdown_rx,
                frame_gap,
            )
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

        let response = ModbusTcpServer::handle_request(&request, register_bank.as_ref())
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

    #[tokio::test]
    async fn test_tcp_handle_request_rejects_read_trailing_bytes() {
        let register_bank = Arc::new(ModbusRegisterBank::new());

        let request = [
            0x12, 0x34, // transaction id
            0x00, 0x00, // protocol id
            0x00, 0x07, // length includes one trailing byte
            0x01, // unit id
            0x03, // read holding registers
            0x00, 0x00, // address
            0x00, 0x01, // quantity
            0x99, // invalid trailing byte
        ];

        let err = ModbusTcpServer::handle_request(&request, register_bank.as_ref())
            .await
            .unwrap_err();
        assert!(matches!(err, ModbusError::Frame { .. }));
    }

    #[tokio::test]
    async fn test_tcp_handle_request_rejects_write_trailing_bytes() {
        let register_bank = Arc::new(ModbusRegisterBank::new());

        let request = [
            0x12, 0x34, // transaction id
            0x00, 0x00, // protocol id
            0x00, 0x0A, // length includes one trailing byte
            0x01, // unit id
            0x10, // write multiple registers
            0x00, 0x00, // address
            0x00, 0x01, // quantity
            0x02, // byte count
            0x12, 0x34, // register value
            0x99, // invalid trailing byte
        ];

        let err = ModbusTcpServer::handle_request(&request, register_bank.as_ref())
            .await
            .unwrap_err();
        assert!(matches!(err, ModbusError::Frame { .. }));
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
            slave_id: 17,
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

    #[tokio::test]
    async fn test_tcp_handle_request_mask_write() {
        let register_bank = Arc::new(ModbusRegisterBank::new());
        register_bank.write_06(4, 0x0012).unwrap();

        // Spec example: current=0x12, and=0xF2, or=0x25 → (0x12 & 0xF2) | (0x25 & !0xF2) = 0x17
        let request = [
            0x00, 0x01, 0x00, 0x00, 0x00, 0x08, // MBAP, len = unit+fc+addr+and+or
            0x01, 0x16, 0x00, 0x04, 0x00, 0xF2, 0x00, 0x25,
        ];
        let response = ModbusTcpServer::handle_request(&request, register_bank.as_ref())
            .await
            .unwrap();

        // Response echoes the request PDU
        assert_eq!(&response[7..], &[0x16, 0x00, 0x04, 0x00, 0xF2, 0x00, 0x25]);
        assert_eq!(register_bank.read_03(4, 1).unwrap(), vec![0x0017]);
    }

    /// FC22 must be atomic: concurrent mask writes from different connections
    /// to distinct bits of one register must never lose each other's update.
    /// Each task owns one bit; after setting it, that bit must stay set until
    /// the same task clears it.
    #[tokio::test(flavor = "multi_thread", worker_threads = 8)]
    async fn test_tcp_mask_write_is_atomic_under_concurrency() {
        let bank = Arc::new(ModbusRegisterBank::new());
        let mut tasks = Vec::new();
        for bit in 0..16u16 {
            let bank = bank.clone();
            tasks.push(tokio::spawn(async move {
                let mask = 1u16 << bit;
                for _ in 0..2000 {
                    for or in [mask, 0] {
                        let [a0, a1] = (!mask).to_be_bytes();
                        let [o0, o1] = or.to_be_bytes();
                        let request = [
                            0x00, 0x01, 0x00, 0x00, 0x00, 0x08, 0x01, 0x16, 0x00, 0x00, a0, a1, o0,
                            o1,
                        ];
                        ModbusTcpServer::handle_request(&request, bank.as_ref())
                            .await
                            .unwrap();
                        let current = bank.read_03(0, 1).unwrap()[0];
                        if or != 0 && current & mask == 0 {
                            return false;
                        }
                    }
                    tokio::task::yield_now().await;
                }
                true
            }));
        }
        for task in tasks {
            assert!(task.await.unwrap(), "a concurrent mask write was lost");
        }
        assert_eq!(bank.read_03(0, 1).unwrap(), vec![0]);
    }

    #[tokio::test]
    async fn test_tcp_handle_request_read_write_multiple() {
        let register_bank = Arc::new(ModbusRegisterBank::new());
        register_bank.write_06(0, 0x0AAA).unwrap();

        // Read 1 reg from address 0, write 0x1234 to address 10
        let request = [
            0x00, 0x01, 0x00, 0x00, 0x00, 0x0D, // MBAP, len = 1+1+2+2+2+2+1+2
            0x01, 0x17, //
            0x00, 0x00, 0x00, 0x01, // read addr 0, qty 1
            0x00, 0x0A, 0x00, 0x01, // write addr 10, qty 1
            0x02, 0x12, 0x34, // byte count + value
        ];
        let response = ModbusTcpServer::handle_request(&request, register_bank.as_ref())
            .await
            .unwrap();

        assert_eq!(&response[7..], &[0x17, 0x02, 0x0A, 0xAA]);
        assert_eq!(register_bank.read_03(10, 1).unwrap(), vec![0x1234]);
    }

    /// Minimal custom service: serves a fixed FC03 payload, rejects the rest.
    struct FixedService;

    impl ModbusService for FixedService {
        fn handle_pdu<'a>(&'a self, function_code: u8, _data: &'a [u8]) -> ServiceFuture<'a> {
            Box::pin(async move {
                match function_code {
                    0x03 => Ok(vec![0x03, 0x02, 0xBE, 0xEF]),
                    other => Err(ModbusError::invalid_function(other)),
                }
            })
        }
    }

    #[tokio::test]
    async fn test_custom_service_backs_tcp_dispatch() {
        let request = [
            0x00, 0x01, 0x00, 0x00, 0x00, 0x06, //
            0x01, 0x03, 0x00, 0x00, 0x00, 0x01,
        ];
        let response = ModbusTcpServer::handle_request(&request, &FixedService)
            .await
            .unwrap();
        assert_eq!(&response[7..], &[0x03, 0x02, 0xBE, 0xEF]);

        // Unsupported FC surfaces as an error (mapped to exception upstream)
        let bad = [
            0x00, 0x01, 0x00, 0x00, 0x00, 0x06, //
            0x01, 0x04, 0x00, 0x00, 0x00, 0x01,
        ];
        assert!(ModbusTcpServer::handle_request(&bad, &FixedService)
            .await
            .is_err());
    }

    #[tokio::test]
    async fn test_tcp_handle_request_diagnostics_echo() {
        let register_bank = Arc::new(ModbusRegisterBank::new());
        let request = [
            0x00, 0x01, 0x00, 0x00, 0x00, 0x06, //
            0x01, 0x08, 0x00, 0x00, 0xA5, 0x37, // sub 0x0000 echo test
        ];
        let response = ModbusTcpServer::handle_request(&request, register_bank.as_ref())
            .await
            .unwrap();
        assert_eq!(&response[7..], &[0x08, 0x00, 0x00, 0xA5, 0x37]);

        // Unsupported sub-function must error
        let bad = [
            0x00, 0x01, 0x00, 0x00, 0x00, 0x06, //
            0x01, 0x08, 0x00, 0x01, 0x00, 0x00,
        ];
        assert!(
            ModbusTcpServer::handle_request(&bad, register_bank.as_ref())
                .await
                .is_err()
        );
    }

    #[tokio::test]
    async fn test_device_identity_stream_read() {
        let identity = DeviceIdentity::basic("VendorX", "PC-1", "V2.1");
        // Basic stream read from object 0
        let response = identity.handle_request(&[0x0E, 0x01, 0x00]).unwrap();
        assert_eq!(&response[..3], &[0x2B, 0x0E, 0x01]);
        assert_eq!(response[3], 0x81); // basic + individual access
        assert_eq!(response[4], 0x00); // no more follows
        assert_eq!(response[6], 3); // three objects
        assert_eq!(&response[7..9], &[0x00, 7]); // VendorName, len 7
        assert_eq!(&response[9..16], b"VendorX");
    }

    #[tokio::test]
    async fn test_device_identity_individual_read() {
        let identity = DeviceIdentity::basic("VendorX", "PC-1", "V2.1");

        // Code 4: read one specific object
        let response = identity.handle_request(&[0x0E, 0x04, 0x01]).unwrap();
        assert_eq!(response[6], 1);
        assert_eq!(&response[7..9], &[0x01, 4]);
        assert_eq!(&response[9..13], b"PC-1");

        // Unknown object id → error (maps to exception 0x02)
        let err = identity.handle_request(&[0x0E, 0x04, 0x77]).unwrap_err();
        assert_eq!(ModbusTcpServer::exception_code_for_error(&err), 0x02);
    }

    #[tokio::test]
    async fn test_device_identity_served_through_service_wrapper() {
        let bank = Arc::new(ModbusRegisterBank::new());
        bank.write_06(0, 0x1234).unwrap();
        let service: Arc<dyn ModbusService> = bank.clone();
        let identity = Some(Arc::new(DeviceIdentity::basic("V", "P", "1.0")));
        let wrapped = effective_service(&service, &identity);

        // FC 0x2B answered from the identity
        let request = [
            0x00, 0x01, 0x00, 0x00, 0x00, 0x05, //
            0x01, 0x2B, 0x0E, 0x01, 0x00,
        ];
        let response = ModbusTcpServer::handle_request(&request, wrapped.as_ref())
            .await
            .unwrap();
        assert_eq!(response[7], 0x2B);
        assert_eq!(response[13], 3); // object count field: all three basic objects

        // Other function codes still reach the register bank
        let read = [
            0x00, 0x02, 0x00, 0x00, 0x00, 0x06, //
            0x01, 0x03, 0x00, 0x00, 0x00, 0x01,
        ];
        let response = ModbusTcpServer::handle_request(&read, wrapped.as_ref())
            .await
            .unwrap();
        assert_eq!(&response[7..], &[0x03, 0x02, 0x12, 0x34]);
    }

    /// Build a valid RTU frame by appending the CRC to a body.
    #[cfg(feature = "rtu")]
    fn rtu_frame(body: &[u8]) -> Vec<u8> {
        let mut frame = body.to_vec();
        let crc = ModbusRtuServer::calculate_crc(&frame);
        frame.extend_from_slice(&crc.to_le_bytes());
        frame
    }

    /// C1 regression: short noise bursts (including the 3-byte case that used
    /// to panic via `&data[2..len-2]`) must be ignored, never panic.
    #[cfg(feature = "rtu")]
    #[tokio::test]
    async fn test_rtu_process_frame_ignores_short_noise() {
        let bank = Arc::new(ModbusRegisterBank::new());
        for noise in [
            &[][..],
            &[0x01][..],
            &[0x01, 0x03][..],
            &[0x01, 0x03, 0x00][..],
        ] {
            assert!(
                ModbusRtuServer::process_frame(noise, 1, bank.as_ref())
                    .await
                    .is_none(),
                "noise frame {noise:02X?} must be ignored"
            );
        }
    }

    /// C2 regression: frames with a corrupted CRC must be dropped, not executed.
    #[cfg(feature = "rtu")]
    #[tokio::test]
    async fn test_rtu_process_frame_ignores_bad_crc() {
        let bank = Arc::new(ModbusRegisterBank::new());
        // Corrupted broadcast write: must NOT reach the register bank
        let mut frame = rtu_frame(&[0x00, 0x06, 0x00, 0x05, 0xAB, 0xCD]);
        let last = frame.len() - 1;
        frame[last] ^= 0xFF;

        assert!(ModbusRtuServer::process_frame(&frame, 1, bank.as_ref())
            .await
            .is_none());
        assert_eq!(bank.read_03(5, 1).unwrap(), vec![0x0000]);
    }

    /// C3 regression: the answered slave address comes from configuration,
    /// not a hardcoded `1`.
    #[cfg(feature = "rtu")]
    #[tokio::test]
    async fn test_rtu_process_frame_respects_configured_slave_id() {
        let bank = Arc::new(ModbusRegisterBank::new());
        let frame = rtu_frame(&[0x02, 0x03, 0x00, 0x00, 0x00, 0x01]);

        // Addressed to slave 2: ignored when we are slave 1, answered when we are slave 2
        assert!(ModbusRtuServer::process_frame(&frame, 1, bank.as_ref())
            .await
            .is_none());
        assert!(ModbusRtuServer::process_frame(&frame, 2, bank.as_ref())
            .await
            .is_some());
    }

    /// C4 regression: broadcast writes are executed but never answered.
    #[cfg(feature = "rtu")]
    #[tokio::test]
    async fn test_rtu_process_frame_broadcast_write_executes_silently() {
        let bank = Arc::new(ModbusRegisterBank::new());
        let frame = rtu_frame(&[0x00, 0x06, 0x00, 0x05, 0xAB, 0xCD]);

        let response = ModbusRtuServer::process_frame(&frame, 1, bank.as_ref()).await;
        assert!(response.is_none(), "broadcast must never be answered");
        assert_eq!(bank.read_03(5, 1).unwrap(), vec![0xABCD]);
    }

    /// Valid unicast read produces a CRC-terminated response frame.
    #[cfg(feature = "rtu")]
    #[tokio::test]
    async fn test_rtu_process_frame_valid_read_response() {
        let bank = Arc::new(ModbusRegisterBank::new());
        bank.write_06(0, 0x1234).unwrap();
        let frame = rtu_frame(&[0x01, 0x03, 0x00, 0x00, 0x00, 0x01]);

        let response = ModbusRtuServer::process_frame(&frame, 1, bank.as_ref())
            .await
            .expect("valid unicast read must be answered");

        // slave, fc, byte_count, data hi, data lo
        assert_eq!(&response[..5], &[0x01, 0x03, 0x02, 0x12, 0x34]);
        let split = response.len() - 2;
        let crc = u16::from_le_bytes([response[split], response[split + 1]]);
        assert_eq!(crc, ModbusRtuServer::calculate_crc(&response[..split]));
    }

    /// W4 regression: processing errors yield a Modbus exception response
    /// instead of leaving the master to time out.
    #[cfg(feature = "rtu")]
    #[tokio::test]
    async fn test_rtu_process_frame_error_yields_exception_response() {
        let bank = Arc::new(ModbusRegisterBank::new());
        // FC03 with quantity 0 → Illegal Data Value (0x03)
        let frame = rtu_frame(&[0x01, 0x03, 0x00, 0x00, 0x00, 0x00]);

        let response = ModbusRtuServer::process_frame(&frame, 1, bank.as_ref())
            .await
            .expect("error must produce an exception response");

        assert_eq!(response[0], 0x01); // slave id
        assert_eq!(response[1], 0x83); // fc | 0x80
        assert_eq!(response[2], 0x03); // Illegal Data Value
        let split = response.len() - 2;
        let crc = u16::from_le_bytes([response[split], response[split + 1]]);
        assert_eq!(crc, ModbusRtuServer::calculate_crc(&response[..split]));
    }

    /// Spawn the RTU read loop over an in-memory duplex "serial line".
    #[cfg(feature = "rtu")]
    fn spawn_rtu_loop(
        bank: Arc<ModbusRegisterBank>,
        frame_gap: Duration,
    ) -> (
        tokio::io::DuplexStream,
        broadcast::Sender<()>,
        tokio::task::JoinHandle<()>,
    ) {
        let (client, server) = tokio::io::duplex(4096);
        let (tx, rx) = broadcast::channel(1);
        let stats = Arc::new(Mutex::new(ServerStats::default()));
        let task = tokio::spawn(ModbusRtuServer::handle_rtu_communication(
            server, 1, bank, stats, rx, frame_gap,
        ));
        (client, tx, task)
    }

    /// Bug: a frame was only processed after a fixed 100 ms read timeout.
    /// The frame must end after the configured t3.5 gap instead. Uses paused
    /// tokio time, so the elapsed time is virtual and deterministic.
    #[cfg(feature = "rtu")]
    #[tokio::test(start_paused = true)]
    async fn test_rtu_loop_replies_after_frame_gap() {
        let bank = Arc::new(ModbusRegisterBank::new());
        bank.write_06(0, 0x1234).unwrap();
        let (mut client, tx, task) = spawn_rtu_loop(bank, Duration::from_millis(2));

        let start = tokio::time::Instant::now();
        // Request split across two writes with no gap is still one frame
        let request = rtu_frame(&[0x01, 0x03, 0x00, 0x00, 0x00, 0x01]);
        client.write_all(&request[..3]).await.unwrap();
        client.write_all(&request[3..]).await.unwrap();

        let mut response = [0u8; 7];
        tokio::time::timeout(Duration::from_secs(1), client.read_exact(&mut response))
            .await
            .expect("no RTU reply")
            .unwrap();
        assert_eq!(
            &response[..],
            &rtu_frame(&[0x01, 0x03, 0x02, 0x12, 0x34])[..]
        );
        assert!(
            start.elapsed() < Duration::from_millis(20),
            "reply took {:?}",
            start.elapsed()
        );

        tx.send(()).unwrap();
        task.await.unwrap();
    }

    /// Multi-drop bus: another slave's reply is followed, after only a few
    /// t3.5 gaps, by the master's request to us. The gap must split the two
    /// frames (as in 1.0.0); a longer idle threshold would merge them and the
    /// CRC check would silently drop our request.
    #[cfg(feature = "rtu")]
    #[tokio::test(start_paused = true)]
    async fn test_rtu_loop_splits_back_to_back_frames_on_multidrop_bus() {
        let bank = Arc::new(ModbusRegisterBank::new());
        bank.write_06(0, 0x1234).unwrap();
        let (mut client, tx, task) = spawn_rtu_loop(bank, Duration::from_millis(2));

        // Slave 2's reply to the master, then 3 ms later the request to us
        client
            .write_all(&rtu_frame(&[0x02, 0x03, 0x02, 0xAB, 0xCD]))
            .await
            .unwrap();
        tokio::time::sleep(Duration::from_millis(3)).await;
        client
            .write_all(&rtu_frame(&[0x01, 0x03, 0x00, 0x00, 0x00, 0x01]))
            .await
            .unwrap();

        let mut response = [0u8; 7];
        tokio::time::timeout(Duration::from_secs(1), client.read_exact(&mut response))
            .await
            .expect("request after another slave's reply was dropped")
            .unwrap();
        assert_eq!(
            &response[..],
            &rtu_frame(&[0x01, 0x03, 0x02, 0x12, 0x34])[..]
        );

        tx.send(()).unwrap();
        task.await.unwrap();
    }

    #[cfg(feature = "rtu")]
    fn spawn_rtu_loop_as(
        own_slave_id: u8,
        bank: Arc<ModbusRegisterBank>,
        frame_gap: Duration,
    ) -> (
        tokio::io::DuplexStream,
        broadcast::Sender<()>,
        tokio::task::JoinHandle<()>,
    ) {
        let (client, server) = tokio::io::duplex(4096);
        let (tx, rx) = broadcast::channel(1);
        let stats = Arc::new(Mutex::new(ServerStats::default()));
        let task = tokio::spawn(ModbusRtuServer::handle_rtu_communication(
            server,
            own_slave_id,
            bank,
            stats,
            rx,
            frame_gap,
        ));
        (client, tx, task)
    }

    #[cfg(feature = "rtu")]
    async fn expect_reply(client: &mut tokio::io::DuplexStream, body: &[u8]) {
        let expected = rtu_frame(body);
        let mut response = vec![0u8; expected.len()];
        tokio::time::timeout(Duration::from_secs(1), client.read_exact(&mut response))
            .await
            .expect("no RTU reply")
            .unwrap();
        assert_eq!(response, expected);
    }

    /// USB-RS485 adapters (FTDI latency timer: 16 ms) deliver a request in
    /// chunks far more than t3.5 apart (1.75 ms above 19200 baud). The server
    /// waits for the rest of a request to it instead of dropping each chunk.
    #[cfg(feature = "rtu")]
    #[tokio::test(start_paused = true)]
    async fn test_rtu_loop_reassembles_usb_chunked_request() {
        for own in [1u8, 7, 0x10, 247] {
            let bank = Arc::new(ModbusRegisterBank::new());
            bank.write_06(0, 0x1234).unwrap();
            let (mut client, tx, task) = spawn_rtu_loop_as(own, bank, Duration::from_micros(1750));
            let request = rtu_frame(&[own, 0x03, 0x00, 0x00, 0x00, 0x01]);
            for cut in 1..request.len() {
                client.write_all(&request[..cut]).await.unwrap();
                tokio::time::sleep(Duration::from_millis(16)).await;
                client.write_all(&request[cut..]).await.unwrap();
                expect_reply(&mut client, &[own, 0x03, 0x02, 0x12, 0x34]).await;
            }
            tx.send(()).unwrap();
            task.await.unwrap();
        }
    }

    /// A long request takes longer than the 50 ms grace on the wire; the
    /// grace runs from the last byte, so chunk after chunk keeps it alive.
    /// 129-byte FC10 at 9600 baud (~134 ms), 16 bytes every 16 ms.
    #[cfg(feature = "rtu")]
    #[tokio::test(start_paused = true)]
    async fn test_rtu_loop_reassembles_long_chunked_request() {
        let bank = Arc::new(ModbusRegisterBank::new());
        let (mut client, tx, task) = spawn_rtu_loop_as(1, bank.clone(), Duration::from_millis(4));
        let mut body = vec![0x01, 0x10, 0x00, 0x00, 0x00, 60, 120];
        for i in 0..60u16 {
            body.extend_from_slice(&i.to_be_bytes());
        }
        let request = rtu_frame(&body);
        assert_eq!(request.len(), 129);
        for chunk in request.chunks(16) {
            client.write_all(chunk).await.unwrap();
            tokio::time::sleep(Duration::from_millis(16)).await;
        }
        expect_reply(&mut client, &[0x01, 0x10, 0x00, 0x00, 0x00, 60]).await;
        assert_eq!(bank.read_03(59, 1).unwrap(), vec![59]);
        tx.send(()).unwrap();
        task.await.unwrap();
    }

    /// A partial request that never completes is dropped after the grace,
    /// and the next request is answered normally.
    #[cfg(feature = "rtu")]
    #[tokio::test(start_paused = true)]
    async fn test_rtu_loop_drops_request_that_never_completes() {
        let bank = Arc::new(ModbusRegisterBank::new());
        bank.write_06(0, 0x1234).unwrap();
        let (mut client, tx, task) = spawn_rtu_loop(bank, Duration::from_micros(1750));
        client.write_all(&[0x01, 0x03, 0x00]).await.unwrap();
        tokio::time::sleep(Duration::from_millis(200)).await;
        client
            .write_all(&rtu_frame(&[0x01, 0x03, 0x00, 0x00, 0x00, 0x01]))
            .await
            .unwrap();
        expect_reply(&mut client, &[0x01, 0x03, 0x02, 0x12, 0x34]).await;
        tx.send(()).unwrap();
        task.await.unwrap();
    }

    /// Other slaves' traffic is never waited on: our request right after
    /// another slave's frame is answered at t3.5, not after the grace.
    #[cfg(feature = "rtu")]
    #[tokio::test(start_paused = true)]
    async fn test_rtu_loop_does_not_wait_on_other_slaves() {
        let bank = Arc::new(ModbusRegisterBank::new());
        bank.write_06(0, 0x1234).unwrap();
        let (mut client, tx, task) = spawn_rtu_loop(bank, Duration::from_millis(2));
        // Another slave's request, cut short (only part of it heard)
        client.write_all(&[0x02, 0x03, 0x00]).await.unwrap();
        tokio::time::sleep(Duration::from_millis(3)).await;
        let start = tokio::time::Instant::now();
        client
            .write_all(&rtu_frame(&[0x01, 0x03, 0x00, 0x00, 0x00, 0x01]))
            .await
            .unwrap();
        expect_reply(&mut client, &[0x01, 0x03, 0x02, 0x12, 0x34]).await;
        assert!(
            start.elapsed() < Duration::from_millis(10),
            "{:?}",
            start.elapsed()
        );
        tx.send(()).unwrap();
        task.await.unwrap();
    }

    /// Cases where 1.0.1 answered and the partial-request wait must not
    /// change that (review round 4): the wait may only ever add a frame —
    /// a reassembled request — never lose one 1.0.1 would have processed.
    #[cfg(feature = "rtu")]
    async fn answered_after(own: u8, frame_gap: Duration, steps: &[(&[u8], u64)]) {
        let bank = Arc::new(ModbusRegisterBank::new());
        bank.write_06(0, 0x1234).unwrap();
        let (mut client, tx, task) = spawn_rtu_loop_as(own, bank, frame_gap);
        for (bytes, pause_ms) in steps {
            client.write_all(bytes).await.unwrap();
            tokio::time::sleep(Duration::from_millis(*pause_ms)).await;
        }
        expect_reply(&mut client, &[own, 0x03, 0x02, 0x12, 0x34]).await;
        tx.send(()).unwrap();
        task.await.unwrap();
    }

    /// A configured frame gap longer than the 50 ms grace is still honored.
    #[cfg(feature = "rtu")]
    #[tokio::test(start_paused = true)]
    async fn test_rtu_loop_partial_wait_honors_long_frame_gap() {
        let request = rtu_frame(&[0x01, 0x03, 0x00, 0x00, 0x00, 0x01]);
        answered_after(
            1,
            Duration::from_millis(100),
            &[(&request[..3], 70), (&request[3..], 0)],
        )
        .await;
    }

    /// A glitch byte that looks like the start of a request to us, then the
    /// master's real request within the grace: the request is answered.
    #[cfg(feature = "rtu")]
    #[tokio::test(start_paused = true)]
    async fn test_rtu_loop_glitch_then_request_within_grace() {
        for own in [1u8, 0x10, 0x2B, 100] {
            let request = rtu_frame(&[own, 0x03, 0x00, 0x00, 0x00, 0x01]);
            for glitch in [0x00u8, own] {
                answered_after(
                    own,
                    Duration::from_micros(1750),
                    &[(&[glitch], 10), (&request, 0)],
                )
                .await;
            }
        }
    }

    /// A truncated request, then the master's retry within the grace: the
    /// retry is answered.
    #[cfg(feature = "rtu")]
    #[tokio::test(start_paused = true)]
    async fn test_rtu_loop_retry_after_truncated_request() {
        let request = rtu_frame(&[0x01, 0x03, 0x00, 0x00, 0x00, 0x01]);
        answered_after(
            1,
            Duration::from_micros(1750),
            &[(&request[..3], 30), (&request, 0)],
        )
        .await;
    }

    /// Review round 5, example A: a truncated request whose byte count points
    /// to a long layout, then the master retries every 30 ms. Each retry must
    /// be answered at t3.5, as 1.0.1 does — not held back and burst out.
    #[cfg(feature = "rtu")]
    #[tokio::test(start_paused = true)]
    async fn test_rtu_loop_retries_after_stuck_partial_are_answered_promptly() {
        for (own, junk) in [
            (1u8, vec![0x01, 0x10, 0x00, 0x00, 0x00, 0x7B, 0xF6]),
            // Example B: glitch equal to own ID 0x10 makes the retry's own
            // bytes read as a long FC10 layout
            (0x10, vec![0x10]),
            (0x10, vec![0x00]),
        ] {
            let bank = Arc::new(ModbusRegisterBank::new());
            bank.write_06(0, 0x1234).unwrap();
            let (mut client, tx, task) = spawn_rtu_loop_as(own, bank, Duration::from_micros(1750));
            let retry = rtu_frame(&[own, 0x03, 0x00, 0x00, 0x00, 0x7D]);
            client.write_all(&junk).await.unwrap();
            for _ in 0..5 {
                tokio::time::sleep(Duration::from_millis(30)).await;
                let start = tokio::time::Instant::now();
                client.write_all(&retry).await.unwrap();
                let mut header = [0u8; 3];
                tokio::time::timeout(Duration::from_secs(1), client.read_exact(&mut header))
                    .await
                    .expect("no reply")
                    .unwrap();
                assert_eq!(header, [own, 0x03, 250], "own {own:#04X}");
                let mut rest = [0u8; 252];
                client.read_exact(&mut rest).await.unwrap();
                assert!(
                    start.elapsed() < Duration::from_millis(5),
                    "own {own:#04X} junk {junk:02X?}: reply after {:?}",
                    start.elapsed()
                );
            }
            tx.send(()).unwrap();
            task.await.unwrap();
        }
    }

    /// Example C: a truncated request, then a maximum-size retry 30 ms later.
    /// The retry must be executed (the stale bytes must not overflow it away).
    #[cfg(feature = "rtu")]
    #[tokio::test(start_paused = true)]
    async fn test_rtu_loop_max_size_retry_after_truncated_request() {
        let bank = Arc::new(ModbusRegisterBank::new());
        let (mut client, tx, task) = spawn_rtu_loop(bank.clone(), Duration::from_micros(1750));
        let mut body = vec![0x01, 0x10, 0x00, 0x00, 0x00, 123, 246];
        for i in 0..123u16 {
            body.extend_from_slice(&i.to_be_bytes());
        }
        let retry = rtu_frame(&body);
        assert_eq!(retry.len(), 255);
        client.write_all(&retry[..7]).await.unwrap();
        tokio::time::sleep(Duration::from_millis(30)).await;
        client.write_all(&retry).await.unwrap();
        expect_reply(&mut client, &[0x01, 0x10, 0x00, 0x00, 0x00, 123]).await;
        assert_eq!(bank.read_03(122, 1).unwrap(), vec![122]);
        tx.send(()).unwrap();
        task.await.unwrap();
    }

    /// CRC-16/MODBUS has no output XOR: a CRC-valid frame followed by 0x00
    /// bytes still passes. A malformed-but-CRC-valid broadcast that keeps the
    /// wait open, then a 0x00 glitch, must not become an executed write.
    #[cfg(feature = "rtu")]
    #[tokio::test(start_paused = true)]
    async fn test_rtu_loop_zero_padding_does_not_create_a_write() {
        let bank = Arc::new(ModbusRegisterBank::new());
        let (mut client, tx, task) = spawn_rtu_loop(bank.clone(), Duration::from_micros(1750));
        // 7-byte FC06 broadcast: CRC-valid but one byte short of its layout
        client
            .write_all(&rtu_frame(&[0x00, 0x06, 0x00, 0x01, 0x55]))
            .await
            .unwrap();
        tokio::time::sleep(Duration::from_millis(10)).await;
        client.write_all(&[0x00]).await.unwrap();
        tokio::time::sleep(Duration::from_millis(200)).await;
        assert_eq!(bank.read_03(1, 1).unwrap(), vec![0]);
        tx.send(()).unwrap();
        task.await.unwrap();
    }

    /// Review round 6: a chunked request followed by 0x00 still passes the
    /// CRC as a whole. It is not reassembled (wrong length), so the server
    /// stays silent, exactly as 1.0.1 does for these gap-split segments.
    #[cfg(feature = "rtu")]
    #[tokio::test(start_paused = true)]
    async fn test_rtu_loop_does_not_reassemble_zero_padded_request() {
        let bank = Arc::new(ModbusRegisterBank::new());
        let (mut client, tx, task) = spawn_rtu_loop_as(0x06, bank, Duration::from_micros(1750));
        let request = rtu_frame(&[0x06, 0x06, 0x00, 0x01, 0x00, 0x03]);
        client.write_all(&request[..1]).await.unwrap();
        tokio::time::sleep(Duration::from_millis(16)).await;
        client.write_all(&request[1..]).await.unwrap();
        tokio::time::sleep(Duration::from_millis(1)).await;
        client.write_all(&[0x00]).await.unwrap();
        let mut reply = [0u8; 1];
        assert!(
            tokio::time::timeout(Duration::from_millis(200), client.read(&mut reply))
                .await
                .is_err(),
            "server answered a zero-padded reassembly"
        );
        tx.send(()).unwrap();
        task.await.unwrap();
    }

    /// EOF (`read` returning 0) used to spin the loop at 100% CPU.
    #[cfg(feature = "rtu")]
    #[tokio::test(start_paused = true)]
    async fn test_rtu_loop_exits_on_eof() {
        let bank = Arc::new(ModbusRegisterBank::new());
        let (client, _tx, task) = spawn_rtu_loop(bank, Duration::from_millis(2));
        drop(client);
        tokio::time::timeout(Duration::from_secs(1), task)
            .await
            .expect("RTU loop kept running after EOF")
            .unwrap();
    }

    /// A wait that does not end in a CRC-valid frame yields exactly the
    /// segments 1.0.1 would have produced at each gap.
    #[cfg(feature = "rtu")]
    #[test]
    fn test_rtu_accumulator_falls_back_to_gap_segments() {
        let request = rtu_frame(&[0x01, 0x03, 0x00, 0x00, 0x00, 0x01]);

        // Reassembled: one frame
        let mut acc = RtuFrameAccumulator::new(0x01);
        acc.push(&request[..3]);
        acc.mark_gap();
        acc.push(&request[3..]);
        assert_eq!(acc.end_frame(), vec![request.clone()]);

        // Glitch, gap, request: the 1.0.1 segments
        let mut acc = RtuFrameAccumulator::new(0x01);
        acc.push(&[0x00]);
        acc.mark_gap();
        acc.push(&request);
        assert_eq!(acc.end_frame(), vec![vec![0x00], request.clone()]);

        // Truncated request, gap, retry
        let mut acc = RtuFrameAccumulator::new(0x01);
        acc.push(&request[..3]);
        acc.mark_gap();
        acc.mark_gap(); // repeated timeouts at the same length record one gap
        acc.push(&request);
        assert_eq!(
            acc.end_frame(),
            vec![request[..3].to_vec(), request.clone()]
        );
    }

    /// Bug: continuous line noise grew `frame_buffer` without bound. An
    /// over-long burst (> 256-byte RTU ADU) is discarded as a whole — even a
    /// valid-looking tail is not answered — and the next frame works.
    #[cfg(feature = "rtu")]
    #[tokio::test(start_paused = true)]
    async fn test_rtu_loop_discards_overlong_burst() {
        let bank = Arc::new(ModbusRegisterBank::new());
        bank.write_06(0, 0x1111).unwrap();
        bank.write_06(1, 0x2222).unwrap();
        let (mut client, tx, task) = spawn_rtu_loop(bank, Duration::from_millis(2));

        let mut burst = vec![0xAAu8; 1000];
        burst.extend_from_slice(&rtu_frame(&[0x01, 0x03, 0x00, 0x01, 0x00, 0x01]));
        client.write_all(&burst).await.unwrap();
        tokio::time::sleep(Duration::from_millis(10)).await;

        client
            .write_all(&rtu_frame(&[0x01, 0x03, 0x00, 0x00, 0x00, 0x01]))
            .await
            .unwrap();
        let mut response = [0u8; 7];
        tokio::time::timeout(Duration::from_secs(1), client.read_exact(&mut response))
            .await
            .expect("no RTU reply")
            .unwrap();
        // Reply to the second request only (register 0), not the burst tail
        assert_eq!(
            &response[..],
            &rtu_frame(&[0x01, 0x03, 0x02, 0x11, 0x11])[..]
        );

        tx.send(()).unwrap();
        task.await.unwrap();
    }

    #[cfg(feature = "rtu")]
    #[test]
    fn test_rtu_frame_accumulator_caps_buffer() {
        let mut acc = RtuFrameAccumulator::new(0x01);
        assert!(acc.is_idle());

        // Exactly one max-size ADU is accepted
        acc.push(&[0x55; MAX_RTU_ADU_SIZE]);
        assert_eq!(
            acc.end_frame().iter().map(Vec::len).collect::<Vec<_>>(),
            vec![MAX_RTU_ADU_SIZE]
        );
        assert!(acc.is_idle());

        // Overflow discards the buffer and the rest of the burst
        for _ in 0..100 {
            acc.push(&[0xAA; 64]);
            assert!(acc.buffered_len() <= MAX_RTU_ADU_SIZE);
        }
        assert!(!acc.is_idle());
        assert!(acc.end_frame().is_empty());
        assert!(acc.is_idle());

        // Next frame after the gap is accumulated normally
        acc.push(&[1, 2, 3, 4]);
        assert_eq!(acc.end_frame(), vec![vec![1, 2, 3, 4]]);
        assert!(acc.end_frame().is_empty());
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
