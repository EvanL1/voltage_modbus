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
/// [`ModbusRtuServer::set_device_identity`]; FC 0x2B requests are then
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

        let current = register_bank.read_03(address, 1)?[0];
        let result = (current & and_mask) | (or_mask & !and_mask);
        register_bank.write_06(address, result)?;

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

        // Write first, then read (spec-mandated ordering)
        register_bank.write_10(write_address, &values)?;
        let registers = register_bank.read_03(read_address, read_quantity)?;

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
    async fn handle_rtu_communication(
        mut port: tokio_serial::SerialStream,
        own_slave_id: u8,
        service: Arc<dyn ModbusService>,
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
                                    own_slave_id,
                                    service.as_ref(),
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
                                    own_slave_id,
                                    service.as_ref(),
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
        own_slave_id: u8,
        service: &dyn ModbusService,
        stats: &Arc<Mutex<ServerStats>>,
    ) {
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
