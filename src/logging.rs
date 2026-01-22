use std::fmt::Write;
use std::sync::Arc;

/// Format bytes as hex string efficiently
///
/// Uses direct string writing for better performance than collect/join.
#[inline]
fn format_hex(data: &[u8]) -> String {
    if data.is_empty() {
        return String::new();
    }
    let mut result = String::with_capacity(data.len() * 3 - 1);
    for (i, b) in data.iter().enumerate() {
        if i > 0 {
            result.push(' ');
        }
        let _ = write!(result, "{:02X}", b);
    }
    result
}

/// Log levels for the callback logging system
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LogLevel {
    /// Error messages
    Error,
    /// Warning messages
    Warn,
    /// Informational messages
    Info,
    /// Debug messages
    Debug,
}

/// Logging mode for packet display
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoggingMode {
    /// Show raw packet data only
    Raw,
    /// Show interpreted packet data with field descriptions
    Interpreted,
    /// Show both raw and interpreted data
    Both,
}

impl LogLevel {
    /// Convert log level to string
    pub fn as_str(&self) -> &'static str {
        match self {
            LogLevel::Error => "ERROR",
            LogLevel::Warn => "WARN",
            LogLevel::Info => "INFO",
            LogLevel::Debug => "DEBUG",
        }
    }
}

/// Type alias for log callback functions
///
/// The callback receives a log level and message string
pub type LogCallback = Box<dyn Fn(LogLevel, &str) + Send + Sync>;

/// Logger that uses callbacks for flexible logging
#[derive(Clone)]
pub struct CallbackLogger {
    callback: Option<Arc<LogCallback>>,
    min_level: LogLevel,
    mode: LoggingMode,
}

impl CallbackLogger {
    /// Create a new callback logger
    pub fn new(callback: Option<LogCallback>, min_level: LogLevel) -> Self {
        Self {
            callback: callback.map(Arc::new),
            min_level,
            mode: LoggingMode::Interpreted,
        }
    }

    /// Create a new callback logger with specific mode
    pub fn with_mode(
        callback: Option<LogCallback>,
        min_level: LogLevel,
        mode: LoggingMode,
    ) -> Self {
        Self {
            callback: callback.map(Arc::new),
            min_level,
            mode,
        }
    }

    /// Create a logger with default console output
    pub fn console() -> Self {
        let callback: LogCallback = Box::new(|level, message| {
            use std::time::SystemTime;
            let now = SystemTime::now()
                .duration_since(SystemTime::UNIX_EPOCH)
                .unwrap_or_default();
            let secs = now.as_secs();
            let millis = now.subsec_millis();
            match level {
                LogLevel::Error => eprintln!("[{}.{:03}] ERROR: {}", secs, millis, message),
                LogLevel::Warn => eprintln!("[{}.{:03}] WARN: {}", secs, millis, message),
                LogLevel::Info => println!("[{}.{:03}] INFO: {}", secs, millis, message),
                LogLevel::Debug => println!("[{}.{:03}] DEBUG: {}", secs, millis, message),
            }
        });
        Self::new(Some(callback), LogLevel::Info)
    }

    /// Create a logger that outputs nothing (disabled)
    pub fn disabled() -> Self {
        Self::new(None, LogLevel::Error)
    }

    /// Set logging mode
    pub fn set_mode(&mut self, mode: LoggingMode) {
        self.mode = mode;
    }

    /// Get current logging mode
    pub fn get_mode(&self) -> LoggingMode {
        self.mode
    }

    /// Log a message at the specified level
    pub fn log(&self, level: LogLevel, message: &str) {
        if self.should_log(level) {
            if let Some(ref callback) = self.callback {
                callback(level, message);
            }
        }
    }

    /// Log an error message
    pub fn error(&self, message: &str) {
        self.log(LogLevel::Error, message);
    }

    /// Log a warning message
    pub fn warn(&self, message: &str) {
        self.log(LogLevel::Warn, message);
    }

    /// Log an info message
    pub fn info(&self, message: &str) {
        self.log(LogLevel::Info, message);
    }

    /// Log a debug message
    pub fn debug(&self, message: &str) {
        self.log(LogLevel::Debug, message);
    }

    /// Check if a message at the given level should be logged
    fn should_log(&self, level: LogLevel) -> bool {
        self.callback.is_some() && level as u8 <= self.min_level as u8
    }

    /// Log packet data with hex dump
    pub fn log_packet(&self, level: LogLevel, direction: &str, data: &[u8]) {
        if !self.should_log(level) {
            return;
        }

        let hex_data = format_hex(data);
        let message = format!("{} packet ({} bytes): {}", direction, data.len(), hex_data);
        self.log(level, &message);
    }

    /// Log a Modbus request with different modes
    ///
    /// # Arguments
    ///
    /// * `transaction_id` - Optional transaction ID for TCP (None for RTU/ASCII)
    /// * `slave_id` - The Modbus slave/unit ID
    /// * `function_code` - The Modbus function code
    /// * `address` - The starting address
    /// * `quantity` - The quantity of registers/coils
    /// * `data` - Additional data bytes
    pub fn log_request(
        &self,
        transaction_id: Option<u16>,
        slave_id: u8,
        function_code: u8,
        address: u16,
        quantity: u16,
        data: &[u8],
    ) {
        match self.mode {
            LoggingMode::Raw => {
                let raw_packet = self.build_raw_request_packet(
                    transaction_id.unwrap_or(1),
                    slave_id,
                    function_code,
                    address,
                    quantity,
                    data,
                );
                let hex_data = format_hex(&raw_packet);
                let message = format!("Modbus Request -> Raw: {}", hex_data);
                self.info(&message);
            }
            LoggingMode::Interpreted => {
                let function_name = self.get_function_name(function_code);
                let message = format!(
                    "Modbus Request -> Slave: {}, Function: {} (0x{:02X}), Address: {}, Quantity: {}",
                    slave_id, function_name, function_code, address, quantity
                );
                self.info(&message);
            }
            LoggingMode::Both => {
                // Log interpreted first
                let function_name = self.get_function_name(function_code);
                let interpreted = format!(
                    "Modbus Request -> Slave: {}, Function: {} (0x{:02X}), Address: {}, Quantity: {}",
                    slave_id, function_name, function_code, address, quantity
                );
                self.info(&interpreted);

                // Then log raw
                let raw_packet = self.build_raw_request_packet(
                    transaction_id.unwrap_or(1),
                    slave_id,
                    function_code,
                    address,
                    quantity,
                    data,
                );
                let hex_data = format_hex(&raw_packet);
                let raw_message = format!("Modbus Request -> Raw: {}", hex_data);
                self.debug(&raw_message);
            }
        }
    }

    /// Log a Modbus response with different modes
    ///
    /// # Arguments
    ///
    /// * `transaction_id` - Optional transaction ID for TCP (None for RTU/ASCII)
    /// * `slave_id` - The Modbus slave/unit ID
    /// * `function_code` - The Modbus function code
    /// * `data` - Response data bytes
    pub fn log_response(
        &self,
        transaction_id: Option<u16>,
        slave_id: u8,
        function_code: u8,
        data: &[u8],
    ) {
        match self.mode {
            LoggingMode::Raw => {
                let raw_packet = self.build_raw_response_packet(
                    transaction_id.unwrap_or(1),
                    slave_id,
                    function_code,
                    data,
                );
                let hex_data = format_hex(&raw_packet);
                let message = format!("Modbus Response <- Raw: {}", hex_data);
                self.info(&message);
            }
            LoggingMode::Interpreted => {
                let function_name = self.get_function_name(function_code);
                let interpreted_data = self.interpret_response_data(function_code, data);
                let message = format!(
                    "Modbus Response <- Slave: {}, Function: {} (0x{:02X}), {}",
                    slave_id, function_name, function_code, interpreted_data
                );
                self.info(&message);
            }
            LoggingMode::Both => {
                // Log interpreted first
                let function_name = self.get_function_name(function_code);
                let interpreted_data = self.interpret_response_data(function_code, data);
                let interpreted = format!(
                    "Modbus Response <- Slave: {}, Function: {} (0x{:02X}), {}",
                    slave_id, function_name, function_code, interpreted_data
                );
                self.info(&interpreted);

                // Then log raw
                let raw_packet = self.build_raw_response_packet(
                    transaction_id.unwrap_or(1),
                    slave_id,
                    function_code,
                    data,
                );
                let hex_data = format_hex(&raw_packet);
                let raw_message = format!("Modbus Response <- Raw: {}", hex_data);
                self.debug(&raw_message);
            }
        }
    }

    /// Build raw request packet for logging
    fn build_raw_request_packet(
        &self,
        transaction_id: u16,
        slave_id: u8,
        function_code: u8,
        address: u16,
        quantity: u16,
        data: &[u8],
    ) -> Vec<u8> {
        let mut packet = Vec::new();
        // TCP MBAP header
        packet.extend_from_slice(&transaction_id.to_be_bytes()); // Transaction ID
        packet.extend_from_slice(&[0x00, 0x00]); // Protocol ID
        packet.extend_from_slice(&[0x00, (6 + data.len()) as u8]); // Length
        packet.push(slave_id);
        packet.push(function_code);
        packet.extend_from_slice(&address.to_be_bytes());
        packet.extend_from_slice(&quantity.to_be_bytes());
        packet.extend_from_slice(data);
        packet
    }

    /// Build raw response packet for logging
    fn build_raw_response_packet(
        &self,
        transaction_id: u16,
        slave_id: u8,
        function_code: u8,
        data: &[u8],
    ) -> Vec<u8> {
        let mut packet = Vec::new();
        // TCP MBAP header
        packet.extend_from_slice(&transaction_id.to_be_bytes()); // Transaction ID
        packet.extend_from_slice(&[0x00, 0x00]); // Protocol ID
        packet.extend_from_slice(&[0x00, (2 + data.len()) as u8]); // Length
        packet.push(slave_id);
        packet.push(function_code);
        packet.extend_from_slice(data);
        packet
    }

    /// Simple hex encoding helper (no spaces between bytes)
    fn hex_encode(data: &[u8]) -> String {
        // Pre-allocate: 2 hex chars per byte
        let mut result = String::with_capacity(data.len() * 2);
        for b in data {
            let _ = write!(result, "{:02X}", b);
        }
        result
    }

    /// Get human-readable function name
    fn get_function_name(&self, function_code: u8) -> &'static str {
        match function_code {
            0x01 => "Read Coils",
            0x02 => "Read Discrete Inputs",
            0x03 => "Read Holding Registers",
            0x04 => "Read Input Registers",
            0x05 => "Write Single Coil",
            0x06 => "Write Single Register",
            0x0F => "Write Multiple Coils",
            0x10 => "Write Multiple Registers",
            _ => "Unknown Function",
        }
    }

    /// Interpret response data based on function code
    fn interpret_response_data(&self, function_code: u8, data: &[u8]) -> String {
        if data.is_empty() {
            return "No data".to_string();
        }

        match function_code {
            0x01 | 0x02 => {
                // Coils or discrete inputs
                if data.len() >= 2 {
                    let byte_count = data[0];
                    let mut coils = Vec::new();
                    for i in 1..=byte_count as usize {
                        if i < data.len() {
                            for bit in 0..8 {
                                coils.push((data[i] & (1 << bit)) != 0);
                            }
                        }
                    }
                    format!(
                        "Byte count: {}, Coils: {:?}",
                        byte_count,
                        &coils[..coils.len().min(16)]
                    )
                } else {
                    format!("Data: {}", Self::hex_encode(data))
                }
            }
            0x03 | 0x04 => {
                // Holding registers or input registers
                if data.len() >= 3 {
                    let byte_count = data[0];
                    let mut registers = Vec::new();
                    for i in (1..data.len()).step_by(2) {
                        if i + 1 < data.len() {
                            let value = u16::from_be_bytes([data[i], data[i + 1]]);
                            registers.push(value);
                        }
                    }
                    format!(
                        "Byte count: {}, Registers: {:?}",
                        byte_count,
                        &registers[..registers.len().min(8)]
                    )
                } else {
                    format!("Data: {}", Self::hex_encode(data))
                }
            }
            0x05 => {
                // Write single coil response
                if data.len() >= 4 {
                    let address = u16::from_be_bytes([data[0], data[1]]);
                    let value = u16::from_be_bytes([data[2], data[3]]);
                    format!(
                        "Address: {}, Value: 0x{:04X} ({})",
                        address,
                        value,
                        if value == 0xFF00 { "ON" } else { "OFF" }
                    )
                } else {
                    format!("Data: {}", Self::hex_encode(data))
                }
            }
            0x06 => {
                // Write single register response
                if data.len() >= 4 {
                    let address = u16::from_be_bytes([data[0], data[1]]);
                    let value = u16::from_be_bytes([data[2], data[3]]);
                    format!("Address: {}, Value: {} (0x{:04X})", address, value, value)
                } else {
                    format!("Data: {}", Self::hex_encode(data))
                }
            }
            0x0F | 0x10 => {
                // Write multiple coils/registers response
                if data.len() >= 4 {
                    let address = u16::from_be_bytes([data[0], data[1]]);
                    let quantity = u16::from_be_bytes([data[2], data[3]]);
                    format!("Address: {}, Quantity: {}", address, quantity)
                } else {
                    format!("Data: {}", Self::hex_encode(data))
                }
            }
            _ => {
                format!("Data: {}", Self::hex_encode(data))
            }
        }
    }
}

impl Default for CallbackLogger {
    fn default() -> Self {
        Self::disabled()
    }
}

/// Convenience macro for creating a simple console logger
#[macro_export]
macro_rules! console_logger {
    () => {
        $crate::logging::CallbackLogger::console()
    };
}

/// Convenience macro for creating a custom logger
#[macro_export]
macro_rules! custom_logger {
    ($callback:expr) => {
        $crate::logging::CallbackLogger::new(Some($callback), $crate::logging::LogLevel::Info)
    };
    ($callback:expr, $level:expr) => {
        $crate::logging::CallbackLogger::new(Some($callback), $level)
    };
    ($callback:expr, $level:expr, $mode:expr) => {
        $crate::logging::CallbackLogger::with_mode(Some($callback), $level, $mode)
    };
}
