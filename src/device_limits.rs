//! # Device Limits Configuration
//!
//! Configuration for device-specific Modbus protocol limits.
//! Different devices have different capabilities for read/write operations.
//!
//! ## Modbus Specification Limits
//!
//! - **Read Holding/Input Registers (FC03/04)**: Max 125 registers per request
//! - **Read Coils/Discrete Inputs (FC01/02)**: Max 2000 bits per request
//! - **Write Multiple Registers (FC16)**: Max 123 registers per request
//! - **Write Multiple Coils (FC15)**: Max 1968 coils per request
//!
//! Some devices may have lower limits. This module allows configuring
//! per-device limits for optimal communication.

/// Default maximum registers per read operation (Modbus specification).
pub const DEFAULT_MAX_READ_REGISTERS: u16 = 125;

/// Default maximum registers per write operation (Modbus specification).
pub const DEFAULT_MAX_WRITE_REGISTERS: u16 = 123;

/// Default maximum coils per read operation (Modbus specification).
pub const DEFAULT_MAX_READ_COILS: u16 = 2000;

/// Default maximum coils per write operation (Modbus specification).
pub const DEFAULT_MAX_WRITE_COILS: u16 = 1968;

/// Default inter-request delay in milliseconds.
pub const DEFAULT_INTER_REQUEST_DELAY_MS: u64 = 0;

/// Device-specific Modbus protocol limits.
///
/// Use this to configure limits for devices that don't support
/// the full Modbus specification limits.
///
/// # Example
///
/// ```rust
/// use voltage_modbus::DeviceLimits;
///
/// // Create limits for a device that only supports 50 registers per read
/// let limits = DeviceLimits::new()
///     .with_max_read_registers(50)
///     .with_inter_request_delay_ms(10);
///
/// assert_eq!(limits.max_read_registers, 50);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeviceLimits {
    /// Maximum registers per read request.
    pub max_read_registers: u16,
    /// Maximum registers per write request.
    pub max_write_registers: u16,
    /// Maximum coils per read request.
    pub max_read_coils: u16,
    /// Maximum coils per write request.
    pub max_write_coils: u16,
    /// Minimum delay between requests (milliseconds).
    pub inter_request_delay_ms: u64,
}

impl DeviceLimits {
    /// Create new device limits with default (Modbus spec) values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Create limits for a conservative/slow device.
    ///
    /// Uses lower limits suitable for older or less capable devices:
    /// - 50 registers per read/write
    /// - 500 coils per read/write
    /// - 10ms inter-request delay
    pub fn conservative() -> Self {
        Self {
            max_read_registers: 50,
            max_write_registers: 50,
            max_read_coils: 500,
            max_write_coils: 500,
            inter_request_delay_ms: 10,
        }
    }

    /// Set maximum read registers.
    pub fn with_max_read_registers(mut self, count: u16) -> Self {
        self.max_read_registers = count;
        self
    }

    /// Set maximum write registers.
    pub fn with_max_write_registers(mut self, count: u16) -> Self {
        self.max_write_registers = count;
        self
    }

    /// Set maximum read coils.
    pub fn with_max_read_coils(mut self, count: u16) -> Self {
        self.max_read_coils = count;
        self
    }

    /// Set maximum write coils.
    pub fn with_max_write_coils(mut self, count: u16) -> Self {
        self.max_write_coils = count;
        self
    }

    /// Set inter-request delay in milliseconds.
    pub fn with_inter_request_delay_ms(mut self, delay_ms: u64) -> Self {
        self.inter_request_delay_ms = delay_ms;
        self
    }

    /// Calculate the number of read requests needed for a given register count.
    pub fn read_request_count(&self, total_registers: u16) -> u16 {
        if total_registers == 0 {
            return 0;
        }
        total_registers.div_ceil(self.max_read_registers)
    }

    /// Calculate the number of write requests needed for a given register count.
    pub fn write_request_count(&self, total_registers: u16) -> u16 {
        if total_registers == 0 {
            return 0;
        }
        total_registers.div_ceil(self.max_write_registers)
    }

    /// Check if a read request is within limits.
    pub fn is_read_within_limits(&self, register_count: u16) -> bool {
        register_count <= self.max_read_registers
    }

    /// Check if a write request is within limits.
    pub fn is_write_within_limits(&self, register_count: u16) -> bool {
        register_count <= self.max_write_registers
    }

    /// Check if a coil read request is within limits.
    pub fn is_coil_read_within_limits(&self, coil_count: u16) -> bool {
        coil_count <= self.max_read_coils
    }

    /// Check if a coil write request is within limits.
    pub fn is_coil_write_within_limits(&self, coil_count: u16) -> bool {
        coil_count <= self.max_write_coils
    }
}

impl Default for DeviceLimits {
    fn default() -> Self {
        Self {
            max_read_registers: DEFAULT_MAX_READ_REGISTERS,
            max_write_registers: DEFAULT_MAX_WRITE_REGISTERS,
            max_read_coils: DEFAULT_MAX_READ_COILS,
            max_write_coils: DEFAULT_MAX_WRITE_COILS,
            inter_request_delay_ms: DEFAULT_INTER_REQUEST_DELAY_MS,
        }
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_limits() {
        let limits = DeviceLimits::default();
        assert_eq!(limits.max_read_registers, DEFAULT_MAX_READ_REGISTERS);
        assert_eq!(limits.max_write_registers, DEFAULT_MAX_WRITE_REGISTERS);
        assert_eq!(limits.max_read_coils, DEFAULT_MAX_READ_COILS);
        assert_eq!(limits.max_write_coils, DEFAULT_MAX_WRITE_COILS);
        assert_eq!(
            limits.inter_request_delay_ms,
            DEFAULT_INTER_REQUEST_DELAY_MS
        );
    }

    #[test]
    fn test_conservative_limits() {
        let limits = DeviceLimits::conservative();
        assert_eq!(limits.max_read_registers, 50);
        assert_eq!(limits.max_write_registers, 50);
        assert_eq!(limits.inter_request_delay_ms, 10);
    }

    #[test]
    fn test_builder_pattern() {
        let limits = DeviceLimits::new()
            .with_max_read_registers(60)
            .with_max_write_registers(40)
            .with_inter_request_delay_ms(5);

        assert_eq!(limits.max_read_registers, 60);
        assert_eq!(limits.max_write_registers, 40);
        assert_eq!(limits.inter_request_delay_ms, 5);
    }

    #[test]
    fn test_read_request_count() {
        let limits = DeviceLimits::new().with_max_read_registers(50);

        assert_eq!(limits.read_request_count(0), 0);
        assert_eq!(limits.read_request_count(50), 1);
        assert_eq!(limits.read_request_count(51), 2);
        assert_eq!(limits.read_request_count(100), 2);
        assert_eq!(limits.read_request_count(101), 3);
    }

    #[test]
    fn test_write_request_count() {
        let limits = DeviceLimits::new().with_max_write_registers(100);

        assert_eq!(limits.write_request_count(0), 0);
        assert_eq!(limits.write_request_count(100), 1);
        assert_eq!(limits.write_request_count(101), 2);
        assert_eq!(limits.write_request_count(250), 3);
    }

    #[test]
    fn test_is_read_within_limits() {
        let limits = DeviceLimits::new().with_max_read_registers(100);

        assert!(limits.is_read_within_limits(100));
        assert!(!limits.is_read_within_limits(101));
    }

    #[test]
    fn test_is_write_within_limits() {
        let limits = DeviceLimits::new().with_max_write_registers(80);

        assert!(limits.is_write_within_limits(80));
        assert!(!limits.is_write_within_limits(81));
    }

    #[test]
    fn test_is_coil_within_limits() {
        let limits = DeviceLimits::new()
            .with_max_read_coils(1000)
            .with_max_write_coils(500);

        assert!(limits.is_coil_read_within_limits(1000));
        assert!(!limits.is_coil_read_within_limits(1001));

        assert!(limits.is_coil_write_within_limits(500));
        assert!(!limits.is_coil_write_within_limits(501));
    }
}
