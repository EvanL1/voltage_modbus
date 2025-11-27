use crate::error::{ModbusError, ModbusResult};
/// Utility functions and helpers for Modbus operations
///
/// This module contains various utility functions for data conversion,
/// logging, and performance monitoring.
use std::time::{Duration, Instant};
use tracing::{debug, info, warn};

/// Performance metrics for Modbus operations
#[derive(Debug, Clone, Default)]
pub struct PerformanceMetrics {
    pub total_requests: u64,
    pub successful_requests: u64,
    pub failed_requests: u64,
    pub total_duration: Duration,
    pub min_duration: Option<Duration>,
    pub max_duration: Option<Duration>,
    pub avg_duration: Duration,
}

impl PerformanceMetrics {
    /// Create new empty metrics
    pub fn new() -> Self {
        Self::default()
    }

    /// Record a successful operation
    pub fn record_success(&mut self, duration: Duration) {
        self.total_requests += 1;
        self.successful_requests += 1;
        self.total_duration += duration;

        self.min_duration = Some(self.min_duration.map_or(duration, |min| min.min(duration)));
        self.max_duration = Some(self.max_duration.map_or(duration, |max| max.max(duration)));

        if self.total_requests > 0 {
            self.avg_duration = self.total_duration / self.total_requests as u32;
        }
    }

    /// Record a failed operation
    pub fn record_failure(&mut self, duration: Duration) {
        self.total_requests += 1;
        self.failed_requests += 1;
        self.total_duration += duration;

        if self.total_requests > 0 {
            self.avg_duration = self.total_duration / self.total_requests as u32;
        }
    }

    /// Get success rate as percentage
    pub fn success_rate(&self) -> f64 {
        if self.total_requests == 0 {
            return 0.0;
        }
        (self.successful_requests as f64 / self.total_requests as f64) * 100.0
    }

    /// Get requests per second
    pub fn requests_per_second(&self) -> f64 {
        if self.total_duration.is_zero() {
            return 0.0;
        }
        self.total_requests as f64 / self.total_duration.as_secs_f64()
    }

    /// Reset all metrics
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

/// Timer for measuring operation duration
pub struct OperationTimer {
    start: Instant,
    operation_name: String,
}

impl OperationTimer {
    /// Start a new timer
    pub fn start(operation_name: &str) -> Self {
        debug!("Starting operation: {}", operation_name);
        Self {
            start: Instant::now(),
            operation_name: operation_name.to_string(),
        }
    }

    /// Stop the timer and return duration
    pub fn stop(self) -> Duration {
        let duration = self.start.elapsed();
        debug!(
            "Operation '{}' completed in {:?}",
            self.operation_name, duration
        );
        duration
    }

    /// Stop timer and log result
    pub fn stop_and_log(self, success: bool) -> Duration {
        let duration = self.start.elapsed();
        if success {
            info!(
                "✅ Operation '{}' succeeded in {:?}",
                self.operation_name, duration
            );
        } else {
            warn!(
                "❌ Operation '{}' failed after {:?}",
                self.operation_name, duration
            );
        }
        duration
    }
}

/// Data validation utilities
pub mod validation {
    use super::*;

    /// Validate slave ID (1-247)
    pub fn validate_slave_id(slave_id: u8) -> ModbusResult<()> {
        if slave_id == 0 || slave_id > 247 {
            return Err(ModbusError::invalid_data(format!(
                "Invalid slave ID: {} (must be 1-247)",
                slave_id
            )));
        }
        Ok(())
    }

    /// Validate address range
    pub fn validate_address_range(start: u16, count: u16) -> ModbusResult<()> {
        if count == 0 {
            return Err(ModbusError::invalid_address(start, count));
        }

        if (start as u32 + count as u32) > 65536 {
            return Err(ModbusError::invalid_address(start, count));
        }

        Ok(())
    }

    /// Validate register count for read operations
    pub fn validate_register_count(count: u16) -> ModbusResult<()> {
        if count == 0 || count > crate::MAX_READ_REGISTERS as u16 {
            return Err(ModbusError::invalid_data(format!(
                "Invalid register count: {} (must be 1-{})",
                count,
                crate::MAX_READ_REGISTERS
            )));
        }
        Ok(())
    }

    /// Validate coil count for read operations
    pub fn validate_coil_count(count: u16) -> ModbusResult<()> {
        if count == 0 || count > crate::MAX_READ_COILS as u16 {
            return Err(ModbusError::invalid_data(format!(
                "Invalid coil count: {} (must be 1-{})",
                count,
                crate::MAX_READ_COILS
            )));
        }
        Ok(())
    }
}

/// Formatting and display utilities
pub mod format {
    use super::*;

    /// Format byte array as hex string
    pub fn bytes_to_hex(bytes: &[u8]) -> String {
        bytes
            .iter()
            .map(|b| format!("{:02X}", b))
            .collect::<Vec<_>>()
            .join(" ")
    }

    /// Format register values as hex
    pub fn registers_to_hex(registers: &[u16]) -> String {
        registers
            .iter()
            .map(|r| format!("{:04X}", r))
            .collect::<Vec<_>>()
            .join(" ")
    }

    /// Format duration in a human-readable way
    pub fn format_duration(duration: Duration) -> String {
        let millis = duration.as_millis();
        if millis < 1000 {
            format!("{}ms", millis)
        } else if millis < 60_000 {
            format!("{:.2}s", duration.as_secs_f64())
        } else {
            let mins = millis / 60_000;
            let secs = (millis % 60_000) as f64 / 1000.0;
            format!("{}m {:.1}s", mins, secs)
        }
    }

    /// Format transfer rate (bytes/sec)
    pub fn format_transfer_rate(bytes: u64, duration: Duration) -> String {
        if duration.is_zero() {
            return "0 B/s".to_string();
        }

        let rate = bytes as f64 / duration.as_secs_f64();

        if rate < 1024.0 {
            format!("{:.1} B/s", rate)
        } else if rate < 1024.0 * 1024.0 {
            format!("{:.1} KB/s", rate / 1024.0)
        } else {
            format!("{:.1} MB/s", rate / (1024.0 * 1024.0))
        }
    }

    /// Format performance metrics as a table
    pub fn format_metrics(metrics: &PerformanceMetrics) -> String {
        format!(
            "Performance Metrics:\n\
             ├─ Total Requests: {}\n\
             ├─ Successful: {} ({:.1}%)\n\
             ├─ Failed: {}\n\
             ├─ Average Duration: {}\n\
             ├─ Min Duration: {}\n\
             ├─ Max Duration: {}\n\
             └─ Requests/sec: {:.1}",
            metrics.total_requests,
            metrics.successful_requests,
            metrics.success_rate(),
            metrics.failed_requests,
            format_duration(metrics.avg_duration),
            metrics
                .min_duration
                .map_or("N/A".to_string(), format_duration),
            metrics
                .max_duration
                .map_or("N/A".to_string(), format_duration),
            metrics.requests_per_second()
        )
    }
}

/// Logging utilities
pub mod logging {
    use super::*;

    /// Initialize simple logger for testing (no-op, use tracing_subscriber)
    pub fn init_test_logger() {
        // Use tracing_subscriber for logging initialization
        // This is a no-op here; users should initialize tracing themselves
    }

    /// Log request/response for debugging
    pub fn log_request_response(
        slave_id: u8,
        function: &str,
        address: u16,
        data: &[u8],
        duration: Duration,
        success: bool,
    ) {
        let status = if success { "✅" } else { "❌" };
        let data_str = format::bytes_to_hex(data);

        debug!(
            "{} Slave {} {} @ {} | Data: {} | Duration: {}",
            status,
            slave_id,
            function,
            address,
            data_str,
            format::format_duration(duration)
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_performance_metrics() {
        let mut metrics = PerformanceMetrics::new();

        metrics.record_success(Duration::from_millis(100));
        metrics.record_success(Duration::from_millis(200));
        metrics.record_failure(Duration::from_millis(150));

        assert_eq!(metrics.total_requests, 3);
        assert_eq!(metrics.successful_requests, 2);
        assert_eq!(metrics.failed_requests, 1);
        assert!((metrics.success_rate() - 66.67).abs() < 0.1);
    }

    #[test]
    fn test_validation() {
        assert!(validation::validate_slave_id(1).is_ok());
        assert!(validation::validate_slave_id(247).is_ok());
        assert!(validation::validate_slave_id(0).is_err());
        assert!(validation::validate_slave_id(248).is_err());

        assert!(validation::validate_address_range(0, 10).is_ok());
        assert!(validation::validate_address_range(65530, 5).is_ok());
        assert!(validation::validate_address_range(65530, 10).is_err());
    }

    #[test]
    fn test_formatting() {
        let bytes = vec![0x01, 0x03, 0x10, 0xFF];
        assert_eq!(format::bytes_to_hex(&bytes), "01 03 10 FF");

        let registers = vec![0x1234, 0x5678];
        assert_eq!(format::registers_to_hex(&registers), "1234 5678");

        let duration = Duration::from_millis(1500);
        assert_eq!(format::format_duration(duration), "1.50s");
    }
}
