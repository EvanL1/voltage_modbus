//! # Command Batching for Modbus
//!
//! Provides batching functionality to group multiple Modbus write commands
//! for more efficient communication with devices.
//!
//! ## How It Works
//!
//! The batcher collects write commands and groups them by (slave_id, function_code).
//! Commands are released for execution when either:
//! - The time window expires (default 20ms)
//! - The batch size limit is reached (default 100 commands)
//!
//! This reduces network overhead and improves throughput when writing many
//! values to Modbus devices.
//!
//! ## Example
//!
//! ```rust
//! use voltage_modbus::{CommandBatcher, BatchCommand, ModbusValue, ByteOrder};
//!
//! let mut batcher = CommandBatcher::new();
//!
//! // Add commands
//! batcher.add_command(BatchCommand {
//!     point_id: 1,
//!     value: ModbusValue::U16(100),
//!     slave_id: 1,
//!     function_code: 6,
//!     register_address: 100,
//!     data_type: "uint16",
//!     byte_order: ByteOrder::BigEndian,
//! });
//!
//! // Check if batch should execute
//! if batcher.should_execute() {
//!     let commands = batcher.take_commands();
//!     // Process commands...
//! }
//! ```

use std::collections::HashMap;
use std::time::{Duration, Instant};

use crate::bytes::ByteOrder;
use crate::codec::registers_for_type;
use crate::value::ModbusValue;

/// Default batch window in milliseconds.
pub const DEFAULT_BATCH_WINDOW_MS: u64 = 20;

/// Default maximum batch size.
pub const DEFAULT_MAX_BATCH_SIZE: usize = 100;

/// A single command in a batch.
#[derive(Debug, Clone)]
pub struct BatchCommand {
    /// Unique identifier for the point/tag.
    pub point_id: u32,
    /// Value to write.
    pub value: ModbusValue,
    /// Modbus slave/unit ID.
    pub slave_id: u8,
    /// Function code (5, 6, 15, or 16).
    pub function_code: u8,
    /// Starting register address.
    pub register_address: u16,
    /// Data type string (e.g., "uint16", "float32").
    /// Uses `&'static str` to avoid heap allocation since all type names are static.
    pub data_type: &'static str,
    /// Byte order for multi-register types.
    pub byte_order: ByteOrder,
}

/// Command batcher for optimizing Modbus write communications.
///
/// Groups commands by (slave_id, function_code) and releases them
/// based on time window or batch size limits.
#[derive(Debug)]
pub struct CommandBatcher {
    /// Pending commands grouped by (slave_id, function_code).
    pending_commands: HashMap<(u8, u8), Vec<BatchCommand>>,
    /// Last batch execution time.
    last_batch_time: Instant,
    /// Total pending commands count.
    total_pending: usize,
    /// Batch window duration.
    batch_window: Duration,
    /// Maximum batch size.
    max_batch_size: usize,
}

impl CommandBatcher {
    /// Create a new command batcher with default settings.
    pub fn new() -> Self {
        Self {
            pending_commands: HashMap::new(),
            last_batch_time: Instant::now(),
            total_pending: 0,
            batch_window: Duration::from_millis(DEFAULT_BATCH_WINDOW_MS),
            max_batch_size: DEFAULT_MAX_BATCH_SIZE,
        }
    }

    /// Create a batcher with custom settings.
    ///
    /// # Arguments
    /// * `batch_window_ms` - Time window in milliseconds before batch is released
    /// * `max_batch_size` - Maximum number of commands before batch is released
    pub fn with_config(batch_window_ms: u64, max_batch_size: usize) -> Self {
        Self {
            pending_commands: HashMap::new(),
            last_batch_time: Instant::now(),
            total_pending: 0,
            batch_window: Duration::from_millis(batch_window_ms),
            max_batch_size,
        }
    }

    /// Get the number of pending commands.
    #[inline]
    pub fn pending_count(&self) -> usize {
        self.total_pending
    }

    /// Get time elapsed since last batch.
    #[inline]
    pub fn elapsed_since_last_batch(&self) -> Duration {
        self.last_batch_time.elapsed()
    }

    /// Check if batch should be executed.
    ///
    /// Returns true if:
    /// - Time window has expired, OR
    /// - Batch size limit has been reached
    pub fn should_execute(&self) -> bool {
        self.last_batch_time.elapsed() >= self.batch_window
            || self.total_pending >= self.max_batch_size
    }

    /// Take all pending commands and reset the batcher.
    ///
    /// Returns commands grouped by (slave_id, function_code).
    pub fn take_commands(&mut self) -> HashMap<(u8, u8), Vec<BatchCommand>> {
        self.last_batch_time = Instant::now();
        self.total_pending = 0;
        std::mem::take(&mut self.pending_commands)
    }

    /// Add a command to the pending batch.
    pub fn add_command(&mut self, command: BatchCommand) {
        let key = (command.slave_id, command.function_code);
        self.pending_commands.entry(key).or_default().push(command);
        self.total_pending += 1;
    }

    /// Check if registers are strictly consecutive (for FC16 batch write).
    ///
    /// This is useful for determining if multiple writes can be combined
    /// into a single FC16 (Write Multiple Registers) request.
    pub fn are_strictly_consecutive(commands: &[BatchCommand]) -> bool {
        if commands.len() < 2 {
            return false;
        }

        // Use index sorting to avoid cloning the entire command slice
        let mut indices: Vec<usize> = (0..commands.len()).collect();
        indices.sort_by_key(|&i| commands[i].register_address);

        let mut expected_addr = commands[indices[0]].register_address;

        for &idx in &indices {
            if commands[idx].register_address != expected_addr {
                return false;
            }
            // Calculate registers used by this data type
            expected_addr += Self::get_register_count(commands[idx].data_type);
        }
        true
    }

    /// Get number of 16-bit registers used by a data type.
    #[inline]
    pub fn get_register_count(data_type: &str) -> u16 {
        registers_for_type(data_type) as u16
    }

    /// Clear all pending commands without executing.
    pub fn clear(&mut self) {
        self.pending_commands.clear();
        self.total_pending = 0;
    }

    /// Check if the batcher is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.total_pending == 0
    }
}

impl Default for CommandBatcher {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_command(
        point_id: u32,
        slave_id: u8,
        function_code: u8,
        register_address: u16,
        data_type: &'static str,
    ) -> BatchCommand {
        BatchCommand {
            point_id,
            value: ModbusValue::F32(100.0),
            slave_id,
            function_code,
            register_address,
            data_type,
            byte_order: ByteOrder::BigEndian,
        }
    }

    #[test]
    fn test_new_creates_empty_batcher() {
        let batcher = CommandBatcher::new();
        assert_eq!(batcher.pending_count(), 0);
        assert!(batcher.is_empty());
    }

    #[test]
    fn test_default_is_equivalent_to_new() {
        let batcher1 = CommandBatcher::new();
        let batcher2 = CommandBatcher::default();
        assert_eq!(batcher1.pending_count(), batcher2.pending_count());
    }

    #[test]
    fn test_with_config() {
        let batcher = CommandBatcher::with_config(50, 200);
        assert_eq!(batcher.batch_window, Duration::from_millis(50));
        assert_eq!(batcher.max_batch_size, 200);
    }

    #[test]
    fn test_pending_count_after_add() {
        let mut batcher = CommandBatcher::new();

        batcher.add_command(create_test_command(1, 1, 6, 100, "uint16"));
        assert_eq!(batcher.pending_count(), 1);
        assert!(!batcher.is_empty());

        batcher.add_command(create_test_command(2, 1, 6, 101, "uint16"));
        assert_eq!(batcher.pending_count(), 2);
    }

    #[test]
    fn test_pending_count_resets_after_take() {
        let mut batcher = CommandBatcher::new();

        batcher.add_command(create_test_command(1, 1, 6, 100, "uint16"));
        batcher.add_command(create_test_command(2, 1, 6, 101, "uint16"));
        assert_eq!(batcher.pending_count(), 2);

        let _ = batcher.take_commands();
        assert_eq!(batcher.pending_count(), 0);
        assert!(batcher.is_empty());
    }

    #[test]
    fn test_should_execute_false_when_empty_and_recent() {
        let batcher = CommandBatcher::new();
        assert!(!batcher.should_execute());
    }

    #[test]
    fn test_should_execute_true_at_max_batch_size() {
        let mut batcher = CommandBatcher::new();

        for i in 0..DEFAULT_MAX_BATCH_SIZE {
            batcher.add_command(create_test_command(i as u32, 1, 6, i as u16, "uint16"));
        }

        assert!(batcher.should_execute());
    }

    #[test]
    fn test_take_commands_returns_all_pending() {
        let mut batcher = CommandBatcher::new();

        batcher.add_command(create_test_command(1, 1, 6, 100, "uint16"));
        batcher.add_command(create_test_command(2, 1, 6, 101, "uint16"));
        batcher.add_command(create_test_command(3, 2, 6, 200, "uint16"));

        let commands = batcher.take_commands();

        // Commands should be grouped by (slave_id, function_code)
        assert_eq!(commands.len(), 2); // Two groups: (1, 6) and (2, 6)
        assert_eq!(commands.get(&(1, 6)).map(|v| v.len()), Some(2));
        assert_eq!(commands.get(&(2, 6)).map(|v| v.len()), Some(1));
    }

    #[test]
    fn test_add_command_groups_by_slave_and_function() {
        let mut batcher = CommandBatcher::new();

        // Same slave, same function code
        batcher.add_command(create_test_command(1, 1, 6, 100, "uint16"));
        batcher.add_command(create_test_command(2, 1, 6, 101, "uint16"));

        // Different slave
        batcher.add_command(create_test_command(3, 2, 6, 100, "uint16"));

        // Different function code
        batcher.add_command(create_test_command(4, 1, 16, 100, "uint16"));

        let commands = batcher.take_commands();

        assert_eq!(commands.len(), 3); // (1,6), (2,6), (1,16)
        assert_eq!(commands.get(&(1, 6)).map(|v| v.len()), Some(2));
        assert_eq!(commands.get(&(2, 6)).map(|v| v.len()), Some(1));
        assert_eq!(commands.get(&(1, 16)).map(|v| v.len()), Some(1));
    }

    #[test]
    fn test_consecutive_single_command_returns_false() {
        let commands = vec![create_test_command(1, 1, 6, 100, "uint16")];
        assert!(!CommandBatcher::are_strictly_consecutive(&commands));
    }

    #[test]
    fn test_consecutive_empty_returns_false() {
        let commands: Vec<BatchCommand> = vec![];
        assert!(!CommandBatcher::are_strictly_consecutive(&commands));
    }

    #[test]
    fn test_consecutive_uint16_sequence() {
        let commands = vec![
            create_test_command(1, 1, 6, 100, "uint16"),
            create_test_command(2, 1, 6, 101, "uint16"),
            create_test_command(3, 1, 6, 102, "uint16"),
        ];
        assert!(CommandBatcher::are_strictly_consecutive(&commands));
    }

    #[test]
    fn test_consecutive_float32_sequence() {
        // float32 uses 2 registers each
        let commands = vec![
            create_test_command(1, 1, 6, 100, "float32"),
            create_test_command(2, 1, 6, 102, "float32"),
            create_test_command(3, 1, 6, 104, "float32"),
        ];
        assert!(CommandBatcher::are_strictly_consecutive(&commands));
    }

    #[test]
    fn test_consecutive_mixed_types() {
        // Mixed: uint16(1 reg) + float32(2 regs) + uint16(1 reg)
        let commands = vec![
            create_test_command(1, 1, 6, 100, "uint16"),
            create_test_command(2, 1, 6, 101, "float32"),
            create_test_command(3, 1, 6, 103, "uint16"),
        ];
        assert!(CommandBatcher::are_strictly_consecutive(&commands));
    }

    #[test]
    fn test_non_consecutive_gap() {
        let commands = vec![
            create_test_command(1, 1, 6, 100, "uint16"),
            create_test_command(2, 1, 6, 105, "uint16"), // Gap at 101-104
        ];
        assert!(!CommandBatcher::are_strictly_consecutive(&commands));
    }

    #[test]
    fn test_consecutive_out_of_order_input() {
        // Registers should be sorted internally
        let commands = vec![
            create_test_command(3, 1, 6, 102, "uint16"),
            create_test_command(1, 1, 6, 100, "uint16"),
            create_test_command(2, 1, 6, 101, "uint16"),
        ];
        assert!(CommandBatcher::are_strictly_consecutive(&commands));
    }

    #[test]
    fn test_register_count() {
        assert_eq!(CommandBatcher::get_register_count("uint16"), 1);
        assert_eq!(CommandBatcher::get_register_count("int16"), 1);
        assert_eq!(CommandBatcher::get_register_count("uint32"), 2);
        assert_eq!(CommandBatcher::get_register_count("float32"), 2);
        assert_eq!(CommandBatcher::get_register_count("uint64"), 4);
        assert_eq!(CommandBatcher::get_register_count("float64"), 4);
        assert_eq!(CommandBatcher::get_register_count("unknown"), 1);
    }

    #[test]
    fn test_clear() {
        let mut batcher = CommandBatcher::new();

        batcher.add_command(create_test_command(1, 1, 6, 100, "uint16"));
        batcher.add_command(create_test_command(2, 1, 6, 101, "uint16"));
        assert_eq!(batcher.pending_count(), 2);

        batcher.clear();
        assert_eq!(batcher.pending_count(), 0);
        assert!(batcher.is_empty());
    }

    #[test]
    fn test_batch_workflow() {
        let mut batcher = CommandBatcher::new();

        // Initially empty
        assert_eq!(batcher.pending_count(), 0);

        // Add commands
        for i in 0..5 {
            batcher.add_command(create_test_command(i, 1, 6, 100 + i as u16, "uint16"));
        }
        assert_eq!(batcher.pending_count(), 5);

        // Take and verify
        let batch = batcher.take_commands();
        assert_eq!(batch.get(&(1, 6)).unwrap().len(), 5);

        // Should be empty after take
        assert_eq!(batcher.pending_count(), 0);
        assert!(batcher.is_empty());
    }
}
