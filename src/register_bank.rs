//! Modbus register bank for server-side data storage
//!
//! This module provides thread-safe storage for Modbus data including coils,
//! discrete inputs, holding registers, and input registers.

use crate::error::{ModbusError, ModbusResult};
use std::collections::HashMap;
use std::sync::{Arc, RwLock};

/// Default register bank size
const DEFAULT_COILS_SIZE: usize = 10000;
const DEFAULT_DISCRETE_INPUTS_SIZE: usize = 10000;
const DEFAULT_HOLDING_REGISTERS_SIZE: usize = 10000;
const DEFAULT_INPUT_REGISTERS_SIZE: usize = 10000;

/// Modbus register bank for storing coils, discrete inputs, holding registers, and input registers
///
/// This structure provides thread-safe access to Modbus data through Arc<RwLock<_>> wrappers.
/// All register operations use 0-based addressing internally.
#[derive(Debug, Clone)]
pub struct ModbusRegisterBank {
    /// Coils (read/write) - 1 bit each
    coils: Arc<RwLock<HashMap<u16, bool>>>,
    /// Discrete inputs (read-only) - 1 bit each
    discrete_inputs: Arc<RwLock<HashMap<u16, bool>>>,
    /// Holding registers (read/write) - 16 bits each
    holding_registers: Arc<RwLock<HashMap<u16, u16>>>,
    /// Input registers (read-only) - 16 bits each  
    input_registers: Arc<RwLock<HashMap<u16, u16>>>,
}

impl ModbusRegisterBank {
    /// Create a new register bank with empty data
    pub fn new() -> Self {
        Self {
            coils: Arc::new(RwLock::new(HashMap::new())),
            discrete_inputs: Arc::new(RwLock::new(HashMap::new())),
            holding_registers: Arc::new(RwLock::new(HashMap::new())),
            input_registers: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    /// Create a new register bank with pre-allocated capacity
    ///
    /// This avoids HashMap reallocations when populating the register bank.
    /// Use this when you know the approximate number of registers you'll need.
    ///
    /// # Arguments
    /// * `coils_cap` - Expected number of coils
    /// * `discrete_inputs_cap` - Expected number of discrete inputs
    /// * `holding_registers_cap` - Expected number of holding registers
    /// * `input_registers_cap` - Expected number of input registers
    pub fn with_capacity(
        coils_cap: usize,
        discrete_inputs_cap: usize,
        holding_registers_cap: usize,
        input_registers_cap: usize,
    ) -> Self {
        Self {
            coils: Arc::new(RwLock::new(HashMap::with_capacity(coils_cap))),
            discrete_inputs: Arc::new(RwLock::new(HashMap::with_capacity(discrete_inputs_cap))),
            holding_registers: Arc::new(RwLock::new(HashMap::with_capacity(holding_registers_cap))),
            input_registers: Arc::new(RwLock::new(HashMap::with_capacity(input_registers_cap))),
        }
    }

    /// Create a new register bank with default capacity
    ///
    /// Uses the default sizes: 10000 for each register type.
    /// This is suitable for typical industrial applications.
    pub fn with_default_capacity() -> Self {
        Self::with_capacity(
            DEFAULT_COILS_SIZE,
            DEFAULT_DISCRETE_INPUTS_SIZE,
            DEFAULT_HOLDING_REGISTERS_SIZE,
            DEFAULT_INPUT_REGISTERS_SIZE,
        )
    }

    /// Read coils starting at address (function code 0x01)
    pub fn read_coils(&self, address: u16, quantity: u16) -> ModbusResult<Vec<bool>> {
        validate_address_range(address, quantity)?;
        let coils = self
            .coils
            .read()
            .map_err(|_| ModbusError::internal("Failed to lock coils"))?;
        let mut result = Vec::with_capacity(quantity as usize);

        for i in 0..quantity {
            let addr = checked_address(address, i)?;
            result.push(coils.get(&addr).copied().unwrap_or(false));
        }

        Ok(result)
    }

    /// Alias for read_coils using function code naming
    pub fn read_01(&self, address: u16, quantity: u16) -> ModbusResult<Vec<bool>> {
        self.read_coils(address, quantity)
    }

    /// Write single coil (function code 0x05)
    pub fn write_05(&self, address: u16, value: bool) -> ModbusResult<()> {
        let mut coils = self
            .coils
            .write()
            .map_err(|_| ModbusError::internal("Failed to lock coils"))?;
        coils.insert(address, value);
        Ok(())
    }

    /// Write multiple coils (function code 0x0F)
    pub fn write_0f(&self, address: u16, values: &[bool]) -> ModbusResult<()> {
        let quantity = u16::try_from(values.len())
            .map_err(|_| ModbusError::invalid_address(address, u16::MAX))?;
        validate_address_range(address, quantity)?;
        let mut coils = self
            .coils
            .write()
            .map_err(|_| ModbusError::internal("Failed to lock coils"))?;
        for (i, &value) in values.iter().enumerate() {
            let addr = checked_address(address, i)?;
            coils.insert(addr, value);
        }
        Ok(())
    }

    /// Read discrete inputs starting at address (function code 0x02)
    pub fn read_discrete_inputs(&self, address: u16, quantity: u16) -> ModbusResult<Vec<bool>> {
        validate_address_range(address, quantity)?;
        let inputs = self
            .discrete_inputs
            .read()
            .map_err(|_| ModbusError::internal("Failed to lock discrete inputs"))?;
        let mut result = Vec::with_capacity(quantity as usize);

        for i in 0..quantity {
            let addr = checked_address(address, i)?;
            result.push(inputs.get(&addr).copied().unwrap_or(false));
        }

        Ok(result)
    }

    /// Alias for read_discrete_inputs using function code naming
    pub fn read_02(&self, address: u16, quantity: u16) -> ModbusResult<Vec<bool>> {
        self.read_discrete_inputs(address, quantity)
    }

    /// Read holding registers starting at address (function code 0x03)
    pub fn read_holding_registers(&self, address: u16, quantity: u16) -> ModbusResult<Vec<u16>> {
        validate_address_range(address, quantity)?;
        let registers = self
            .holding_registers
            .read()
            .map_err(|_| ModbusError::internal("Failed to lock holding registers"))?;
        let mut result = Vec::with_capacity(quantity as usize);

        for i in 0..quantity {
            let addr = checked_address(address, i)?;
            result.push(registers.get(&addr).copied().unwrap_or(0));
        }

        Ok(result)
    }

    /// Alias for read_holding_registers using function code naming
    pub fn read_03(&self, address: u16, quantity: u16) -> ModbusResult<Vec<u16>> {
        self.read_holding_registers(address, quantity)
    }

    /// Write single register (function code 0x06)
    pub fn write_06(&self, address: u16, value: u16) -> ModbusResult<()> {
        let mut registers = self
            .holding_registers
            .write()
            .map_err(|_| ModbusError::internal("Failed to lock holding registers"))?;
        registers.insert(address, value);
        Ok(())
    }

    /// Write multiple registers (function code 0x10)
    pub fn write_10(&self, address: u16, values: &[u16]) -> ModbusResult<()> {
        let quantity = u16::try_from(values.len())
            .map_err(|_| ModbusError::invalid_address(address, u16::MAX))?;
        validate_address_range(address, quantity)?;
        let mut registers = self
            .holding_registers
            .write()
            .map_err(|_| ModbusError::internal("Failed to lock holding registers"))?;
        for (i, &value) in values.iter().enumerate() {
            let addr = checked_address(address, i)?;
            registers.insert(addr, value);
        }
        Ok(())
    }

    /// Mask write register (function code 0x16), atomically.
    ///
    /// Computes `(current & and_mask) | (or_mask & !and_mask)` and stores it
    /// under a single write lock, so concurrent writers cannot interleave
    /// between the read and the write. Returns the new register value.
    pub(crate) fn mask_write_register(
        &self,
        address: u16,
        and_mask: u16,
        or_mask: u16,
    ) -> ModbusResult<u16> {
        let mut registers = self
            .holding_registers
            .write()
            .map_err(|_| ModbusError::internal("Failed to lock holding registers"))?;
        let current = registers.get(&address).copied().unwrap_or(0);
        let result = (current & and_mask) | (or_mask & !and_mask);
        registers.insert(address, result);
        Ok(result)
    }

    /// Write then read holding registers (function code 0x17), atomically.
    ///
    /// Per spec the write is performed before the read; both happen under a
    /// single write lock. Both ranges are validated before anything is
    /// written, so an invalid request leaves the bank unchanged.
    pub(crate) fn write_read_registers(
        &self,
        write_address: u16,
        values: &[u16],
        read_address: u16,
        read_quantity: u16,
    ) -> ModbusResult<Vec<u16>> {
        let write_quantity = u16::try_from(values.len())
            .map_err(|_| ModbusError::invalid_address(write_address, u16::MAX))?;
        validate_address_range(write_address, write_quantity)?;
        validate_address_range(read_address, read_quantity)?;
        let mut registers = self
            .holding_registers
            .write()
            .map_err(|_| ModbusError::internal("Failed to lock holding registers"))?;
        for (i, &value) in values.iter().enumerate() {
            registers.insert(checked_address(write_address, i)?, value);
        }
        (0..read_quantity)
            .map(|i| {
                checked_address(read_address, i)
                    .map(|addr| registers.get(&addr).copied().unwrap_or(0))
            })
            .collect()
    }

    /// Read input registers starting at address (function code 0x04)
    pub fn read_input_registers(&self, address: u16, quantity: u16) -> ModbusResult<Vec<u16>> {
        validate_address_range(address, quantity)?;
        let registers = self
            .input_registers
            .read()
            .map_err(|_| ModbusError::internal("Failed to lock input registers"))?;
        let mut result = Vec::with_capacity(quantity as usize);

        for i in 0..quantity {
            let addr = checked_address(address, i)?;
            result.push(registers.get(&addr).copied().unwrap_or(0));
        }

        Ok(result)
    }

    /// Alias for read_input_registers using function code naming
    pub fn read_04(&self, address: u16, quantity: u16) -> ModbusResult<Vec<u16>> {
        self.read_input_registers(address, quantity)
    }

    /// Set input register value (for simulation/testing)
    pub fn set_input_register(&self, address: u16, value: u16) -> ModbusResult<()> {
        let mut registers = self
            .input_registers
            .write()
            .map_err(|_| ModbusError::internal("Failed to lock input registers"))?;
        registers.insert(address, value);
        Ok(())
    }

    /// Set discrete input value (for simulation/testing)
    pub fn set_discrete_input(&self, address: u16, value: bool) -> ModbusResult<()> {
        let mut inputs = self
            .discrete_inputs
            .write()
            .map_err(|_| ModbusError::internal("Failed to lock discrete inputs"))?;
        inputs.insert(address, value);
        Ok(())
    }

    /// Get register bank statistics
    pub fn get_stats(&self) -> RegisterBankStats {
        RegisterBankStats {
            coils_count: self.coils.read().map(|coils| coils.len()).unwrap_or(0),
            discrete_inputs_count: self
                .discrete_inputs
                .read()
                .map(|inputs| inputs.len())
                .unwrap_or(0),
            holding_registers_count: self
                .holding_registers
                .read()
                .map(|registers| registers.len())
                .unwrap_or(0),
            input_registers_count: self
                .input_registers
                .read()
                .map(|registers| registers.len())
                .unwrap_or(0),
        }
    }
}

fn validate_address_range(address: u16, quantity: u16) -> ModbusResult<()> {
    if quantity == 0 {
        return Err(ModbusError::invalid_address(address, quantity));
    }

    if address.checked_add(quantity - 1).is_none() {
        return Err(ModbusError::invalid_address(address, quantity));
    }

    Ok(())
}

fn checked_address(address: u16, offset: impl TryInto<u16>) -> ModbusResult<u16> {
    let offset = offset
        .try_into()
        .map_err(|_| ModbusError::invalid_address(address, u16::MAX))?;
    address
        .checked_add(offset)
        .ok_or_else(|| ModbusError::invalid_address(address, offset.saturating_add(1)))
}

impl Default for ModbusRegisterBank {
    fn default() -> Self {
        Self::new()
    }
}

/// Register bank statistics
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct RegisterBankStats {
    pub coils_count: usize,
    pub discrete_inputs_count: usize,
    pub holding_registers_count: usize,
    pub input_registers_count: usize,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_coil_operations() {
        let bank = ModbusRegisterBank::new();

        // Test single coil write and read
        bank.write_05(10, true).unwrap();
        let coils = bank.read_01(10, 1).unwrap();
        assert!(coils[0]);

        // Test multiple coil operations
        bank.write_0f(20, &[true, false, true]).unwrap();
        let coils = bank.read_01(20, 3).unwrap();
        assert_eq!(coils, vec![true, false, true]);
    }

    #[test]
    fn test_register_operations() {
        let bank = ModbusRegisterBank::new();

        // Test single register write and read
        bank.write_06(5, 42).unwrap();
        let registers = bank.read_03(5, 1).unwrap();
        assert_eq!(registers[0], 42);

        // Test multiple register operations
        bank.write_10(100, &[100, 200, 300]).unwrap();
        let registers = bank.read_03(100, 3).unwrap();
        assert_eq!(registers, vec![100, 200, 300]);
    }

    #[test]
    fn test_mask_write_register() {
        let bank = ModbusRegisterBank::new();
        bank.write_06(4, 0x0012).unwrap();

        // Spec example: (0x12 & 0xF2) | (0x25 & !0xF2) = 0x17
        assert_eq!(bank.mask_write_register(4, 0x00F2, 0x0025).unwrap(), 0x0017);
        assert_eq!(bank.read_03(4, 1).unwrap(), vec![0x0017]);

        // Unset register reads as 0; OR bits inside the AND mask are ignored
        assert_eq!(bank.mask_write_register(9, 0xFFFF, 0x00FF).unwrap(), 0x0000);
        assert_eq!(bank.mask_write_register(9, 0x0000, 0x00FF).unwrap(), 0x00FF);
    }

    #[test]
    fn test_write_read_registers_writes_before_read() {
        let bank = ModbusRegisterBank::new();
        bank.write_10(0, &[1, 2, 3, 4]).unwrap();

        // Overlapping ranges: the read must observe the write
        let read = bank.write_read_registers(1, &[20, 30], 0, 4).unwrap();
        assert_eq!(read, vec![1, 20, 30, 4]);

        // Invalid read range fails without performing the write
        assert!(bank.write_read_registers(0, &[99], u16::MAX, 2).is_err());
        assert_eq!(bank.read_03(0, 1).unwrap(), vec![1]);
        assert!(bank.write_read_registers(0, &[], 0, 1).is_err());
    }

    #[test]
    fn test_range_overflow_is_rejected() {
        let bank = ModbusRegisterBank::new();

        assert!(bank.read_03(u16::MAX, 2).is_err());
        assert!(bank.write_10(u16::MAX, &[1, 2]).is_err());
        assert!(bank.read_01(10, 0).is_err());
    }
}
