/// Modbus register bank for server-side data storage
/// 
/// This module provides thread-safe storage for Modbus data including coils,
/// discrete inputs, holding registers, and input registers.

use std::collections::HashMap;
use std::sync::{Arc, RwLock};
use tracing::{debug, error, info, warn};
use crate::error::{ModbusError, ModbusResult};

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
        let coils = self.coils.read().map_err(|_| ModbusError::internal("Failed to lock coils"))?;
        let mut result = Vec::with_capacity(quantity as usize);
        
        for i in 0..quantity {
            let addr = address.wrapping_add(i);
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
        let mut coils = self.coils.write().map_err(|_| ModbusError::internal("Failed to lock coils"))?;
        coils.insert(address, value);
        Ok(())
    }
    
    /// Write multiple coils (function code 0x0F)
    pub fn write_0f(&self, address: u16, values: &[bool]) -> ModbusResult<()> {
        let mut coils = self.coils.write().map_err(|_| ModbusError::internal("Failed to lock coils"))?;
        for (i, &value) in values.iter().enumerate() {
            let addr = address.wrapping_add(i as u16);
            coils.insert(addr, value);
        }
        Ok(())
    }
    
    /// Read discrete inputs starting at address (function code 0x02)
    pub fn read_discrete_inputs(&self, address: u16, quantity: u16) -> ModbusResult<Vec<bool>> {
        let inputs = self.discrete_inputs.read().map_err(|_| ModbusError::internal("Failed to lock discrete inputs"))?;
        let mut result = Vec::with_capacity(quantity as usize);
        
        for i in 0..quantity {
            let addr = address.wrapping_add(i);
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
        let registers = self.holding_registers.read().map_err(|_| ModbusError::internal("Failed to lock holding registers"))?;
        let mut result = Vec::with_capacity(quantity as usize);
        
        for i in 0..quantity {
            let addr = address.wrapping_add(i);
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
        let mut registers = self.holding_registers.write().map_err(|_| ModbusError::internal("Failed to lock holding registers"))?;
        registers.insert(address, value);
        Ok(())
    }
    
    /// Write multiple registers (function code 0x10)
    pub fn write_10(&self, address: u16, values: &[u16]) -> ModbusResult<()> {
        let mut registers = self.holding_registers.write().map_err(|_| ModbusError::internal("Failed to lock holding registers"))?;
        for (i, &value) in values.iter().enumerate() {
            let addr = address.wrapping_add(i as u16);
            registers.insert(addr, value);
        }
        Ok(())
    }
    
    /// Read input registers starting at address (function code 0x04)
    pub fn read_input_registers(&self, address: u16, quantity: u16) -> ModbusResult<Vec<u16>> {
        let registers = self.input_registers.read().map_err(|_| ModbusError::internal("Failed to lock input registers"))?;
        let mut result = Vec::with_capacity(quantity as usize);
        
        for i in 0..quantity {
            let addr = address.wrapping_add(i);
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
        let mut registers = self.input_registers.write().map_err(|_| ModbusError::internal("Failed to lock input registers"))?;
        registers.insert(address, value);
        Ok(())
    }
    
    /// Set discrete input value (for simulation/testing)
    pub fn set_discrete_input(&self, address: u16, value: bool) -> ModbusResult<()> {
        let mut inputs = self.discrete_inputs.write().map_err(|_| ModbusError::internal("Failed to lock discrete inputs"))?;
        inputs.insert(address, value);
        Ok(())
    }
    
    /// Get register bank statistics
    pub fn get_stats(&self) -> RegisterBankStats {
        RegisterBankStats {
            coils_count: self.coils.read().unwrap().len(),
            discrete_inputs_count: self.discrete_inputs.read().unwrap().len(),
            holding_registers_count: self.holding_registers.read().unwrap().len(),
            input_registers_count: self.input_registers.read().unwrap().len(),
        }
    }
}

impl Default for ModbusRegisterBank {
    fn default() -> Self {
        Self::new()
    }
}

/// Register bank statistics
#[derive(Debug, Clone)]
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
        assert_eq!(coils[0], true);
        
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
} 