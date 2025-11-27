//! Optimized Modbus PDU data structure
//!
//! Use a fixed-size stack array to avoid heap allocation and improve performance.

use tracing::debug;

use crate::constants::MAX_PDU_SIZE;
use crate::error::{ModbusError, ModbusResult};

/// High-performance PDU with stack-allocated fixed array
#[derive(Debug, Clone)]
pub struct ModbusPdu {
    /// Fixed-size buffer (stack)
    data: [u8; MAX_PDU_SIZE],
    /// Actual data length
    len: usize,
}

impl ModbusPdu {
    /// Create an empty PDU
    #[inline]
    pub fn new() -> Self {
        Self {
            data: [0; MAX_PDU_SIZE],
            len: 0,
        }
    }

    /// Create a PDU from a byte slice
    #[inline]
    pub fn from_slice(data: &[u8]) -> ModbusResult<Self> {
        debug!("Parsing PDU from slice: {} bytes", data.len());

        if data.len() > MAX_PDU_SIZE {
            return Err(ModbusError::Protocol {
                message: format!("PDU too large: {} bytes (max {})", data.len(), MAX_PDU_SIZE),
            });
        }

        let mut pdu = Self::new();
        pdu.data[..data.len()].copy_from_slice(data);
        pdu.len = data.len();

        // Log function code details
        if let Some(fc) = pdu.function_code() {
            let fc_desc = Self::function_code_description(fc);
            if pdu.is_exception() {
                let exc_code = pdu.exception_code().unwrap_or(0);
                debug!(
                    "PDU parsed: FC={:02X} (Exception: {}), exception_code={:02X}",
                    fc, fc_desc, exc_code
                );
            } else {
                debug!(
                    "PDU parsed: FC={:02X} ({}), data_len={}",
                    fc,
                    fc_desc,
                    pdu.len - 1
                );
            }
        } else {
            debug!("PDU parsed: empty PDU");
        }

        Ok(pdu)
    }

    /// Push a single byte
    #[inline]
    pub fn push(&mut self, byte: u8) -> ModbusResult<()> {
        if self.len >= MAX_PDU_SIZE {
            return Err(ModbusError::Protocol {
                message: "PDU buffer full".to_string(),
            });
        }
        self.data[self.len] = byte;
        self.len += 1;
        Ok(())
    }

    /// Push u16 in big-endian
    #[inline]
    pub fn push_u16(&mut self, value: u16) -> ModbusResult<()> {
        self.push((value >> 8) as u8)?;
        self.push((value & 0xFF) as u8)?;
        Ok(())
    }

    /// Extend with a byte slice
    #[inline]
    pub fn extend(&mut self, data: &[u8]) -> ModbusResult<()> {
        if self.len + data.len() > MAX_PDU_SIZE {
            return Err(ModbusError::Protocol {
                message: format!(
                    "PDU would exceed max size: {} + {} > {}",
                    self.len,
                    data.len(),
                    MAX_PDU_SIZE
                ),
            });
        }
        self.data[self.len..self.len + data.len()].copy_from_slice(data);
        self.len += data.len();
        Ok(())
    }

    /// Get immutable data slice
    #[inline]
    pub fn as_slice(&self) -> &[u8] {
        &self.data[..self.len]
    }

    /// Get mutable data slice
    #[inline]
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        &mut self.data[..self.len]
    }

    /// Get current length
    #[inline]
    pub fn len(&self) -> usize {
        self.len
    }

    /// Check if empty
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Clear PDU
    #[inline]
    pub fn clear(&mut self) {
        self.len = 0;
    }

    /// Get function code (first byte)
    #[inline]
    pub fn function_code(&self) -> Option<u8> {
        if self.len > 0 {
            Some(self.data[0])
        } else {
            None
        }
    }

    /// Check if exception response
    #[inline]
    pub fn is_exception(&self) -> bool {
        self.function_code()
            .map(|fc| fc & 0x80 != 0)
            .unwrap_or(false)
    }

    /// Get exception code
    #[inline]
    pub fn exception_code(&self) -> Option<u8> {
        if self.is_exception() && self.len > 1 {
            Some(self.data[1])
        } else {
            None
        }
    }

    /// Get human-readable function code description
    pub fn function_code_description(fc: u8) -> &'static str {
        match fc & 0x7F {
            0x01 => "Read Coils",
            0x02 => "Read Discrete Inputs",
            0x03 => "Read Holding Registers",
            0x04 => "Read Input Registers",
            0x05 => "Write Single Coil",
            0x06 => "Write Single Register",
            0x0F => "Write Multiple Coils",
            0x10 => "Write Multiple Registers",
            0x17 => "Read/Write Multiple Registers",
            _ => "Unknown Function",
        }
    }
}

impl Default for ModbusPdu {
    fn default() -> Self {
        Self::new()
    }
}

/// PDU builder - fluent API
pub struct PduBuilder {
    pdu: ModbusPdu,
}

impl Default for PduBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl PduBuilder {
    /// Create a new builder
    #[inline]
    pub fn new() -> Self {
        Self {
            pdu: ModbusPdu::new(),
        }
    }

    /// Set function code
    #[inline]
    pub fn function_code(mut self, fc: u8) -> ModbusResult<Self> {
        self.pdu.push(fc)?;
        Ok(self)
    }

    /// Add address
    #[inline]
    pub fn address(mut self, addr: u16) -> ModbusResult<Self> {
        self.pdu.push_u16(addr)?;
        Ok(self)
    }

    /// Add quantity
    #[inline]
    pub fn quantity(mut self, qty: u16) -> ModbusResult<Self> {
        self.pdu.push_u16(qty)?;
        Ok(self)
    }

    /// Add a byte
    #[inline]
    pub fn byte(mut self, b: u8) -> ModbusResult<Self> {
        self.pdu.push(b)?;
        Ok(self)
    }

    /// Add data
    #[inline]
    pub fn data(mut self, data: &[u8]) -> ModbusResult<Self> {
        self.pdu.extend(data)?;
        Ok(self)
    }

    /// Build the PDU
    #[inline]
    pub fn build(self) -> ModbusPdu {
        if let Some(fc) = self.pdu.function_code() {
            let fc_desc = ModbusPdu::function_code_description(fc);
            debug!(
                "PDU built: FC={:02X} ({}), total_len={}",
                fc,
                fc_desc,
                self.pdu.len()
            );
        } else {
            debug!("PDU built: empty PDU");
        }

        self.pdu
    }

    /// Build a read request PDU for FC01-04
    ///
    /// # Arguments
    /// * `fc` - Function code (1, 2, 3, or 4)
    /// * `start_address` - Starting address for the read operation
    /// * `quantity` - Number of coils (FC01/02) or registers (FC03/04) to read
    pub fn build_read_request(
        fc: u8,
        start_address: u16,
        quantity: u16,
    ) -> ModbusResult<ModbusPdu> {
        if !matches!(fc, 0x01..=0x04) {
            return Err(ModbusError::InvalidFunction { code: fc });
        }
        Ok(PduBuilder::new()
            .function_code(fc)?
            .address(start_address)?
            .quantity(quantity)?
            .build())
    }

    /// Build a write single coil PDU (FC05)
    ///
    /// # Arguments
    /// * `address` - Coil address
    /// * `value` - Coil value (true = ON, false = OFF)
    pub fn build_write_single_coil(address: u16, value: bool) -> ModbusResult<ModbusPdu> {
        let coil_value: u16 = if value { 0xFF00 } else { 0x0000 };
        Ok(PduBuilder::new()
            .function_code(0x05)?
            .address(address)?
            .quantity(coil_value)?
            .build())
    }

    /// Build a write single register PDU (FC06)
    ///
    /// # Arguments
    /// * `address` - Register address
    /// * `value` - Register value
    pub fn build_write_single_register(address: u16, value: u16) -> ModbusResult<ModbusPdu> {
        Ok(PduBuilder::new()
            .function_code(0x06)?
            .address(address)?
            .quantity(value)?
            .build())
    }

    /// Build a write multiple coils PDU (FC15)
    ///
    /// # Arguments
    /// * `address` - Starting address
    /// * `values` - Coil values
    pub fn build_write_multiple_coils(address: u16, values: &[bool]) -> ModbusResult<ModbusPdu> {
        let quantity = values.len() as u16;
        let byte_count = (values.len() + 7) / 8;

        // Pack bits into bytes
        let mut coil_bytes = vec![0u8; byte_count];
        for (i, &value) in values.iter().enumerate() {
            if value {
                coil_bytes[i / 8] |= 1 << (i % 8);
            }
        }

        Ok(PduBuilder::new()
            .function_code(0x0F)?
            .address(address)?
            .quantity(quantity)?
            .byte(byte_count as u8)?
            .data(&coil_bytes)?
            .build())
    }

    /// Build a write multiple registers PDU (FC16)
    ///
    /// # Arguments
    /// * `address` - Starting address
    /// * `values` - Register values
    pub fn build_write_multiple_registers(address: u16, values: &[u16]) -> ModbusResult<ModbusPdu> {
        let quantity = values.len() as u16;
        let byte_count = (values.len() * 2) as u8;

        let mut builder = PduBuilder::new()
            .function_code(0x10)?
            .address(address)?
            .quantity(quantity)?
            .byte(byte_count)?;

        // Add register values in big-endian
        for &value in values {
            builder = builder
                .byte((value >> 8) as u8)?
                .byte((value & 0xFF) as u8)?;
        }

        Ok(builder.build())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pdu_basic_operations() {
        let mut pdu = ModbusPdu::new();
        assert_eq!(pdu.len(), 0);
        assert!(pdu.is_empty());

        pdu.push(0x03).unwrap();
        assert_eq!(pdu.function_code(), Some(0x03));
        assert!(!pdu.is_exception());

        pdu.push_u16(0x0100).unwrap();
        pdu.push_u16(0x000A).unwrap();

        assert_eq!(pdu.len(), 5);
        assert_eq!(pdu.as_slice(), &[0x03, 0x01, 0x00, 0x00, 0x0A]);
    }

    #[test]
    fn test_pdu_builder() {
        let pdu = PduBuilder::new()
            .function_code(0x03)
            .unwrap()
            .address(0x0100)
            .unwrap()
            .quantity(0x000A)
            .unwrap()
            .build();

        assert_eq!(pdu.len(), 5);
        assert_eq!(pdu.as_slice(), &[0x03, 0x01, 0x00, 0x00, 0x0A]);
    }

    #[test]
    fn test_exception_response() {
        let mut pdu = ModbusPdu::new();
        pdu.push(0x83).unwrap();
        pdu.push(0x02).unwrap();

        assert!(pdu.is_exception());
        assert_eq!(pdu.exception_code(), Some(0x02));
    }

    #[test]
    fn test_build_read_request() {
        let pdu = PduBuilder::build_read_request(0x03, 0x006B, 3).unwrap();

        assert_eq!(pdu.function_code(), Some(0x03));
        let data = pdu.as_slice();
        assert_eq!(data.len(), 5);
        assert_eq!(data, &[0x03, 0x00, 0x6B, 0x00, 0x03]);
    }

    #[test]
    fn test_build_write_single_coil() {
        let pdu = PduBuilder::build_write_single_coil(0x00AC, true).unwrap();

        assert_eq!(pdu.function_code(), Some(0x05));
        assert_eq!(pdu.as_slice(), &[0x05, 0x00, 0xAC, 0xFF, 0x00]);
    }

    #[test]
    fn test_build_write_single_register() {
        let pdu = PduBuilder::build_write_single_register(0x0001, 0x0003).unwrap();

        assert_eq!(pdu.function_code(), Some(0x06));
        assert_eq!(pdu.as_slice(), &[0x06, 0x00, 0x01, 0x00, 0x03]);
    }

    #[test]
    fn test_build_write_multiple_registers() {
        let pdu = PduBuilder::build_write_multiple_registers(0x0001, &[0x000A, 0x0102]).unwrap();

        assert_eq!(pdu.function_code(), Some(0x10));
        assert_eq!(
            pdu.as_slice(),
            &[0x10, 0x00, 0x01, 0x00, 0x02, 0x04, 0x00, 0x0A, 0x01, 0x02]
        );
    }
}
