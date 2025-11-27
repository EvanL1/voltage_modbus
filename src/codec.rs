//! # Modbus Codec
//!
//! Encoding and decoding of Modbus data types with byte order support.
//! Provides conversion between raw registers and typed values.
//!
//! ## Supported Data Types
//!
//! | Type | Registers | Aliases |
//! |------|-----------|---------|
//! | bool | 1 (coil) | boolean |
//! | u16 | 1 | uint16, word |
//! | i16 | 1 | int16, short |
//! | u32 | 2 | uint32, dword |
//! | i32 | 2 | int32, long |
//! | f32 | 2 | float32, float, real |
//! | u64 | 4 | uint64, qword |
//! | i64 | 4 | int64, longlong |
//! | f64 | 4 | float64, double, lreal |

use crate::bytes::{bytes_4_to_regs, bytes_8_to_regs, regs_to_bytes_4, regs_to_bytes_8, ByteOrder};
use crate::constants;
use crate::error::{ModbusError, ModbusResult};
use crate::pdu::{ModbusPdu, PduBuilder};
use crate::value::ModbusValue;

/// Modbus codec for data encoding/decoding.
pub struct ModbusCodec;

// ============================================================================
// Decoding Functions
// ============================================================================

/// Decode Modbus register values to ModbusValue based on data format.
///
/// Supports multiple data types with configurable byte ordering:
/// - `bool`: Single bit extraction from register (0-15 bit position)
/// - `uint16`, `int16`: Single 16-bit register
/// - `uint32`, `int32`, `float32`: Two 16-bit registers
/// - `uint64`, `int64`, `float64`: Four 16-bit registers
///
/// # Arguments
/// * `registers` - Raw register values from Modbus response
/// * `data_type` - Data type string (e.g., "uint16", "float32", "bool")
/// * `bit_position` - For bool type: which bit to extract (0-15, LSB=0)
/// * `byte_order` - Byte ordering for multi-register types
///
/// # Example
///
/// ```rust
/// use voltage_modbus::{decode_register_value, ByteOrder, ModbusValue};
///
/// // Decode a 32-bit unsigned integer from 2 registers
/// let registers = [0x1234, 0x5678];
/// let value = decode_register_value(&registers, "uint32", 0, ByteOrder::BigEndian).unwrap();
/// assert_eq!(value, ModbusValue::U32(0x12345678));
/// ```
pub fn decode_register_value(
    registers: &[u16],
    data_type: &str,
    bit_position: u8,
    byte_order: ByteOrder,
) -> ModbusResult<ModbusValue> {
    match data_type.to_lowercase().as_str() {
        "bool" | "boolean" | "coil" => {
            if registers.is_empty() {
                return Err(ModbusError::InvalidData {
                    message: "No registers for bool".to_string(),
                });
            }

            if bit_position > 15 {
                return Err(ModbusError::InvalidData {
                    message: format!("Invalid bit position: {} (must be 0-15)", bit_position),
                });
            }

            let value = registers[0];
            let bit_value = (value >> bit_position) & 0x01;
            Ok(ModbusValue::Bool(bit_value != 0))
        }

        "uint16" | "u16" | "word" => {
            if registers.is_empty() {
                return Err(ModbusError::InvalidData {
                    message: "No registers for uint16".to_string(),
                });
            }
            Ok(ModbusValue::U16(registers[0]))
        }

        "int16" | "i16" | "short" => {
            if registers.is_empty() {
                return Err(ModbusError::InvalidData {
                    message: "No registers for int16".to_string(),
                });
            }
            Ok(ModbusValue::I16(registers[0] as i16))
        }

        "uint32" | "u32" | "dword" => {
            if registers.len() < 2 {
                return Err(ModbusError::InvalidData {
                    message: "Not enough registers for uint32".to_string(),
                });
            }
            let regs: [u16; 2] = [registers[0], registers[1]];
            let bytes = regs_to_bytes_4(&regs, byte_order);
            Ok(ModbusValue::U32(u32::from_be_bytes(bytes)))
        }

        "int32" | "i32" | "long" => {
            if registers.len() < 2 {
                return Err(ModbusError::InvalidData {
                    message: "Not enough registers for int32".to_string(),
                });
            }
            let regs: [u16; 2] = [registers[0], registers[1]];
            let bytes = regs_to_bytes_4(&regs, byte_order);
            Ok(ModbusValue::I32(i32::from_be_bytes(bytes)))
        }

        "float32" | "f32" | "float" | "real" => {
            if registers.len() < 2 {
                return Err(ModbusError::InvalidData {
                    message: "Not enough registers for float32".to_string(),
                });
            }
            let regs: [u16; 2] = [registers[0], registers[1]];
            let bytes = regs_to_bytes_4(&regs, byte_order);
            Ok(ModbusValue::F32(f32::from_be_bytes(bytes)))
        }

        "uint64" | "u64" | "qword" => {
            if registers.len() < 4 {
                return Err(ModbusError::InvalidData {
                    message: "Not enough registers for uint64".to_string(),
                });
            }
            let regs: [u16; 4] = [registers[0], registers[1], registers[2], registers[3]];
            let bytes = regs_to_bytes_8(&regs, byte_order);
            Ok(ModbusValue::U64(u64::from_be_bytes(bytes)))
        }

        "int64" | "i64" | "longlong" => {
            if registers.len() < 4 {
                return Err(ModbusError::InvalidData {
                    message: "Not enough registers for int64".to_string(),
                });
            }
            let regs: [u16; 4] = [registers[0], registers[1], registers[2], registers[3]];
            let bytes = regs_to_bytes_8(&regs, byte_order);
            Ok(ModbusValue::I64(i64::from_be_bytes(bytes)))
        }

        "float64" | "f64" | "double" | "lreal" => {
            if registers.len() < 4 {
                return Err(ModbusError::InvalidData {
                    message: "Not enough registers for float64".to_string(),
                });
            }
            let regs: [u16; 4] = [registers[0], registers[1], registers[2], registers[3]];
            let bytes = regs_to_bytes_8(&regs, byte_order);
            Ok(ModbusValue::F64(f64::from_be_bytes(bytes)))
        }

        _ => Err(ModbusError::InvalidData {
            message: format!("Unsupported data type: {}", data_type),
        }),
    }
}

/// Clamp a value to the valid range for a given Modbus data type.
///
/// Prevents overflow when writing values that exceed the target register's
/// capacity (e.g., writing 70000 to a uint16 register).
///
/// # Arguments
/// * `value` - The value to clamp
/// * `data_type` - Target data type (e.g., "uint16", "int32", "float32")
///
/// # Returns
/// The clamped value, or the original value if the type is unknown/boolean
pub fn clamp_to_data_type(value: f64, data_type: &str) -> f64 {
    let (min, max): (f64, f64) = match data_type.to_lowercase().as_str() {
        "uint16" | "u16" => (0.0, 65535.0),
        "int16" | "i16" => (-32768.0, 32767.0),
        "uint32" | "u32" => (0.0, 4294967295.0),
        "int32" | "i32" => (-2147483648.0, 2147483647.0),
        "uint64" | "u64" => (0.0, u64::MAX as f64),
        "int64" | "i64" => (i64::MIN as f64, i64::MAX as f64),
        "float32" | "f32" => (f32::MIN as f64, f32::MAX as f64),
        "float64" | "f64" => (f64::MIN, f64::MAX),
        // Boolean types don't need range clamping
        "bool" | "boolean" | "coil" => return value,
        // Unknown type - return as-is
        _ => return value,
    };

    value.clamp(min, max)
}

/// Parse a Modbus response PDU and extract register data.
///
/// This function implements graceful degradation - it will attempt to parse
/// as much valid data as possible even when the response is incomplete.
///
/// # Arguments
/// * `pdu` - The Modbus PDU to parse
/// * `function_code` - Expected function code (1, 2, 3, or 4)
/// * `expected_count` - Expected number of coils (FC01/02) or registers (FC03/04)
///
/// # Returns
/// - For FC01/02: Vec of bytes (each stored as u16 for uniform processing)
/// - For FC03/04: Vec of 16-bit register values
pub fn parse_read_response(
    pdu: &ModbusPdu,
    function_code: u8,
    expected_count: u16,
) -> ModbusResult<Vec<u16>> {
    let pdu_data = pdu.as_slice();

    // Minimum viable PDU check
    if pdu_data.len() < 2 {
        return Ok(Vec::new()); // Return empty instead of failing
    }

    let actual_fc = pdu.function_code().unwrap_or(0);
    if actual_fc != function_code {
        return Err(ModbusError::Protocol {
            message: format!(
                "Function code mismatch: expected {}, got {}",
                function_code, actual_fc
            ),
        });
    }

    let byte_count = pdu_data[1] as usize;
    let available_bytes = pdu_data.len().saturating_sub(2);
    let actual_byte_count = byte_count.min(available_bytes);

    match function_code {
        1 | 2 => {
            // FC 01/02: coils/discrete inputs
            let _expected_bytes = expected_count.div_ceil(8) as usize;
            let mut registers = Vec::new();
            for &byte in &pdu_data[2..2 + actual_byte_count] {
                registers.push(u16::from(byte));
            }
            Ok(registers)
        }
        3 | 4 => {
            // FC 03/04: holding/input registers
            let mut registers = Vec::new();
            let complete_pairs = actual_byte_count / 2;

            for i in 0..complete_pairs {
                let offset = 2 + i * 2;
                if offset + 1 < pdu_data.len() {
                    let value =
                        (u16::from(pdu_data[offset]) << 8) | u16::from(pdu_data[offset + 1]);
                    registers.push(value);
                }
            }
            Ok(registers)
        }
        _ => Err(ModbusError::Protocol {
            message: format!("Unsupported function code: {}", function_code),
        }),
    }
}

// ============================================================================
// Encoding Functions
// ============================================================================

/// Encode a ModbusValue for Modbus transmission.
///
/// Converts typed values to register arrays with proper byte ordering.
///
/// # Example
///
/// ```rust
/// use voltage_modbus::{encode_value, ByteOrder, ModbusValue};
///
/// let value = ModbusValue::U32(0x12345678);
/// let registers = encode_value(&value, ByteOrder::BigEndian).unwrap();
/// assert_eq!(registers, vec![0x1234, 0x5678]);
/// ```
pub fn encode_value(value: &ModbusValue, byte_order: ByteOrder) -> ModbusResult<Vec<u16>> {
    match value {
        ModbusValue::Bool(b) => Ok(vec![if *b { 1 } else { 0 }]),
        ModbusValue::U16(v) => Ok(vec![*v]),
        ModbusValue::I16(v) => Ok(vec![*v as u16]),
        ModbusValue::U32(v) => {
            let bytes = v.to_be_bytes();
            Ok(bytes_4_to_regs(&bytes, byte_order).to_vec())
        }
        ModbusValue::I32(v) => {
            let bytes = v.to_be_bytes();
            Ok(bytes_4_to_regs(&bytes, byte_order).to_vec())
        }
        ModbusValue::F32(v) => {
            let bytes = v.to_be_bytes();
            Ok(bytes_4_to_regs(&bytes, byte_order).to_vec())
        }
        ModbusValue::U64(v) => {
            let bytes = v.to_be_bytes();
            Ok(bytes_8_to_regs(&bytes, byte_order).to_vec())
        }
        ModbusValue::I64(v) => {
            let bytes = v.to_be_bytes();
            Ok(bytes_8_to_regs(&bytes, byte_order).to_vec())
        }
        ModbusValue::F64(v) => {
            let bytes = v.to_be_bytes();
            Ok(bytes_8_to_regs(&bytes, byte_order).to_vec())
        }
    }
}

/// Encode a value from f64 with specified data type for Modbus transmission.
///
/// This is useful when you have a generic numeric value and need to encode
/// it as a specific Modbus data type.
///
/// # Example
///
/// ```rust
/// use voltage_modbus::{encode_f64_as_type, ByteOrder};
///
/// let registers = encode_f64_as_type(123.456, "float32", ByteOrder::BigEndian).unwrap();
/// assert_eq!(registers.len(), 2);
/// ```
pub fn encode_f64_as_type(
    value: f64,
    data_type: &str,
    byte_order: ByteOrder,
) -> ModbusResult<Vec<u16>> {
    let clamped = clamp_to_data_type(value, data_type);

    match data_type.to_lowercase().as_str() {
        "bool" | "boolean" | "coil" => Ok(vec![if clamped != 0.0 { 1 } else { 0 }]),
        "uint16" | "u16" | "word" => Ok(vec![clamped as u16]),
        "int16" | "i16" | "short" => Ok(vec![(clamped as i16) as u16]),
        "uint32" | "u32" | "dword" => {
            let bytes = (clamped as u32).to_be_bytes();
            Ok(bytes_4_to_regs(&bytes, byte_order).to_vec())
        }
        "int32" | "i32" | "long" => {
            let bytes = (clamped as i32).to_be_bytes();
            Ok(bytes_4_to_regs(&bytes, byte_order).to_vec())
        }
        "float32" | "f32" | "float" | "real" => {
            let bytes = (clamped as f32).to_be_bytes();
            Ok(bytes_4_to_regs(&bytes, byte_order).to_vec())
        }
        "uint64" | "u64" | "qword" => {
            let bytes = (clamped as u64).to_be_bytes();
            Ok(bytes_8_to_regs(&bytes, byte_order).to_vec())
        }
        "int64" | "i64" | "longlong" => {
            let bytes = (clamped as i64).to_be_bytes();
            Ok(bytes_8_to_regs(&bytes, byte_order).to_vec())
        }
        "float64" | "f64" | "double" | "lreal" => {
            let bytes = clamped.to_be_bytes();
            Ok(bytes_8_to_regs(&bytes, byte_order).to_vec())
        }
        _ => Err(ModbusError::InvalidData {
            message: format!("Unsupported data type: {}", data_type),
        }),
    }
}

// ============================================================================
// PDU Building Functions
// ============================================================================

impl ModbusCodec {
    /// Build write PDU for FC05 (Write Single Coil).
    pub fn build_fc05_pdu(address: u16, value: bool) -> ModbusResult<ModbusPdu> {
        Ok(PduBuilder::new()
            .function_code(0x05)?
            .address(address)?
            .byte(if value { 0xFF } else { 0x00 })?
            .byte(0x00)?
            .build())
    }

    /// Build write PDU for FC06 (Write Single Register).
    pub fn build_fc06_pdu(address: u16, value: u16) -> ModbusResult<ModbusPdu> {
        Ok(PduBuilder::new()
            .function_code(0x06)?
            .address(address)?
            .quantity(value)?
            .build())
    }

    /// Build write PDU for FC15 (Write Multiple Coils).
    pub fn build_fc15_pdu(start_address: u16, values: &[bool]) -> ModbusResult<ModbusPdu> {
        if values.is_empty() || values.len() > constants::MAX_WRITE_COILS {
            return Err(ModbusError::InvalidData {
                message: "Invalid coil count for FC15".to_string(),
            });
        }

        let mut pdu = ModbusPdu::new();

        // Function code
        pdu.push(0x0F)?;

        // Starting address
        pdu.push_u16(start_address)?;

        // Quantity of coils
        let quantity = values.len() as u16;
        pdu.push_u16(quantity)?;

        // Byte count
        let byte_count = values.len().div_ceil(8) as u8;
        pdu.push(byte_count)?;

        // Coil values (packed as bits)
        let mut current_byte = 0u8;
        let mut bit_index = 0;

        for &value in values {
            if value {
                current_byte |= 1 << bit_index;
            }
            bit_index += 1;

            if bit_index == 8 {
                pdu.push(current_byte)?;
                current_byte = 0;
                bit_index = 0;
            }
        }

        // Push last byte if needed
        if bit_index > 0 {
            pdu.push(current_byte)?;
        }

        Ok(pdu)
    }

    /// Build write PDU for FC16 (Write Multiple Registers).
    pub fn build_fc16_pdu(start_address: u16, values: &[u16]) -> ModbusResult<ModbusPdu> {
        if values.is_empty() || values.len() > constants::MAX_WRITE_REGISTERS {
            return Err(ModbusError::InvalidData {
                message: "Invalid register count for FC16".to_string(),
            });
        }

        let mut pdu = ModbusPdu::new();

        // Function code
        pdu.push(0x10)?;

        // Starting address
        pdu.push_u16(start_address)?;

        // Quantity of registers
        let quantity = values.len() as u16;
        pdu.push_u16(quantity)?;

        // Byte count
        let byte_count = (values.len() * 2) as u8;
        pdu.push(byte_count)?;

        // Register values
        for &value in values {
            pdu.push_u16(value)?;
        }

        Ok(pdu)
    }

    /// Parse write response PDU.
    pub fn parse_write_response(pdu: &ModbusPdu, expected_fc: u8) -> ModbusResult<bool> {
        let data = pdu.as_slice();

        if data.is_empty() {
            return Err(ModbusError::Protocol {
                message: "Empty response PDU".to_string(),
            });
        }

        // Check for exception response
        if data[0] & 0x80 != 0 {
            let exception_code = if data.len() > 1 { data[1] } else { 0 };
            return Err(ModbusError::Exception {
                function: data[0] & 0x7F,
                code: exception_code,
                message: format!("Exception code {:02X}", exception_code),
            });
        }

        // Verify function code
        if data[0] != expected_fc {
            return Err(ModbusError::Protocol {
                message: format!(
                    "Function code mismatch: expected {:02X}, got {:02X}",
                    expected_fc, data[0]
                ),
            });
        }

        Ok(true)
    }
}

/// Get the number of registers required for a data type.
pub fn registers_for_type(data_type: &str) -> usize {
    match data_type.to_lowercase().as_str() {
        "bool" | "boolean" | "coil" => 0, // Coils use separate addressing
        "uint16" | "u16" | "word" | "int16" | "i16" | "short" => 1,
        "uint32" | "u32" | "dword" | "int32" | "i32" | "long" | "float32" | "f32" | "float"
        | "real" => 2,
        "uint64" | "u64" | "qword" | "int64" | "i64" | "longlong" | "float64" | "f64"
        | "double" | "lreal" => 4,
        _ => 1, // Default to 1 register for unknown types
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_decode_uint16() {
        let registers = [0x1234];
        let value = decode_register_value(&registers, "uint16", 0, ByteOrder::BigEndian).unwrap();
        assert_eq!(value, ModbusValue::U16(0x1234));
    }

    #[test]
    fn test_decode_int16() {
        let registers = [0xFFFF]; // -1 in two's complement
        let value = decode_register_value(&registers, "int16", 0, ByteOrder::BigEndian).unwrap();
        assert_eq!(value, ModbusValue::I16(-1));
    }

    #[test]
    fn test_decode_uint32_big_endian() {
        let registers = [0x1234, 0x5678];
        let value = decode_register_value(&registers, "uint32", 0, ByteOrder::BigEndian).unwrap();
        assert_eq!(value, ModbusValue::U32(0x12345678));
    }

    #[test]
    fn test_decode_uint32_big_endian_swap() {
        // CDAB: Word-swapped big-endian (common in Modbus)
        let registers = [0x5678, 0x1234]; // Swapped order
        let value =
            decode_register_value(&registers, "uint32", 0, ByteOrder::BigEndianSwap).unwrap();
        assert_eq!(value, ModbusValue::U32(0x12345678));
    }

    #[test]
    fn test_decode_float32() {
        // 25.0 in IEEE 754: 0x41C80000
        let registers = [0x41C8, 0x0000];
        let value = decode_register_value(&registers, "float32", 0, ByteOrder::BigEndian).unwrap();
        if let ModbusValue::F32(f) = value {
            assert!((f - 25.0).abs() < f32::EPSILON);
        } else {
            panic!("Expected F32");
        }
    }

    #[test]
    fn test_decode_bool_bit_extraction() {
        let registers = [0b0000_0100]; // Bit 2 is set
        let value = decode_register_value(&registers, "bool", 2, ByteOrder::BigEndian).unwrap();
        assert_eq!(value, ModbusValue::Bool(true));

        let value = decode_register_value(&registers, "bool", 0, ByteOrder::BigEndian).unwrap();
        assert_eq!(value, ModbusValue::Bool(false));
    }

    #[test]
    fn test_encode_uint32_roundtrip() {
        let original = ModbusValue::U32(0x12345678);
        for order in [
            ByteOrder::BigEndian,
            ByteOrder::LittleEndian,
            ByteOrder::BigEndianSwap,
            ByteOrder::LittleEndianSwap,
        ] {
            let registers = encode_value(&original, order).unwrap();
            let decoded = decode_register_value(&registers, "uint32", 0, order).unwrap();
            assert_eq!(decoded, original, "Roundtrip failed for {:?}", order);
        }
    }

    #[test]
    fn test_encode_float32_roundtrip() {
        let original = ModbusValue::F32(123.456);
        for order in [
            ByteOrder::BigEndian,
            ByteOrder::LittleEndian,
            ByteOrder::BigEndianSwap,
            ByteOrder::LittleEndianSwap,
        ] {
            let registers = encode_value(&original, order).unwrap();
            let decoded = decode_register_value(&registers, "float32", 0, order).unwrap();
            if let (ModbusValue::F32(orig), ModbusValue::F32(dec)) = (&original, &decoded) {
                assert!(
                    (orig - dec).abs() < 0.001,
                    "Roundtrip failed for {:?}",
                    order
                );
            } else {
                panic!("Type mismatch");
            }
        }
    }

    #[test]
    fn test_clamp_to_data_type() {
        assert_eq!(clamp_to_data_type(70000.0, "uint16"), 65535.0);
        assert_eq!(clamp_to_data_type(-100.0, "uint16"), 0.0);
        assert_eq!(clamp_to_data_type(40000.0, "int16"), 32767.0);
        assert_eq!(clamp_to_data_type(-40000.0, "int16"), -32768.0);
    }

    #[test]
    fn test_registers_for_type() {
        assert_eq!(registers_for_type("bool"), 0);
        assert_eq!(registers_for_type("uint16"), 1);
        assert_eq!(registers_for_type("int32"), 2);
        assert_eq!(registers_for_type("float64"), 4);
    }

    #[test]
    fn test_build_fc05_pdu() {
        let pdu = ModbusCodec::build_fc05_pdu(0x0100, true).unwrap();
        assert_eq!(pdu.as_slice(), &[0x05, 0x01, 0x00, 0xFF, 0x00]);
    }

    #[test]
    fn test_build_fc06_pdu() {
        let pdu = ModbusCodec::build_fc06_pdu(0x0100, 0x1234).unwrap();
        assert_eq!(pdu.as_slice(), &[0x06, 0x01, 0x00, 0x12, 0x34]);
    }

    #[test]
    fn test_build_fc15_pdu() {
        let pdu = ModbusCodec::build_fc15_pdu(0x0100, &[true, false, true]).unwrap();
        // FC15: [FC, AddrH, AddrL, QtyH, QtyL, ByteCount, Data...]
        assert_eq!(
            pdu.as_slice(),
            &[0x0F, 0x01, 0x00, 0x00, 0x03, 0x01, 0b0000_0101]
        );
    }

    #[test]
    fn test_build_fc16_pdu() {
        let pdu = ModbusCodec::build_fc16_pdu(0x0100, &[0x1234, 0x5678]).unwrap();
        assert_eq!(
            pdu.as_slice(),
            &[0x10, 0x01, 0x00, 0x00, 0x02, 0x04, 0x12, 0x34, 0x56, 0x78]
        );
    }
}
