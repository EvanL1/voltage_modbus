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
use crate::error::{ModbusError, ModbusResult};
use crate::value::ModbusValue;

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
    let dt = data_type;
    if dt.eq_ignore_ascii_case("bool")
        || dt.eq_ignore_ascii_case("boolean")
        || dt.eq_ignore_ascii_case("coil")
    {
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
        return Ok(ModbusValue::Bool(bit_value != 0));
    }

    if dt.eq_ignore_ascii_case("uint16")
        || dt.eq_ignore_ascii_case("u16")
        || dt.eq_ignore_ascii_case("word")
    {
        if registers.is_empty() {
            return Err(ModbusError::InvalidData {
                message: "No registers for uint16".to_string(),
            });
        }
        return Ok(ModbusValue::U16(registers[0]));
    }

    if dt.eq_ignore_ascii_case("int16")
        || dt.eq_ignore_ascii_case("i16")
        || dt.eq_ignore_ascii_case("short")
    {
        if registers.is_empty() {
            return Err(ModbusError::InvalidData {
                message: "No registers for int16".to_string(),
            });
        }
        return Ok(ModbusValue::I16(registers[0] as i16));
    }

    if dt.eq_ignore_ascii_case("uint32")
        || dt.eq_ignore_ascii_case("u32")
        || dt.eq_ignore_ascii_case("dword")
    {
        if registers.len() < 2 {
            return Err(ModbusError::InvalidData {
                message: "Not enough registers for uint32".to_string(),
            });
        }
        let regs: [u16; 2] = [registers[0], registers[1]];
        let bytes = regs_to_bytes_4(&regs, byte_order);
        return Ok(ModbusValue::U32(u32::from_be_bytes(bytes)));
    }

    if dt.eq_ignore_ascii_case("int32")
        || dt.eq_ignore_ascii_case("i32")
        || dt.eq_ignore_ascii_case("long")
    {
        if registers.len() < 2 {
            return Err(ModbusError::InvalidData {
                message: "Not enough registers for int32".to_string(),
            });
        }
        let regs: [u16; 2] = [registers[0], registers[1]];
        let bytes = regs_to_bytes_4(&regs, byte_order);
        return Ok(ModbusValue::I32(i32::from_be_bytes(bytes)));
    }

    if dt.eq_ignore_ascii_case("float32")
        || dt.eq_ignore_ascii_case("f32")
        || dt.eq_ignore_ascii_case("float")
        || dt.eq_ignore_ascii_case("real")
    {
        if registers.len() < 2 {
            return Err(ModbusError::InvalidData {
                message: "Not enough registers for float32".to_string(),
            });
        }
        let regs: [u16; 2] = [registers[0], registers[1]];
        let bytes = regs_to_bytes_4(&regs, byte_order);
        return Ok(ModbusValue::F32(f32::from_be_bytes(bytes)));
    }

    if dt.eq_ignore_ascii_case("uint64")
        || dt.eq_ignore_ascii_case("u64")
        || dt.eq_ignore_ascii_case("qword")
    {
        if registers.len() < 4 {
            return Err(ModbusError::InvalidData {
                message: "Not enough registers for uint64".to_string(),
            });
        }
        let regs: [u16; 4] = [registers[0], registers[1], registers[2], registers[3]];
        let bytes = regs_to_bytes_8(&regs, byte_order);
        return Ok(ModbusValue::U64(u64::from_be_bytes(bytes)));
    }

    if dt.eq_ignore_ascii_case("int64")
        || dt.eq_ignore_ascii_case("i64")
        || dt.eq_ignore_ascii_case("longlong")
    {
        if registers.len() < 4 {
            return Err(ModbusError::InvalidData {
                message: "Not enough registers for int64".to_string(),
            });
        }
        let regs: [u16; 4] = [registers[0], registers[1], registers[2], registers[3]];
        let bytes = regs_to_bytes_8(&regs, byte_order);
        return Ok(ModbusValue::I64(i64::from_be_bytes(bytes)));
    }

    if dt.eq_ignore_ascii_case("float64")
        || dt.eq_ignore_ascii_case("f64")
        || dt.eq_ignore_ascii_case("double")
        || dt.eq_ignore_ascii_case("lreal")
    {
        if registers.len() < 4 {
            return Err(ModbusError::InvalidData {
                message: "Not enough registers for float64".to_string(),
            });
        }
        let regs: [u16; 4] = [registers[0], registers[1], registers[2], registers[3]];
        let bytes = regs_to_bytes_8(&regs, byte_order);
        return Ok(ModbusValue::F64(f64::from_be_bytes(bytes)));
    }

    Err(ModbusError::InvalidData {
        message: format!("Unsupported data type: {}", data_type),
    })
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
    let dt = data_type;
    let (min, max): (f64, f64) =
        if dt.eq_ignore_ascii_case("uint16") || dt.eq_ignore_ascii_case("u16") {
            (0.0, 65535.0)
        } else if dt.eq_ignore_ascii_case("int16") || dt.eq_ignore_ascii_case("i16") {
            (-32768.0, 32767.0)
        } else if dt.eq_ignore_ascii_case("uint32") || dt.eq_ignore_ascii_case("u32") {
            (0.0, 4294967295.0)
        } else if dt.eq_ignore_ascii_case("int32") || dt.eq_ignore_ascii_case("i32") {
            (-2147483648.0, 2147483647.0)
        } else if dt.eq_ignore_ascii_case("uint64") || dt.eq_ignore_ascii_case("u64") {
            (0.0, u64::MAX as f64)
        } else if dt.eq_ignore_ascii_case("int64") || dt.eq_ignore_ascii_case("i64") {
            (i64::MIN as f64, i64::MAX as f64)
        } else if dt.eq_ignore_ascii_case("float32") || dt.eq_ignore_ascii_case("f32") {
            (f32::MIN as f64, f32::MAX as f64)
        } else if dt.eq_ignore_ascii_case("float64") || dt.eq_ignore_ascii_case("f64") {
            (f64::MIN, f64::MAX)
        } else {
            // Boolean types and unknown types — return as-is
            return value;
        };

    value.clamp(min, max)
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
    let dt = data_type;

    if dt.eq_ignore_ascii_case("bool")
        || dt.eq_ignore_ascii_case("boolean")
        || dt.eq_ignore_ascii_case("coil")
    {
        return Ok(vec![if clamped != 0.0 { 1 } else { 0 }]);
    }
    if dt.eq_ignore_ascii_case("uint16")
        || dt.eq_ignore_ascii_case("u16")
        || dt.eq_ignore_ascii_case("word")
    {
        return Ok(vec![clamped as u16]);
    }
    if dt.eq_ignore_ascii_case("int16")
        || dt.eq_ignore_ascii_case("i16")
        || dt.eq_ignore_ascii_case("short")
    {
        return Ok(vec![(clamped as i16) as u16]);
    }
    if dt.eq_ignore_ascii_case("uint32")
        || dt.eq_ignore_ascii_case("u32")
        || dt.eq_ignore_ascii_case("dword")
    {
        let bytes = (clamped as u32).to_be_bytes();
        return Ok(bytes_4_to_regs(&bytes, byte_order).to_vec());
    }
    if dt.eq_ignore_ascii_case("int32")
        || dt.eq_ignore_ascii_case("i32")
        || dt.eq_ignore_ascii_case("long")
    {
        let bytes = (clamped as i32).to_be_bytes();
        return Ok(bytes_4_to_regs(&bytes, byte_order).to_vec());
    }
    if dt.eq_ignore_ascii_case("float32")
        || dt.eq_ignore_ascii_case("f32")
        || dt.eq_ignore_ascii_case("float")
        || dt.eq_ignore_ascii_case("real")
    {
        let bytes = (clamped as f32).to_be_bytes();
        return Ok(bytes_4_to_regs(&bytes, byte_order).to_vec());
    }
    if dt.eq_ignore_ascii_case("uint64")
        || dt.eq_ignore_ascii_case("u64")
        || dt.eq_ignore_ascii_case("qword")
    {
        let bytes = (clamped as u64).to_be_bytes();
        return Ok(bytes_8_to_regs(&bytes, byte_order).to_vec());
    }
    if dt.eq_ignore_ascii_case("int64")
        || dt.eq_ignore_ascii_case("i64")
        || dt.eq_ignore_ascii_case("longlong")
    {
        let bytes = (clamped as i64).to_be_bytes();
        return Ok(bytes_8_to_regs(&bytes, byte_order).to_vec());
    }
    if dt.eq_ignore_ascii_case("float64")
        || dt.eq_ignore_ascii_case("f64")
        || dt.eq_ignore_ascii_case("double")
        || dt.eq_ignore_ascii_case("lreal")
    {
        let bytes = clamped.to_be_bytes();
        return Ok(bytes_8_to_regs(&bytes, byte_order).to_vec());
    }

    Err(ModbusError::InvalidData {
        message: format!("Unsupported data type: {}", data_type),
    })
}

/// Get the number of registers required for a data type.
pub fn registers_for_type(data_type: &str) -> usize {
    let dt = data_type;
    if dt.eq_ignore_ascii_case("bool")
        || dt.eq_ignore_ascii_case("boolean")
        || dt.eq_ignore_ascii_case("coil")
    {
        0 // Coils use separate addressing
    } else if dt.eq_ignore_ascii_case("uint16")
        || dt.eq_ignore_ascii_case("u16")
        || dt.eq_ignore_ascii_case("word")
        || dt.eq_ignore_ascii_case("int16")
        || dt.eq_ignore_ascii_case("i16")
        || dt.eq_ignore_ascii_case("short")
    {
        1
    } else if dt.eq_ignore_ascii_case("uint32")
        || dt.eq_ignore_ascii_case("u32")
        || dt.eq_ignore_ascii_case("dword")
        || dt.eq_ignore_ascii_case("int32")
        || dt.eq_ignore_ascii_case("i32")
        || dt.eq_ignore_ascii_case("long")
        || dt.eq_ignore_ascii_case("float32")
        || dt.eq_ignore_ascii_case("f32")
        || dt.eq_ignore_ascii_case("float")
        || dt.eq_ignore_ascii_case("real")
    {
        2
    } else if dt.eq_ignore_ascii_case("uint64")
        || dt.eq_ignore_ascii_case("u64")
        || dt.eq_ignore_ascii_case("qword")
        || dt.eq_ignore_ascii_case("int64")
        || dt.eq_ignore_ascii_case("i64")
        || dt.eq_ignore_ascii_case("longlong")
        || dt.eq_ignore_ascii_case("float64")
        || dt.eq_ignore_ascii_case("f64")
        || dt.eq_ignore_ascii_case("double")
        || dt.eq_ignore_ascii_case("lreal")
    {
        4
    } else {
        1 // Default to 1 register for unknown types
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
}
