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

use crate::bytes::{
    bytes_4_to_regs, bytes_8_to_regs, reg_to_u16, regs_to_bytes_4, regs_to_bytes_8, ByteOrder,
};
use crate::error::{ModbusError, ModbusResult};
use crate::value::ModbusValue;

// ============================================================================
// Data Type Parsing
// ============================================================================

/// Canonical data type, parsed once from the user-facing type string.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DataType {
    Bool,
    U16,
    I16,
    U32,
    I32,
    F32,
    U64,
    I64,
    F64,
}

/// Single source of truth for type-name aliases (matched case-insensitively).
/// The first name of each entry is the canonical name used in error messages.
const DATA_TYPE_ALIASES: &[(DataType, &[&str])] = &[
    (DataType::Bool, &["bool", "boolean", "coil"]),
    (DataType::U16, &["uint16", "u16", "word"]),
    (DataType::I16, &["int16", "i16", "short"]),
    (DataType::U32, &["uint32", "u32", "dword"]),
    (DataType::I32, &["int32", "i32", "long"]),
    (DataType::F32, &["float32", "f32", "float", "real"]),
    (DataType::U64, &["uint64", "u64", "qword"]),
    (DataType::I64, &["int64", "i64", "longlong"]),
    (DataType::F64, &["float64", "f64", "double", "lreal"]),
];

impl DataType {
    fn parse(s: &str) -> Option<Self> {
        DATA_TYPE_ALIASES
            .iter()
            .find(|(_, names)| names.iter().any(|n| n.eq_ignore_ascii_case(s)))
            .map(|(dt, _)| *dt)
    }

    fn name(self) -> &'static str {
        DATA_TYPE_ALIASES
            .iter()
            .find(|(dt, _)| *dt == self)
            .map_or("unknown", |(_, names)| names[0])
    }

    /// Registers occupied by this type (bool is 0: coils use separate addressing).
    fn register_count(self) -> usize {
        match self {
            Self::Bool => 0,
            Self::U16 | Self::I16 => 1,
            Self::U32 | Self::I32 | Self::F32 => 2,
            Self::U64 | Self::I64 | Self::F64 => 4,
        }
    }

    /// Representable range, or `None` for bool (no clamping).
    fn range(self) -> Option<(f64, f64)> {
        match self {
            Self::Bool => None,
            Self::U16 => Some((0.0, u16::MAX as f64)),
            Self::I16 => Some((i16::MIN as f64, i16::MAX as f64)),
            Self::U32 => Some((0.0, u32::MAX as f64)),
            Self::I32 => Some((i32::MIN as f64, i32::MAX as f64)),
            Self::U64 => Some((0.0, u64::MAX as f64)),
            Self::I64 => Some((i64::MIN as f64, i64::MAX as f64)),
            Self::F32 => Some((f32::MIN as f64, f32::MAX as f64)),
            Self::F64 => Some((f64::MIN, f64::MAX)),
        }
    }

    /// Convert an (already clamped) f64 into a typed value.
    fn value_from_f64(self, v: f64) -> ModbusValue {
        match self {
            Self::Bool => ModbusValue::Bool(v != 0.0),
            Self::U16 => ModbusValue::U16(v as u16),
            Self::I16 => ModbusValue::I16(v as i16),
            Self::U32 => ModbusValue::U32(v as u32),
            Self::I32 => ModbusValue::I32(v as i32),
            Self::F32 => ModbusValue::F32(v as f32),
            Self::U64 => ModbusValue::U64(v as u64),
            Self::I64 => ModbusValue::I64(v as i64),
            Self::F64 => ModbusValue::F64(v),
        }
    }
}

fn unsupported_type(data_type: &str) -> ModbusError {
    ModbusError::InvalidData {
        message: format!("Unsupported data type: {}", data_type),
    }
}

/// Inverse of [`reg_to_u16`]: the only single-register transform is a byte
/// swap (for `LittleEndian16`), which is its own inverse.
#[inline]
fn u16_to_reg(value: u16, order: ByteOrder) -> u16 {
    reg_to_u16(value, order)
}

#[inline]
fn regs2(r: &[u16], order: ByteOrder) -> [u8; 4] {
    regs_to_bytes_4(&[r[0], r[1]], order)
}

#[inline]
fn regs4(r: &[u16], order: ByteOrder) -> [u8; 8] {
    regs_to_bytes_8(&[r[0], r[1], r[2], r[3]], order)
}

// ============================================================================
// Decoding Functions
// ============================================================================

/// Decode Modbus register values to ModbusValue based on data format.
///
/// Supports multiple data types with configurable byte ordering:
/// - `bool`: Single bit extraction from register (0-15 bit position)
/// - `uint16`, `int16`: Single 16-bit register (bytes swapped for
///   `ByteOrder::LittleEndian16`, unchanged for every other order)
/// - `uint32`, `int32`, `float32`: Two 16-bit registers
/// - `uint64`, `int64`, `float64`: Four 16-bit registers
///
/// Type names are case-insensitive and accept the aliases listed in the
/// module documentation.
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
    let dt = DataType::parse(data_type).ok_or_else(|| unsupported_type(data_type))?;

    let needed = dt.register_count().max(1);
    if registers.len() < needed {
        let prefix = if needed == 1 {
            "No registers"
        } else {
            "Not enough registers"
        };
        return Err(ModbusError::InvalidData {
            message: format!("{} for {}", prefix, dt.name()),
        });
    }

    let (r, o) = (registers, byte_order);
    Ok(match dt {
        DataType::Bool => {
            if bit_position > 15 {
                return Err(ModbusError::InvalidData {
                    message: format!("Invalid bit position: {} (must be 0-15)", bit_position),
                });
            }
            ModbusValue::Bool((r[0] >> bit_position) & 0x01 != 0)
        }
        DataType::U16 => ModbusValue::U16(reg_to_u16(r[0], o)),
        DataType::I16 => ModbusValue::I16(reg_to_u16(r[0], o) as i16),
        DataType::U32 => ModbusValue::U32(u32::from_be_bytes(regs2(r, o))),
        DataType::I32 => ModbusValue::I32(i32::from_be_bytes(regs2(r, o))),
        DataType::F32 => ModbusValue::F32(f32::from_be_bytes(regs2(r, o))),
        DataType::U64 => ModbusValue::U64(u64::from_be_bytes(regs4(r, o))),
        DataType::I64 => ModbusValue::I64(i64::from_be_bytes(regs4(r, o))),
        DataType::F64 => ModbusValue::F64(f64::from_be_bytes(regs4(r, o))),
    })
}

/// Clamp a value to the valid range for a given Modbus data type.
///
/// Prevents overflow when writing values that exceed the target register's
/// capacity (e.g., writing 70000 to a uint16 register). All type aliases
/// (e.g. `word`, `real`, `lreal`) clamp identically to their canonical type.
///
/// # Arguments
/// * `value` - The value to clamp
/// * `data_type` - Target data type (e.g., "uint16", "int32", "float32")
///
/// # Returns
/// The clamped value, or the original value if the type is unknown/boolean
pub fn clamp_to_data_type(value: f64, data_type: &str) -> f64 {
    match DataType::parse(data_type).and_then(DataType::range) {
        Some((min, max)) => value.clamp(min, max),
        None => value,
    }
}

// ============================================================================
// Encoding Functions
// ============================================================================

/// Encode a ModbusValue for Modbus transmission.
///
/// Converts typed values to register arrays with proper byte ordering.
/// 16-bit values are byte-swapped for `ByteOrder::LittleEndian16` (the
/// inverse of decoding) and written unchanged for every other order.
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
    let o = byte_order;
    Ok(match value {
        ModbusValue::Bool(b) => vec![u16::from(*b)],
        ModbusValue::U16(v) => vec![u16_to_reg(*v, o)],
        ModbusValue::I16(v) => vec![u16_to_reg(*v as u16, o)],
        ModbusValue::U32(v) => bytes_4_to_regs(&v.to_be_bytes(), o).to_vec(),
        ModbusValue::I32(v) => bytes_4_to_regs(&v.to_be_bytes(), o).to_vec(),
        ModbusValue::F32(v) => bytes_4_to_regs(&v.to_be_bytes(), o).to_vec(),
        ModbusValue::U64(v) => bytes_8_to_regs(&v.to_be_bytes(), o).to_vec(),
        ModbusValue::I64(v) => bytes_8_to_regs(&v.to_be_bytes(), o).to_vec(),
        ModbusValue::F64(v) => bytes_8_to_regs(&v.to_be_bytes(), o).to_vec(),
    })
}

/// Encode a value from f64 with specified data type for Modbus transmission.
///
/// This is useful when you have a generic numeric value and need to encode
/// it as a specific Modbus data type. The value is first clamped with
/// [`clamp_to_data_type`], then encoded with [`encode_value`].
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
    let dt = DataType::parse(data_type).ok_or_else(|| unsupported_type(data_type))?;
    let clamped = clamp_to_data_type(value, data_type);
    encode_value(&dt.value_from_f64(clamped), byte_order)
}

/// Get the number of registers required for a data type.
///
/// Returns 0 for bool (coils use separate addressing) and 1 for unknown types.
pub fn registers_for_type(data_type: &str) -> usize {
    DataType::parse(data_type).map_or(1, DataType::register_count)
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

    const ALL_ORDERS: [ByteOrder; 6] = [
        ByteOrder::BigEndian,
        ByteOrder::LittleEndian,
        ByteOrder::BigEndianSwap,
        ByteOrder::LittleEndianSwap,
        ByteOrder::BigEndian16,
        ByteOrder::LittleEndian16,
    ];

    /// (canonical name, aliases) — mirrors the alias table in the module doc.
    const ALIAS_TABLE: &[(&str, &[&str])] = &[
        ("bool", &["boolean", "coil", "BOOL"]),
        ("uint16", &["u16", "word", "WORD", "Uint16"]),
        ("int16", &["i16", "short", "SHORT"]),
        ("uint32", &["u32", "dword", "DWORD"]),
        ("int32", &["i32", "long", "LONG"]),
        ("float32", &["f32", "float", "real", "REAL"]),
        ("uint64", &["u64", "qword", "QWORD"]),
        ("int64", &["i64", "longlong", "LongLong"]),
        ("float64", &["f64", "double", "lreal", "LREAL"]),
    ];

    #[test]
    fn test_aliases_behave_like_canonical() {
        let regs = [0x1234, 0x5678, 0x9ABC, 0xDEF0];
        let samples = [1e40, -1e40, 70000.0, -40000.0, 12.5, -5.0, 0.0];
        for (canonical, aliases) in ALIAS_TABLE {
            for alias in *aliases {
                assert_eq!(
                    registers_for_type(alias),
                    registers_for_type(canonical),
                    "registers_for_type({alias}) != {canonical}"
                );
                for v in samples {
                    assert_eq!(
                        clamp_to_data_type(v, alias).to_bits(),
                        clamp_to_data_type(v, canonical).to_bits(),
                        "clamp({v}, {alias}) != {canonical}"
                    );
                }
                for order in ALL_ORDERS {
                    assert_eq!(
                        decode_register_value(&regs, alias, 3, order).unwrap(),
                        decode_register_value(&regs, canonical, 3, order).unwrap(),
                        "decode({alias}, {order:?}) != {canonical}"
                    );
                    for v in samples {
                        assert_eq!(
                            encode_f64_as_type(v, alias, order).unwrap(),
                            encode_f64_as_type(v, canonical, order).unwrap(),
                            "encode({v}, {alias}, {order:?}) != {canonical}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn test_clamp_float_aliases_saturate_to_f32_max() {
        let expected = encode_value(&ModbusValue::F32(f32::MAX), ByteOrder::BigEndian).unwrap();
        for dt in ["float32", "f32", "float", "real"] {
            assert_eq!(clamp_to_data_type(1e40, dt), f32::MAX as f64, "{dt}");
            assert_eq!(
                encode_f64_as_type(1e40, dt, ByteOrder::BigEndian).unwrap(),
                expected,
                "{dt}"
            );
        }
        assert_eq!(clamp_to_data_type(70000.0, "word"), 65535.0);
        assert_eq!(clamp_to_data_type(-40000.0, "short"), -32768.0);
    }

    #[test]
    fn test_unknown_type_behavior_preserved() {
        assert!(decode_register_value(&[1], "nope", 0, ByteOrder::BigEndian).is_err());
        assert!(encode_f64_as_type(1.0, "nope", ByteOrder::BigEndian).is_err());
        assert_eq!(clamp_to_data_type(1e40, "nope"), 1e40);
        assert_eq!(clamp_to_data_type(1e40, "bool"), 1e40);
        assert_eq!(registers_for_type("nope"), 1);
    }

    #[test]
    fn test_decode_16bit_little_endian16() {
        assert_eq!(
            decode_register_value(&[0x3412], "uint16", 0, ByteOrder::LittleEndian16).unwrap(),
            ModbusValue::U16(0x1234)
        );
        assert_eq!(
            decode_register_value(&[0xFFFE], "int16", 0, ByteOrder::LittleEndian16).unwrap(),
            ModbusValue::I16(0xFEFFu16 as i16)
        );
        assert_eq!(
            encode_value(&ModbusValue::U16(0x1234), ByteOrder::LittleEndian16).unwrap(),
            vec![0x3412]
        );
        assert_eq!(
            encode_f64_as_type(4660.0, "uint16", ByteOrder::LittleEndian16).unwrap(),
            vec![0x3412]
        );
    }

    #[test]
    fn test_16bit_matches_bytes_helpers_and_roundtrips_all_orders() {
        use crate::bytes::{reg_to_i16, reg_to_u16};
        for order in ALL_ORDERS {
            for raw in [0x0000u16, 0x1234, 0x00FF, 0x8001, 0xFFFF] {
                let u = decode_register_value(&[raw], "uint16", 0, order).unwrap();
                assert_eq!(u, ModbusValue::U16(reg_to_u16(raw, order)), "{order:?}");
                let i = decode_register_value(&[raw], "int16", 0, order).unwrap();
                assert_eq!(i, ModbusValue::I16(reg_to_i16(raw, order)), "{order:?}");
                assert_eq!(encode_value(&u, order).unwrap(), vec![raw], "{order:?}");
                assert_eq!(encode_value(&i, order).unwrap(), vec![raw], "{order:?}");
            }
            for (v, dt) in [
                (ModbusValue::U16(0x1234), "uint16"),
                (ModbusValue::I16(-12345), "int16"),
            ] {
                let regs = encode_value(&v, order).unwrap();
                assert_eq!(decode_register_value(&regs, dt, 0, order).unwrap(), v);
                let via_f64 = encode_f64_as_type(v.as_f64(), dt, order).unwrap();
                assert_eq!(via_f64, regs, "{order:?}");
            }
        }
    }
}
