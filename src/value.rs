//! # Modbus Value Types
//!
//! Self-contained data types for industrial Modbus applications.
//! Designed for register encoding/decoding with minimal allocations.

use std::fmt;

/// Industrial data type enumeration for Modbus register values.
///
/// This enum represents all numeric types commonly used in industrial
/// automation and SCADA systems. It is designed to be self-contained
/// (no external dependencies) for use with `ModbusCodec`.
///
/// # Register Mapping
///
/// | Type | Registers | Description |
/// |------|-----------|-------------|
/// | Bool | 1 (coil) | Single bit value |
/// | U16/I16 | 1 | Single 16-bit register |
/// | U32/I32/F32 | 2 | Two consecutive registers |
/// | U64/I64/F64 | 4 | Four consecutive registers |
///
/// # Example
///
/// ```rust
/// use voltage_modbus::ModbusValue;
///
/// let temp = ModbusValue::F32(25.5);
/// assert_eq!(temp.register_count(), 2);
/// assert!((temp.as_f64() - 25.5).abs() < 0.001);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub enum ModbusValue {
    /// Boolean value (typically from coils)
    Bool(bool),
    /// Unsigned 16-bit integer (1 register)
    U16(u16),
    /// Signed 16-bit integer (1 register)
    I16(i16),
    /// Unsigned 32-bit integer (2 registers)
    U32(u32),
    /// Signed 32-bit integer (2 registers)
    I32(i32),
    /// 32-bit floating point (2 registers)
    F32(f32),
    /// Unsigned 64-bit integer (4 registers)
    U64(u64),
    /// Signed 64-bit integer (4 registers)
    I64(i64),
    /// 64-bit floating point (4 registers)
    F64(f64),
}

impl ModbusValue {
    /// Convert the value to f64 for uniform numeric handling.
    ///
    /// This is useful for calculations, comparisons, and storing
    /// values in a normalized format.
    #[inline]
    pub fn as_f64(&self) -> f64 {
        match self {
            ModbusValue::Bool(b) => {
                if *b {
                    1.0
                } else {
                    0.0
                }
            }
            ModbusValue::U16(v) => f64::from(*v),
            ModbusValue::I16(v) => f64::from(*v),
            ModbusValue::U32(v) => f64::from(*v),
            ModbusValue::I32(v) => f64::from(*v),
            ModbusValue::F32(v) => f64::from(*v),
            ModbusValue::U64(v) => *v as f64,
            ModbusValue::I64(v) => *v as f64,
            ModbusValue::F64(v) => *v,
        }
    }

    /// Convert the value to i64 for integer operations.
    ///
    /// Float values are rounded to the nearest integer.
    #[inline]
    pub fn as_i64(&self) -> i64 {
        match self {
            ModbusValue::Bool(b) => i64::from(*b),
            ModbusValue::U16(v) => i64::from(*v),
            ModbusValue::I16(v) => i64::from(*v),
            ModbusValue::U32(v) => i64::from(*v),
            ModbusValue::I32(v) => i64::from(*v),
            ModbusValue::F32(v) => v.round() as i64,
            ModbusValue::U64(v) => *v as i64,
            ModbusValue::I64(v) => *v,
            ModbusValue::F64(v) => v.round() as i64,
        }
    }

    /// Returns the number of 16-bit Modbus registers required for this value.
    ///
    /// # Returns
    ///
    /// - `0` for Bool (coils use separate addressing)
    /// - `1` for U16/I16
    /// - `2` for U32/I32/F32
    /// - `4` for U64/I64/F64
    #[inline]
    pub fn register_count(&self) -> usize {
        match self {
            ModbusValue::Bool(_) => 0, // Coils don't use registers
            ModbusValue::U16(_) | ModbusValue::I16(_) => 1,
            ModbusValue::U32(_) | ModbusValue::I32(_) | ModbusValue::F32(_) => 2,
            ModbusValue::U64(_) | ModbusValue::I64(_) | ModbusValue::F64(_) => 4,
        }
    }

    /// Check if the value is zero or false.
    #[inline]
    pub fn is_zero(&self) -> bool {
        match self {
            ModbusValue::Bool(b) => !*b,
            ModbusValue::U16(v) => *v == 0,
            ModbusValue::I16(v) => *v == 0,
            ModbusValue::U32(v) => *v == 0,
            ModbusValue::I32(v) => *v == 0,
            ModbusValue::F32(v) => *v == 0.0,
            ModbusValue::U64(v) => *v == 0,
            ModbusValue::I64(v) => *v == 0,
            ModbusValue::F64(v) => *v == 0.0,
        }
    }

    /// Returns the type name as a string for logging/debugging.
    pub fn type_name(&self) -> &'static str {
        match self {
            ModbusValue::Bool(_) => "bool",
            ModbusValue::U16(_) => "u16",
            ModbusValue::I16(_) => "i16",
            ModbusValue::U32(_) => "u32",
            ModbusValue::I32(_) => "i32",
            ModbusValue::F32(_) => "f32",
            ModbusValue::U64(_) => "u64",
            ModbusValue::I64(_) => "i64",
            ModbusValue::F64(_) => "f64",
        }
    }
}

impl fmt::Display for ModbusValue {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ModbusValue::Bool(v) => write!(f, "{}", v),
            ModbusValue::U16(v) => write!(f, "{}", v),
            ModbusValue::I16(v) => write!(f, "{}", v),
            ModbusValue::U32(v) => write!(f, "{}", v),
            ModbusValue::I32(v) => write!(f, "{}", v),
            ModbusValue::F32(v) => write!(f, "{}", v),
            ModbusValue::U64(v) => write!(f, "{}", v),
            ModbusValue::I64(v) => write!(f, "{}", v),
            ModbusValue::F64(v) => write!(f, "{}", v),
        }
    }
}

impl Default for ModbusValue {
    fn default() -> Self {
        ModbusValue::U16(0)
    }
}

// ============================================================================
// From implementations for ergonomic construction
// ============================================================================

impl From<bool> for ModbusValue {
    fn from(v: bool) -> Self {
        ModbusValue::Bool(v)
    }
}

impl From<u16> for ModbusValue {
    fn from(v: u16) -> Self {
        ModbusValue::U16(v)
    }
}

impl From<i16> for ModbusValue {
    fn from(v: i16) -> Self {
        ModbusValue::I16(v)
    }
}

impl From<u32> for ModbusValue {
    fn from(v: u32) -> Self {
        ModbusValue::U32(v)
    }
}

impl From<i32> for ModbusValue {
    fn from(v: i32) -> Self {
        ModbusValue::I32(v)
    }
}

impl From<f32> for ModbusValue {
    fn from(v: f32) -> Self {
        ModbusValue::F32(v)
    }
}

impl From<u64> for ModbusValue {
    fn from(v: u64) -> Self {
        ModbusValue::U64(v)
    }
}

impl From<i64> for ModbusValue {
    fn from(v: i64) -> Self {
        ModbusValue::I64(v)
    }
}

impl From<f64> for ModbusValue {
    fn from(v: f64) -> Self {
        ModbusValue::F64(v)
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_as_f64() {
        assert_eq!(ModbusValue::Bool(true).as_f64(), 1.0);
        assert_eq!(ModbusValue::Bool(false).as_f64(), 0.0);
        assert_eq!(ModbusValue::U16(100).as_f64(), 100.0);
        assert_eq!(ModbusValue::I16(-50).as_f64(), -50.0);
        assert_eq!(ModbusValue::U32(100000).as_f64(), 100000.0);
        assert_eq!(ModbusValue::I32(-100000).as_f64(), -100000.0);
        assert!((ModbusValue::F32(3.14).as_f64() - 3.14).abs() < 0.001);
        assert_eq!(ModbusValue::F64(3.14159265359).as_f64(), 3.14159265359);
    }

    #[test]
    fn test_as_i64() {
        assert_eq!(ModbusValue::Bool(true).as_i64(), 1);
        assert_eq!(ModbusValue::F32(3.7).as_i64(), 4); // rounded
        assert_eq!(ModbusValue::F64(-2.3).as_i64(), -2); // rounded
        assert_eq!(ModbusValue::U64(u64::MAX).as_i64(), -1); // overflow wraps
    }

    #[test]
    fn test_register_count() {
        assert_eq!(ModbusValue::Bool(true).register_count(), 0);
        assert_eq!(ModbusValue::U16(0).register_count(), 1);
        assert_eq!(ModbusValue::I16(0).register_count(), 1);
        assert_eq!(ModbusValue::U32(0).register_count(), 2);
        assert_eq!(ModbusValue::I32(0).register_count(), 2);
        assert_eq!(ModbusValue::F32(0.0).register_count(), 2);
        assert_eq!(ModbusValue::U64(0).register_count(), 4);
        assert_eq!(ModbusValue::I64(0).register_count(), 4);
        assert_eq!(ModbusValue::F64(0.0).register_count(), 4);
    }

    #[test]
    fn test_is_zero() {
        assert!(ModbusValue::Bool(false).is_zero());
        assert!(!ModbusValue::Bool(true).is_zero());
        assert!(ModbusValue::U16(0).is_zero());
        assert!(!ModbusValue::U16(1).is_zero());
        assert!(ModbusValue::F32(0.0).is_zero());
        assert!(!ModbusValue::F32(0.001).is_zero());
    }

    #[test]
    fn test_from_primitives() {
        let _: ModbusValue = true.into();
        let _: ModbusValue = 100u16.into();
        let _: ModbusValue = (-50i16).into();
        let _: ModbusValue = 100000u32.into();
        let _: ModbusValue = (-100000i32).into();
        let _: ModbusValue = 3.14f32.into();
        let _: ModbusValue = 100000000u64.into();
        let _: ModbusValue = (-100000000i64).into();
        let _: ModbusValue = 3.14159265359f64.into();
    }

    #[test]
    fn test_display() {
        assert_eq!(format!("{}", ModbusValue::Bool(true)), "true");
        assert_eq!(format!("{}", ModbusValue::U16(1234)), "1234");
        assert_eq!(format!("{}", ModbusValue::I16(-1234)), "-1234");
    }

    #[test]
    fn test_type_name() {
        assert_eq!(ModbusValue::Bool(true).type_name(), "bool");
        assert_eq!(ModbusValue::U16(0).type_name(), "u16");
        assert_eq!(ModbusValue::F32(0.0).type_name(), "f32");
    }
}
