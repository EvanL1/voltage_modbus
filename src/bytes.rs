//! # Byte Order Handling for Modbus
//!
//! Unified byte/word order representation for industrial protocols.
//! Supports ABCD, DCBA, CDAB, BADC patterns commonly used in PLCs and SCADA systems.
//!
//! ## Terminology
//!
//! - **Byte order**: Order of bytes within multi-byte values (endianness)
//! - **Word order**: Order of 16-bit words when combining to form 32/64-bit values
//!
//! ## Naming Convention
//!
//! Uses ABCD notation where:
//! - A = Most significant byte (MSB)
//! - B = Second byte
//! - C = Third byte
//! - D = Least significant byte (LSB)
//!
//! For 32-bit value `0x12345678`:
//! - `BigEndian (ABCD)`: \[0x12, 0x34, 0x56, 0x78\]
//! - `LittleEndian (DCBA)`: \[0x78, 0x56, 0x34, 0x12\]
//! - `BigEndianSwap (CDAB)`: \[0x56, 0x78, 0x12, 0x34\] (Modbus common)
//! - `LittleEndianSwap (BADC)`: \[0x34, 0x12, 0x78, 0x56\]

use std::fmt;

/// Unified byte/word order representation for 16/32/64-bit values.
///
/// # Example
///
/// ```rust
/// use voltage_modbus::ByteOrder;
///
/// let order = ByteOrder::from_str("CDAB").unwrap();
/// assert_eq!(order, ByteOrder::BigEndianSwap);
/// assert!(order.has_word_swap());
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ByteOrder {
    /// Big-endian: ABCD (most significant byte first)
    ///
    /// Network byte order, used in most protocols.
    /// Example: 0x12345678 → \[0x12, 0x34, 0x56, 0x78\]
    BigEndian,

    /// Little-endian: DCBA (least significant byte first)
    ///
    /// Intel x86 native byte order.
    /// Example: 0x12345678 → \[0x78, 0x56, 0x34, 0x12\]
    LittleEndian,

    /// Big-endian with swapped words: CDAB
    ///
    /// Common in Modbus and some PLCs. Words are big-endian but swapped.
    /// Example: 0x12345678 → \[0x56, 0x78, 0x12, 0x34\]
    BigEndianSwap,

    /// Little-endian with swapped words: BADC
    ///
    /// Rare, but exists in some devices.
    /// Example: 0x12345678 → \[0x34, 0x12, 0x78, 0x56\]
    LittleEndianSwap,

    /// 16-bit big-endian: AB
    ///
    /// For 16-bit values only.
    /// Example: 0x1234 → \[0x12, 0x34\]
    BigEndian16,

    /// 16-bit little-endian: BA
    ///
    /// For 16-bit values only.
    /// Example: 0x1234 → \[0x34, 0x12\]
    LittleEndian16,
}

impl ByteOrder {
    /// Convert from legacy string formats.
    ///
    /// Supports various common string representations:
    /// - "ABCD", "AB-CD" → BigEndian
    /// - "DCBA", "DC-BA" → LittleEndian
    /// - "CDAB", "CD-AB" → BigEndianSwap
    /// - "BADC", "BA-DC" → LittleEndianSwap
    /// - "BE", "BIG_ENDIAN" → BigEndian
    /// - "LE", "LITTLE_ENDIAN" → LittleEndian
    /// - "AB" → BigEndian16
    /// - "BA" → LittleEndian16
    #[allow(clippy::should_implement_trait)]
    pub fn from_str(s: &str) -> Option<Self> {
        // Normalize in single pass: uppercase + remove hyphens/underscores
        let normalized: String = s
            .chars()
            .filter(|c| *c != '-' && *c != '_')
            .map(|c| c.to_ascii_uppercase())
            .collect();
        match normalized.as_str() {
            // 32/64-bit patterns
            "ABCD" | "BE" | "BIG_ENDIAN" | "BIGENDIAN" | "ABCDEFGH" => Some(Self::BigEndian),
            "DCBA" | "LE" | "LITTLE_ENDIAN" | "LITTLEENDIAN" | "HGFEDCBA" => {
                Some(Self::LittleEndian)
            }
            "CDAB" | "BIG_ENDIAN_SWAP" | "BIGENDIANSWAP" => Some(Self::BigEndianSwap),
            "BADC" | "LITTLE_ENDIAN_SWAP" | "LITTLEENDIANSWAP" => Some(Self::LittleEndianSwap),

            // 16-bit patterns
            "AB" => Some(Self::BigEndian16),
            "BA" => Some(Self::LittleEndian16),

            _ => None,
        }
    }

    /// Get descriptive name.
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::BigEndian => "ABCD (Big-Endian)",
            Self::LittleEndian => "DCBA (Little-Endian)",
            Self::BigEndianSwap => "CDAB (Big-Endian Swap)",
            Self::LittleEndianSwap => "BADC (Little-Endian Swap)",
            Self::BigEndian16 => "AB (Big-Endian 16)",
            Self::LittleEndian16 => "BA (Little-Endian 16)",
        }
    }

    /// Check if this is a 16-bit only byte order.
    #[inline]
    pub fn is_16bit_only(&self) -> bool {
        matches!(self, Self::BigEndian16 | Self::LittleEndian16)
    }

    /// Check if this is a big-endian variant.
    #[inline]
    pub fn is_big_endian(&self) -> bool {
        matches!(
            self,
            Self::BigEndian | Self::BigEndianSwap | Self::BigEndian16
        )
    }

    /// Check if this is a little-endian variant.
    #[inline]
    pub fn is_little_endian(&self) -> bool {
        matches!(
            self,
            Self::LittleEndian | Self::LittleEndianSwap | Self::LittleEndian16
        )
    }

    /// Check if words are swapped (for 32/64-bit values).
    #[inline]
    pub fn has_word_swap(&self) -> bool {
        matches!(self, Self::BigEndianSwap | Self::LittleEndianSwap)
    }
}

impl fmt::Display for ByteOrder {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.as_str())
    }
}

impl Default for ByteOrder {
    /// Default to big-endian (network byte order).
    fn default() -> Self {
        Self::BigEndian
    }
}

// ============================================================================
// Register to Bytes Conversions
// ============================================================================

/// Convert 2 u16 registers to 4 bytes with specified byte order.
///
/// # Example
///
/// ```rust
/// use voltage_modbus::{ByteOrder, regs_to_bytes_4};
///
/// let regs = [0x1234, 0x5678];
/// assert_eq!(regs_to_bytes_4(&regs, ByteOrder::BigEndian), [0x12, 0x34, 0x56, 0x78]);
/// assert_eq!(regs_to_bytes_4(&regs, ByteOrder::BigEndianSwap), [0x56, 0x78, 0x12, 0x34]);
/// ```
#[inline]
pub fn regs_to_bytes_4(regs: &[u16; 2], order: ByteOrder) -> [u8; 4] {
    let [h0, h1] = [regs[0].to_be_bytes(), regs[1].to_be_bytes()];

    match order {
        ByteOrder::BigEndian | ByteOrder::BigEndian16 => [h0[0], h0[1], h1[0], h1[1]], // ABCD
        ByteOrder::LittleEndian | ByteOrder::LittleEndian16 => [h1[1], h1[0], h0[1], h0[0]], // DCBA
        ByteOrder::BigEndianSwap => [h1[0], h1[1], h0[0], h0[1]],                      // CDAB
        ByteOrder::LittleEndianSwap => [h0[1], h0[0], h1[1], h1[0]],                   // BADC
    }
}

/// Convert 4 u16 registers to 8 bytes with specified byte order.
///
/// # Example
///
/// ```rust
/// use voltage_modbus::{ByteOrder, regs_to_bytes_8};
///
/// let regs = [0x1234, 0x5678, 0x9ABC, 0xDEF0];
/// let bytes = regs_to_bytes_8(&regs, ByteOrder::BigEndian);
/// assert_eq!(bytes, [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0]);
/// ```
#[inline]
pub fn regs_to_bytes_8(regs: &[u16; 4], order: ByteOrder) -> [u8; 8] {
    let [h0, h1, h2, h3] = [
        regs[0].to_be_bytes(),
        regs[1].to_be_bytes(),
        regs[2].to_be_bytes(),
        regs[3].to_be_bytes(),
    ];

    match order {
        ByteOrder::BigEndian | ByteOrder::BigEndian16 => [
            h0[0], h0[1], h1[0], h1[1], h2[0], h2[1], h3[0], h3[1], // ABCDEFGH
        ],
        ByteOrder::LittleEndian | ByteOrder::LittleEndian16 => [
            h3[1], h3[0], h2[1], h2[0], h1[1], h1[0], h0[1], h0[0], // HGFEDCBA
        ],
        ByteOrder::BigEndianSwap => [
            h3[0], h3[1], h2[0], h2[1], h1[0], h1[1], h0[0], h0[1], // GHEFCDAB
        ],
        ByteOrder::LittleEndianSwap => [
            h0[1], h0[0], h1[1], h1[0], h2[1], h2[0], h3[1], h3[0], // BADCFEHG
        ],
    }
}

// ============================================================================
// Register to Numeric Type Conversions
// ============================================================================

/// Convert single u16 register to bytes.
#[inline]
pub fn reg_to_bytes_2(reg: u16, order: ByteOrder) -> [u8; 2] {
    match order {
        ByteOrder::BigEndian | ByteOrder::BigEndian16 => reg.to_be_bytes(),
        ByteOrder::LittleEndian | ByteOrder::LittleEndian16 => reg.to_le_bytes(),
        _ => reg.to_be_bytes(),
    }
}

/// Convert single u16 register to u16 (with byte swapping if needed).
#[inline]
pub fn reg_to_u16(reg: u16, order: ByteOrder) -> u16 {
    match order {
        ByteOrder::LittleEndian16 => reg.swap_bytes(),
        _ => reg,
    }
}

/// Convert single u16 register to i16.
#[inline]
pub fn reg_to_i16(reg: u16, order: ByteOrder) -> i16 {
    reg_to_u16(reg, order) as i16
}

/// Convert 2 u16 registers to f32.
#[inline]
pub fn regs_to_f32(regs: &[u16; 2], order: ByteOrder) -> f32 {
    let bytes = regs_to_bytes_4(regs, order);
    f32::from_be_bytes(bytes)
}

/// Convert 4 u16 registers to f64.
#[inline]
pub fn regs_to_f64(regs: &[u16; 4], order: ByteOrder) -> f64 {
    let bytes = regs_to_bytes_8(regs, order);
    f64::from_be_bytes(bytes)
}

/// Convert 2 u16 registers to u32.
#[inline]
pub fn regs_to_u32(regs: &[u16; 2], order: ByteOrder) -> u32 {
    let bytes = regs_to_bytes_4(regs, order);
    u32::from_be_bytes(bytes)
}

/// Convert 2 u16 registers to i32.
#[inline]
pub fn regs_to_i32(regs: &[u16; 2], order: ByteOrder) -> i32 {
    let bytes = regs_to_bytes_4(regs, order);
    i32::from_be_bytes(bytes)
}

/// Convert 4 u16 registers to u64.
#[inline]
pub fn regs_to_u64(regs: &[u16; 4], order: ByteOrder) -> u64 {
    let bytes = regs_to_bytes_8(regs, order);
    u64::from_be_bytes(bytes)
}

/// Convert 4 u16 registers to i64.
#[inline]
pub fn regs_to_i64(regs: &[u16; 4], order: ByteOrder) -> i64 {
    let bytes = regs_to_bytes_8(regs, order);
    i64::from_be_bytes(bytes)
}

// ============================================================================
// Numeric Type to Register Conversions (for encoding)
// ============================================================================

/// Convert u32 to 2 u16 registers with specified byte order.
#[inline]
pub fn u32_to_regs(value: u32, order: ByteOrder) -> [u16; 2] {
    let bytes = value.to_be_bytes();
    bytes_4_to_regs(&bytes, order)
}

/// Convert i32 to 2 u16 registers with specified byte order.
#[inline]
pub fn i32_to_regs(value: i32, order: ByteOrder) -> [u16; 2] {
    u32_to_regs(value as u32, order)
}

/// Convert f32 to 2 u16 registers with specified byte order.
#[inline]
pub fn f32_to_regs(value: f32, order: ByteOrder) -> [u16; 2] {
    let bytes = value.to_be_bytes();
    bytes_4_to_regs(&bytes, order)
}

/// Convert u64 to 4 u16 registers with specified byte order.
#[inline]
pub fn u64_to_regs(value: u64, order: ByteOrder) -> [u16; 4] {
    let bytes = value.to_be_bytes();
    bytes_8_to_regs(&bytes, order)
}

/// Convert i64 to 4 u16 registers with specified byte order.
#[inline]
pub fn i64_to_regs(value: i64, order: ByteOrder) -> [u16; 4] {
    u64_to_regs(value as u64, order)
}

/// Convert f64 to 4 u16 registers with specified byte order.
#[inline]
pub fn f64_to_regs(value: f64, order: ByteOrder) -> [u16; 4] {
    let bytes = value.to_be_bytes();
    bytes_8_to_regs(&bytes, order)
}

/// Convert 4 bytes (big-endian value) to 2 u16 registers with specified byte order.
#[inline]
pub fn bytes_4_to_regs(bytes: &[u8; 4], order: ByteOrder) -> [u16; 2] {
    match order {
        ByteOrder::BigEndian | ByteOrder::BigEndian16 => [
            u16::from_be_bytes([bytes[0], bytes[1]]),
            u16::from_be_bytes([bytes[2], bytes[3]]),
        ],
        ByteOrder::LittleEndian | ByteOrder::LittleEndian16 => [
            u16::from_be_bytes([bytes[3], bytes[2]]),
            u16::from_be_bytes([bytes[1], bytes[0]]),
        ],
        ByteOrder::BigEndianSwap => [
            u16::from_be_bytes([bytes[2], bytes[3]]),
            u16::from_be_bytes([bytes[0], bytes[1]]),
        ],
        ByteOrder::LittleEndianSwap => [
            u16::from_be_bytes([bytes[1], bytes[0]]),
            u16::from_be_bytes([bytes[3], bytes[2]]),
        ],
    }
}

/// Convert 8 bytes (big-endian value) to 4 u16 registers with specified byte order.
#[inline]
pub fn bytes_8_to_regs(bytes: &[u8; 8], order: ByteOrder) -> [u16; 4] {
    match order {
        ByteOrder::BigEndian | ByteOrder::BigEndian16 => [
            u16::from_be_bytes([bytes[0], bytes[1]]),
            u16::from_be_bytes([bytes[2], bytes[3]]),
            u16::from_be_bytes([bytes[4], bytes[5]]),
            u16::from_be_bytes([bytes[6], bytes[7]]),
        ],
        ByteOrder::LittleEndian | ByteOrder::LittleEndian16 => [
            u16::from_be_bytes([bytes[7], bytes[6]]),
            u16::from_be_bytes([bytes[5], bytes[4]]),
            u16::from_be_bytes([bytes[3], bytes[2]]),
            u16::from_be_bytes([bytes[1], bytes[0]]),
        ],
        ByteOrder::BigEndianSwap => [
            u16::from_be_bytes([bytes[6], bytes[7]]),
            u16::from_be_bytes([bytes[4], bytes[5]]),
            u16::from_be_bytes([bytes[2], bytes[3]]),
            u16::from_be_bytes([bytes[0], bytes[1]]),
        ],
        ByteOrder::LittleEndianSwap => [
            u16::from_be_bytes([bytes[1], bytes[0]]),
            u16::from_be_bytes([bytes[3], bytes[2]]),
            u16::from_be_bytes([bytes[5], bytes[4]]),
            u16::from_be_bytes([bytes[7], bytes[6]]),
        ],
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_from_str_valid() {
        assert_eq!(ByteOrder::from_str("ABCD"), Some(ByteOrder::BigEndian));
        assert_eq!(ByteOrder::from_str("AB-CD"), Some(ByteOrder::BigEndian));
        assert_eq!(ByteOrder::from_str("be"), Some(ByteOrder::BigEndian));

        assert_eq!(ByteOrder::from_str("DCBA"), Some(ByteOrder::LittleEndian));
        assert_eq!(ByteOrder::from_str("LE"), Some(ByteOrder::LittleEndian));

        assert_eq!(ByteOrder::from_str("CDAB"), Some(ByteOrder::BigEndianSwap));
        assert_eq!(
            ByteOrder::from_str("BADC"),
            Some(ByteOrder::LittleEndianSwap)
        );

        assert_eq!(ByteOrder::from_str("AB"), Some(ByteOrder::BigEndian16));
        assert_eq!(ByteOrder::from_str("BA"), Some(ByteOrder::LittleEndian16));
    }

    #[test]
    fn test_from_str_invalid() {
        assert_eq!(ByteOrder::from_str("invalid"), None);
        assert_eq!(ByteOrder::from_str(""), None);
    }

    #[test]
    fn test_properties() {
        assert!(ByteOrder::BigEndian16.is_16bit_only());
        assert!(!ByteOrder::BigEndian.is_16bit_only());

        assert!(ByteOrder::BigEndian.is_big_endian());
        assert!(!ByteOrder::LittleEndian.is_big_endian());

        assert!(ByteOrder::BigEndianSwap.has_word_swap());
        assert!(!ByteOrder::BigEndian.has_word_swap());
    }

    #[test]
    fn test_default() {
        assert_eq!(ByteOrder::default(), ByteOrder::BigEndian);
    }

    #[test]
    fn test_regs_to_bytes_4_all_orders() {
        let regs = [0x1234, 0x5678];

        assert_eq!(
            regs_to_bytes_4(&regs, ByteOrder::BigEndian),
            [0x12, 0x34, 0x56, 0x78]
        );
        assert_eq!(
            regs_to_bytes_4(&regs, ByteOrder::LittleEndian),
            [0x78, 0x56, 0x34, 0x12]
        );
        assert_eq!(
            regs_to_bytes_4(&regs, ByteOrder::BigEndianSwap),
            [0x56, 0x78, 0x12, 0x34]
        );
        assert_eq!(
            regs_to_bytes_4(&regs, ByteOrder::LittleEndianSwap),
            [0x34, 0x12, 0x78, 0x56]
        );
    }

    #[test]
    fn test_regs_to_bytes_8_all_orders() {
        let regs = [0x1234, 0x5678, 0x9ABC, 0xDEF0];

        assert_eq!(
            regs_to_bytes_8(&regs, ByteOrder::BigEndian),
            [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0]
        );
        assert_eq!(
            regs_to_bytes_8(&regs, ByteOrder::LittleEndian),
            [0xF0, 0xDE, 0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12]
        );
    }

    #[test]
    fn test_regs_to_f32() {
        // 25.0 in IEEE 754: 0x41C80000
        let regs = [0x41C8, 0x0000];
        let value = regs_to_f32(&regs, ByteOrder::BigEndian);
        assert!((value - 25.0).abs() < f32::EPSILON);
    }

    #[test]
    fn test_regs_to_u32() {
        let regs = [0x1234, 0x5678];
        assert_eq!(regs_to_u32(&regs, ByteOrder::BigEndian), 0x12345678);
        assert_eq!(regs_to_u32(&regs, ByteOrder::LittleEndian), 0x78563412);
    }

    #[test]
    fn test_u32_to_regs_roundtrip() {
        let value = 0x12345678u32;
        for order in [
            ByteOrder::BigEndian,
            ByteOrder::LittleEndian,
            ByteOrder::BigEndianSwap,
            ByteOrder::LittleEndianSwap,
        ] {
            let regs = u32_to_regs(value, order);
            let decoded = regs_to_u32(&regs, order);
            assert_eq!(decoded, value, "Roundtrip failed for {:?}", order);
        }
    }

    #[test]
    fn test_f32_to_regs_roundtrip() {
        let value = 123.456f32;
        for order in [
            ByteOrder::BigEndian,
            ByteOrder::LittleEndian,
            ByteOrder::BigEndianSwap,
            ByteOrder::LittleEndianSwap,
        ] {
            let regs = f32_to_regs(value, order);
            let decoded = regs_to_f32(&regs, order);
            assert!(
                (decoded - value).abs() < 0.001,
                "Roundtrip failed for {:?}",
                order
            );
        }
    }

    #[test]
    fn test_f64_to_regs_roundtrip() {
        let value = 123456.789012345f64;
        for order in [
            ByteOrder::BigEndian,
            ByteOrder::LittleEndian,
            ByteOrder::BigEndianSwap,
            ByteOrder::LittleEndianSwap,
        ] {
            let regs = f64_to_regs(value, order);
            let decoded = regs_to_f64(&regs, order);
            assert!(
                (decoded - value).abs() < 1e-9,
                "Roundtrip failed for {:?}",
                order
            );
        }
    }
}
