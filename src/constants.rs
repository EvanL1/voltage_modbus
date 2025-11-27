//! Modbus protocol constants based on official specification
//!
//! These constants are derived from the official Modbus specification:
//! - Maximum PDU size: 253 bytes (inherited from RS485 ADU limit of 256 bytes)
//! - Register/coil limits are calculated to fit within the PDU size constraint

// ============================================================================
// Frame Size Constants
// ============================================================================

/// Modbus MBAP header length for TCP
/// Format: Transaction ID(2) + Protocol ID(2) + Length(2) + Unit ID(1) = 7 bytes
/// Note: Length field itself is not counted in MBAP_HEADER_LEN for frame parsing
pub const MBAP_HEADER_LEN: usize = 6;

/// Maximum PDU (Protocol Data Unit) size per Modbus specification
/// This is the fundamental limit inherited from RS485 implementation:
/// RS485 ADU (256 bytes) - Slave Address (1 byte) - CRC (2 bytes) = 253 bytes
pub const MAX_PDU_SIZE: usize = 253;

/// Maximum MBAP length field value (Unit ID + PDU)
/// Used for validating the Length field in MBAP header
/// = 1 (Unit ID) + 253 (Max PDU) = 254 bytes
pub const MAX_MBAP_LENGTH: usize = 1 + MAX_PDU_SIZE;

/// Response buffer size for receiving Modbus frames
///
/// Calculation:
/// - MBAP Header: 6 bytes (MBAP_HEADER_LEN)
/// - Max MBAP Length (Unit ID + PDU): 254 bytes (MAX_MBAP_LENGTH)
/// - Theoretical max frame: 6 + 254 = 260 bytes
/// - Buffer size: 512 bytes (provides safety margin)
pub const MODBUS_RESPONSE_BUFFER_SIZE: usize = 512;

// ============================================================================
// Register Operation Limits
// ============================================================================

/// Maximum number of registers for FC03/FC04 (Read Holding/Input Registers)
///
/// Calculation for response PDU:
/// - Function Code: 1 byte
/// - Byte Count: 1 byte
/// - Register Data: N × 2 bytes
/// - Total: 1 + 1 + (N × 2) ≤ 253
/// - Therefore: N ≤ (253 - 2) / 2 = 125.5 → 125 registers
pub const MAX_READ_REGISTERS: usize = 125;

/// Maximum number of registers for FC16 (Write Multiple Registers)
///
/// Calculation for request PDU:
/// - Function Code: 1 byte
/// - Starting Address: 2 bytes
/// - Quantity of Registers: 2 bytes
/// - Byte Count: 1 byte
/// - Register Values: N × 2 bytes
/// - Total: 1 + 2 + 2 + 1 + (N × 2) ≤ 253
/// - Therefore: N ≤ (253 - 6) / 2 = 123.5 → 123 registers
pub const MAX_WRITE_REGISTERS: usize = 123;

// ============================================================================
// Coil Operation Limits
// ============================================================================

/// Maximum number of coils for FC01/FC02 (Read Coils/Discrete Inputs)
///
/// Calculation for response PDU:
/// - Function Code: 1 byte
/// - Byte Count: 1 byte
/// - Coil Data: ceil(N / 8) bytes
/// - Total: 1 + 1 + ceil(N / 8) ≤ 253
/// - Therefore: ceil(N / 8) ≤ 251, N ≤ 251 × 8 = 2008
/// - Spec defines: N ≤ 2000 (rounded for practical use)
pub const MAX_READ_COILS: usize = 2000;

/// Maximum number of coils for FC15 (Write Multiple Coils)
///
/// Calculation for request PDU:
/// - Function Code: 1 byte
/// - Starting Address: 2 bytes
/// - Quantity of Outputs: 2 bytes
/// - Byte Count: 1 byte
/// - Coil Values: ceil(N / 8) bytes
/// - Total: 1 + 2 + 2 + 1 + ceil(N / 8) ≤ 253
/// - Therefore: ceil(N / 8) ≤ 247, N ≤ 247 × 8 = 1976
/// - Spec defines: N ≤ 1968 (0x7B0, conservative practical limit)
pub const MAX_WRITE_COILS: usize = 1968;

// ============================================================================
// Modbus Function Codes
// ============================================================================

/// Read Coils (FC01)
pub const FC_READ_COILS: u8 = 0x01;

/// Read Discrete Inputs (FC02)
pub const FC_READ_DISCRETE_INPUTS: u8 = 0x02;

/// Read Holding Registers (FC03)
pub const FC_READ_HOLDING_REGISTERS: u8 = 0x03;

/// Read Input Registers (FC04)
pub const FC_READ_INPUT_REGISTERS: u8 = 0x04;

/// Write Single Coil (FC05)
pub const FC_WRITE_SINGLE_COIL: u8 = 0x05;

/// Write Single Register (FC06)
pub const FC_WRITE_SINGLE_REGISTER: u8 = 0x06;

/// Write Multiple Coils (FC15)
pub const FC_WRITE_MULTIPLE_COILS: u8 = 0x0F;

/// Write Multiple Registers (FC16)
pub const FC_WRITE_MULTIPLE_REGISTERS: u8 = 0x10;

// ============================================================================
// Modbus Exception Codes
// ============================================================================

/// Illegal Function
pub const EXCEPTION_ILLEGAL_FUNCTION: u8 = 0x01;

/// Illegal Data Address
pub const EXCEPTION_ILLEGAL_DATA_ADDRESS: u8 = 0x02;

/// Illegal Data Value
pub const EXCEPTION_ILLEGAL_DATA_VALUE: u8 = 0x03;

/// Server Device Failure
pub const EXCEPTION_SERVER_DEVICE_FAILURE: u8 = 0x04;

/// Acknowledge
pub const EXCEPTION_ACKNOWLEDGE: u8 = 0x05;

/// Server Device Busy
pub const EXCEPTION_SERVER_DEVICE_BUSY: u8 = 0x06;

/// Memory Parity Error
pub const EXCEPTION_MEMORY_PARITY_ERROR: u8 = 0x08;

/// Gateway Path Unavailable
pub const EXCEPTION_GATEWAY_PATH_UNAVAILABLE: u8 = 0x0A;

/// Gateway Target Device Failed to Respond
pub const EXCEPTION_GATEWAY_TARGET_FAILED: u8 = 0x0B;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_frame_size_constants() {
        assert_eq!(MBAP_HEADER_LEN, 6);
        assert_eq!(MAX_PDU_SIZE, 253);
        assert_eq!(MAX_MBAP_LENGTH, 254);
    }

    #[test]
    fn test_register_limits() {
        // Verify read register limit calculation
        let read_pdu_size = 1 + 1 + (MAX_READ_REGISTERS * 2);
        assert!(read_pdu_size <= MAX_PDU_SIZE);
        assert_eq!(MAX_READ_REGISTERS, 125);

        // Verify write register limit calculation
        let write_pdu_size = 1 + 2 + 2 + 1 + (MAX_WRITE_REGISTERS * 2);
        assert!(write_pdu_size <= MAX_PDU_SIZE);
        assert_eq!(MAX_WRITE_REGISTERS, 123);
    }

    #[test]
    fn test_coil_limits() {
        // Verify read coil limit
        let read_coil_bytes = MAX_READ_COILS.div_ceil(8);
        let read_coil_pdu = 1 + 1 + read_coil_bytes;
        assert!(read_coil_pdu <= MAX_PDU_SIZE);
        assert_eq!(MAX_READ_COILS, 2000);

        // Verify write coil limit
        let write_coil_bytes = MAX_WRITE_COILS.div_ceil(8);
        let write_coil_pdu = 1 + 2 + 2 + 1 + write_coil_bytes;
        assert!(write_coil_pdu <= MAX_PDU_SIZE);
        assert_eq!(MAX_WRITE_COILS, 1968);
    }
}
