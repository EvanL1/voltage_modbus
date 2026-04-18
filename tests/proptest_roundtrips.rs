//! Property-based tests for voltage_modbus pure-logic modules.
//!
//! Tests byte-order roundtrips, PDU builder invariants, and read-coalescer
//! structural invariants using proptest with 256 randomized cases each.

use proptest::collection::vec;
use proptest::prelude::*;
use voltage_modbus::bytes::{
    f32_to_regs, f64_to_regs, i32_to_regs, i64_to_regs, regs_to_f32, regs_to_f64, regs_to_i32,
    regs_to_i64, regs_to_u32, regs_to_u64, u32_to_regs, u64_to_regs,
};
use voltage_modbus::coalescer::{ReadCoalescer, ReadRequest};
use voltage_modbus::pdu::PduBuilder;
use voltage_modbus::ByteOrder;

// ============================================================================
// Helpers
// ============================================================================

/// All four 32/64-bit byte orders as an index 0..=3.
fn byte_order_32(idx: u8) -> ByteOrder {
    match idx % 4 {
        0 => ByteOrder::BigEndian,
        1 => ByteOrder::LittleEndian,
        2 => ByteOrder::BigEndianSwap,
        _ => ByteOrder::LittleEndianSwap,
    }
}

// ============================================================================
// Byte-order roundtrip properties — 32-bit types
// ============================================================================

proptest! {
    /// u32 → registers → u32 must be identity for all four byte orders.
    #[test]
    fn prop_u32_roundtrip(value: u32, order_idx in 0u8..4) {
        let order = byte_order_32(order_idx);
        let regs = u32_to_regs(value, order);
        let decoded = regs_to_u32(&regs, order);
        prop_assert_eq!(decoded, value, "u32 roundtrip failed for {:?}", order);
    }

    /// i32 → registers → i32 must be identity for all four byte orders.
    #[test]
    fn prop_i32_roundtrip(value: i32, order_idx in 0u8..4) {
        let order = byte_order_32(order_idx);
        let regs = i32_to_regs(value, order);
        let decoded = regs_to_i32(&regs, order);
        prop_assert_eq!(decoded, value, "i32 roundtrip failed for {:?}", order);
    }

    /// f32 roundtrip via bit-level identity (handles NaN bit patterns correctly).
    #[test]
    fn prop_f32_roundtrip(bits: u32, order_idx in 0u8..4) {
        let value = f32::from_bits(bits);
        let order = byte_order_32(order_idx);
        let regs = f32_to_regs(value, order);
        let decoded = regs_to_f32(&regs, order);
        // Compare bit-for-bit so NaN bit patterns survive the roundtrip.
        prop_assert_eq!(
            decoded.to_bits(),
            value.to_bits(),
            "f32 roundtrip failed for {:?} (bits={:#010x})",
            order,
            bits
        );
    }
}

// ============================================================================
// Byte-order roundtrip properties — 64-bit types
// ============================================================================

proptest! {
    /// u64 → registers → u64 must be identity for all four byte orders.
    #[test]
    fn prop_u64_roundtrip(value: u64, order_idx in 0u8..4) {
        let order = byte_order_32(order_idx);
        let regs = u64_to_regs(value, order);
        let decoded = regs_to_u64(&regs, order);
        prop_assert_eq!(decoded, value, "u64 roundtrip failed for {:?}", order);
    }

    /// i64 → registers → i64 must be identity for all four byte orders.
    #[test]
    fn prop_i64_roundtrip(value: i64, order_idx in 0u8..4) {
        let order = byte_order_32(order_idx);
        let regs = i64_to_regs(value, order);
        let decoded = regs_to_i64(&regs, order);
        prop_assert_eq!(decoded, value, "i64 roundtrip failed for {:?}", order);
    }

    /// f64 roundtrip via bit-level identity (handles NaN bit patterns correctly).
    #[test]
    fn prop_f64_roundtrip(bits: u64, order_idx in 0u8..4) {
        let value = f64::from_bits(bits);
        let order = byte_order_32(order_idx);
        let regs = f64_to_regs(value, order);
        let decoded = regs_to_f64(&regs, order);
        prop_assert_eq!(
            decoded.to_bits(),
            value.to_bits(),
            "f64 roundtrip failed for {:?} (bits={:#018x})",
            order,
            bits
        );
    }
}

// ============================================================================
// PDU builder properties
// ============================================================================

proptest! {
    /// FC 1–4 with valid quantity (1..=125) must succeed and produce a 5-byte PDU.
    #[test]
    fn prop_pdu_read_request_valid_fc(
        fc in 1u8..=4,
        address: u16,
        quantity in 1u16..=125,
    ) {
        let result = PduBuilder::build_read_request(fc, address, quantity);
        prop_assert!(result.is_ok(), "FC {:02X} address={} qty={} should succeed", fc, address, quantity);
        let pdu = result.unwrap();
        prop_assert_eq!(pdu.len(), 5, "read request PDU must be 5 bytes");
        prop_assert_eq!(pdu.as_slice()[0], fc, "first byte must be function code");
    }

    /// FC outside 1–4 (excluding 0, to avoid accidental hits) must return Err.
    #[test]
    fn prop_pdu_read_request_invalid_fc(fc in 5u8..=255) {
        let result = PduBuilder::build_read_request(fc, 0, 1);
        prop_assert!(result.is_err(), "FC {:02X} should be rejected", fc);
    }

    /// FC 0 must also be rejected.
    #[test]
    fn prop_pdu_read_request_fc_zero(_dummy: u8) {
        let result = PduBuilder::build_read_request(0, 0, 1);
        prop_assert!(result.is_err(), "FC 0x00 should be rejected");
    }
}

// ============================================================================
// Read coalescer structural invariants
// ============================================================================

/// Strategy that generates a `ReadRequest` with Modbus-legal quantities.
fn arb_read_request() -> impl Strategy<Value = ReadRequest> {
    (
        any::<u8>(),              // slave_id (0–255)
        prop_oneof![Just(3u8), Just(4u8)],  // function: 0x03 or 0x04
        any::<u16>(),             // address
        1u16..=125u16,            // quantity within Modbus limit
    )
        .prop_map(|(slave_id, function, address, quantity)| ReadRequest {
            slave_id,
            function,
            address,
            quantity,
        })
}

proptest! {
    /// Every input request appears in exactly one CoalescedRead, and the total
    /// mapping count across all groups equals the number of inputs.
    #[test]
    fn prop_coalescer_covers_all_requests(
        requests in vec(arb_read_request(), 0..=32),
    ) {
        let coalescer = ReadCoalescer::new();
        let coalesced = coalescer.coalesce(&requests);

        // Sum of mappings == number of input requests.
        let total_mappings: usize = coalesced.iter().map(|c| c.mappings.len()).sum();
        prop_assert_eq!(
            total_mappings,
            requests.len(),
            "total mapping count must equal input count"
        );
    }

    /// Each original index appears exactly once across all CoalescedRead mappings.
    #[test]
    fn prop_coalescer_no_duplicate_mappings(
        requests in vec(arb_read_request(), 0..=32),
    ) {
        let coalescer = ReadCoalescer::new();
        let coalesced = coalescer.coalesce(&requests);

        let mut seen = vec![false; requests.len()];
        for group in &coalesced {
            for &(orig_idx, _offset, _qty) in &group.mappings {
                prop_assert!(
                    orig_idx < requests.len(),
                    "orig_idx {} out of bounds (len={})", orig_idx, requests.len()
                );
                prop_assert!(
                    !seen[orig_idx],
                    "orig_idx {} appears more than once", orig_idx
                );
                seen[orig_idx] = true;
            }
        }
        // Every index must have been seen.
        for (i, &was_seen) in seen.iter().enumerate() {
            prop_assert!(was_seen, "orig_idx {} not covered by any CoalescedRead", i);
        }
    }

    /// Every CoalescedRead must have quantity > 0 and quantity <= MAX_READ_REGISTERS (125).
    #[test]
    fn prop_coalescer_quantity_within_limits(
        requests in vec(arb_read_request(), 0..=32),
    ) {
        let coalescer = ReadCoalescer::new();
        let coalesced = coalescer.coalesce(&requests);

        for group in &coalesced {
            prop_assert!(
                group.quantity > 0,
                "CoalescedRead quantity must be > 0, got {}",
                group.quantity
            );
            prop_assert!(
                group.quantity <= 125,
                "CoalescedRead quantity {} exceeds MAX_READ_REGISTERS=125",
                group.quantity
            );
        }
    }

    /// All mappings within a CoalescedRead have offsets that respect the merged
    /// address range, accounting for u16 saturation in `end_address()`.
    ///
    /// The coalescer computes its window using `saturating_add`, so when
    /// `address + quantity` overflows u16, `group.quantity` may be smaller than
    /// the individual request's quantity. We therefore only assert the invariant
    /// when no u16 overflow occurs.
    #[test]
    fn prop_coalescer_mapping_offsets_in_range(
        requests in vec(arb_read_request(), 0..=32),
    ) {
        let coalescer = ReadCoalescer::new();
        let coalesced = coalescer.coalesce(&requests);

        for group in &coalesced {
            for &(orig_idx, offset, orig_qty) in &group.mappings {
                let req = &requests[orig_idx];
                // Only assert when the original request end_address does not saturate.
                let overflows = req.address.checked_add(req.quantity).is_none();
                if !overflows {
                    let end = offset + orig_qty; // safe: offset and qty fit in a u16 window
                    prop_assert!(
                        end <= group.quantity,
                        "mapping orig_idx={} offset={} + qty={} = {} exceeds group quantity={} (addr={}, req_qty={})",
                        orig_idx, offset, orig_qty, end, group.quantity, req.address, req.quantity
                    );
                }
            }
        }
    }

    /// Requests that belong to the same CoalescedRead must share the same
    /// slave_id and function code.
    #[test]
    fn prop_coalescer_homogeneous_groups(
        requests in vec(arb_read_request(), 0..=32),
    ) {
        let coalescer = ReadCoalescer::new();
        let coalesced = coalescer.coalesce(&requests);

        for group in &coalesced {
            for &(orig_idx, _offset, _qty) in &group.mappings {
                let req = &requests[orig_idx];
                prop_assert_eq!(
                    req.slave_id, group.slave_id,
                    "orig_idx={} slave_id={} but group slave_id={}",
                    orig_idx, req.slave_id, group.slave_id
                );
                prop_assert_eq!(
                    req.function, group.function,
                    "orig_idx={} function={} but group function={}",
                    orig_idx, req.function, group.function
                );
            }
        }
    }

    /// Empty input always produces empty output.
    #[test]
    fn prop_coalescer_empty_input(_dummy: u8) {
        let coalescer = ReadCoalescer::new();
        let result = coalescer.coalesce(&[]);
        prop_assert!(result.is_empty(), "empty input must yield empty output");
    }
}
