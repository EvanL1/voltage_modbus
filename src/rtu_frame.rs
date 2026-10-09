//! Recognizes a request to this RTU server that has only partly arrived.
//!
//! The server delimits frames by t3.5 silence and checks each frame's CRC as a
//! whole. USB-RS485 adapters deliver bytes in chunks (FTDI's default latency
//! timer is 16 ms), far longer apart than t3.5 at high baud rates, which would
//! split one request into several "frames" that all fail the CRC. So at a
//! silence the server first asks [`is_partial_request`]: when the buffer is the
//! beginning of a fixed-layout request addressed to it (or broadcast) and still
//! shorter than that layout, it waits for the rest instead of dropping it.
//!
//! Nothing else changes: frames are still only processed whole and CRC-valid,
//! so this can delay a frame but never invent one.

/// Length of a request layout, if enough of the frame has arrived to tell.
enum Expected {
    Len(usize),
    /// The byte that determines the length has not arrived yet.
    NeedsMore,
}

/// Byte-count-dependent length: `fixed + buf[idx]`.
fn with_byte_count(buf: &[u8], idx: usize, fixed: usize) -> Expected {
    buf.get(idx).map_or(Expected::NeedsMore, |&bc| {
        Expected::Len(fixed + bc as usize)
    })
}

/// Total length of a master request with this function code (`None`: unknown
/// code or variable length, which is never waited for).
fn request_len(buf: &[u8]) -> Option<Expected> {
    Some(match buf[1] {
        0x01..=0x06 | 0x08 => Expected::Len(8), // addr/sub + qty/value
        0x07 | 0x0B | 0x0C | 0x11 => Expected::Len(4),
        0x0F | 0x10 => with_byte_count(buf, 6, 9), // ..qty + bc + data + crc
        0x14 | 0x15 => with_byte_count(buf, 2, 5), // file record: bc + data
        0x16 => Expected::Len(10),
        0x17 => with_byte_count(buf, 10, 13),
        0x18 => Expected::Len(6), // FIFO pointer address
        0x2B => Expected::Len(7), // MEI 0x0E: type + read code + object id
        _ => return None,
    })
}

/// True when `buf` is the beginning of a request to `own_slave_id` (or a
/// broadcast) that is shorter than its layout — i.e. the rest is still coming.
pub(crate) fn is_partial_request(buf: &[u8], own_slave_id: u8) -> bool {
    match buf {
        [] => false,
        [slave, ..] if *slave != own_slave_id && *slave != 0 => false,
        [_] => true, // function code not here yet
        _ => match request_len(buf) {
            Some(Expected::Len(n)) => buf.len() < n,
            Some(Expected::NeedsMore) => true,
            None => false,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crc::{Crc, CRC_16_MODBUS};

    const CRC_MODBUS: Crc<u16> = Crc::<u16>::new(&CRC_16_MODBUS);
    const OWN: u8 = 0x07;

    fn frame(body: &[u8]) -> Vec<u8> {
        let mut f = body.to_vec();
        f.extend_from_slice(&CRC_MODBUS.checksum(body).to_le_bytes());
        f
    }

    /// Every proper prefix of a request to us is partial; the full request is not.
    fn assert_prefixes_partial(f: &[u8]) {
        for cut in 1..f.len() {
            assert!(
                is_partial_request(&f[..cut], OWN),
                "prefix {cut} of {f:02X?}"
            );
        }
        assert!(!is_partial_request(f, OWN), "full {f:02X?}");
    }

    #[test]
    fn prefixes_of_requests_to_us_are_partial() {
        for fc in 1..=6u8 {
            assert_prefixes_partial(&frame(&[OWN, fc, 0x00, 0x10, 0x00, 0x02]));
        }
        for fc in [0x07, 0x0B, 0x0C, 0x11] {
            assert_prefixes_partial(&frame(&[OWN, fc]));
        }
        assert_prefixes_partial(&frame(&[OWN, 0x08, 0x00, 0x00, 0xA5, 0x37]));
        assert_prefixes_partial(&frame(&[
            OWN, 0x0F, 0x00, 0x13, 0x00, 0x0A, 0x02, 0xCD, 0x01,
        ]));
        assert_prefixes_partial(&frame(&[
            OWN, 0x10, 0x00, 0x01, 0x00, 0x02, 0x04, 0x00, 0x0A, 0x01, 0x02,
        ]));
        assert_prefixes_partial(&frame(&[
            OWN, 0x14, 0x07, 0x06, 0x00, 0x04, 0x00, 0x01, 0x00, 0x02,
        ]));
        assert_prefixes_partial(&frame(&[OWN, 0x16, 0x00, 0x04, 0x00, 0xF2, 0x00, 0x25]));
        assert_prefixes_partial(&frame(&[
            OWN, 0x17, 0x00, 0x03, 0x00, 0x01, 0x00, 0x0E, 0x00, 0x01, 0x02, 0x00, 0xFF,
        ]));
        assert_prefixes_partial(&frame(&[OWN, 0x18, 0x04, 0xDE]));
        assert_prefixes_partial(&frame(&[OWN, 0x2B, 0x0E, 0x01, 0x00]));
        // Broadcast requests are ours too
        assert_prefixes_partial(&frame(&[0x00, 0x06, 0x00, 0x01, 0x00, 0x03]));
        // A maximum-size FC10 (123 registers)
        let mut body = vec![OWN, 0x10, 0x00, 0x00, 0x00, 123, 246];
        body.extend_from_slice(&[0x5A; 246]);
        assert_prefixes_partial(&frame(&body));
    }

    #[test]
    fn other_slaves_traffic_is_never_waited_for() {
        let other = frame(&[0x02, 0x03, 0x00, 0x00, 0x00, 0x01]);
        for cut in 1..=other.len() {
            assert!(!is_partial_request(&other[..cut], OWN));
        }
    }

    #[test]
    fn unknown_or_overlong_is_not_partial() {
        // Unknown function code: length unknown, never waited for
        assert!(!is_partial_request(&[OWN, 0x41, 0x00], OWN));
        // Longer than its layout (e.g. a request plus trailing bytes)
        let mut f = frame(&[OWN, 0x03, 0x00, 0x00, 0x00, 0x01]);
        f.push(0xFF);
        assert!(!is_partial_request(&f, OWN));
        assert!(!is_partial_request(&[], OWN));
    }
}
