//! Shared traits for request scheduling components.
//!
//! `voltage_modbus` has two distinct scheduling strategies whose internals are
//! intentionally different:
//!
//! - [`CommandBatcher`](crate::batcher::CommandBatcher) — a stateful,
//!   time-windowed buffer for *write* commands.
//! - [`ReadCoalescer`](crate::coalescer::ReadCoalescer) — a stateless merger
//!   for overlapping/adjacent *read* requests.
//!
//! Forcing both into a single scheduler trait would create a leaky abstraction:
//! their input/output shapes and statefulness differ fundamentally. Instead,
//! this module exposes the minimum surface they genuinely share —
//! [`ScheduledRequest`], which lets callers route, log, or aggregate either
//! request type by slave + function code.

/// A request that can be routed to a Modbus slave with a known function code.
///
/// Implemented by both [`BatchCommand`](crate::batcher::BatchCommand) (writes)
/// and [`ReadRequest`](crate::coalescer::ReadRequest) (reads). Allows
/// downstream code (logging, per-slave rate limiting, tracing enrichment) to
/// treat either uniformly without caring which scheduler produced them.
pub trait ScheduledRequest {
    /// Modbus slave/unit ID (0 = broadcast).
    fn slave_id(&self) -> u8;
    /// Modbus function code (e.g. 0x03 read holding, 0x10 write multiple).
    fn function_code(&self) -> u8;
}

impl ScheduledRequest for crate::batcher::BatchCommand {
    fn slave_id(&self) -> u8 {
        self.slave_id
    }
    fn function_code(&self) -> u8 {
        self.function_code
    }
}

impl ScheduledRequest for crate::coalescer::ReadRequest {
    fn slave_id(&self) -> u8 {
        self.slave_id
    }
    fn function_code(&self) -> u8 {
        self.function
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::batcher::BatchCommand;
    use crate::bytes::ByteOrder;
    use crate::coalescer::ReadRequest;
    use crate::value::ModbusValue;

    fn sid<R: ScheduledRequest>(r: &R) -> u8 {
        r.slave_id()
    }
    fn fc<R: ScheduledRequest>(r: &R) -> u8 {
        r.function_code()
    }

    #[test]
    fn read_request_implements_trait() {
        let r = ReadRequest::new(7, 0x03, 100, 10);
        assert_eq!(sid(&r), 7);
        assert_eq!(fc(&r), 0x03);
    }

    #[test]
    fn batch_command_implements_trait() {
        let cmd = BatchCommand {
            point_id: 1,
            value: ModbusValue::U16(1),
            slave_id: 3,
            function_code: 0x10,
            register_address: 0,
            data_type: "uint16",
            byte_order: ByteOrder::BigEndian,
        };
        assert_eq!(sid(&cmd), 3);
        assert_eq!(fc(&cmd), 0x10);
    }
}
