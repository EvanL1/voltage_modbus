# Voltage Modbus

[![Crates.io](https://img.shields.io/crates/v/voltage_modbus.svg)](https://crates.io/crates/voltage_modbus)
[![docs.rs](https://docs.rs/voltage_modbus/badge.svg)](https://docs.rs/voltage_modbus)
[![Rust](https://img.shields.io/badge/rust-1.85+-blue.svg)](https://www.rust-lang.org)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

High-performance Modbus TCP/RTU library for Rust, designed for industrial automation and IoT applications.

## Why voltage_modbus?

| Feature | voltage_modbus | tokio-modbus |
|---------|---------------|-------------|
| Request pipelining | **Yes** — N requests in ~1 RTT | No — strictly sequential |
| Read coalescing | **Yes** — auto-merge adjacent reads | No |
| Zero-alloc hot path | **Yes** — stack frames, persistent buffers | No — per-request heap alloc |
| TCP_NODELAY | **Yes** — eliminates Nagle delay | No |
| Broadcast (slave=0) | **Yes** — no-timeout write | Triggers timeout error |
| `no_std` core | **Yes** — PDU/protocol/error | No |
| Auto reconnection | **Yes** | No |
| Float/multi-register codec | **Yes** — f32/f64/i32 with byte order | No |
| Async trait | RPITIT (zero-cost) | `async-trait` (Box alloc) |

## Features

- **Request pipelining** — send multiple requests in a single write, match responses by Transaction ID
- **Read coalescing** — automatically merge adjacent register reads to minimize network round-trips
- **Zero-alloc hot path** — stack-allocated frames, persistent read buffers, no per-request heap allocation
- **Modbus TCP and RTU** — generic client architecture, shared PDU logic
- **`no_std` support** — core modules (PDU, protocol, error, constants) work without std
- **Broadcast support** — slave_id=0 write operations return immediately without waiting for response
- **Zero unsafe code** — pure safe Rust
- **Async/await** — built on Tokio with zero-cost async traits (RPITIT)

## Installation

```bash
cargo add voltage_modbus
```

For RTU (serial) support:

```toml
voltage_modbus = { version = "0.5", features = ["rtu"] }
```

For `no_std` (PDU encoding/decoding only):

```toml
voltage_modbus = { version = "0.5", default-features = false }
```

## Quick Start

### TCP Client

```rust
use voltage_modbus::{ModbusTcpClient, ModbusClient, ModbusResult};
use std::time::Duration;

#[tokio::main]
async fn main() -> ModbusResult<()> {
    let mut client = ModbusTcpClient::from_address("127.0.0.1:502", Duration::from_secs(5)).await?;

    // Read holding registers
    let values = client.read_03(1, 0, 10).await?;
    println!("Registers: {:?}", values);

    // Write single register
    client.write_06(1, 100, 0x1234).await?;

    client.close().await?;
    Ok(())
}
```

### RTU Client

```rust
use voltage_modbus::{ModbusRtuClient, ModbusClient, ModbusResult};
use std::time::Duration;

#[tokio::main]
async fn main() -> ModbusResult<()> {
    let mut client = ModbusRtuClient::new("/dev/ttyUSB0", 9600)?;

    let coils = client.read_01(1, 0, 8).await?;
    println!("Coils: {:?}", coils);

    client.close().await?;
    Ok(())
}
```

### Pipelining — N Requests in ~1 RTT

```rust
use voltage_modbus::{ModbusTcpClient, ModbusResult};
use std::time::Duration;

#[tokio::main]
async fn main() -> ModbusResult<()> {
    let mut client = ModbusTcpClient::from_address("127.0.0.1:502", Duration::from_secs(5)).await?;

    // Read 3 different register regions in a single network round-trip
    let results = client.pipeline_reads(1, &[
        (0, 10),    // registers 0-9
        (100, 5),   // registers 100-104
        (200, 3),   // registers 200-202
    ], Duration::from_secs(5)).await?;

    for (i, result) in results.iter().enumerate() {
        match result {
            Ok(regs) => println!("Region {}: {:?}", i, regs),
            Err(e) => println!("Region {} failed: {}", i, e),
        }
    }

    client.close().await?;
    Ok(())
}
```

### Read Coalescing — Auto-merge Adjacent Reads

```rust
use voltage_modbus::{ModbusTcpClient, ModbusClient, ModbusResult};
use std::time::Duration;

#[tokio::main]
async fn main() -> ModbusResult<()> {
    let mut client = ModbusTcpClient::from_address("127.0.0.1:502", Duration::from_secs(5)).await?;

    // These 3 regions are close together — the library automatically merges them
    // into fewer network requests based on a configurable gap threshold
    let results = client.read_holding_registers_coalesced(1, &[
        (0, 2),    // temperature (registers 0-1)
        (2, 2),    // pressure (registers 2-3)
        (10, 2),   // flow rate (registers 10-11)
    ]).await?;

    // Results are returned in the original order
    println!("Temperature regs: {:?}", results[0]);
    println!("Pressure regs: {:?}", results[1]);
    println!("Flow rate regs: {:?}", results[2]);

    client.close().await?;
    Ok(())
}
```

## Supported Function Codes

| Code | Function                 | Method |
| ---- | ------------------------ | ------ |
| 0x01 | Read Coils               | `read_01()` / `read_coils()` |
| 0x02 | Read Discrete Inputs     | `read_02()` / `read_discrete_inputs()` |
| 0x03 | Read Holding Registers   | `read_03()` / `read_holding_registers()` |
| 0x04 | Read Input Registers     | `read_04()` / `read_input_registers()` |
| 0x05 | Write Single Coil        | `write_05()` / `write_single_coil()` |
| 0x06 | Write Single Register    | `write_06()` / `write_single_register()` |
| 0x0F | Write Multiple Coils     | `write_0f()` / `write_multiple_coils()` |
| 0x10 | Write Multiple Registers | `write_10()` / `write_multiple_registers()` |

## Architecture

```
┌───────────────────────────────────────────────┐
│             Application Layer                 │
│                                               │
│  ┌──────────────────┐  ┌─────────────────┐    │
│  │ ModbusTcpClient  │  │ ModbusRtuClient │    │
│  │  + pipeline()    │  └────────┬────────┘    │
│  │  + coalesced()   │           │             │
│  └────────┬─────────┘           │             │
│           │─────────────────────┘             │
│  ┌────────┴──────────────────┐                │
│  │    GenericModbusClient    │                │
│  │     (Shared PDU Logic)    │                │
│  └───────────────────────────┘                │
└───────────────────────────────────────────────┘
┌───────────────────────────────────────────────┐
│             Transport Layer                   │
│                                               │
│  ┌─────────────────┐  ┌─────────────────┐     │
│  │  TcpTransport   │  │  RtuTransport   │     │
│  │  TCP_NODELAY    │  │  CRC-16         │     │
│  │  zero-alloc I/O │  │  frame gap      │     │
│  └─────────────────┘  └─────────────────┘     │
└───────────────────────────────────────────────┘
┌───────────────────────────────────────────────┐
│        Core (no_std compatible)               │
│                                               │
│  ModbusPdu · ModbusFunction · ModbusError     │
│  constants · protocol · CRC                   │
└───────────────────────────────────────────────┘
```

TCP and RTU share identical PDU (Protocol Data Unit), differing only in transport framing.

## Documentation

- [API Reference](https://docs.rs/voltage_modbus)
- [GitHub](https://github.com/EvanL1/voltage_modbus)

## License

MIT License - see [LICENSE](LICENSE) for details.
