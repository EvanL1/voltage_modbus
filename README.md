# Voltage Modbus

[![Crates.io](https://img.shields.io/crates/v/voltage_modbus.svg)](https://crates.io/crates/voltage_modbus)
[![docs.rs](https://docs.rs/voltage_modbus/badge.svg)](https://docs.rs/voltage_modbus)
[![Rust](https://img.shields.io/badge/rust-1.85+-blue.svg)](https://www.rust-lang.org)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

High-performance Modbus TCP/RTU library for Rust, designed for industrial automation and IoT applications.

## Features

- Async/await with Tokio
- Modbus TCP and RTU protocols
- Zero unsafe code
- Generic client architecture (TCP/RTU share application logic)

## Installation

```bash
cargo add voltage_modbus
```

For RTU (serial) support:

```toml
voltage_modbus = { version = "0.4", features = ["rtu"] }
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
    // Simple: default 8N1 configuration
    let mut client = ModbusRtuClient::new("/dev/ttyUSB0", 9600)?;

    // Or with full configuration (data bits, stop bits, parity, timeout)
    let mut client = ModbusRtuClient::with_config_and_logging(
        "/dev/ttyUSB0",
        9600,
        tokio_serial::DataBits::Eight,
        tokio_serial::StopBits::One,
        tokio_serial::Parity::Even,  // Even parity
        Duration::from_secs(1),
        None,
    )?;

    let coils = client.read_01(1, 0, 8).await?;
    println!("Coils: {:?}", coils);

    client.close().await?;
    Ok(())
}
```

## Supported Function Codes

| Code | Function                 | Status |
| ---- | ------------------------ | ------ |
| 0x01 | Read Coils               | ✅     |
| 0x02 | Read Discrete Inputs     | ✅     |
| 0x03 | Read Holding Registers   | ✅     |
| 0x04 | Read Input Registers     | ✅     |
| 0x05 | Write Single Coil        | ✅     |
| 0x06 | Write Single Register    | ✅     |
| 0x0F | Write Multiple Coils     | ✅     |
| 0x10 | Write Multiple Registers | ✅     |

## Architecture

```
┌─────────────────────────────────────────────────┐
│              Application Layer                  │
│  ┌─────────────────┐    ┌─────────────────┐     │
│  │ ModbusTcpClient │    │ ModbusRtuClient │     │
│  └────────┬────────┘    └────────┬────────┘     │
│           └──────────┬───────────┘              │
│           ┌──────────┴───────────┐              │
│           │ GenericModbusClient  │              │
│           │  (Shared PDU Logic)  │              │
│           └──────────────────────┘              │
└─────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────┐
│               Transport Layer                   │
│  ┌─────────────────┐    ┌─────────────────┐     │
│  │  TcpTransport   │    │  RtuTransport   │     │
│  └─────────────────┘    └─────────────────┘     │
└─────────────────────────────────────────────────┘
```

TCP and RTU share identical PDU (Protocol Data Unit), differing only in transport framing.

## Documentation

- [API Reference](https://docs.rs/voltage_modbus)
- [GitHub](https://github.com/EvanL1/voltage_modbus)

## License

MIT License - see [LICENSE](LICENSE) for details.
