//! Voltage Modbus Demo
//!
//! Demonstrates the voltage_modbus library features including:
//! - Basic Modbus TCP client operations (read/write registers and coils)
//! - Industrial Enhancement features (ModbusValue, ByteOrder, ModbusCodec)
//! - Simplified API with function code naming (read_03, write_06, etc.)
//!
//! Usage: cargo run --bin demo [server_address]
//! Example: cargo run --bin demo 127.0.0.1:502

use std::time::Duration;
use tokio::time::sleep;
use voltage_modbus::{
    regs_to_f32, ByteOrder, DeviceLimits, ModbusClient, ModbusTcpClient, ModbusValue,
};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🚀 Voltage Modbus v0.4.0 Demo");
    println!("=============================");
    println!("Industrial Enhancement Features Showcase\n");

    // =========================================================================
    // Part 1: Industrial Data Types Demo (No connection required)
    // =========================================================================
    println!("📦 Part 1: Industrial Data Types (ModbusValue)");
    println!("-----------------------------------------------");

    // ModbusValue - Industrial data type abstraction
    let values = [
        ModbusValue::U16(1234),
        ModbusValue::I16(-500),
        ModbusValue::U32(100000),
        ModbusValue::I32(-50000),
        ModbusValue::F32(std::f32::consts::PI),
        ModbusValue::F64(std::f64::consts::E),
        ModbusValue::Bool(true),
    ];

    for value in &values {
        println!(
            "  {} -> as_f64: {:.4}, registers: {}",
            value,
            value.as_f64(),
            value.register_count()
        );
    }

    // =========================================================================
    // Part 2: Byte Order Demo
    // =========================================================================
    println!("\n🔄 Part 2: Byte Order Handling");
    println!("-------------------------------");

    let byte_orders = [
        ByteOrder::BigEndian,
        ByteOrder::LittleEndian,
        ByteOrder::BigEndianSwap,
        ByteOrder::LittleEndianSwap,
    ];

    let test_regs: [u16; 2] = [0x4248, 0x0000]; // 50.0 as F32 in BigEndian
    println!("  Test registers: {:04X} {:04X}", test_regs[0], test_regs[1]);

    for order in &byte_orders {
        let f32_val = voltage_modbus::regs_to_f32(&test_regs, *order);
        println!("    {:?} -> f32: {:.2}", order, f32_val);
    }

    // =========================================================================
    // Part 3: Data Encoding/Decoding Demo
    // =========================================================================
    println!("\n📊 Part 3: Data Encoding/Decoding");
    println!("----------------------------------");

    // Encode F32 to registers (BigEndian)
    let f32_value: f32 = 123.456;
    let f32_bytes = f32_value.to_be_bytes();
    let encoded_f32 = [
        u16::from_be_bytes([f32_bytes[0], f32_bytes[1]]),
        u16::from_be_bytes([f32_bytes[2], f32_bytes[3]]),
    ];
    println!("  F32 {} -> registers: {:04X} {:04X}", f32_value, encoded_f32[0], encoded_f32[1]);

    // Decode back using regs_to_f32
    let decoded_f32 = regs_to_f32(&encoded_f32, ByteOrder::BigEndian);
    println!("  Decoded F32: {:.3}", decoded_f32);

    // U32 encoding example
    let u32_value: u32 = 0x12345678;
    let encoded_u32 = [(u32_value >> 16) as u16, (u32_value & 0xFFFF) as u16];
    println!("  U32 0x{:08X} -> registers: {:04X} {:04X}", u32_value, encoded_u32[0], encoded_u32[1]);

    // =========================================================================
    // Part 4: DeviceLimits Demo
    // =========================================================================
    println!("\n🎛️  Part 4: DeviceLimits - Protocol Configuration");
    println!("-------------------------------------------------");

    let default_limits = DeviceLimits::default();
    println!("  Default limits:");
    println!("    Max read registers: {}", default_limits.max_read_registers);
    println!("    Max write registers: {}", default_limits.max_write_registers);
    println!("    Max read coils: {}", default_limits.max_read_coils);
    println!("    Inter-request delay: {}ms", default_limits.inter_request_delay_ms);

    let custom_limits = DeviceLimits::new()
        .with_max_read_registers(50)
        .with_max_write_registers(20)
        .with_inter_request_delay_ms(100);
    println!("  Custom limits:");
    println!("    Max read registers: {}", custom_limits.max_read_registers);
    println!("    Inter-request delay: {}ms", custom_limits.inter_request_delay_ms);

    // =========================================================================
    // Part 5: TCP Client Demo (requires Modbus server)
    // =========================================================================
    println!("\n🔌 Part 5: TCP Client Operations");
    println!("---------------------------------");

    let server_address = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "127.0.0.1:502".to_string());

    println!("  Connecting to {}...", server_address);

    let timeout = Duration::from_secs(5);

    let mut client = match ModbusTcpClient::from_address(&server_address, timeout).await {
        Ok(client) => {
            println!("  ✅ Connected successfully!");
            client
        }
        Err(e) => {
            println!("  ⚠️  Connection failed: {}", e);
            println!("  (This is expected if no Modbus server is running)");
            println!("\n🎉 Demo completed! (TCP operations skipped)");
            return Ok(());
        }
    };

    let slave_id = 1;

    // Read operations using simplified API
    println!("\n  📖 Read Operations:");

    match client.read_03(slave_id, 0, 5).await {
        Ok(values) => {
            println!("    FC03 Holding registers 0-4: {:?}", values);

            // Decode as F32 using regs_to_f32
            if values.len() >= 2 {
                let f32_val = regs_to_f32(&[values[0], values[1]], ByteOrder::BigEndian);
                println!("    -> First 2 registers as F32: {:.4}", f32_val);
            }
        }
        Err(e) => println!("    FC03 Error: {}", e),
    }

    sleep(Duration::from_millis(50)).await;

    match client.read_01(slave_id, 0, 8).await {
        Ok(coils) => {
            let states: Vec<&str> = coils.iter().map(|&c| if c { "ON" } else { "OFF" }).collect();
            println!("    FC01 Coils 0-7: {:?}", states);
        }
        Err(e) => println!("    FC01 Error: {}", e),
    }

    // Write operations
    println!("\n  ✏️  Write Operations:");

    match client.write_06(slave_id, 100, 0x1234).await {
        Ok(_) => println!("    FC06 Wrote register 100 = 0x1234"),
        Err(e) => println!("    FC06 Error: {}", e),
    }

    sleep(Duration::from_millis(50)).await;

    // Write F32 value (encode manually)
    let temp: f32 = 98.6;
    let temp_bytes = temp.to_be_bytes();
    let f32_regs = [
        u16::from_be_bytes([temp_bytes[0], temp_bytes[1]]),
        u16::from_be_bytes([temp_bytes[2], temp_bytes[3]]),
    ];
    match client.write_10(slave_id, 200, &f32_regs).await {
        Ok(_) => println!("    FC16 Wrote F32 98.6 to registers 200-201"),
        Err(e) => println!("    FC16 Error: {}", e),
    }

    // Statistics
    let stats = client.get_stats();
    println!("\n  📊 Statistics:");
    println!("    Requests: {}, Responses: {}", stats.requests_sent, stats.responses_received);
    println!("    Bytes sent: {}, received: {}", stats.bytes_sent, stats.bytes_received);

    if let Err(e) = client.close().await {
        eprintln!("  ⚠️  Close error: {}", e);
    }

    println!("\n🎉 Demo completed!");
    println!("📚 Documentation: https://docs.rs/voltage_modbus");
    println!("🔗 Repository: https://github.com/EvanL1/voltage_modbus");

    Ok(())
}
