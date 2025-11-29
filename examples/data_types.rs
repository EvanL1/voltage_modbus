//! Data Types Example
//!
//! Demonstrates how to work with different industrial data types
//! using `ModbusValue` and `ByteOrder`.
//!
//! # Industrial Data Types
//!
//! Modbus registers are 16-bit (u16), but industrial devices often store
//! larger data types across multiple registers:
//!
//! | Type    | Registers | Description |
//! |---------|-----------|-------------|
//! | U16/I16 | 1         | Single register |
//! | U32/I32 | 2         | Two registers |
//! | F32     | 2         | IEEE 754 float |
//! | U64/I64 | 4         | Four registers |
//! | F64     | 4         | IEEE 754 double |
//!
//! # Byte Order
//!
//! Different manufacturers use different byte orders:
//!
//! - **BigEndian**: Most significant byte first (common in PLCs)
//! - **LittleEndian**: Least significant byte first
//! - **BigEndianSwap**: Big endian with swapped register order
//! - **LittleEndianSwap**: Little endian with swapped register order
//!
//! # Running this example
//!
//! ```bash
//! cargo run --example data_types
//! ```

use voltage_modbus::{regs_to_f32, regs_to_f64, regs_to_i32, regs_to_u32, ByteOrder, ModbusValue};

fn main() {
    // =========================================================================
    // Part 1: ModbusValue - Type-safe industrial values
    // =========================================================================
    println!("╔════════════════════════════════════════════════════════════╗");
    println!("║              ModbusValue Type Examples                     ║");
    println!("╚════════════════════════════════════════════════════════════╝\n");

    let values = [
        ("U16", ModbusValue::U16(1234)),
        ("I16", ModbusValue::I16(-500)),
        ("U32", ModbusValue::U32(100000)),
        ("I32", ModbusValue::I32(-50000)),
        ("F32", ModbusValue::F32(std::f32::consts::PI)),
        ("F64", ModbusValue::F64(std::f64::consts::E)),
        ("Bool", ModbusValue::Bool(true)),
    ];

    println!(
        "{:<8} {:<20} {:<12} {}",
        "Type", "Value", "as_f64()", "Registers"
    );
    println!("{}", "-".repeat(60));

    for (name, value) in &values {
        println!(
            "{:<8} {:<20} {:<12.6} {}",
            name,
            format!("{}", value),
            value.as_f64(),
            value.register_count()
        );
    }

    // =========================================================================
    // Part 2: ByteOrder - Different byte orderings
    // =========================================================================
    println!("\n╔════════════════════════════════════════════════════════════╗");
    println!("║              ByteOrder Decoding Examples                   ║");
    println!("╚════════════════════════════════════════════════════════════╝\n");

    // These registers represent 50.0 as Float32 in BigEndian format
    // 0x4248 0x0000 = 50.0 in IEEE 754 BigEndian
    let test_regs: [u16; 2] = [0x4248, 0x0000];

    println!(
        "Input registers: [0x{:04X}, 0x{:04X}]",
        test_regs[0], test_regs[1]
    );
    println!("\nFloat32 interpretation with different byte orders:\n");

    let orders = [
        (
            ByteOrder::BigEndian,
            "Most common - PLCs, industrial devices",
        ),
        (ByteOrder::LittleEndian, "Some PC-based systems"),
        (ByteOrder::BigEndianSwap, "Modicon/Schneider PLCs"),
        (ByteOrder::LittleEndianSwap, "Rare"),
    ];

    for (order, description) in &orders {
        let f32_val = regs_to_f32(&test_regs, *order);
        println!(
            "  {:20} → {:>12.4}  ({})",
            format!("{:?}", order),
            f32_val,
            description
        );
    }

    // =========================================================================
    // Part 3: U32 Decoding
    // =========================================================================
    println!("\n╔════════════════════════════════════════════════════════════╗");
    println!("║              U32/I32 Decoding Examples                     ║");
    println!("╚════════════════════════════════════════════════════════════╝\n");

    let u32_regs: [u16; 2] = [0x1234, 0x5678];
    println!(
        "Input registers: [0x{:04X}, 0x{:04X}]\n",
        u32_regs[0], u32_regs[1]
    );

    println!(
        "{:<20} {:>15} {:>15}",
        "Byte Order", "U32 (hex)", "U32 (decimal)"
    );
    println!("{}", "-".repeat(50));

    for order in &[
        ByteOrder::BigEndian,
        ByteOrder::LittleEndian,
        ByteOrder::BigEndianSwap,
        ByteOrder::LittleEndianSwap,
    ] {
        let u32_val = regs_to_u32(&u32_regs, *order);
        println!(
            "{:<20} 0x{:08X} {:>15}",
            format!("{:?}", order),
            u32_val,
            u32_val
        );
    }

    // I32 example (signed)
    println!("\nSigned I32 interpretation:");
    let i32_val = regs_to_i32(&u32_regs, ByteOrder::BigEndian);
    println!("  BigEndian: {} (0x{:08X})", i32_val, i32_val as u32);

    // =========================================================================
    // Part 4: F64 (Double) Decoding
    // =========================================================================
    println!("\n╔════════════════════════════════════════════════════════════╗");
    println!("║              F64 (Double) Decoding Example                 ║");
    println!("╚════════════════════════════════════════════════════════════╝\n");

    // These 4 registers represent PI as Float64 in BigEndian
    // IEEE 754 double: 0x400921FB54442D18
    let f64_regs: [u16; 4] = [0x4009, 0x21FB, 0x5444, 0x2D18];
    println!(
        "Input registers: [0x{:04X}, 0x{:04X}, 0x{:04X}, 0x{:04X}]",
        f64_regs[0], f64_regs[1], f64_regs[2], f64_regs[3]
    );

    let f64_val = regs_to_f64(&f64_regs, ByteOrder::BigEndian);
    println!("\nFloat64 BigEndian: {:.15}", f64_val);
    println!("Expected (π):      {:.15}", std::f64::consts::PI);

    // =========================================================================
    // Part 5: Practical Tips
    // =========================================================================
    println!("\n╔════════════════════════════════════════════════════════════╗");
    println!("║              Practical Tips                                ║");
    println!("╚════════════════════════════════════════════════════════════╝\n");

    println!("1. Check your device's documentation for byte order");
    println!("2. Most industrial PLCs use BigEndian");
    println!("3. Modicon/Schneider often use BigEndianSwap");
    println!("4. When in doubt, try reading a known value (like 1.0)");
    println!("5. Use ModbusValue for type-safe value handling");
    println!("\nCommon Float32 test values:");
    println!("  1.0  → BigEndian: [0x3F80, 0x0000]");
    println!("  50.0 → BigEndian: [0x4248, 0x0000]");
    println!("  100.0→ BigEndian: [0x42C8, 0x0000]");
}
