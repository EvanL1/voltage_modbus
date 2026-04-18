//! Throughput micro-benchmarks for voltage_modbus hot paths.
//!
//! All benches are pure-CPU (no network/serial I/O). Run with:
//!   cargo bench --bench throughput

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use voltage_modbus::bytes::{regs_to_f32, regs_to_f64, regs_to_u32, ByteOrder};
use voltage_modbus::coalescer::{ReadCoalescer, ReadRequest};
use voltage_modbus::pdu::PduBuilder;

fn bench_pdu_builder(c: &mut Criterion) {
    let mut g = c.benchmark_group("pdu_builder");

    g.bench_function("read_holding_registers_fc03", |b| {
        b.iter(|| {
            PduBuilder::build_read_request(black_box(0x03), black_box(100), black_box(10)).unwrap()
        })
    });

    g.bench_function("read_coils_fc01", |b| {
        b.iter(|| {
            PduBuilder::build_read_request(black_box(0x01), black_box(0), black_box(2000)).unwrap()
        })
    });

    g.bench_function("write_single_register_fc06", |b| {
        b.iter(|| {
            PduBuilder::build_write_single_register(black_box(100), black_box(0x1234)).unwrap()
        })
    });

    g.finish();
}

fn bench_byte_order_decode(c: &mut Criterion) {
    let mut g = c.benchmark_group("byte_order_decode");
    let regs2 = [0x4048u16, 0xF5C3];
    let regs4 = [0x4009u16, 0x21FB, 0x5444, 0x2D18];

    for order in [
        ByteOrder::BigEndian,
        ByteOrder::LittleEndian,
        ByteOrder::BigEndianSwap,
        ByteOrder::LittleEndianSwap,
    ] {
        g.bench_with_input(BenchmarkId::new("f32", format!("{order:?}")), &order, |b, &o| {
            b.iter(|| regs_to_f32(black_box(&regs2), o))
        });
        g.bench_with_input(BenchmarkId::new("u32", format!("{order:?}")), &order, |b, &o| {
            b.iter(|| regs_to_u32(black_box(&regs2), o))
        });
        g.bench_with_input(BenchmarkId::new("f64", format!("{order:?}")), &order, |b, &o| {
            b.iter(|| regs_to_f64(black_box(&regs4), o))
        });
    }
    g.finish();
}

fn bench_coalescer(c: &mut Criterion) {
    let mut g = c.benchmark_group("coalescer");
    let coalescer = ReadCoalescer::new();

    // Scenario 1: Adjacent ranges that should merge into one read.
    let adjacent: Vec<ReadRequest> = (0..16)
        .map(|i| ReadRequest::new(1, 0x03, i * 10, 10))
        .collect();

    // Scenario 2: Sparse requests that can't merge (spacing > default gap).
    let sparse: Vec<ReadRequest> = (0..16)
        .map(|i| ReadRequest::new(1, 0x03, i * 500, 10))
        .collect();

    for (name, input) in [("adjacent_16", &adjacent), ("sparse_16", &sparse)] {
        g.throughput(Throughput::Elements(input.len() as u64));
        g.bench_with_input(BenchmarkId::from_parameter(name), input, |b, reqs| {
            b.iter(|| coalescer.coalesce(black_box(reqs)))
        });
    }
    g.finish();
}

criterion_group!(benches, bench_pdu_builder, bench_byte_order_decode, bench_coalescer);
criterion_main!(benches);
