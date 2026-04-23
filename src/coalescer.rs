//! Read Coalescing — 智能读合并模块
//!
//! 自动将相邻或重叠的寄存器读请求合并为更少的网络请求，减少 RTT 延迟。
//!
//! # 工作原理
//!
//! 在工业 SCADA/DCS 场景中，用户经常需要读取分布在相邻寄存器的多个数据点。
//! 例如：
//! - 温度在寄存器 0-1（f32, 2 registers）
//! - 压力在寄存器 2-3（f32, 2 registers）
//! - 流量在寄存器 10-11（f32, 2 registers）
//!
//! 每个数据点独立调用 `read_03` 将产生 3 次网络请求。通过读合并，
//! 当间隙 ≤ `gap_threshold`（默认 10）时，只需 1 次请求。
//!
//! # Example
//!
//! ```rust
//! use voltage_modbus::coalescer::{ReadCoalescer, ReadRequest};
//!
//! let coalescer = ReadCoalescer::new();
//! let requests = vec![
//!     ReadRequest { slave_id: 1, function: 0x03, address: 0, quantity: 2 },
//!     ReadRequest { slave_id: 1, function: 0x03, address: 2, quantity: 2 },
//!     ReadRequest { slave_id: 1, function: 0x03, address: 10, quantity: 2 },
//! ];
//!
//! let coalesced = coalescer.coalesce(&requests);
//! // 三个请求合并为一个: address=0, quantity=12
//! assert_eq!(coalesced.len(), 1);
//! ```

use crate::constants::MAX_READ_REGISTERS;

/// 默认合并间隙阈值（寄存器数）
///
/// 当两个读请求之间的间隙小于此值时，将它们合并为一次请求，
/// 以牺牲少量额外传输数据换取减少 RTT 开销。
pub const DEFAULT_GAP_THRESHOLD: u16 = 10;

/// 读请求描述
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReadRequest {
    /// 从站 ID
    pub slave_id: u8,
    /// 功能码（0x03 或 0x04）
    pub function: u8,
    /// 起始地址
    pub address: u16,
    /// 寄存器数量
    pub quantity: u16,
}

impl ReadRequest {
    /// 创建新的读请求
    pub fn new(slave_id: u8, function: u8, address: u16, quantity: u16) -> Self {
        Self {
            slave_id,
            function,
            address,
            quantity,
        }
    }

    /// 返回此请求覆盖的末尾地址（不含）
    #[inline]
    fn end_address(&self) -> u32 {
        u32::from(self.address) + u32::from(self.quantity)
    }
}

/// 合并后的读请求组
///
/// 表示多个原始 `ReadRequest` 被合并为一次 Modbus 读操作的结果。
/// `mappings` 记录了每个原始请求在合并后数据中的位置。
#[derive(Debug)]
pub struct CoalescedRead {
    /// 从站 ID
    pub slave_id: u8,
    /// 功能码
    pub function: u8,
    /// 合并后的起始地址
    pub address: u16,
    /// 合并后的寄存器总数
    pub quantity: u16,
    /// 原始请求在合并后数据中的偏移映射
    ///
    /// 每个元素：`(original_index, offset_in_merged_data, original_quantity)`
    /// - `original_index`: 原始请求在输入列表中的索引
    /// - `offset_in_merged_data`: 该请求数据在合并响应中的偏移（寄存器数）
    /// - `original_quantity`: 该请求的原始寄存器数量
    pub mappings: Vec<(usize, u16, u16)>,
}

/// 读合并器
///
/// 将多个寄存器读请求按照配置的间隙阈值合并为更少的网络请求。
///
/// # 合并规则
///
/// 1. 只合并相同 `slave_id` 和 `function` 的请求
/// 2. 按起始地址排序
/// 3. 如果两个请求之间的间隙 ≤ `gap_threshold`，合并为一个
/// 4. 合并后的请求不超过 `max_registers`（默认 125）
pub struct ReadCoalescer {
    /// 合并间隙阈值（寄存器数）
    gap_threshold: u16,
    /// 单次读取的最大寄存器数
    max_registers: u16,
}

impl Default for ReadCoalescer {
    fn default() -> Self {
        Self::new()
    }
}

impl ReadCoalescer {
    /// 创建默认配置的合并器（gap_threshold=10, max_registers=125）
    pub fn new() -> Self {
        Self {
            gap_threshold: DEFAULT_GAP_THRESHOLD,
            max_registers: MAX_READ_REGISTERS as u16,
        }
    }

    /// 创建自定义间隙阈值的合并器
    pub fn with_gap_threshold(gap_threshold: u16) -> Self {
        Self {
            gap_threshold,
            max_registers: MAX_READ_REGISTERS as u16,
        }
    }

    /// 创建完整自定义配置的合并器
    pub fn with_config(gap_threshold: u16, max_registers: u16) -> Self {
        Self {
            gap_threshold,
            max_registers,
        }
    }

    /// 将多个读请求合并为更少的请求
    ///
    /// # Arguments
    ///
    /// * `requests` - 待合并的读请求列表
    ///
    /// # Returns
    ///
    /// 合并后的请求列表，每个元素包含合并请求和映射关系。
    /// 返回顺序稳定，每个原始请求只出现在一个 `CoalescedRead` 中。
    pub fn coalesce(&self, requests: &[ReadRequest]) -> Vec<CoalescedRead> {
        if requests.is_empty() {
            return Vec::new();
        }

        // 按 (slave_id, function, address) 排序，保留原始索引
        let mut indexed: Vec<(usize, &ReadRequest)> = requests.iter().enumerate().collect();
        indexed.sort_by_key(|(_, r)| (r.slave_id, r.function, r.address));

        let mut result: Vec<CoalescedRead> = Vec::new();

        // 当前合并组的状态
        let mut group_slave = indexed[0].1.slave_id;
        let mut group_fn = indexed[0].1.function;
        let mut group_start = indexed[0].1.address;
        let mut group_end = indexed[0].1.end_address(); // exclusive
        let mut group_mappings: Vec<(usize, u16, u16)> = Vec::new();

        // 将第一个元素加入当前组
        {
            let (orig_idx, req) = indexed[0];
            let offset = req.address - group_start;
            group_mappings.push((orig_idx, offset, req.quantity));
        }

        for &(orig_idx, req) in &indexed[1..] {
            let same_group = req.slave_id == group_slave && req.function == group_fn;

            if same_group {
                // 计算合并后的总范围
                let new_end = req.end_address().max(group_end);
                let merged_qty = new_end - u32::from(group_start);

                if merged_qty <= u32::from(self.max_registers) {
                    // 检查间隙：req.address 到 group_end 的距离
                    let gap = u32::from(req.address).saturating_sub(group_end);
                    if gap <= u32::from(self.gap_threshold) || u32::from(req.address) <= group_end {
                        // 合并到当前组
                        group_end = new_end;
                        let offset = (u32::from(req.address) - u32::from(group_start)) as u16;
                        group_mappings.push((orig_idx, offset, req.quantity));
                        continue;
                    }
                }
            }

            // 提交当前组，开始新组
            result.push(CoalescedRead {
                slave_id: group_slave,
                function: group_fn,
                address: group_start,
                quantity: Self::window_quantity(group_start, group_end),
                mappings: std::mem::take(&mut group_mappings),
            });

            group_slave = req.slave_id;
            group_fn = req.function;
            group_start = req.address;
            group_end = req.end_address();
            group_mappings.push((orig_idx, 0, req.quantity));
        }

        // 提交最后一组
        result.push(CoalescedRead {
            slave_id: group_slave,
            function: group_fn,
            address: group_start,
            quantity: Self::window_quantity(group_start, group_end),
            mappings: group_mappings,
        });

        result
    }

    #[inline]
    fn window_quantity(start: u16, end: u32) -> u16 {
        let quantity = end - u32::from(start);
        debug_assert!(quantity <= u32::from(u16::MAX));
        quantity as u16
    }

    /// 从合并后的响应数据中提取原始请求对应的数据
    ///
    /// # Arguments
    ///
    /// * `coalesced` - 合并后的读请求（包含映射关系）
    /// * `data` - 合并请求返回的寄存器数据
    ///
    /// # Returns
    ///
    /// 按 `mappings` 中 `original_index` 的顺序返回各原始请求的数据切片。
    /// 结果顺序与 `mappings` 中的顺序一致（即排序后的顺序）。
    pub fn extract_results(&self, coalesced: &CoalescedRead, data: &[u16]) -> Vec<Vec<u16>> {
        coalesced
            .mappings
            .iter()
            .map(|&(_, offset, qty)| {
                let start = offset as usize;
                let end = (offset + qty) as usize;
                if end <= data.len() {
                    data[start..end].to_vec()
                } else {
                    Vec::new()
                }
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn req(slave_id: u8, function: u8, address: u16, quantity: u16) -> ReadRequest {
        ReadRequest::new(slave_id, function, address, quantity)
    }

    // -------------------------------------------------------------------------
    // test_empty_requests — 空输入返回空结果
    // -------------------------------------------------------------------------
    #[test]
    fn test_empty_requests() {
        let coalescer = ReadCoalescer::new();
        let result = coalescer.coalesce(&[]);
        assert!(result.is_empty());
    }

    // -------------------------------------------------------------------------
    // test_single_request — 单个请求不变
    // -------------------------------------------------------------------------
    #[test]
    fn test_single_request() {
        let coalescer = ReadCoalescer::new();
        let requests = vec![req(1, 0x03, 10, 5)];
        let result = coalescer.coalesce(&requests);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].address, 10);
        assert_eq!(result[0].quantity, 5);
        assert_eq!(result[0].mappings.len(), 1);
        assert_eq!(result[0].mappings[0], (0, 0, 5));
    }

    #[test]
    fn test_single_request_at_u16_max_address_keeps_quantity() {
        let coalescer = ReadCoalescer::new();
        let requests = vec![req(1, 0x03, u16::MAX, 1)];
        let result = coalescer.coalesce(&requests);

        assert_eq!(result.len(), 1);
        assert_eq!(result[0].address, u16::MAX);
        assert_eq!(result[0].quantity, 1);
        assert_eq!(result[0].mappings, vec![(0, 0, 1)]);
    }

    // -------------------------------------------------------------------------
    // test_adjacent_merge — 相邻请求合并
    // -------------------------------------------------------------------------
    #[test]
    fn test_adjacent_merge() {
        let coalescer = ReadCoalescer::new();
        let requests = vec![
            req(1, 0x03, 0, 2), // 寄存器 0-1
            req(1, 0x03, 2, 2), // 寄存器 2-3，紧邻前一个
        ];
        let result = coalescer.coalesce(&requests);
        assert_eq!(result.len(), 1, "相邻请求应合并为一个");
        assert_eq!(result[0].address, 0);
        assert_eq!(result[0].quantity, 4);
        // 验证映射
        assert_eq!(result[0].mappings.len(), 2);
        // 找到原始索引 0 的映射：offset=0, qty=2
        let m0 = result[0].mappings.iter().find(|m| m.0 == 0).unwrap();
        assert_eq!(m0.1, 0);
        assert_eq!(m0.2, 2);
        // 找到原始索引 1 的映射：offset=2, qty=2
        let m1 = result[0].mappings.iter().find(|m| m.0 == 1).unwrap();
        assert_eq!(m1.1, 2);
        assert_eq!(m1.2, 2);
    }

    // -------------------------------------------------------------------------
    // test_overlapping_merge — 重叠请求合并
    // -------------------------------------------------------------------------
    #[test]
    fn test_overlapping_merge() {
        let coalescer = ReadCoalescer::new();
        let requests = vec![
            req(1, 0x03, 0, 5), // 寄存器 0-4
            req(1, 0x03, 3, 5), // 寄存器 3-7，与前一个重叠
        ];
        let result = coalescer.coalesce(&requests);
        assert_eq!(result.len(), 1, "重叠请求应合并为一个");
        assert_eq!(result[0].address, 0);
        assert_eq!(result[0].quantity, 8); // 0..8
    }

    // -------------------------------------------------------------------------
    // test_gap_merge — 间隙内请求合并（间隙 ≤ threshold）
    // -------------------------------------------------------------------------
    #[test]
    fn test_gap_merge() {
        // gap_threshold = 10
        let coalescer = ReadCoalescer::new();
        let requests = vec![
            req(1, 0x03, 0, 2),  // 寄存器 0-1，end=2
            req(1, 0x03, 2, 2),  // 寄存器 2-3，end=4
            req(1, 0x03, 10, 2), // 寄存器 10-11，gap=6 ≤ 10，应合并
        ];
        let result = coalescer.coalesce(&requests);
        assert_eq!(result.len(), 1, "间隙 ≤ threshold，应合并为一个");
        assert_eq!(result[0].address, 0);
        assert_eq!(result[0].quantity, 12);
        // 映射验证：原始请求 2 在 offset=10，qty=2
        let m2 = result[0].mappings.iter().find(|m| m.0 == 2).unwrap();
        assert_eq!(m2.1, 10);
        assert_eq!(m2.2, 2);
    }

    // -------------------------------------------------------------------------
    // test_no_merge — 间隙超过阈值，不合并
    // -------------------------------------------------------------------------
    #[test]
    fn test_no_merge() {
        // gap_threshold = 5
        let coalescer = ReadCoalescer::with_gap_threshold(5);
        let requests = vec![
            req(1, 0x03, 0, 2),  // end=2
            req(1, 0x03, 10, 2), // gap=8 > 5，不应合并
        ];
        let result = coalescer.coalesce(&requests);
        assert_eq!(result.len(), 2, "间隙 > threshold，不应合并");
        assert_eq!(result[0].address, 0);
        assert_eq!(result[0].quantity, 2);
        assert_eq!(result[1].address, 10);
        assert_eq!(result[1].quantity, 2);
    }

    // -------------------------------------------------------------------------
    // test_max_registers_split — 超过 max_registers 时分拆
    // -------------------------------------------------------------------------
    #[test]
    fn test_max_registers_split() {
        // max_registers = 10，gap_threshold = 100（确保不因间隙分拆）
        let coalescer = ReadCoalescer::with_config(100, 10);
        let requests = vec![
            req(1, 0x03, 0, 6), // 寄存器 0-5
            req(1, 0x03, 6, 6), // 寄存器 6-11，合并后 qty=12 > 10
        ];
        let result = coalescer.coalesce(&requests);
        assert_eq!(result.len(), 2, "合并后超过 max_registers，应分拆");
        assert_eq!(result[0].address, 0);
        assert_eq!(result[0].quantity, 6);
        assert_eq!(result[1].address, 6);
        assert_eq!(result[1].quantity, 6);
    }

    // -------------------------------------------------------------------------
    // test_different_slaves_no_merge — 不同 slave_id 不合并
    // -------------------------------------------------------------------------
    #[test]
    fn test_different_slaves_no_merge() {
        let coalescer = ReadCoalescer::new();
        let requests = vec![
            req(1, 0x03, 0, 2), // slave 1
            req(2, 0x03, 2, 2), // slave 2，不应合并
        ];
        let result = coalescer.coalesce(&requests);
        assert_eq!(result.len(), 2, "不同 slave_id 不应合并");
        // 按排序顺序：slave 1 先，slave 2 后
        let s1 = result.iter().find(|r| r.slave_id == 1).unwrap();
        let s2 = result.iter().find(|r| r.slave_id == 2).unwrap();
        assert_eq!(s1.quantity, 2);
        assert_eq!(s2.quantity, 2);
    }

    // -------------------------------------------------------------------------
    // test_different_functions_no_merge — 不同 function code 不合并
    // -------------------------------------------------------------------------
    #[test]
    fn test_different_functions_no_merge() {
        let coalescer = ReadCoalescer::new();
        let requests = vec![
            req(1, 0x03, 0, 2), // FC03 Holding Registers
            req(1, 0x04, 2, 2), // FC04 Input Registers，不应合并
        ];
        let result = coalescer.coalesce(&requests);
        assert_eq!(result.len(), 2, "不同 function code 不应合并");
        let fc03 = result.iter().find(|r| r.function == 0x03).unwrap();
        let fc04 = result.iter().find(|r| r.function == 0x04).unwrap();
        assert_eq!(fc03.quantity, 2);
        assert_eq!(fc04.quantity, 2);
    }

    // -------------------------------------------------------------------------
    // test_extract_results — 从合并数据中正确提取原始区域
    // -------------------------------------------------------------------------
    #[test]
    fn test_extract_results() {
        let coalescer = ReadCoalescer::new();
        // 模拟合并后的数据：12 个寄存器 (0..12)
        let merged_data: Vec<u16> = (0..12).collect();

        let coalesced = CoalescedRead {
            slave_id: 1,
            function: 0x03,
            address: 0,
            quantity: 12,
            mappings: vec![
                (0, 0, 2),  // 原始请求 0: offset=0, qty=2
                (1, 2, 2),  // 原始请求 1: offset=2, qty=2
                (2, 10, 2), // 原始请求 2: offset=10, qty=2
            ],
        };

        let extracted = coalescer.extract_results(&coalesced, &merged_data);
        assert_eq!(extracted.len(), 3);
        assert_eq!(extracted[0], vec![0, 1]); // merged_data[0..2]
        assert_eq!(extracted[1], vec![2, 3]); // merged_data[2..4]
        assert_eq!(extracted[2], vec![10, 11]); // merged_data[10..12]
    }

    // -------------------------------------------------------------------------
    // test_coalesce_full_example — 文档中的完整示例
    // -------------------------------------------------------------------------
    #[test]
    fn test_coalesce_full_example() {
        // 文档中的示例：温度(0-1)、压力(2-3)、流量(10-11)
        // gap_threshold=10，三个请求应合并为一个
        let coalescer = ReadCoalescer::new();
        let requests = vec![req(1, 0x03, 0, 2), req(1, 0x03, 2, 2), req(1, 0x03, 10, 2)];
        let coalesced = coalescer.coalesce(&requests);
        assert_eq!(coalesced.len(), 1);
        assert_eq!(coalesced[0].address, 0);
        assert_eq!(coalesced[0].quantity, 12);
        assert_eq!(coalesced[0].mappings.len(), 3);

        // 模拟响应数据，提取各区域
        let data: Vec<u16> = (100..112).collect();
        let results = coalescer.extract_results(&coalesced[0], &data);
        assert_eq!(results.len(), 3);
        assert_eq!(results[0], vec![100, 101]); // 温度
        assert_eq!(results[1], vec![102, 103]); // 压力
        assert_eq!(results[2], vec![110, 111]); // 流量
    }

    // -------------------------------------------------------------------------
    // test_extract_results_out_of_bounds — 数据不足时返回空
    // -------------------------------------------------------------------------
    #[test]
    fn test_extract_results_out_of_bounds() {
        let coalescer = ReadCoalescer::new();
        let short_data: Vec<u16> = vec![1, 2, 3]; // 只有 3 个寄存器

        let coalesced = CoalescedRead {
            slave_id: 1,
            function: 0x03,
            address: 0,
            quantity: 10,
            mappings: vec![
                (0, 0, 2), // 正常范围
                (1, 8, 2), // 超出数据范围
            ],
        };

        let extracted = coalescer.extract_results(&coalesced, &short_data);
        assert_eq!(extracted[0], vec![1, 2]); // 正常提取
        assert!(extracted[1].is_empty()); // 超出范围返回空
    }
}
