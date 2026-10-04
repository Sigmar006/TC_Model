/**
 * @file SarScanSarFocusing.h
 * @brief 扫描模式(ScanSAR)逐 burst Omega-K 聚焦编排器。
 *
 * 数据流(契约 scansar_mode.md §4):raw history → 按 burst_pulse_ranges 切分为各 burst
 * 子矩阵 → 每 burst 独立调 FocusStripmapOmegaK(broadside,offset=0,等价条带聚焦)→
 * 同子带内 burst 方位拼接(幅度域对齐)→ 跨子带距离拼接 → 宽测绘带图像。
 *
 * 关键不变量:单 burst 聚焦 = 条带 broadside 聚焦(ScanSAR burst 内零 squint),故复用
 * FocusStripmapOmegaK,聚焦引擎零改动。工作量全在 burst 切分 + 拼接(编排层)。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_SCANSAR_FOCUSING_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_SCANSAR_FOCUSING_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "sar/geometry/SarScanBurst.h"
#include "sar/imaging/SarOmegaKFocusing.h"
#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

/**
 * @brief burst 脉冲区间(半开 [start_pulse, end_pulse))。
 *
 * 标识 raw history 中属于该 burst 的连续脉冲行。end_pulse 为开区间上界(= 末脉冲 + 1)。
 */
struct ScanBurstPulseRange {
  std::size_t start_pulse{0U};
  std::size_t end_pulse{0U};  // 开区间上界
};

/**
 * @brief 单子带聚焦配置。
 *
 * 每个子带有自己的 Omega-K 物理参数(reference_range_m 取子带中心斜距)和 burst 列表。
 */
struct ScanSarSubswathConfig {
  OmegaKConfig omega_k{};                          /**< 该子带 Omega-K 参数 */
  std::vector<ScanBurstPulseRange> burst_ranges{};  /**< 该子带各 burst 的脉冲区间 */
};

/**
 * @brief ScanSAR 聚焦编排配置。
 */
struct ScanSarFocusConfig {
  std::vector<ScanSarSubswathConfig> subswaths{};  /**< N_swath 个子带配置 */
};

/**
 * @brief 单子带聚焦结果(各 burst 拼接后的连续子图像)。
 */
struct FocusedScanSarSubswath {
  signal::ComplexMatrix image{};                   /**< burst 拼接后子图像 */
  std::vector<OmegaKDiagnostics> burst_diagnostics{};  /**< 各 burst 诊断 */
  std::vector<std::size_t> burst_azimuth_offsets{};    /**< 各 burst 在子图像中的方位行偏移 */
  bool valid{false};
};

/**
 * @brief ScanSAR 聚焦完整结果。
 */
struct FocusedScanSarImage {
  std::vector<FocusedScanSarSubswath> subswaths{}; /**< N_swath 个子带，每个子带是各 burst 拼接后的连续子图像 */
  std::string failure_stage = "none";
};

/**
 * @brief ScanSAR 逐 burst Omega-K 聚焦编排器。
 *
 * 流程:
 * 1. 对每个子带的每个 burst,从 raw_history 抽出 [start, end) 行的子矩阵。
 * 2. 调 FocusStripmapOmegaK(broadside,offset=0)对该 burst 子矩阵聚焦——与条带等价。
 * 3. 同子带内 burst 按脉冲区间方位顺序拼接(burst 子图像纵向堆叠,重叠区幅度平均)。
 *
 * 不变量(契约 §4.2):
 * - 单子带退化:N_swath=1 且单 burst 覆盖全程 → 输出 == FocusStripmapOmegaK。
 * - 单 burst = 条带子集:任一 burst 独立聚焦与对该脉冲区间跑条带聚焦逐样本一致。
 * - burst 内零 squint(Stolt 成立)。
 * - 确定性 + 无 NaN/Inf。
 *
 * @param config 子带 + burst 区间配置。
 * @param raw_pulse_history 完整 raw history(rows = azimuth_pulse_count,cols = range_sample_count)。
 * @param output 聚焦结果。
 * @return 全部 burst 聚焦 + 拼接成功则 true;任一 burst 失败或参数非法则 false。
 */
bool FocusScanSarOmegaK(const ScanSarFocusConfig& config,
                        const signal::ComplexMatrix& raw_pulse_history,
                        FocusedScanSarImage* output);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_SCANSAR_FOCUSING_H_
