/**
 * @file MtiMtdAcceptanceBank.h
 * @brief 验收旁路 MTI/MTD 频谱效能核（不进检测 SINR）。
 *
 * 常量：N=8、2 脉冲 MTI、杂波谱宽 σ_v=0.25 m/s。不算四增益偏置。
 */

#ifndef COMMON_RADAR_MTI_MTD_ACCEPTANCE_BANK_H_
#define COMMON_RADAR_MTI_MTD_ACCEPTANCE_BANK_H_

#include <array>
#include <cstddef>

namespace oneq {
namespace common {
namespace radar {

constexpr std::size_t kMtiMtdChannelCount = 8U;
constexpr double kMtiMtdClutterSigmaVelocityMps = 0.25;
constexpr double kMtiNoiseFactor = 2.0;

/** @brief 验收旁路干扰单音（链路多普勒 + 到达功率）。 */
struct MtiMtdInterferenceTone {
  double doppler_hz{0.0};
  double power_w{0.0};
};

/** @brief 验收旁路 MTI/MTD 输入（物理瓦数，未加偏置）。 */
struct MtiMtdAcceptanceInput {
  double echo_power_w{0.0};
  double thermal_noise_power_w{0.0};
  double clutter_power_w{0.0};
  double two_way_doppler_shift_hz{0.0};
  double prf_hz{0.0};
  double center_frequency_hz{0.0};
  const MtiMtdInterferenceTone* tones{nullptr};
  std::size_t tone_count{0U};
};

/** @brief 验收旁路 MTI/MTD 输出。 */
struct MtiMtdAcceptanceResult {
  double mti_gain_db{0.0};
  double mtd_gain_db{0.0};
  std::size_t selected_channel{0U};
  std::array<double, kMtiMtdChannelCount> target_w{};
  std::array<double, kMtiMtdChannelCount> noise_w{};
  std::array<double, kMtiMtdChannelCount> clutter_w{};
  std::array<double, kMtiMtdChannelCount> jam_w{};
  double mti_residual_clutter_w{0.0};
  double mti_residual_jam_w{0.0};
  double mtd_equivalent_noise_w{0.0};
  bool has_jam_channels{false};
};

/**
 * @brief 按冻结公式求验收旁路 MTI/MTD。
 * @return 成功返回 true；非法输入原子拒绝且不修改 @p result。
 */
bool TryResolveMtiMtdAcceptanceBank(const MtiMtdAcceptanceInput& input,
                                    MtiMtdAcceptanceResult* result);

/**
 * @brief 验收旁路功率比（dB）= 10 log10(input / residual)。
 * @return 两边均有限且为正时写 @p ratio_db 并返回 true；否则不改输出。
 */
bool TryAcceptancePowerRatioDb(double input_w, double residual_w, double* ratio_db);

}  // namespace radar
}  // namespace common
}  // namespace oneq

#endif
