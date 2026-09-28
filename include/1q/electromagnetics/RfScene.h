/**
 * @file RfScene.h
 * @brief 定义工程 RF v2 的世界场景、参数化波形与单程入射链路公共 API。
 * @note 本文件保持 C++11 可用，不生成复数 IQ，也不包含任何传感器探测或受扰判决。
 */

#ifndef ONEQ_ELECTROMAGNETICS_RF_SCENE_H_
#define ONEQ_ELECTROMAGNETICS_RF_SCENE_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"

namespace oneq {
namespace electromagnetics {

/**
 * @brief RF v2 支持的参数化波形类别。
 * @note 类别只决定统计级活动判定（某时刻是否发、瞬时频率是多少），不生成真实信号采样点。
 *       四类共用 RfWaveformSchedule，仅与所选类别配套的字段有意义，其余为零或被忽略。
 */
enum class RfSceneWaveformKind : std::uint8_t {
  kContinuous = 0,     /**< 连续载波：活动期间常开，频率固定在中心频率。 */
  kPulseTrain = 1,     /**< 脉冲列：按 PRI 周期性发短脉冲，仅在脉宽内判定有信号；含确定性 jitter。 */
  kLinearSweep = 2,    /**< 线性扫频：中心频率在 sweep 周期内由起点匀速滑到终点，瞬时频率随时刻变化。 */
  kBandLimitedNoise = 3, /**< 带限噪声：占用带宽内的噪声功率；活动判定同连续载波，频率固定在中心频率。 */
};

/**
 * @brief RF v2 使用的名义极化或极化工作模式类别。
 * @note 极化描述电场摆动方向；收发两侧配对使用（emission 与 receiver 各填一个），
 *       配对结果决定 polarization_mismatch_loss_db：同值 0 dB；同为线极化互垂或同为
 *       圆极化旋向相反取 cross_polarization_isolation_db；线圆互配或任一侧非极化固定 3.01 dB；
 *       全极化为两条相互垂直的极化通道同时工作——全极化接收对任意固定极化合并收满 0 dB，
 *       单极化接收全极化发射只收一半 3.01 dB，全极化对全极化 0 dB，对非极化仍 3.01 dB。
 */
enum class RfScenePolarization : std::uint8_t {
  kHorizontal = 0,        /**< 水平线极化：电场沿水平方向直线摆动。 */
  kVertical = 1,          /**< 垂直线极化：电场沿垂直方向直线摆动，与水平互垂。 */
  kRightHandCircular = 2, /**< 右旋圆极化：电场端点按右手方向画圆旋转。 */
  kLeftHandCircular = 3,  /**< 左旋圆极化：电场端点按左手方向画圆旋转，与右旋相反。 */
  kUnpolarized = 4,       /**< 非极化：摆动方向不固定，与任一侧配对均取 3.01 dB 保守损耗。 */
  kFullPolarization = 5,  /**< 全极化：两条相互垂直的极化通道同时工作。 */
};

/** @brief ECEF 坐标系中的方向向量；求解时会归一化。 */
struct ONEQ_API RfSceneDirection {
  double x{1.0};
  double y{0.0};
  double z{0.0};
};

/** @brief 轴对称远场天线方向图的统计级近似。 */
struct ONEQ_API RfSceneAntennaPattern {
  RfSceneDirection boresight_ecef{};
  double peak_gain_dbi{0.0};
  double half_power_beamwidth_deg{60.0};
  double sidelobe_level_db{-20.0};
  double backlobe_level_db{-40.0};
  double cross_polarization_isolation_db{30.0};
};

/** @brief 发射事实的 sensor_entity_id 与 emission 唯一身份。 */
struct ONEQ_API RfEmissionIdentity {
  std::uint32_t sensor_entity_id{0U};
  std::uint64_t emission_id{0U};
};

/**
 * @brief 身份二元组同一性判定（`sensor_entity_id` + `emission_id`）。
 * @note 本判定是"自身链剔除"的唯一谓词单源：检测单元解析、干扰源写出与控制器
 *       接受侧过滤必须共用它，避免同一规则在三处各写一遍而口径漂移。
 */
inline bool operator==(const RfEmissionIdentity& left, const RfEmissionIdentity& right) {
  return left.sensor_entity_id == right.sensor_entity_id &&
         left.emission_id == right.emission_id;
}

/** @brief 身份二元组不同一（见 `operator==`）。 */
inline bool operator!=(const RfEmissionIdentity& left, const RfEmissionIdentity& right) {
  return !(left == right);
}

/**
 * @brief 一个绝对 world-time 轴上的参数化 RF 活动。
 * @note 调用方应通过 `TryCreateRf*Waveform` 构造；默认值是无效占位值。
 */
struct ONEQ_API RfWaveformSchedule {
  RfSceneWaveformKind kind{RfSceneWaveformKind::kContinuous};
  double activity_start_time_s{0.0};
  double activity_duration_s{0.0};
  double center_frequency_hz{0.0};
  double occupied_bandwidth_hz{0.0};
  double transmit_power_w{0.0};
  double pulse_width_s{0.0};
  double pulse_repetition_interval_s{0.0};
  double first_pulse_time_s{0.0};
  std::uint32_t pulse_count{0};
  double pulse_jitter_fraction{0.0};
  std::uint64_t timing_seed{0};
  std::uint64_t timing_epoch{0};
  double sweep_start_frequency_hz{0.0};
  double sweep_stop_frequency_hz{0.0};
  double sweep_period_s{0.0};
};

/** @brief RF v2 中的一条实际发射事实。 */
struct ONEQ_API RfSceneEmission {
  RfEmissionIdentity identity{};
  coordinate::EcefPositionM position_ecef_m{};
  coordinate::EcefVelocityMps velocity_ecef_mps{};
  RfSceneAntennaPattern antenna{};
  RfScenePolarization polarization{RfScenePolarization::kUnpolarized};
  RfWaveformSchedule waveform{};
};

/** @brief 一个接收世界窗口内由 orchestrator 冻结的 RF 发射场景。 */
struct ONEQ_API RfSceneFrame {
  std::uint64_t world_cycle_index{0};
  double window_start_time_s{0.0};
  double window_duration_s{0.0};
  std::vector<RfSceneEmission> emissions{};
};

/**
 * @brief 模块间直接交换的 RF 发射事实帧。
 * @note 该名称是普通调用方合同；`RfSceneFrame` 保留为 RF 求解器内部语义别名。
 */
using RfEmissionFrame = RfSceneFrame;

/** @brief 接收设备针对指定发射源的硬件隔离路径。 */
struct ONEQ_API RfCoSiteIsolationPath {
  std::uint32_t transmitter_sensor_entity_id{0U};
  double isolation_db{0.0};
  RfCoSiteIsolationPath() = default;
  RfCoSiteIsolationPath(std::uint32_t transmitter_id, double isolation)
      : transmitter_sensor_entity_id(transmitter_id),
        isolation_db(isolation) {}
};

/** @brief 接收设备在一个世界窗口内冻结的硬件与调谐状态。 */
struct ONEQ_API RfSceneReceiverState {
  std::uint32_t sensor_entity_id{0U};
  coordinate::EcefPositionM position_ecef_m{};
  coordinate::EcefVelocityMps velocity_ecef_mps{};
  RfSceneAntennaPattern antenna{};
  RfScenePolarization polarization{RfScenePolarization::kUnpolarized};
  double window_start_time_s{0.0};
  double window_duration_s{0.0};
  double center_frequency_hz{0.0};
  double bandwidth_hz{0.0};
  double receiver_system_loss_db{0.0};
  double minimum_far_field_range_m{1.0};
  std::vector<RfCoSiteIsolationPath> co_site_paths{};
};

/** @brief 调用方提供的单程传播附加损耗。 */
struct ONEQ_API RfIncidentLinkConfig {
  double additional_propagation_loss_db{0.0};
};

/** @brief 一个 emission 到接收设备输入端的统计级入射链路结果。 */
struct ONEQ_API RfIncidentLinkResult {
  RfEmissionIdentity identity{};
  RfWaveformSchedule emission_waveform{}; /**< 发射端参数化波形。 */
  std::uint32_t receiver_sensor_entity_id{0U};
  bool is_co_site{false};
  double path_length_m{0.0};
  double propagation_delay_s{0.0};
  double doppler_shift_hz{0.0};
  double transmit_antenna_gain_dbi{0.0};
  double receive_antenna_gain_dbi{0.0};
  double free_space_loss_db{0.0};
  double additional_propagation_loss_db{0.0};
  double polarization_mismatch_loss_db{0.0};
  double receiver_system_loss_db{0.0};
  double co_site_isolation_db{0.0};
  double arrival_start_time_s{0.0};
  double arrival_end_time_s{0.0};
  double time_overlap_fraction{0.0};
  double frequency_overlap_fraction{0.0};
  double received_power_before_overlap_w{0.0};
  double received_power_w{0.0};
  double received_power_spectral_density_w_per_hz{0.0};
};

/**
 * @brief 求参数化波形在接收端某一绝对时刻的活动状态和到达中心频率。
 * @param[in] waveform 发射端参数化波形。
 * @param[in] propagation_delay_s 单程传播时延。
 * @param[in] doppler_shift_hz 接收约定的单程 Doppler。
 * @param[in] arrival_time_s 接收端绝对 world time。
 * @param[out] active 该时刻是否存在到达 RF 活动。
 * @param[out] arrival_center_frequency_hz 活动时的到达中心频率；不活动时为 0。
 * @return 输入合法时返回 true；失败时不修改输出。
 */
ONEQ_API bool TryEvaluateRfArrivalActivity(const RfWaveformSchedule& waveform,
                                           double propagation_delay_s,
                                           double doppler_shift_hz, double arrival_time_s,
                                           bool* active,
                                           double* arrival_center_frequency_hz);

/**
 * @brief 解析 pulse-train 中指定脉冲的实际绝对发射起点。
 * @param[in] waveform 由 TryCreateRfPulseTrainWaveform 构造的波形。
 * @param[in] pulse_index 从 0 开始的脉冲索引。
 * @param[out] pulse_start_time_s 实际脉冲起点，包含确定性 jitter。
 * @return 输入合法且索引存在时返回 true；失败时不修改输出。
 */
ONEQ_API bool TryResolveRfPulseStartTime(const RfWaveformSchedule& waveform,
                                         std::uint32_t pulse_index,
                                         double* pulse_start_time_s);

/**
 * @brief 构造连续载波参数化波形。
 * @param[in] start_time_s 绝对 world-time 起点。
 * @param[in] duration_s 活动持续时间。
 * @param[in] center_frequency_hz 中心频率。
 * @param[in] bandwidth_hz 占用带宽。
 * @param[in] transmit_power_w 活动期间的总发射功率。
 * @param[out] waveform 成功时原子写入波形。
 * @return 全部输入合法时返回 true。
 */
ONEQ_API bool TryCreateRfContinuousWaveform(double start_time_s, double duration_s,
                                            double center_frequency_hz, double bandwidth_hz,
                                            double transmit_power_w, RfWaveformSchedule* waveform);

/**
 * @brief 构造带限噪声参数化波形。
 * @param[in] start_time_s 绝对 world-time 起点。
 * @param[in] duration_s 活动持续时间。
 * @param[in] center_frequency_hz 中心频率。
 * @param[in] bandwidth_hz 噪声占用带宽。
 * @param[in] transmit_power_w 活动期间的带内总发射功率。
 * @param[out] waveform 成功时原子写入波形。
 * @return 全部输入合法时返回 true。
 */
ONEQ_API bool TryCreateRfNoiseWaveform(double start_time_s, double duration_s,
                                       double center_frequency_hz, double bandwidth_hz,
                                       double transmit_power_w, RfWaveformSchedule* waveform);

/**
 * @brief 构造具有确定性 jitter 的有限脉冲列。
 * @param[in] first_pulse_time_s 首脉冲绝对 world time。
 * @param[in] center_frequency_hz 中心频率。
 * @param[in] bandwidth_hz 占用带宽。
 * @param[in] peak_power_w 脉冲峰值功率。
 * @param[in] pulse_width_s 脉宽。
 * @param[in] pulse_repetition_interval_s 名义 PRI。
 * @param[in] pulse_count 脉冲数量。
 * @param[in] jitter_fraction PRI 的最大相对抖动，范围 `[0, 0.25]`。
 * @param[in] timing_seed 确定性时序种子。
 * @param[in] timing_epoch 确定性时序 epoch。
 * @param[out] waveform 成功时原子写入波形。
 * @return 全部输入合法且脉冲互不重叠时返回 true。
 */
ONEQ_API bool TryCreateRfPulseTrainWaveform(double first_pulse_time_s, double center_frequency_hz,
                                            double bandwidth_hz, double peak_power_w,
                                            double pulse_width_s,
                                            double pulse_repetition_interval_s,
                                            std::uint32_t pulse_count, double jitter_fraction,
                                            std::uint64_t timing_seed, std::uint64_t timing_epoch,
                                            RfWaveformSchedule* waveform);

/**
 * @brief 构造周期重复的线性扫频波形。
 * @param[in] start_time_s 绝对 world-time 起点。
 * @param[in] duration_s 活动持续时间。
 * @param[in] start_frequency_hz 每次 sweep 的起始中心频率。
 * @param[in] stop_frequency_hz 每次 sweep 的终止中心频率。
 * @param[in] instantaneous_bandwidth_hz 瞬时占用带宽。
 * @param[in] transmit_power_w 活动期间的总发射功率。
 * @param[in] sweep_period_s 一次线性 sweep 的周期。
 * @param[out] waveform 成功时原子写入波形。
 * @return 全部输入合法且起止频率不相等时返回 true。
 */
ONEQ_API bool TryCreateRfLinearSweepWaveform(double start_time_s, double duration_s,
                                             double start_frequency_hz, double stop_frequency_hz,
                                             double instantaneous_bandwidth_hz,
                                             double transmit_power_w, double sweep_period_s,
                                             RfWaveformSchedule* waveform);

/**
 * @brief 校验冻结 RF scene 的身份、窗口、方向图与全部波形。
 * @param[in] scene 待校验场景。
 * @return 场景完整合法且 emission ID 唯一时返回 true。
 */
ONEQ_API bool TryValidateRfSceneFrame(const RfSceneFrame& scene);

/**
 * @brief 判定非空 RF emission frame 的 envelope 是否与消费者周期窗口完全一致。
 * @param[in] frame 待校验帧。调用方须先完成空帧豁免策略；本函数不解释空帧语义。
 * @param[in] cycle_index 消费者周期号（uint32_t 输入经隐式提升参与比较）。
 * @param[in] cycle_start_time_s 周期绝对起点（s）。
 * @param[in] window_duration_s 周期时长（s）。
 * @return frame 的 world_cycle_index、window_start_time_s、window_duration_s 与消费者
 *         周期窗口三字段全部精确相等时返回 true。
 * @note 冻结于 docs/common/contract.md §工程 RF 契约 条款 7（非空部分）。空帧 envelope
 *       语义各模块自行裁定，本函数不承担空帧豁免判断。
 */
ONEQ_API bool RfFrameMatchesCycleWindow(const RfSceneFrame& frame, std::uint64_t cycle_index,
                                        double cycle_start_time_s, double window_duration_s);

/**
 * @brief 求解一个 RF v2 emission 到冻结接收状态的单程入射链路。
 * @param[in] emission 发射事实。
 * @param[in] receiver 接收设备状态。
 * @param[in] config 附加传播损耗。
 * @param[out] result 成功时原子写入结果，失败时保持原值。
 * @return 输入合法且近场或 co-site 路径可求解时返回 true。
 * @note Doppler 约定为闭合时正值，发射远离接收机时为负值。
 */
ONEQ_API bool TryEvaluateRfIncidentLink(const RfSceneEmission& emission,
                                        const RfSceneReceiverState& receiver,
                                        const RfIncidentLinkConfig& config,
                                        RfIncidentLinkResult* result);

/**
 * @brief 按 emission 身份稳定排序并聚合接收功率。
 * @param[in] links 同一接收设备的链路结果。
 * @param[out] total_received_power_w 成功时写入线性功率和。
 * @return 接收设备一致、身份唯一且功率合法时返回 true。
 */
ONEQ_API bool TryAggregateRfIncidentPower(const std::vector<RfIncidentLinkResult>& links,
                                          double* total_received_power_w);

}  // namespace electromagnetics
}  // namespace oneq

#endif  // ONEQ_ELECTROMAGNETICS_RF_SCENE_H_
