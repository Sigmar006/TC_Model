/**
 * @file AcceptanceInterfaceRecord.h
 * @brief 验收用「接口日志统一记录」结构体（雷达 + 红外 + 场景输入，内/外部接口）。
 *
 * 背景：验收判据里的接口类条目分散在六个日志文件、十余个条目中（第 57/58/61–68/70–77 项），
 * 每项字段命名与排列各不相同，外部核对时要在多个文件之间来回比对。本文件把**所有接口类
 * 条目的字段**收拢到一个结构体里，只做数据承载（不参与运行、不改变任何日志输出），
 * 供后续"一条记录 = 一个接口事件"的导出/核对使用。
 *
 * 使用约定（当前阶段：只有定义，没有实现——按 2026-11-07 要求）：
 * - 一个实例对应**一次接口交互**（一个仿真周期内、一对源/宿之间的一条记录）；
 * - 只填与该接口相关的字段，其余字段保持默认值；
 * - 字段含义与验收判据/现有日志逐项对应，注释里标注所属验收项，便于对照。
 *
 * 命名：字段名用下划线小写；单位一律写入注释（m / m/s / W / dB / deg）。
 */

#ifndef ONEQ_FOUNDATION_ACCEPTANCE_INTERFACE_RECORD_H_
#define ONEQ_FOUNDATION_ACCEPTANCE_INTERFACE_RECORD_H_

#include <cstdint>

namespace oneq {
namespace foundation {

/// @brief 接口类别：决定本记录哪些字段有效。
enum class AcceptanceInterfaceKind : std::uint8_t {
  kRadarExternal = 0,  ///< 雷达外部接口（第 61/64/65 项）：平台参数、点迹量测、航迹识别。
  kRadarInternal = 1,  ///< 雷达内部接口（第 70–73 项）：回波→信号处理→检测→量测→航迹。
  kIrExternal = 2,     ///< 红外外部接口（第 57/68 项）：平台载荷配置、场景输入、观测输出。
  kIrInternal = 3,     ///< 红外内部接口（第 74–77 项）：几何门控、辐射噪声、调度、量测滤波。
  kSceneInput = 4,     ///< 场景/干扰输入（第 58/62/63 项）：目标场景、红外环境、外部干扰。
};

/// @brief 雷达工作模式（第 61 项 平台与工作参数配置）。
enum class AcceptanceRadarWorkMode : std::uint8_t {
  kUnknown = 0,
  kSearch = 1,   ///< 搜索
  kTrack = 2,    ///< 跟踪
  kIdentify = 3, ///< 识别
};

/// @brief 航迹生命周期状态（第 65 项 航迹识别输出）。
enum class AcceptanceTrackLifecycle : std::uint8_t {
  kUnknown = 0,
  kTentative = 1,   ///< 暂定
  kConfirmed = 2,   ///< 确认
  kCoasting = 3,    ///< 外推
  kLost = 4,        ///< 丢失
};

/// @brief 红外观测阶段（第 68 项 观测输出）。
enum class AcceptanceIrObservationStage : std::uint8_t {
  kUnknown = 0,
  kWfovCandidate = 1, ///< 宽场候选
  kNfovTrack = 2,     ///< 窄场跟踪
  kLost = 3,          ///< 丢失
};

/**
 * @brief 验收接口日志统一记录：一次接口交互的全部字段。
 *
 * @note 字段按"标识 → 雷达外部 → 雷达内部 → 红外外部 → 红外内部 → 场景/干扰输入"分组；
 *       未使用字段保持默认值（0 / false / kUnknown），不做哨兵值填充。
 */
struct AcceptanceInterfaceRecord {
  // ---------------------------------------------------------------------
  // 0) 公共标识（所有接口类别都填）
  // ---------------------------------------------------------------------
  AcceptanceInterfaceKind kind{AcceptanceInterfaceKind::kRadarExternal}; ///< 接口类别
  std::uint32_t cycle{0U};        ///< 仿真周期序号（从 1 起）
  double sim_time_sec{0.0};       ///< 仿真时间（s）
  std::uint32_t radar_id{0U};     ///< 雷达实体 ID（雷达侧接口）
  std::uint32_t satellite_id{0U}; ///< 卫星实体 ID（红外侧接口）
  std::uint32_t target_id{0U};    ///< 目标 ID
  std::uint32_t track_id{0U};     ///< 航迹 ID
  std::uint32_t plot_id{0U};      ///< 点迹 ID（第 64 项）
  std::uint32_t emission_id{0U};  ///< 发射/干扰源 ID（第 63/64 项）
  std::uint32_t batch_id{0U};     ///< 批次号（第 65 项）
  std::uint32_t count{0U};        ///< 本周期同类对象数量（目标数/点迹数/航迹数/观测数/干扰数）
  std::uint32_t effective_pulse_count{0U}; ///< 有效积累脉冲数（第 70 项）

  // ---------------------------------------------------------------------
  // 1) 雷达外部接口（第 61 项 平台与工作参数配置 / 第 64 项 点迹 / 第 65 项 航迹）
  // ---------------------------------------------------------------------
  AcceptanceRadarWorkMode radar_work_mode{AcceptanceRadarWorkMode::kUnknown}; ///< 工作模式
  double scan_center_az_deg{0.0};        ///< 扫描中心方位（deg）
  double scan_center_el_deg{0.0};        ///< 扫描中心仰角（deg）
  double boresight_az_deg{0.0};          ///< 当前波束指向方位（deg）
  double boresight_el_deg{0.0};          ///< 当前波束指向仰角（deg）
  double look_az_deg{0.0};               ///< 视线方位（deg）
  double look_el_deg{0.0};               ///< 视线仰角（deg）
  std::uint32_t available_track_capacity{0U}; ///< 可用跟踪容量（条）
  std::uint32_t alarm_count{0U};              ///< 告警数量
  std::uint32_t alarm_code{0U};               ///< 主告警码
  bool alarm_recovered{false};                ///< 是否已恢复
  double occultation_margin_m{0.0};      ///< 遮挡余量（m，地影/地平线裕度）

  // 点迹（第 64 项）
  double plot_range_m{0.0};              ///< 点迹斜距（m）
  double plot_az_deg{0.0};               ///< 点迹方位（deg）
  double plot_el_deg{0.0};               ///< 点迹俯仰（deg）
  double plot_snr_db{0.0};               ///< 点迹 SNR（dB）
  double plot_detection_probability{0.0}; ///< 检测概率（0–1）
  bool plot_detected{false};             ///< 检测标志
  std::uint8_t plot_data_level{0U};      ///< 数据层级（原始/点迹/航迹）

  // 航迹（第 65 项）
  AcceptanceTrackLifecycle track_lifecycle{AcceptanceTrackLifecycle::kUnknown}; ///< 生命周期
  double track_position_enu_m[3]{};      ///< 位置 ENU（m）
  double track_velocity_mps[3]{};        ///< 速度（m/s）
  double track_acceleration_mps2[3]{};   ///< 加速度（m/s²）
  std::uint8_t track_coordinate_frame{0U};            ///< 坐标系标识
  std::uint32_t cluster_target_count{0U};             ///< 集群目标数量
  const char* target_category{nullptr};               ///< 目标类别文本（只读，不拥有）
  const char* target_model{nullptr};                  ///< 目标型号文本（只读，不拥有）
  const char* hrrp_feature_summary{nullptr};          ///< 宽带一维像特征摘要（只读）
  const char* rcs_feature_summary{nullptr};           ///< RCS 特征摘要（只读）
  const char* polarization_feature_summary{nullptr};  ///< 极化特征摘要（只读）

  // ---------------------------------------------------------------------
  // 2) 雷达内部接口（第 70 项 回波→处理 / 71 项 处理→检测 / 72 项 检测→航迹 / 73 项 航迹→识别）
  // ---------------------------------------------------------------------
  // 回波（第 70 项）
  double echo_range_m{0.0};                 ///< 斜距（m）
  double echo_rcs_m2{0.0};                  ///< RCS（m²）
  double echo_offaxis_gain_dbi{0.0};        ///< 波束离轴增益（dBi）
  double echo_power_w{0.0};                 ///< 回波功率（处理前，W）
  double echo_delay_s{0.0};                 ///< 回波时延（s）
  double echo_two_way_doppler_hz{0.0};      ///< 双程多普勒（Hz）
  double pulse_compression_gain_db{0.0};    ///< 脉压增益（dB）
  double bandwidth_hz{0.0};                 ///< 信号带宽（Hz）
  double prf_hz{0.0};                       ///< 脉冲重复频率（Hz）

  // 检测（第 71 项）
  double processor_output_snr_db{0.0};      ///< 处理输出 SNR（dB）
  double detector_input_snr_db{0.0};        ///< 检测器输入 SNR（dB）
  double integrated_snr_db{0.0};            ///< 积累 SNR（dB）
  double total_noise_power_w{0.0};          ///< 总噪声功率（W）
  double detection_threshold_db{0.0};       ///< 统计检测门限（dB）
  double detection_probability{0.0};        ///< 检测概率（0–1）
  bool detected{false};                     ///< 检测标志
  double mti_gain_db{0.0};                  ///< MTI 增益（dB，含目标响应损失）
  double mtd_gain_db{0.0};                  ///< MTD 增益（dB，含跨 bin 损失）

  // 量测（第 72 项）
  double measured_position_enu_m[3]{};      ///< 量测位置 ENU（m）
  double measured_covariance[9]{};                       ///< 量测协方差（3×3 行主序）
  double velocity_seed_mps[3]{};            ///< 两点速度种子（m/s）
  std::uint64_t association_key{0ULL};                   ///< 关联键
  std::uint32_t source_target_index{0U};                 ///< 源目标索引
  bool measurement_consumed{false};                      ///< 是否被航迹处理消费

  // 识别（第 73 项）
  double feature_snr_db{0.0};               ///< 特征提取用 SNR（dB）
  std::uint32_t feature_mask{0U};           ///< 特征有效掩码
  std::uint32_t feature_batch{0U};          ///< 特征批次
  std::uint32_t feature_source_cycle{0U};   ///< 特征来源周期
  std::uint32_t recognition_batch{0U};      ///< 识别批次
  std::uint32_t recognition_source_cycle{0U}; ///< 识别来源周期
  double recognition_best_score{0.0};       ///< 综合分（最佳候选）
  double recognition_runner_up_score{0.0};  ///< 次优分
  std::uint32_t valid_feature_dimension_count{0U}; ///< 有效维度数
  std::uint32_t observation_count{0U};      ///< 积累观测数

  // ---------------------------------------------------------------------
  // 3) 红外外部接口（第 57 项 卫星平台与载荷配置 / 第 68 项 观测与航迹输出）
  // ---------------------------------------------------------------------
  double ir_radiant_intensity_w_per_sr{0.0};  ///< 目标红外辐射强度（W/sr）
  double ir_background_radiance_w_sr_m2{0.0}; ///< 背景辐射亮度（W/(sr·m²)）
  double ir_atmospheric_transmittance{0.0};   ///< 大气透过率（0–1）
  double ir_environment_temperature_k{0.0};   ///< 环境温度（K）
  double ir_detector_temperature_k{0.0};      ///< 探测器温度（K）
  double ir_relative_humidity{0.0};           ///< 相对湿度（0–1）
  double ir_snr{0.0};                         ///< 观测信噪比（线性）
  double ir_observation_id{0.0};              ///< 观测 ID（数值化，便于统一承载）
  double ir_state_vector[6]{};                ///< 状态向量（位置 3 + 速度 3）
  double ir_state_covariance[36]{};           ///< 状态协方差（6×6 行主序）
  AcceptanceIrObservationStage ir_observation_stage{AcceptanceIrObservationStage::kUnknown};
  bool ir_detected{false};                    ///< 检测标志
  const char* ir_state_estimator{nullptr};    ///< 估计器名称（只读）
  const char* ir_state_source{nullptr};       ///< 状态估计来源（只读）

  // ---------------------------------------------------------------------
  // 4) 红外内部接口（第 74 几何/视场门控、75 辐射噪声/判决、76 宽场候选/窄场调度、77 量测/滤波）
  // ---------------------------------------------------------------------
  double satellite_position_eci_m[3]{}; ///< 卫星 ECI 位置（m）
  double satellite_position_ecef_m[3]{}; ///< 卫星 ECEF 位置（m）
  double satellite_attitude_deg[3]{};   ///< 卫星姿态（deg）
  double target_position_eci_m[3]{};     ///< 目标 ECI 位置（m）
  double target_position_ecef_m[3]{};    ///< 目标 ECEF 位置（m）
  double ir_slant_range_m{0.0};              ///< 斜距（m）
  double ir_installation_angle_deg{0.0};     ///< 安装角（deg）
  double ir_scan_center_deg[2]{};    ///< 扫描中心（方位/俯仰，deg）
  double ir_occultation_margin_deg{0.0};     ///< 遮挡余量（deg）
  double ir_julian_date_utc{0.0};            ///< UTC 儒略日
  bool ir_fov_gate_passed{false};            ///< 视场门结果
  bool ir_visibility_gate_passed{false};     ///< 门控结果

  double ir_received_power_w{0.0};           ///< 接收功率（W）
  double ir_signal_energy_j{0.0};            ///< 信号能量（J）
  double ir_effective_noise_energy_j{0.0};   ///< 有效噪声能量（J）
  double ir_photon_noise_energy_j{0.0};      ///< 光子噪声（J）
  double ir_thermal_noise_energy_j{0.0};     ///< 热噪声（J）
  double ir_aperture_m{0.0};                 ///< 接收孔径（m）
  double ir_optical_transmission{0.0};       ///< 光学透过率（0–1）
  double ir_integration_time_s{0.0};         ///< 积分时间（s）
  bool ir_snr_gate_passed{false};            ///< SNR 门结果
  bool ir_stable_frame_gate_passed{false};   ///< 稳定帧门结果
  bool ir_admission_passed{false};           ///< 综合准入结果

  const char* ir_candidate_source{nullptr};   ///< 候选来源（只读）
  const char* ir_candidate_state{nullptr};    ///< 候选状态（只读）
  const char* ir_pointing_state{nullptr};     ///< 指向状态（只读）
  const char* ir_capture_result{nullptr};     ///< 捕获结果（只读）
  const char* ir_capture_failure_reason{nullptr}; ///< 捕获失败原因（只读）
  double ir_candidate_angle_deg[2]{}; ///< 候选测角（方位/俯仰，deg）
  double ir_candidate_measure_time_s{0.0};    ///< 候选量测时刻（s）
  double ir_remaining_angle_deg{0.0};         ///< 剩余角（deg）
  double ir_served_duration_s{0.0};           ///< 已服务时长（s）
  std::uint32_t ir_guide_source_satellite_id{0U}; ///< 引导来源卫星 ID
  double ir_command_angle_deg[2]{};   ///< 指令方位/俯仰（deg）
  double ir_capture_input_snr{0.0};           ///< 捕获判决输入 SNR

  double ir_predicted_angle_deg[2]{}; ///< 预测角（deg）
  double ir_filtered_angle_deg[2]{};  ///< 滤波输出角（deg）
  double ir_los_rate_deg_per_s{0.0};          ///< 视线角速率（deg/s）
  double ir_angle_rate_deg_per_s{0.0};        ///< 角速度（deg/s）
  double ir_measurement_covariance[4]{};      ///< 量测协方差（2×2 行主序）
  double ir_measure_time_s{0.0};              ///< 量测时刻（s）
  double ir_nis{0.0};                         ///< 归一化新息平方（NIS）
  bool ir_nis_gate_passed{false};             ///< NIS 门结果
  std::uint32_t ir_frame_count{0U};           ///< 帧数
  const char* ir_track_state{nullptr};        ///< 跟踪状态（只读）
  const char* ir_filter_output_stage{nullptr}; ///< 输出阶段（只读）

  // ---------------------------------------------------------------------
  // 5) 场景/干扰输入（第 58 项 目标与红外环境 / 62 项 目标场景 / 63 项 外部干扰）
  // ---------------------------------------------------------------------
  double scene_position_ecef_m[3]{}; ///< 目标位置 ECEF（m）
  double scene_velocity_ecef_mps[3]{}; ///< 目标速度 ECEF（m/s）
  double scene_position_enu_m[3]{};  ///< 目标位置 ENU（m）
  double scene_rcs_m2{0.0};                  ///< 目标 RCS（m²）
  double scene_swelling_model{0.0};          ///< 起伏模型编号（Swirling 0–4）
  double jammer_center_frequency_hz{0.0};    ///< 干扰中心频率（Hz）
  double jammer_bandwidth_hz{0.0};           ///< 干扰带宽（Hz）
  double jammer_peak_gain_dbi{0.0};          ///< 干扰峰值增益（dBi）
  double jammer_transmit_power_w{0.0};       ///< 干扰发射功率（W）
  double jammer_active_start_s{0.0};         ///< 干扰活动起点（s）
  double jammer_active_duration_s{0.0};      ///< 干扰持续时间（s）
  double jammer_position_ecef_m[3]{}; ///< 干扰源位置 ECEF（m）
  double jammer_velocity_ecef_mps[3]{}; ///< 干扰源速度 ECEF（m/s）
  const char* jammer_waveform{nullptr};      ///< 干扰波形（只读）
  const char* jammer_polarization{nullptr};  ///< 干扰极化（只读）
  const char* scene_target_name{nullptr};    ///< 目标名称（只读）
  std::uint32_t world_cycle{0U};             ///< 世界周期（干扰输入侧）
};

}  // namespace foundation
}  // namespace oneq

#endif  // ONEQ_FOUNDATION_ACCEPTANCE_INTERFACE_RECORD_H_
