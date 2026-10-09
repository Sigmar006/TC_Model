/**
 * @file RirTrackRequest.h
 * @brief 定义外部目标跟踪请求：位置 + 速度 + 中心邻域跟踪窗口。
 *
 * 设计定位（2026-10-04 实测驱动）：
 * 本接口不是「改整机扫描基准」，而是「指定一个被持续跟踪的照射中心」。
 *
 * 为什么必须区分这两种语义：实测表明（见
 * docs/review/ir_external_position_driven_rir_e2e_2026-10-04.md）
 * - 仅设 `RirRuntimeConfigPatch::has_scan_center`（扫描基准语义）：零偏移、完美指向下
 *   仍 **0/25 周期检出**，且一直报 `rir.target_outside_beam_coverage`——失败是**静默的**；
 * - 指定照射语义（每周期首个驻留对准目标）：**25/25 周期检出**。
 * 原因是扫描基准语义下，目标方向只在波位表游标转过去时才被照到一次，间隔可达上千周期。
 *
 * 本接口的机理：
 * 1、调用方给目标位置与速度（ECEF）；
 * 2、库内按速度把位置**外推到本周期时刻**（避免时间偏斜直接变成指向误差）；
 * 3、换系到雷达局部 ENU 得到照射中心；
 * 4、以该中心为原点、按 `window` 给出的方位/俯仰半宽铺驻留波位（中心优先、由近到远）；
 * 5、未收到新请求时按最后速度继续外推，超时释放。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_TRACK_REQUEST_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_TRACK_REQUEST_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"

namespace remote_identification_radar {
namespace session {

/**
 * @brief RirTrackWindowDeg 中心邻域跟踪窗口：以照射中心为原点的两轴半宽（deg）。
 * @note 两轴**均为相对中心的半宽**（方位左右各 az_half、俯仰上下各 el_half），
 *       与全局扫描的「方位相对 scan_center、俯仰绝对」口径不同——本窗口只服务外部跟踪，
 *       不影响 `RirMissionConfig::scan_window_deg` 的既有语义。
 * @note 窗口大小与重访率互斥：波位点数 ≈ (2·az_half/波束宽)·(2·el_half/波束宽)，
 *       一圈周期数 = 点数 / 每周期可用槽位数。应满足
 *       「一圈周期数 ≤ RirLifecyclePolicyConfig::max_miss_before_lost」，
 *       否则失配计数会先到、航迹先丢。默认 8x4 度在默认 4 度波束 / 10 槽位下约 2 周期一圈。
 */
struct ONEQ_API RirTrackWindowDeg {
  float az_half_deg{8.0f}; /**< 方位半宽（deg），>0。 */
  float el_half_deg{4.0f}; /**< 俯仰半宽（deg），>0。 */
};

/**
 * @brief RirExternalTrackPhase 外部跟踪生命周期阶段。
 */
enum class ONEQ_API RirExternalTrackPhase : std::uint8_t {
  kNone = 0,     /**< 未启用外部跟踪。 */
  kFresh = 1,    /**< 本周期收到新请求（或距上次请求不超过一个周期）。 */
  kCoasting = 2, /**< 无新请求但仍在外推保持期内（窗口继续跟随外推中心）。 */
  kExpired = 3,  /**< 保持期耗尽，已释放外部槽位并回到既定调度。 */
};

/**
 * @brief RirExternalTrackRequest 外部目标跟踪请求（一次提交，逐周期重发以持续跟踪）。
 */
struct ONEQ_API RirExternalTrackRequest {
  /** @brief 目标位置（ECEF，m）；必填，有限且模长非退化。 */
  oneq::coordinate::EcefPositionM position_ecef_m{};
  /** @brief 目标速度（ECEF，m/s）；必填——库内按时延外推中心，无速度则指向必然偏差。 */
  oneq::coordinate::EcefVelocityMps velocity_ecef_mps{};

  /** @brief 位置不确定度 1σ（m）；0 = 未提供（仅诊断用途，不参与门控）。 */
  float position_sigma_m{0.0f};
  /** @brief 速度不确定度 1σ（m/s）；0 = 未提供（仅诊断用途，不参与门控）。 */
  float velocity_sigma_mps{0.0f};

  /** @brief 上述状态对应的量测时刻（绝对仿真秒）；库内据此把位置外推到本周期。 */
  double source_time_s{0.0};

  /** @brief 中心邻域跟踪窗口。 */
  RirTrackWindowDeg window{};

  /** @brief 有效至周期号；0 = 由库内按速度与位置误差推导保持期。 */
  std::uint32_t valid_until_cycle_index{0U};

  /** @brief 来源侧目标标识（溯源与人读；不参与门控）。 */
  std::uint64_t source_target_id{0U};
  /** @brief 来源设备标识（红外/融合通道；诊断与回放溯源）。 */
  std::uint32_t source_entity_id{0U};
};

/**
 * @brief RirExternalTrackState 外部跟踪当前状态快照（供集成方判断跟踪是否仍在维持）。
 */
struct ONEQ_API RirExternalTrackState {
  RirExternalTrackPhase phase{RirExternalTrackPhase::kNone}; /**< 当前阶段。 */
  std::uint32_t last_request_cycle_index{0U};    /**< 最近一次成功提交的周期号；0 = 从未提交。 */
  std::uint32_t coasting_cycles{0U};             /**< 当前已连续外推的周期数。 */
  float extrapolated_age_sec{0.0f};              /**< 外推时长（本周期时刻 − source_time_s）。 */
};

}  // namespace session
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_TRACK_REQUEST_H_
