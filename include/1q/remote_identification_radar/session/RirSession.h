/**
 * @file RirSession.h
 * @brief 定义面向外部接入的远程识别雷达会话门面。
 *
 * 会话门面（一步一帧执行、运行期补丁）的主头文件。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_SESSION_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_SESSION_H_

#include <memory>

#include "1q/api.hpp"
#include "1q/foundation/SensorContract.h"
#include "1q/remote_identification_radar/config/RirSessionConfig.h"
#include "1q/remote_identification_radar/config/RirSessionConfigValidation.h"
#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "1q/remote_identification_radar/session/RirCycleResult.h"
#include "1q/remote_identification_radar/session/RirOutputTypes.h"
#include "1q/remote_identification_radar/session/RirRecognitionResult.h"
#include "1q/remote_identification_radar/session/RirTrackRequest.h"

namespace remote_identification_radar {
namespace config {
struct RirRuntimeConfigPatch;
}  // namespace config
}  // namespace remote_identification_radar

namespace remote_identification_radar {
namespace session {

class RirTrackLifecycleRecorder;
class RirExclusionCauseRecorder;

/**
 * @brief RirSession 提供"一步一帧"的外部接入门面。
 */
class ONEQ_API RirSession {
 public:
  RirSession();
  ~RirSession();

  RirSession(const RirSession&) = delete;
  RirSession& operator=(const RirSession&) = delete;
  RirSession(RirSession&&) noexcept;
  RirSession& operator=(RirSession&&) noexcept;

  /**
   * @brief 执行一个识别处理周期。
   * @param[in] input 当前周期输入。
   * @return 当前周期生成的识别输出帧拷贝。
   * @note 非法或被拒绝的周期返回默认的当前帧，不复用历史输出。
   */
  RirOutputFrame Step(const RirCycleInput& input);

  /**
   * @brief 执行一个识别处理周期，并返回聚合结果。
   * @param[in] input 当前周期输入。
   * @return 当前周期聚合结果。
   * @note `status` 是结果有效性的唯一真相；只有 `kCompleted` 携带本周期识别输出。
   */
  RirCycleResult StepWithResult(const RirCycleInput& input);

  /**
   * @brief 尝试应用运行期可变配置补丁。
   * @param[in] patch 运行期可变配置补丁。
   * @note 未设置的字段保持现值不变；补丁在下一个成功执行周期边界提交。
   * @return 补丁被接受并暂存成功时返回 true；补丁无效或无变更时返回 false。
   */
  bool TryApplyRuntimeConfig(const config::RirRuntimeConfigPatch& patch);

  /**
   * @brief 注册航迹生命周期记录器（规则 10/11）。
   *
   * Session 在 `StepWithResult()`/`Step()` 内部自动调用 `Update()`，调用方无需
   * （也不应）手动调用；注册与否不影响 `Step()`/`StepWithResult()` 的返回值
   * 和执行语义（纯观测工具，零行为改变）。
   *
   * @param[in] recorder 生命周期记录器；传入 nullptr 解除注册。
   * @note Session 持有非拥有裸指针，调用方须保证 recorder 生命周期长于注册期。
   */
  void AttachTrackLifecycleRecorder(RirTrackLifecycleRecorder* recorder) noexcept;

  /**
   * @brief 注册排除原因差分记录器（规则 10/11/13e）。
   *
   * 与 `AttachTrackLifecycleRecorder` 独立并列：各自注册、各自驱动、各自
   * `GetLastEvents()` 通道，互不影响。非执行周期两记录器均不推进内部状态。
   *
   * @param[in] recorder 排除原因差分记录器；传入 nullptr 解除注册。
   * @note Session 持有非拥有裸指针，调用方须保证 recorder 生命周期长于注册期。
   */
  void AttachExclusionCauseRecorder(RirExclusionCauseRecorder* recorder) noexcept;

  /** @brief 最近周期是否发布了识别效能摘要。 */
  bool HasLatestRecognitionSummary() const;

  /** @brief 获取最近周期识别效能摘要。 */
  const RirRecognitionCycleSummary& GetLatestRecognitionSummary() const;

  /** @brief 最近一次识别库加载耗时（毫秒）；未加载过为 0。 */
  double LastRecognitionDatabaseLoadMs() const;

  /**
   * @brief 检测链随机数诊断快照（2026-10-20，仅供诊断日志使用）。
   *
   * @details 背景：`UpdateRuntime`/`SetHardware` 曾在每次运行时补丁后重设随机种子，
   *   而宿主几乎每周期都下发补丁 ⇒ CFAR 抽签被冻结、概率判决退化为固定门限。
   *   本结构把"种子被实际施加的次数"与"进入判决流程的次数"暴露给宿主诊断日志，
   *   用于**自证**修复是否生效（正常推进时种子只应在配置变化时施加一次）。
   * @note 不参与任何数值计算，也不写入验收日志。
   */
  struct DetectionRngDiagnostics {
    std::uint32_t applied_seed{0U};      /**< 当前生效的检测随机种子。 */
    std::uint32_t seed_apply_count{0U};  /**< 运行至今实际调用 SetRandomSeed 的次数。 */
    std::uint64_t decision_count{0U};    /**< 运行至今进入检测判决流程的次数。 */
    std::uint64_t false_decision_count{0U}; /**< 其中判为"未检测"的次数。 */
  };

  /**
   * @brief 读取检测链随机数诊断快照。
   * @param[out] out 非空时写入快照。
   * @return 会话已初始化返回 true。
   */
  bool GetDetectionRngDiagnostics(DetectionRngDiagnostics* out) const;

  /**
   * @brief 提交外部目标跟踪请求：位置 + 速度 + 中心邻域跟踪窗口。
   *
   * 机理（本接口不是「改整机扫描基准」）：
   * 1、库内按速度把位置外推到本周期时刻；
   * 2、换系到雷达局部 ENU 得照射中心；
   * 3、以中心为原点按 `window` 铺驻留波位（中心优先、由近到远），逐周期重建；
   * 4、未收到新请求时按最后速度继续外推，超时释放。
   *
   * @param[in] request 外部跟踪请求。
   * @return 请求合法（位置/速度有限、窗口为正、位置非退化）返回 true；
   *         非法时**原子拒绝**——不改变当前跟踪状态，并写入统一问题列表。
   * @note 逐周期提交以持续跟踪；不提交则进入外推保持（kCoasting）。
   * @note 指向结果经 `RirCycleResult::pointing_source` / `track_center_deg` /
   *       `external_track_phase` 逐周期暴露。
   */
  bool SubmitExternalTrack(const session::RirExternalTrackRequest& request);

  /** @brief 清除外部跟踪，立即回到既定扫描/跟踪调度。 */
  void ClearExternalTrack();

  /** @brief 获取外部跟踪当前状态（阶段 / 外推时长 / 已外推周期数）。 */
  session::RirExternalTrackState GetExternalTrackState() const;

  /**
   * @brief 注入传感器实体标识（设备 ID / 融合通道）。
   *
   * 用于在日志与验收记录中区分同型多实例传感器，并作为融合通道 source_id。
   * 传 0 恢复缺省（fusion::kRirSourceId = 5U）；下一周期起生效。
   * @param[in] sensor_entity_id 传感器实体标识（非零；0 为恢复缺省哨兵）。
   */
  void SetSensorEntityId(std::uint32_t sensor_entity_id);

  /** @brief 使用四域配置创建会话（推荐入口，信任路径，不做配置校验）。 */
  static RirSession Create(const config::RirSessionConfig& config = {});

  /**
   * @brief 创建会话并报告配置校验结果（校验路径）。
   * @param[in] config 四域会话配置。
   * @param[out] issues 校验问题输出；传入 nullptr 则不写回但仍构造会话。
   * @return 构造完成的会话。
   * @note 无论 @p issues 是否为空都会构造并返回会话（不阻断）。
   */
  static RirSession CreateWithDiagnostics(const config::RirSessionConfig& config,
                                          RirIssueList* issues);

 private:
  friend class RirSessionReplayAccess;

  struct Impl;
  explicit RirSession(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace remote_identification_radar

// 跨域传感器会话形状契约：锚定 Step/StepWithResult 签名。
ONEQ_SENSOR_SESSION_CONTRACT(remote_identification_radar::session::RirSession,
                             remote_identification_radar::session::RirCycleInput,
                             remote_identification_radar::session::RirOutputFrame,
                             remote_identification_radar::session::RirCycleResult);

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_SESSION_H_
