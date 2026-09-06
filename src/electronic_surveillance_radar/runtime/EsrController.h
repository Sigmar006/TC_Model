/**
 * @file EsrController.h
 * @brief 定义电子侦察核心调度控制器接口。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_RUNTIME_ESR_CONTROLLER_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_RUNTIME_ESR_CONTROLLER_H_

#include <memory>

#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"

namespace electronic_surveillance_radar {
namespace pipeline {
class InterceptPipeline;
}
namespace environment {
class IEsrEnvironmentService;
}

/**
 * @brief EsrControllerRuntimeState 描述 ESR 控制器运行态快照。
 */
struct EsrControllerRuntimeState {
  const void* owner_identity{nullptr};
  std::uint32_t schema_version{0U};
  bool has_latest_output{false};
  session::EsrOutputFrame latest_output{};
  std::uint64_t next_batch_id{1U};
  session::EsrPipelineAbortReason last_abort_reason{
      session::EsrPipelineAbortReason::kNone};
};

namespace extension {

/**
 * @brief EsrController 负责调度环境冻结、侦察流水线执行与输出缓存。
 */
class EsrController {
 public:
  /**
   * @brief 构造电子侦察控制器。
   * @param[in] pipeline 侦察流水线实现。
   * @param[in] environment_service 环境服务接口实现。
   */
  EsrController(pipeline::InterceptPipeline& pipeline,
                environment::IEsrEnvironmentService& environment_service);
  ~EsrController();

  EsrController(const EsrController&) = delete;
  EsrController& operator=(const EsrController&) = delete;

  /**
   * @brief 设置传感器设备实体 ID（透传给 InterceptPipeline）。
   */
  void SetSensorEntityId(std::uint32_t sensor_entity_id);

  /**
   * @brief 执行一次电子侦察处理周期。
   * @param[in] input 当前周期输入。
   */
  void RunOnce(const session::EsrCycleInput& input);

  /**
   * @brief 判断当前是否有可读取的最新输出帧。
   * @return 若已有输出帧则返回 `true`。
   */
  bool HasLatestInterceptOutputFrame() const;

  /**
   * @brief 获取最新输出帧。
   * @return 最新电子侦察输出帧。
   */
  const session::EsrOutputFrame& GetLatestInterceptOutputFrame() const;

  /**
   * @brief 返回最近一次 RunOnce 装配并缓存的单周期聚合结果。
   * @return 最近一次周期的聚合结果。
   */
  session::EsrCycleResult BuildCycleResult() const;

  /**
   * @brief 捕获当前控制器自有运行态快照。
   * @note 流水线累积状态由 session 事务边界独立捕获，不属于控制器快照。
   * @return 当前运行态快照。
   */
  EsrControllerRuntimeState CaptureRuntimeState() const;

  /**
   * @brief 使用快照恢复控制器运行态。
   * @param[in] state 待恢复的快照。
   * @return 恢复成功返回 `true`。
   */
  bool RestoreRuntimeState(const EsrControllerRuntimeState& state);

  /**
   * @brief 获取当前控制器绑定的环境服务实例。
   */
  environment::IEsrEnvironmentService& GetEnvironmentService();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace extension

}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_RUNTIME_ESR_CONTROLLER_H_
