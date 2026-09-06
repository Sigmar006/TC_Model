/**
 * @file SignalPipeline.h
 * @brief 定义信号处理流水线的默认内部实现。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_H_

#include <memory>
#include <vector>

#include "airborne_radar/signal/pipeline/ISignalPipeline.h"
#include "airborne_radar/signal/pipeline/SignalPipelineExecutionConfig.h"
#include "airborne_radar/signal/pipeline/SignalPipelineRuntimeTypes.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
/**
 * @brief SignalPipeline 提供可配置的信号处理默认实现。
 */
class SignalPipeline final : public ISignalPipeline {
 public:
  /**
   * @brief 构造信号处理流水线。
   * @param config execution 运行配置。
   */
  explicit SignalPipeline(const ExecutionConfig& config = {});

  /**
   * @brief 构造信号处理流水线（会话配置桥接入口）。
   * @param config 四域会话配置。
   */
  explicit SignalPipeline(const config::ArSessionConfig& config);
  /**
   * @brief 析构信号处理流水线。
   */
  ~SignalPipeline() override;

  /**
   * @brief 执行单周期信号处理流程。
   * @param input 本周期输入结构体（捆绑 scene_targets、RF v2 detection 上下文、
   *              干扰观测与欺骗候选量测）。
   * @param environment 当前环境服务。
   * @return 当前周期的信号处理输出。
   */
  session::SignalCycleResult RunCycle(
      const SignalCycleInput& input,
      const environment::IEnvironmentService& environment) override;

  /**
   * @brief 获取上一周期生成的跟踪量测列表。
   * @return 与上一周期运行结果对应的跟踪量测集合。
   */
  std::vector<tracking::TrackMeasurement> GetLastTrackMeasurements() const;

  /**
   * @brief 获取上一周期的关联质量指标。
   * @return 上一周期缓存的关联质量指标。
   */
  session::AssociationQualityMetrics GetLastAssociationQualityMetrics() const override;

  SignalPipelineRuntimeState CaptureRuntimeState() const override;

  void RestoreRuntimeState(const SignalPipelineRuntimeState& state) override;

  /**
   * @brief 进入 manual association seed override 模式。
   * @param seeds 由调用方显式注入的轨迹种子；空输入或非法输入会清除 override 并恢复
   *              默认 lifecycle seed 模式。
   */
  void SetAssociationSeeds(const std::vector<tracking::AssociationTrackSeed>& seeds);

  /**
   * @brief 清空外部注入的关联种子并恢复默认 lifecycle seed 模式。
   */
  void ClearManualAssociationSeeds();

  /**
   * @brief 按当前配置构造默认生命周期管理器。
   * @return 生命周期管理器实例。
   */
  std::unique_ptr<tracking::ITrackLifecycleManager> CreateAutoLifecycleManager() const;

  /**
   * @brief 更新平台姿态输入。
   * @param platform_attitude_deg 当前平台姿态。
   */
  void UpdatePlatformAttitude(const config::PlatformAttitudeDeg& platform_attitude_deg) override;
  void UpdatePlatformAltitudeM(float platform_altitude_m) override;

  /**
   * @brief 获取当前缓存的平台姿态。
   * @return 当前平台姿态。
   */
  config::PlatformAttitudeDeg GetPlatformAttitude() const override;
  float GetPlatformAltitudeM() const override;

  /**
   * @brief 更新当前生效的控制真值。
   * @param control_profile 控制真值。
   */
  void SetControlProfile(const session::ArControlProfile& control_profile) override;

  /**
   * @brief 获取当前缓存的控制真值。
   * @return 当前控制真值。
   */
  session::ArControlProfile GetControlProfile() const override;

  void SetSensorEntityId(std::uint32_t sensor_entity_id) override;

  /**
   * @brief 更新流水线运行配置。
   * @param config 四域会话配置。
   */
  bool UpdateConfig(const config::ArSessionConfig& config) override;

  /**
   * @brief 以 execution 配置直接更新流水线运行配置。
   * @param config execution 运行配置。
   */
  bool UpdateExecutionConfig(const ExecutionConfig& config);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_H_
