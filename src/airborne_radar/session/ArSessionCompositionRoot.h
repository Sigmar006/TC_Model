/**
 * @file ArSessionCompositionRoot.h
 * @brief 定义 AR 会话的组合根（composition root），统一装配上下文、流水线、环境服务与控制器。
 */

#ifndef AIRBORNE_RADAR_CORE_SESSION_AR_SESSION_COMPOSITION_ROOT_H_
#define AIRBORNE_RADAR_CORE_SESSION_AR_SESSION_COMPOSITION_ROOT_H_

#include <memory>
#include <utility>

#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/session/ArSession.h"

namespace airborne_radar {
namespace signal {
class ISignalPipeline;
}  // namespace signal
namespace extension {
class ArController;
}  // namespace extension
namespace environment {
class IEnvironmentService;
}
namespace session {
class MutableArContext;

/**
 * @brief 会话装配产物，集中持有运行期实例与其配置快照。
 *
 * 由组件根拥有各 owned_* 实例（通过 unique_ptr 管理生命周期），
 * 同时暴露裸指针别名便于在不转移所有权的情况下访问。
 */
struct ArSessionComposition {
  config::ArHardwareConfig runtime_hardware{};
  config::ArMissionConfig runtime_mission{};
  config::ArOrientationConfig runtime_orientation{};
  config::ArPolicyConfig runtime_policy{};
  config::EnvironmentScenarioConfig runtime_environment_scenario_config{};
  std::unique_ptr<MutableArContext> owned_ar_context;
  std::unique_ptr<signal::ISignalPipeline> owned_signal_pipeline;
  std::unique_ptr<environment::IEnvironmentService> owned_environment_service;
  std::unique_ptr<extension::ArController> owned_controller;

  MutableArContext* ar_context{nullptr};
  signal::ISignalPipeline* signal_pipeline{nullptr};
  environment::IEnvironmentService* environment_service{nullptr};
  extension::ArController* controller{nullptr};
  bool pipeline_config_synced{true};

  // VS2013 不生成隐式移动操作（含移动-only 成员时退化为已删除拷贝），显式实现。
  ArSessionComposition() = default;
  ArSessionComposition(ArSessionComposition&& other) noexcept
      : runtime_hardware(std::move(other.runtime_hardware)),
        runtime_mission(std::move(other.runtime_mission)),
        runtime_orientation(std::move(other.runtime_orientation)),
        runtime_policy(std::move(other.runtime_policy)),
        runtime_environment_scenario_config(std::move(other.runtime_environment_scenario_config)),
        owned_ar_context(std::move(other.owned_ar_context)),
        owned_signal_pipeline(std::move(other.owned_signal_pipeline)),
        owned_environment_service(std::move(other.owned_environment_service)),
        owned_controller(std::move(other.owned_controller)),
        ar_context(other.ar_context),
        signal_pipeline(other.signal_pipeline),
        environment_service(other.environment_service),
        controller(other.controller),
        pipeline_config_synced(other.pipeline_config_synced) {}
  ArSessionComposition& operator=(ArSessionComposition&& other) noexcept {
    runtime_hardware = std::move(other.runtime_hardware);
    runtime_mission = std::move(other.runtime_mission);
    runtime_orientation = std::move(other.runtime_orientation);
    runtime_policy = std::move(other.runtime_policy);
    runtime_environment_scenario_config = std::move(other.runtime_environment_scenario_config);
    owned_ar_context = std::move(other.owned_ar_context);
    owned_signal_pipeline = std::move(other.owned_signal_pipeline);
    owned_environment_service = std::move(other.owned_environment_service);
    owned_controller = std::move(other.owned_controller);
    ar_context = other.ar_context;
    signal_pipeline = other.signal_pipeline;
    environment_service = other.environment_service;
    controller = other.controller;
    pipeline_config_synced = other.pipeline_config_synced;
    return *this;
  }
};

/**
 * @brief AR 会话组合根，负责根据会话配置装配整套运行期对象图。
 */
class ArSessionCompositionRoot {
 public:
  /**
   * @brief 使用默认战术协调器装配会话。
   * @param[in] config 四域会话配置，用于初始化各组件。
   * @return 装配完成的 ArSessionComposition。
   */
  static ArSessionComposition ComposeDefault(const config::ArSessionConfig& config);

};


}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_SESSION_AR_SESSION_COMPOSITION_ROOT_H_
