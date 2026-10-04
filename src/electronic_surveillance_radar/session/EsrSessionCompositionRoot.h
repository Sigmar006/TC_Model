/**
 * @file EsrSessionCompositionRoot.h
 * @brief 定义电子侦察会话组合根。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_SESSION_ESR_SESSION_COMPOSITION_ROOT_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_SESSION_ESR_SESSION_COMPOSITION_ROOT_H_

#include <memory>
#include <utility>

#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "electronic_surveillance_radar/config/EsrInternalExecutionConfig.h"

namespace electronic_surveillance_radar {
namespace environment {
class IEsrEnvironmentService;
}
namespace extension {
class EsrController;
}
namespace pipeline {
class InterceptPipeline;
}
namespace session {

/**
 * @brief EsrSessionComposition 描述会话装配后的组件集合。
 *
 * `owned_*` 持有所有权（组合根拥有生命周期），`pipeline`/`environment_service`/
 * `controller` 为非 owning 指针，指向 `owned_*` 中的对象，便于按接口传递。
 */
struct EsrSessionComposition {
  // VS2013 不生成隐式移动操作（含移动-only 成员时退化为已删除拷贝），显式实现。
  EsrSessionComposition() = default;
  EsrSessionComposition(EsrSessionComposition&& other) noexcept
      : execution_config(std::move(other.execution_config)),
        owned_pipeline(std::move(other.owned_pipeline)),
        owned_environment_service(std::move(other.owned_environment_service)),
        owned_controller(std::move(other.owned_controller)),
        pipeline(other.pipeline),
        environment_service(other.environment_service),
        controller(other.controller) {}
  EsrSessionComposition& operator=(EsrSessionComposition&& other) noexcept {
    execution_config = std::move(other.execution_config);
    owned_pipeline = std::move(other.owned_pipeline);
    owned_environment_service = std::move(other.owned_environment_service);
    owned_controller = std::move(other.owned_controller);
    pipeline = other.pipeline;
    environment_service = other.environment_service;
    controller = other.controller;
    return *this;
  }
  EsrInternalExecutionConfig execution_config{}; /**< 解析后的内部执行态配置 */

  std::unique_ptr<pipeline::InterceptPipeline> owned_pipeline;               /**< 拥有所有权的流水线实例 */
  std::unique_ptr<environment::IEsrEnvironmentService> owned_environment_service; /**< 拥有所有权的环境服务 */
  std::unique_ptr<extension::EsrController> owned_controller;                /**< 拥有所有权的控制器 */

  pipeline::InterceptPipeline* pipeline{nullptr};                       /**< 非 owning 流水线指针 */
  environment::IEsrEnvironmentService* environment_service{nullptr};     /**< 非 owning 环境服务指针 */
  extension::EsrController* controller{nullptr};                        /**< 非 owning 控制器指针 */
};

/**
 * @brief EsrSessionCompositionRoot 负责会话对象图装配。
 */
class EsrSessionCompositionRoot {
 public:
  /**
   * @brief 装配默认会话组件集合。
   * @param[in] config 四域会话配置。
   * @return 装配完成的组件集合（含所有权）。
   */
  static EsrSessionComposition ComposeDefault(const config::EsrSessionConfig& config);
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_SESSION_ESR_SESSION_COMPOSITION_ROOT_H_
