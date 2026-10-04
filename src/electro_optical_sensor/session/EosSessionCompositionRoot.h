/**
 * @file EosSessionCompositionRoot.h
 * @brief 定义 EOS 会话组合根及其装配结果。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SESSION_COMPOSITION_ROOT_H_
#define ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SESSION_COMPOSITION_ROOT_H_

#include <memory>
#include <utility>

#include "1q/electro_optical_sensor/session/EosSession.h"
#include "electro_optical_sensor/config/EosInternalExecutionConfig.h"

namespace electro_optical_sensor {
namespace extension {
class EosController;
}
namespace signal {
namespace pipeline {
class EosPipeline;
}  // namespace pipeline
}  // namespace signal
namespace session {

/**
 * @brief EosSessionComposition 描述会话所需依赖的组合结果。
 * @note 组合结果需保证 `owned_pipeline` 与 `controller` 始终非空。
 * @note 与 AR/ESR 不同，EO 传感器环境为纯观测型（无状态），环境因子以值类型嵌入
 *       CycleInput，ResolveFactors() 为无状态纯函数，因此环境服务托管在 Pipeline 内部
 *       而非在 Composition 层独立管理。
 */
struct EosSessionComposition {
  // VS2013 不生成隐式移动操作（含移动-only 成员时退化为已删除拷贝），显式实现。
  EosSessionComposition() = default;
  EosSessionComposition(EosSessionComposition&& other) noexcept
      : internal_config(std::move(other.internal_config)),
        initial_reset_scan_phase(other.initial_reset_scan_phase),
        owned_pipeline(std::move(other.owned_pipeline)),
        owned_controller(std::move(other.owned_controller)) {}
  EosSessionComposition& operator=(EosSessionComposition&& other) noexcept {
    internal_config = std::move(other.internal_config);
    initial_reset_scan_phase = other.initial_reset_scan_phase;
    owned_pipeline = std::move(other.owned_pipeline);
    owned_controller = std::move(other.owned_controller);
    return *this;
  }
  config::execution::EosInternalExecutionConfig internal_config{}; /**< 解析后的内部执行配置真值 */
  bool initial_reset_scan_phase{true};                              /**< 初始装配时是否重置扫描相位 */

  std::unique_ptr<signal::pipeline::EosPipeline> owned_pipeline;   /**< 拥有的核心管线实例（始终非空） */
  std::unique_ptr<extension::EosController> owned_controller;      /**< 拥有的控制器实例（始终非空） */
};

/**
 * @brief EosSessionCompositionRoot 负责 EOS 会话依赖装配。
 * @note 管线与环境服务已完全内部化，不再支持外部注入。
 */
class EosSessionCompositionRoot {
 public:
  /**
   * @brief 以默认依赖装配会话。
    * @param[in] config 会话初始化配置。
   * @return 完整的会话组合结果。
   */
  static EosSessionComposition ComposeDefault(const config::EosSessionConfig& config);
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SESSION_COMPOSITION_ROOT_H_
