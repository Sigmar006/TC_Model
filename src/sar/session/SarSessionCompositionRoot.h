/**
 * @file SarSessionCompositionRoot.h
 * @brief SAR 会话组合根，按默认依赖装配 pipeline 与 controller。
 */

#ifndef ONEQ_SRC_SAR_SESSION_SAR_SESSION_COMPOSITION_ROOT_H_
#define ONEQ_SRC_SAR_SESSION_SAR_SESSION_COMPOSITION_ROOT_H_

#include <memory>
#include <utility>

#include "1q/sar/config/SarSessionConfig.h"

namespace sar {
namespace pipeline {
class SarProcessingPipeline;
}
namespace extension {
class SarController;
}
namespace session {

/**
 * @brief SAR 会话组合产物。
 *
 * 由组合根装配的 pipeline 与 controller，调用方持有所有权并经 controller 驱动周期执行。
 * @note owned_pipeline 与 owned_controller 由组合根拥有；controller 指向 owned_controller。
 */
struct SarSessionComposition {
  // VS2013 不生成隐式移动操作（含移动-only 成员时退化为已删除拷贝），显式实现。
  SarSessionComposition() = default;
  SarSessionComposition(SarSessionComposition&& other) noexcept
      : owned_pipeline(std::move(other.owned_pipeline)),
        owned_controller(std::move(other.owned_controller)),
        controller(other.controller) {}
  SarSessionComposition& operator=(SarSessionComposition&& other) noexcept {
    owned_pipeline = std::move(other.owned_pipeline);
    owned_controller = std::move(other.owned_controller);
    controller = other.controller;
    return *this;
  }
  std::unique_ptr<pipeline::SarProcessingPipeline> owned_pipeline; /**< 组合根拥有的处理流水线 */
  std::unique_ptr<extension::SarController> owned_controller; /**< 组合根拥有的运行期控制器 */
  extension::SarController* controller{nullptr}; /**< 指向 owned_controller 的便捷句柄 */
};

/**
 * @brief SAR 会话组合根，提供默认依赖装配入口。
 */
class SarSessionCompositionRoot {
 public:
  /**
   * @brief 用默认依赖装配会话组合产物。
   * @param[in] config 初始会话配置。
   * @return 持有 pipeline 与 controller 的组合产物。
   */
  static SarSessionComposition ComposeDefault(const config::SarSessionConfig& config);
};

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SRC_SAR_SESSION_SAR_SESSION_COMPOSITION_ROOT_H_
