/**
 * @file validation_types.h
 * @brief 定义跨模块共享的输入校验基础类型。
 */

#ifndef ONEQ_FOUNDATION_VALIDATION_TYPES_H_
#define ONEQ_FOUNDATION_VALIDATION_TYPES_H_

#include <cstddef>

#include "1q/api.hpp"

namespace oneq {
namespace foundation {

/**
 * @brief ValidationSeverity 表示校验结果严重级别。
 */
enum class ONEQ_API ValidationSeverity {
  kInfo = 0, /**< 仅提示语义，不阻断执行 */
  kWarning,  /**< 调用方应显式关注的潜在问题 */
  kError     /**< 明确的错误输入，建议阻断执行 */
};

/**
 * @brief ValidationLocationKind 表示校验问题定位域。
 */
enum class ONEQ_API ValidationLocationKind {
  kGlobal = 0,  /**< 与具体域或实体无关 */
  kPlatform,    /**< 平台位姿域 */
  kEnvironment, /**< 环境输入域 */
  kSceneEntity  /**< 场景实体域（需配合 entity_index） */
};

/**
 * @brief ValidationLocation 描述校验问题定位信息。
 */
struct ONEQ_API ValidationLocation {
  ValidationLocationKind kind{ValidationLocationKind::kGlobal}; /**< 定位域 */
  std::size_t entity_index{static_cast<std::size_t>(-1)}; /**< 场景实体索引（仅 kSceneEntity 时使用，默认无效值） */
};

}  // namespace foundation
}  // namespace oneq

#endif  // ONEQ_FOUNDATION_VALIDATION_TYPES_H_
