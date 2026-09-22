/**
 * @file ValidationUtils.h
 * @brief 定义校验链路复用的有限值、问题构造与严重级别扫描工具。
 */

#ifndef COMMON_VALIDATION_VALIDATION_UTILS_H_
#define COMMON_VALIDATION_VALIDATION_UTILS_H_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace oneq {
namespace common {
namespace validation {

/**
 * @brief 判断数值是否为有限数。
 * @tparam T 数值类型。
 * @param[in] value 输入数值。
 * @return 输入为有限数时返回 `true`。
 */
template <typename T>
inline bool IsFinite(T value) {
  return std::isfinite(value) != 0;
}

/**
 * @brief 判断数值是否为有限的 [0, 1] 比例值。
 * @tparam T 数值类型。
 * @param[in] value 输入数值。
 * @return 有限且落在 [0, 1] 时返回 true。
 */
template <typename T>
inline bool IsRatio01(T value) {
  return IsFinite(value) && value >= static_cast<T>(0) && value <= static_cast<T>(1);
}

/**
 * @brief 构造定位信息（定位域 + 实体索引）。
 * @tparam LocationT 定位结构类型。
 * @tparam LocationKindT 定位域枚举类型。
 * @param[in] kind 定位域。
 * @param[in] entity_index 实体索引。
 * @return 填充好的定位结构。
 */
template <typename LocationT, typename LocationKindT>
inline LocationT MakeLocation(LocationKindT kind, std::size_t entity_index) {
  LocationT location;
  location.kind = kind;
  location.entity_index = entity_index;
  return location;
}

/**
 * @brief 构造带定位与字段信息的结构化校验问题。
 * @tparam IssueT 问题结构类型。
 * @tparam LocationT 定位结构类型。
 * @tparam SeverityT 严重级别枚举类型。
 * @tparam CodeT 编码枚举类型。
 * @tparam LocationKindT 定位域枚举类型。
 * @param[in] severity 严重级别。
 * @param[in] code 结构化问题编码。
 * @param[in] location_kind 定位域。
 * @param[in] entity_index 实体索引。
 * @param[in] field 关联字段名。
 * @param[in] message 面向调用方的简短说明。
 * @return 构造后的问题结构。
 */
template <typename IssueT, typename LocationT, typename SeverityT, typename CodeT,
          typename LocationKindT>
inline IssueT MakeLocatedIssue(SeverityT severity, CodeT code, LocationKindT location_kind,
                               std::size_t entity_index, const std::string& field,
                               const std::string& message) {
  IssueT issue;
  issue.severity = severity;
  issue.code = code;
  issue.location = MakeLocation<LocationT>(location_kind, entity_index);
  issue.field = field;
  issue.message = message;
  return issue;
}

/**
 * @brief 构造带索引字段的结构化校验问题。
 * @tparam IssueT 问题结构类型。
 * @tparam SeverityT 严重级别枚举类型。
 * @tparam CodeT 编码枚举类型。
 * @tparam IndexField 问题结构中的索引字段成员指针。
 * @param[in] severity 严重级别。
 * @param[in] code 结构化问题编码。
 * @param[in] index 与问题关联的索引值。
 * @param[in] message 面向调用方的简短说明。
 * @return 构造后的问题结构。
 */
template <typename IssueT, typename SeverityT, typename CodeT, std::size_t IssueT::* IndexField>
inline IssueT MakeIndexedIssue(SeverityT severity, CodeT code, std::size_t index,
                               const std::string& message) {
  IssueT issue;
  issue.severity = severity;
  issue.code = code;
  issue.*IndexField = index;
  issue.message = message;
  return issue;
}

/**
 * @brief 构造带索引字段的结构化校验问题。
 * @tparam IssueT 问题结构类型。
 * @tparam SeverityT 严重级别枚举类型。
 * @tparam CodeT 编码枚举类型。
 * @tparam IndexField 问题结构中的索引字段成员指针。
 * @param[in] severity 严重级别。
 * @param[in] code 结构化问题编码。
 * @param[in] index 与问题关联的索引值。
 * @param[in] message 面向调用方的简短说明。
 * @return 构造后的问题结构。
 */
template <typename IssueT, typename SeverityT, typename CodeT, std::int32_t IssueT::* IndexField>
inline IssueT MakeIndexedIssue(SeverityT severity, CodeT code, std::int32_t index,
                               const std::string& message) {
  IssueT issue;
  issue.severity = severity;
  issue.code = code;
  issue.*IndexField = index;
  issue.message = message;
  return issue;
}

/**
 * @brief 判断问题列表中是否包含指定严重级别。
 * @tparam IssueListT 问题列表类型。
 * @tparam SeverityT 严重级别枚举类型。
 * @tparam SeverityField 问题结构中的严重级别字段成员指针。
 * @param[in] issues 问题列表。
 * @param[in] expected 期望严重级别。
 * @return 存在任意一条匹配问题时返回 `true`。
 */
template <typename IssueListT, typename SeverityT,
          SeverityT IssueListT::value_type::* SeverityField>
inline bool HasSeverity(const IssueListT& issues, SeverityT expected) {
  for (typename IssueListT::const_iterator it = issues.begin(); it != issues.end(); ++it) {
    if ((*it).*SeverityField == expected) {
      return true;
    }
  }
  return false;
}

/**
 * @brief 判断问题列表中是否包含"输入校验错误"条目（统一问题列表模型，规则 14）。
 *
 * 各模块 `HasValidationError` 的统一实现（RED-1 收敛）：`phase == kInputValidation`
 * 且 `severity == kError` 即判定为阻断性输入校验错误。成员指针以运行期参数传入，
 * 枚举类型由成员指针自动推导，调用方无需显式模板实参。
 *
 * @tparam IssueT 问题条目类型（各模块 `XxxIssue`）。
 * @tparam PhaseT 来源阶段枚举类型（各模块 `XxxIssuePhase`）。
 * @tparam SeverityT 严重级别枚举类型（各模块 `XxxIssueSeverity`）。
 * @param[in] issues 问题列表。
 * @param[in] phase_field 问题结构中的 phase 字段成员指针。
 * @param[in] severity_field 问题结构中的 severity 字段成员指针。
 * @return 存在任意一条 `kInputValidation + kError` 条目时返回 `true`。
 */
template <typename IssueT, typename PhaseT, typename SeverityT>
inline bool HasValidationPhaseError(const std::vector<IssueT>& issues,
                                    PhaseT IssueT::* phase_field,
                                    SeverityT IssueT::* severity_field) {
  for (const IssueT& issue : issues) {
    if ((issue.*phase_field) == PhaseT::kInputValidation &&
        (issue.*severity_field) == SeverityT::kError) {
      return true;
    }
  }
  return false;
}

}  // namespace validation
}  // namespace common
}  // namespace oneq

#endif  // COMMON_VALIDATION_VALIDATION_UTILS_H_
