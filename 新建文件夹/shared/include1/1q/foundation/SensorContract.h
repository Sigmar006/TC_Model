/**
 * @file SensorContract.h
 * @brief 跨域传感器会话形状契约：编译期锚定各域 Session 的 Step/StepWithResult 签名。
 *
 * 四个传感器域（airborne_radar / electro_optical_sensor /
 * electronic_surveillance_radar / sar）各自维护领域专用的 Session 类型，
 * 不引入跨域公共基类（领域语义差异大，公共基类会压平语义）。
 * 但它们的对外门面遵循同一形状契约：
 *
 *   OutputFrame   Step(const CycleInput&);
 *   CycleResult   StepWithResult(const CycleInput&);
 *
 * 本头提供编译期锚定宏，在任一域 Session 类定义后展开，用 static_assert
 * 钉死上述签名形状。一旦某域把返回类型或参数类型改歪，该域立即编译失败，
 * 防止“伪对称”在无人察觉时漂移成“真不对称”。
 *
 * 全部基于 C++11（static_assert / std::is_same / decltype / std::declval），
 * 不引入运行时成本，不改变任何类型定义。本头可被 include/1q/ 公共头安全包含，
 * 符合“公共头守 C++11 子集”约束。
 */

#ifndef ONEQ_FOUNDATION_SENSOR_CONTRACT_H_
#define ONEQ_FOUNDATION_SENSOR_CONTRACT_H_

#include <type_traits>  // std::is_same
#include <utility>      // std::declval

namespace oneq {
namespace foundation {
namespace contract {

/**
 * @brief 编译期校验：某 SessionT 的 Step(const InputT&) 返回类型恰为 OutputFrameT。
 *
 * 返回 bool 而非直接 static_assert，是为了让宏能在“值不匹配”时给出
 * 可读的 static_assert 信息，而非嵌套模板错误。
 */
template <typename SessionT, typename InputT, typename OutputFrameT>
struct StepReturnMatches {
  // NOTE: 不使用 ONEQ_API（展开为 __declspec(dllimport)），
  // 因为这些模板是纯编译期工具，非 DLL 导出符号。
  // VS2015/MSVC 不接受 static __declspec(dllimport) const bool 的类内初始化。
  static const bool value = std::is_same<
      decltype(std::declval<SessionT>().Step(std::declval<const InputT&>())),
      OutputFrameT>::value;
};

/**
 * @brief 编译期校验：某 SessionT 的 StepWithResult(const InputT&) 返回类型恰为 ResultT。
 */
template <typename SessionT, typename InputT, typename ResultT>
struct StepWithResultReturnMatches {
  static const bool value = std::is_same<
      decltype(std::declval<SessionT>().StepWithResult(std::declval<const InputT&>())),
      ResultT>::value;
};

}  // namespace contract
}  // namespace foundation
}  // namespace oneq

/**
 * @brief 在某域 Session 类定义之后展开，锚定其 Step/StepWithResult 签名形状。
 *
 * 用法（置于命名空间闭合之后、文件末尾附近）：
 *
 *   ONEQ_SENSOR_SESSION_CONTRACT(EosSession, EosCycleInput,
 *                                EosOutputFrame, EosCycleResult);
 *
 * SAR 产品层例外（规则 15c/15e）：Step 返回 SarCycleProduct（元数据 + 聚焦图像），
 * 而非裸 *OutputFrame；产品由同一份周期记录移动取出，不是第二份副本。
 *
 * 注意：宏参数需为当前命名空间下可见的类型名；若跨命名空间调用，
 * 请传入全限定名。该宏不定义符号，仅产生两条 static_assert，
 * 在优化构建中被完全消除。
 */
#define ONEQ_SENSOR_SESSION_CONTRACT(SessionT, InputT, OutputFrameT, ResultT)                 \
  static_assert(                                                                              \
      ::oneq::foundation::contract::StepReturnMatches<SessionT, InputT, OutputFrameT>::value, \
      #SessionT "::Step(const " #InputT "&) must return " #OutputFrameT                       \
      " — 跨域传感器会话形状契约被破坏，见 foundation/SensorContract.h");                       \
  static_assert(                                                                              \
      ::oneq::foundation::contract::StepWithResultReturnMatches<SessionT, InputT, ResultT>::value, \
      #SessionT "::StepWithResult(const " #InputT "&) must return " #ResultT                  \
      " — 跨域传感器会话形状契约被破坏，见 foundation/SensorContract.h")

#endif  // ONEQ_FOUNDATION_SENSOR_CONTRACT_H_
