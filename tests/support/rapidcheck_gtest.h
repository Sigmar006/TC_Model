/**
 * @file rapidcheck_gtest.h
 * @brief 把 RapidCheck 属性检查的结果报告为 GoogleTest 断言。
 *
 * 为什么仓库自带这一层：ConanCenter 的 rapidcheck 配方只执行上游 install，
 * 而上游只安装库、include/、export config 与 .pc —— extras/ 不在安装范围内，
 * 因此属性测试拿不到 rapidcheck/gtest.h。本文件按已安装包的公开 API 重新实现
 * 该适配（与上游 extras/gtest/include/rapidcheck/gtest.h 的 checkGTest 同构），
 * 并额外把可复现的 RC_PARAMS 串写进失败消息。
 *
 * 与上游的差异：只提供 RC_GTEST_PROP，不提供 fixture / typed-fixture 变体。
 * 需要夹具时，在属性体内显式构造被测对象。
 *
 * 用法见 .claude/skills/adversarial-testing/SKILL.md。
 */
#ifndef ONEQ_TEST_RAPIDCHECK_GTEST_H_
#define ONEQ_TEST_RAPIDCHECK_GTEST_H_

#include <gtest/gtest.h>

#include <iostream>
#include <sstream>
#include <string>
#include <utility>

#include "rapidcheck.h"
#include "rapidcheck/detail/Configuration.h"
#include "rapidcheck/detail/Results.h"
#include "rapidcheck/detail/TestMetadata.h"

namespace oneq_test {
namespace detail {

// 运行一次属性检查：成功且分布非空时打印分布；失败时以 FAIL() 报出反例。
// RapidCheck 的 RC_ASSERT 以异常传播失败，但由 checkTestable 内部捕获并转换为
// 结果值，因此失败不会中止测试进程，也不会吞掉同一二进制里的后续用例。
template <typename Testable>
void CheckProperty(Testable &&testable) {
  const ::testing::TestInfo *const test_info =
      ::testing::UnitTest::GetInstance()->current_test_info();

  rc::detail::TestMetadata metadata;
  metadata.id = std::string(test_info->test_suite_name()) + "/" + test_info->name();
  metadata.description = test_info->name();

  const rc::detail::TestResult result =
      rc::detail::checkTestable(std::forward<Testable>(testable), metadata);

  if (result.template is<rc::detail::SuccessResult>()) {
    const rc::detail::SuccessResult &success =
        result.template get<rc::detail::SuccessResult>();
    if (!success.distribution.empty()) {
      std::cout << "- " << metadata.description << std::endl;
      rc::detail::printResultMessage(result, std::cout);
      std::cout << std::endl;
    }
    return;
  }

  // 失败消息必须自带复现手段，否则反例不可重放。
  std::ostringstream message;
  message << metadata.id << ": ";
  rc::detail::printResultMessage(result, message);
  message << "\n复现：RC_PARAMS='"
          << rc::detail::configToMinimalString(rc::detail::configuration()) << "'";
  FAIL() << message.str() << std::endl;
}

}  // namespace detail
}  // namespace oneq_test

// 若上游 extras 头在将来可用，让它优先定义，避免重复定义冲突。
#ifndef RC_GTEST_PROP
#define RC_GTEST_PROP(TestSuite, TestName, ArgList)                          \
  void oneq_test_prop_impl_##TestSuite##_##TestName ArgList;                 \
  TEST(TestSuite, TestName) {                                                \
    ::oneq_test::detail::CheckProperty(                                      \
        &oneq_test_prop_impl_##TestSuite##_##TestName);                      \
  }                                                                          \
  void oneq_test_prop_impl_##TestSuite##_##TestName ArgList
#endif

#endif  // ONEQ_TEST_RAPIDCHECK_GTEST_H_
