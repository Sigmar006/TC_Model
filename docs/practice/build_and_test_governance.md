# 构建与测试治理

Status: active
Last-reviewed: 2026-08-20
Authority: build infrastructure, test infrastructure

本文承载仓库的 CMake 工程边界与测试架构组织规则。这些是工程基础设施规则，不是业务模块契约。
所有模块都必须遵守的跨模块契约见 `docs/common/contract.md`；有 Session 的传感器模块的统一会话
契约见 `docs/common/session_contract.md`。

## CMake 工程边界

1. 顶层 `CMakeLists.txt` 只编排项目生命周期；`src/CMakeLists.txt` 只装配最终产品；
   每个模块 CMake 文件拥有自身 component target、源文件、直接依赖和 replay schema。
   不得恢复中心式 module link matrix 或 schema 注册表。
2. 编译、链接、Unity、PCH、coverage 等策略必须以 target 为作用域；不得以
   `add_compile_options()` 或 `add_link_options()` 向目录树广播项目私有选项。
3. Unity Build 暴露的匿名命名空间重定义必须在源文件属性上显式声明
   `SKIP_UNITY_BUILD_INCLUSION`，并说明冲突原因；不得为此把模块私有 helper 扩大为
   `src/common` 公共设施。
4. Windows/MSVC 支持不因 preset 或 public-header 编译通过而成立。该平台将使用
   仓库拥有的 shell bootstrap 从 GitHub 获取锁定版本依赖；脚本必须固定版本与提交
   标识、校验下载内容并产出 CMake 可消费的 imported targets。只有真实 Windows
   configure、build、install 和外部 consumer job 均通过后，才可宣称 project build support。
   当前 Windows 主线为 v141 预设（`VisualStudio.15.0-amd64`，VS2026 generator +
   v141 工具集，本机 `scripts/1q.sh` 下 bootstrap/configure/build/test/install 全链经
   build preset 驱动，示例层加 `-DENABLE_EXAMPLES=ON` 亦可构建）；VS2015 C++14
   预设（`VisualStudio.14.0-amd64`，无测试）与 no-Conan 模式
   （`VisualStudio.14.0-amd64-none`，含 `scripts/win/fetch_third_party.bat`）仍属未验收脚手架，
   不改变上述支持契约——CI 尚无 Windows job，"已支持 Windows"的宣称仍以真实
   Windows runner 上的全链证据为准。
5. MSVC 配置语义（`cmake/compilers/CompilerMSVC.cmake`，2026-08-20 修订）：**Release
   为真发布档**——/O2 /Ob2 /Oi + /OPT:REF /OPT:ICF，**不生成任何调试产物**
   （无 /Z7、无 PDB）；**RelWithDebInfo 是 /O2 /Ob2 /Oi + /Z7 + /DEBUG:FULL 完整
   符号**的性能可调试档。需要调试符号或变量级可调试性的集成方（如每周期执行
   RDA 聚焦的实时消费工程排障）应链接 RelWithDebInfo 产物。历史版本曾让 Release
   保持 /Od /Ob0 + 全套调试信息（"生产可调试"单档设计），该语义已废止。

## 测试架构

测试代码按"测试类型 × 业务域"组织。`*_test.cpp` 必须位于
`tests/<type>/<domain>/`，其中 type 为 `unit`、`integration`、`replay`、
`contract` 或 `performance`；`compatibility` 存放脚本式兼容性探针，`consumer`
保留为安装后消费者验证，二者不混入进程内 GoogleTest 分区。

规则：

1. 每个 `*_test.cpp` 只能有一个类型、一个业务域、一个编译分区。`TestRegistry.cmake`
   在 configure 时必须拒绝 orphan 和重复归属；不得用 allowlist 长期保留重复编译。
2. 新增进程内测试必须通过 `oneq_add_test_partition()` 注册；不得重新引入按
   GoogleTest suite/case 的 CMake filter。`1q_unit_tests` 等旧名称只可以作为
   aggregate build target，不能再被当作稳定的测试可执行文件路径。
3. 每个 CTest 项必须携带 type 与 domain label；执行策略使用额外 label 表达。
   `ci_required` 是 PR 的阻断关键路径，完整 `unit` 分区同样阻断；`known_limit`
   与 `performance` 不得借重构被静默纳入该门禁。`replay_fast` 仅是 replay 的
   执行策略 label，不是另一种测试类型。
4. `flight_dynamic` 只在目标依赖和执行策略上是特例：稳定源属于
   `unit::flight_dynamic`，边界/性能源属于 `known_limit::flight_dynamic`；不得
   为它恢复独立的 suite filter 体系。
5. 覆盖率 preset 可构建专用 mapping runner，但该 runner 必须在 CTest 中禁用；
   profile 数据仍来自真实 type × domain 分区，避免重复执行同一测试。
6. `test_layout_guard` 负责 type/domain 布局与 CMake filter 禁令；新增 type 或
   domain 前，必须同批更新 guard、分区注册、README 和相关 contract 测试。
7. `tests/consumer/batch_validation` 拥有的端到端可执行程序不是 `*_test.cpp`，不新增 `tests/`
   源码 type，也不进入 GoogleTest 分区。其 sequence 子集可在本框架自身 CMake 中注册为
   `batch_validation::<domain>`，必须同时携带 `batch_validation` 与 domain label；181 个 sweep
   只由显式 `--suite sweep|all` 运行，不得在 CTest 中重复注册。
