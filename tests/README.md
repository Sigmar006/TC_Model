# tests 目录约定

## 分层规则
- `unit/`: 单元测试（含原压力测试）。
- `integration/`: 跨组件或会话级集成测试。
- `replay/`: trace/replay、codec roundtrip 与 divergence 测试。
- `contract/`: 公共 API/头文件稳定性与边界约束测试。
- `performance/`: 性能与质量阈值测试。
- `compatibility/`: 脚本式编译兼容性探针，不属于进程内 GoogleTest 分区。
- `consumer/`: 安装后消费者样例与安装验证。

## 归档规则
- 新增测试文件禁止放在 `tests/` 根目录，必须放入上述分层目录。
- 域归属由第二层域目录（如 `unit/airborne_radar/`、`replay/sar/`）编码；文件名前缀与域目录保持一致。
- 共享测试辅助代码只放 `fixtures/` 或 `mocks/`，避免复制。

## 类型与业务域

进程内测试的有效目录组合由 layout guard 和分区注册共同约束：

| 类型 | 允许的业务域 |
|---|---|
| `unit` | `common`、`examples`、`airborne_radar`、`electronic_surveillance_radar`、`electro_optical_sensor`、`sbirs_sensor`、`sar`、`flight_dynamic` |
| `integration` | `airborne_radar`、`electro_optical_sensor`、`electronic_surveillance_radar`、`sbirs_sensor`、`cross_domain` |
| `replay` | `common`、`airborne_radar`、`electro_optical_sensor`、`electronic_surveillance_radar`、`sar`、`sbirs_sensor` |
| `contract` | `public_api`、`airborne_radar`、`electro_optical_sensor`、`electronic_surveillance_radar`、`sar`、`sbirs_sensor` |
| `performance` | `sar` |

`compatibility/` 当前使用 `public_api` 与 `sar` domain；`consumer/` 分两个子区：
`consumer/` 根为**安装后消费者样例独立工程**（consumer CMake 单独管理，CI 在安装后
构建），`consumer/batch_validation/` 为**树内批量场景验证框架**（独立可执行程序，
由 `tests/CMakeLists.txt` 的 `add_subdirectory` 纳入，见下方注记）。
新增 type/domain 必须同时更新 layout guard、分区 CMake、本文和 `docs/common/contract.md`。

## 命名规则
- 所有测试文件统一命名：`{domain}_{descriptive_name}_test.cpp`。
  - `domain` 取 `common` \| `airborne_radar` \| `electronic_surveillance_radar` \| `electro_optical_sensor` \| `sar` \| `sbirs_sensor` \| `flight_dynamic`，与所在域目录对齐。
  - `contract/` 下的跨域公共 API 测试不带域前缀。
- 文件名中**不**再包含层级后缀（`_unit_`、`_integration_`、`_stress_`），层级由目录路径和 CTest label 编码。
- 示例：`airborne_kalman_filter_test.cpp`、`esr_kdtree_clusterer_test.cpp`、`eos_pipeline_test.cpp`。

## CTest 运行建议
- 全量：`ctest --preset llvm-ninja-debug-local -Q --output-on-failure`
- 仅单元：`ctest --preset llvm-ninja-debug-local -Q --output-on-failure -L unit`
- 仅集成：`ctest --preset llvm-ninja-debug-local -Q --output-on-failure -L integration`
- 仅 replay：`ctest --preset llvm-ninja-debug-local -Q --output-on-failure -L replay`
- 仅契约：`ctest --preset llvm-ninja-debug-local -Q --output-on-failure -L contract`
- PR 关键路径：`ctest --preset llvm-ninja-debug-local -Q --output-on-failure -L ci_required`
- 兼容性探针：`ctest --preset llvm-ninja-debug-local -Q --output-on-failure -L compatibility`
- 五模块专项序列：`ctest --preset llvm-ninja-release-local -Q --output-on-failure -L batch_validation`

## CMake 注册结构

`tests/CMakeLists.txt` 只编排测试生命周期；具体注册按职责位于 `tests/cmake/`：

- `TestSupport.cmake`：分区注册 API、依赖接线与测试源注册表。
- `TestTargets.cmake`：各层 aggregate build targets。
- `partitions/{Unit,Replay,Integration,Contract,Performance}.cmake`：常规 type × domain 分区。
- `FlightDynamicPartitions.cmake`：FD stable unit 与 known-limit 分区，以及 JSBSim 接线。
- `CompatibilityChecks.cmake` / `ContractGuards.cmake`：脚本兼容性检查与源码、文档、CMake 守护。

新增 CTest 入口应放入其拥有的 type × domain 分区；不得重新把专项 filter 或 guard 堆回根入口。

唯一的非 GoogleTest 业务场景入口是 `tests/consumer/batch_validation` 拥有的五个 sequence
可执行套件（验证框架，2026-08-10 由 `examples/batch_validation` 迁入）。它们以
`batch_validation::<domain>` 注册并携带 `batch_validation` 与 domain label，不产生新的
`tests/` 源码 type，也不把 199 个 sweep 重复纳入 CTest。该框架单向依赖 examples 层
共享便利层（`config_loaders` / `json_reader`）与 `examples/basic_config/` 配置 JSON。

每个 `*_test.cpp` 必须只注册到一个分区。`TestRegistry.cmake` 会在 configure 时
拒绝 orphan 或重复归属；不要用重复编译让同一源文件同时承担 unit、replay 或
integration 语义。`1q_unit_tests` 等是 aggregate build target，不应被脚本当作
固定二进制路径调用。

## AR include-direction 护栏
- `airborne_include_direction_guard` 强制 `src/airborne_radar/signal/**` 不得 include `airborne_radar/runtime/**` 或 `airborne_radar/session/**`。
- `airborne_include_direction_guard` 强制 `src/airborne_radar/environment/**` 不得 include `airborne_radar/decision/**` 或 `airborne_radar/signal/**`。
- 当前阶段把“core 只能依赖抽象接口”按 `runtime + session` 落地为告警清单，不作为失败条件。
- 组合根豁免：`src/airborne_radar/session/ArSessionCompositionRoot.cpp` 允许依赖具体实现用于默认装配。
