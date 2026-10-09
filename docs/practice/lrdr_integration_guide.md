# LRDR 集成指南（使用 1q.lib）

Status: active
Last-reviewed: 2026-09-21
Scope: Windows / Visual Studio 消费端，链接已交付的静态库 `1q.lib`。

本文约定：集成方使用同批次公共头文件与 `1q.lib`，通过公开 C++ API 调用 LRDR。
LRDR 对应模块名 `remote_identification_radar`（RIR），入口类为
`remote_identification_radar::session::RirSession`。

## 1. 交付包与集成边界

先核对交付材料：

| 材料 | 要求 |
| --- | --- |
| `lib/1q.lib` | 包含 LRDR 实现的主静态库 |
| `include/1q/` | 与主库同批次的完整公共头，保留目录结构；包含交付的生成导出头 |
| 配套依赖库 | 交付清单要求的 SQLite、zlib 等，架构和运行库配置一致 |
| 配置与数据库 | LRDR 配置模板、识别特征数据库及其版本说明 |
| 构建信息 | 工具集、x64/x86、Release/Debug、CRT、源码版本和验收日志开关 |

**只拿到 `1q.lib` 文件不足以完成接入。** 静态库的第三方依赖未必合并在主库中，必须按该交付批次的链接清单配置。
历史 vendor 交付的配套库名称为 `sqlite3_vendor.lib`、`zlibstatic.lib`；实际名称以收到的包为准。

建议消费目录：

```text
consumer/
  sdk/
    include/1q/...
    lib/1q.lib
    lib/<配套依赖库>
  config/...
  data/<识别特征数据库>
  src/main.cpp
```

集成方负责工程配置、输入装配、会话生命周期和结果消费。
扫描、检测、跟踪、识别等实现由 `1q.lib` 提供。消费工程无需编译
`RirSession.cpp`、`src/common/`、回放生成源码，也无需加入仓库私有 `src` 包含路径。

## 2. Visual Studio 工程配置

先选择与交付库对应的配置和平台，再设置以下属性。以下路径假定工程根目录下有 `sdk`。

| 属性页 | 配置值 |
| --- | --- |
| C/C++ → 常规 → 附加包含目录 | `$(ProjectDir)sdk\include;%(AdditionalIncludeDirectories)` |
| C/C++ → 预处理器 → 预处理器定义 | `ONEQ_STATIC_DEFINE;%(PreprocessorDefinitions)` |
| 链接器 → 常规 → 附加库目录 | `$(ProjectDir)sdk\lib;%(AdditionalLibraryDirectories)` |
| 链接器 → 输入 → 附加依赖项 | `1q.lib`、配套库文件名，并保留 `%(AdditionalDependencies)` |
| C/C++ → 代码生成 → 运行库 | 与交付一致；仓库默认 Release `/MD`、Debug `/MDd` |
| 配置管理器 → 平台 | 与库一致；当前 Windows 主线为 x64 |
| 常规 → 平台工具集 | 按交付构建信息选择兼容且经过验证的工具集 |

若交付的是上述 vendor 包，附加依赖项可写为：

```text
1q.lib;sqlite3_vendor.lib;zlibstatic.lib;%(AdditionalDependencies)
```

`ONEQ_STATIC_DEFINE` 必须对使用 1q 公共头的编译单元生效，包括相关 PCH 的编译。
不要通过定义 DLL 导出宏来代替静态库宏。

不能仅凭文件名判断配置：仓库默认 Debug 名称为 `1qd.lib`，但交付方可能重命名。
以实际构建记录为准，不将 Release 库直接用于 `/MDd` 工程。
公共接口含 STL 类型，编译器、CRT 和迭代器调试配置不匹配也可能导致 ABI 问题。
不要通过忽略默认库或强制链接掩盖这些冲突。

## 3. 首次编译链接检查

先建一个最小控制台工程，只验证公开会话的构造和析构：

```cpp
#include "1q/remote_identification_radar/session/RirSession.h"

int main() {
  remote_identification_radar::session::RirSession radar;
  return 0;
}
```

按第 2 节设置编译和链接选项，构建并运行。通过只表示该调用路径的编译、链接和启动成功，
不表示扫描或识别功能已验收。下一步必须运行实际周期和已知场景。

若消费工程由 CMake 管理，可显式导入原始静态库；以下示例仅适用于已确认
**Release、x64、/MD，且具有所列 vendor 配套库**的交付包：

```cmake
cmake_minimum_required(VERSION 3.16)
project(lrdr_consumer LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
set(ONEQ_SDK "${CMAKE_CURRENT_SOURCE_DIR}/sdk" CACHE PATH "1q SDK directory")

add_library(oneq_sdk STATIC IMPORTED)
set_target_properties(oneq_sdk PROPERTIES
    IMPORTED_LOCATION "${ONEQ_SDK}/lib/1q.lib"
    INTERFACE_INCLUDE_DIRECTORIES "${ONEQ_SDK}/include"
    INTERFACE_COMPILE_DEFINITIONS "ONEQ_STATIC_DEFINE")

add_executable(lrdr_consumer src/main.cpp)
target_link_libraries(lrdr_consumer PRIVATE oneq_sdk
    "${ONEQ_SDK}/lib/sqlite3_vendor.lib"
    "${ONEQ_SDK}/lib/zlibstatic.lib")
```

使用匹配的 VS generator/工具集配置为 x64，并以 Release 构建；实际库名或依赖不同应按交付清单调整。
这段配置不提供 Debug 库映射。只有交付了完整 CMake 包配置时才使用 `find_package`，
仅有 `.lib` 和头文件并不意味着存在 `1qConfig.cmake`。

## 4. 初始化与周期调用

### 4.1 初始化一次

1. 根据任务填写 `config::RirSessionConfig`。当前公开配置包括 `hardware`、`orientation`、
   `mission`、`policy`、`environment` 和顶层电源、日志设置；以交付头文件为准。
2. 调用 `config::ValidateRirSessionConfig(cfg)` 并处理返回诊断。
3. 配置满足运行要求后，调用 `RirSession::Create(cfg)`，保存会话至仿真结束。
4. 多设备场景调用 `SetSensorEntityId` 设置设备标识。

`Create` 是不校验配置的入口；`CreateWithDiagnostics` 即使报告问题也会返回会话。
不能将“对象构造成功”当作“配置有效”。配置 JSON 的读取和字段映射属于宿主适配层，
`RirSession` 不直接接受 JSON 路径。参考 [LRDR 配置模板](../../examples/basic_config/remote_identification_radar.json)。

### 4.2 每周期输入

| `RirCycleInput` 字段 | 约定 |
| --- | --- |
| `input_cycle_index` | 非零周期号，由宿主管理执行顺序 |
| `dt_sec` | 有限且大于零的周期步长，秒 |
| `sim_time_sec` | 仿真绝对时刻，秒，与步长保持一致 |
| `platform_position` | 雷达站点/平台 ECEF 位置，米，必填 |
| `scene_targets` | 站点局部 ENU 目标、运动事实及识别特征；字段见公开 `RirSceneTypes.h` |
| `rf_scene` | 外部 RF 场景，可空，不包括本机发射 |

目标坐标必须先按站点坐标系转换，不能将 ECEF 直接填到 ENU 字段。
核对度/弧度、米/千米与时间单位。完整适配参考 [RIR 示例组件](../../examples/components/rir_sensor_component.cpp)。

以下为宿主中的调用片段，`cfg` 和 `input` 由宿主提供，不是独立运行示例：

```cpp
namespace rir = remote_identification_radar;

// 初始化阶段：cfg 已完成校验。
auto radar = rir::session::RirSession::Create(cfg);

// 周期循环内执行：input 已完成本周期装配。
const auto result = radar.StepWithResult(input);
if (result.status == rir::session::RirCycleStatus::kCompleted) {
  // 消费 result.output_frame。
} else {
  // 记录 result.status、result.issues、result.abort_reason。
}
```

每周期只推进一次会话，不要每帧重建。结果有效性以 `status` 为准：
`kCompleted` 表示周期完成，也可能没有检测或识别结果；非完成周期不得复用上周期输出冒充当前结果。

### 4.3 运行调整与下游

使用 `TryApplyRuntimeConfig` 提交运行期补丁。true 表示已暂存，
在下一个成功执行周期边界提交；false 也可能表示没有变更。
融合应在雷达本周期执行完成后消费当前量测。接线参考
[甲方集成验证清单](customer_integration_verification.md)，先验单会话，再接下游链路。

## 5. 数据库与日志

- 将同批次识别特征数据库部署到可访问路径，并在配置中正确引用。相对路径取决于进程工作目录，
  IDE 启动与部署启动可能不同；首次联调建议使用明确的绝对路径。
- 验收日志能力由构建 `1q.lib` 时的 `ONEQ_ENABLE_RIR_ACCEPTANCE_LOG` 决定。
  给消费工程增加这个宏不能打开一个已关闭该能力的库，需要交付方提供对应版本。
- 当前配置的 `acceptance_log_enabled` 控制运行期开关，`acceptance_log_output_directory`
  可指定输出目录。只使用交付头文件支持的字段，不能单独更新结构体头文件而保留旧库。
- 功能和接口证据参考 `RadarOutput.log`、`RadarInternalInterface.log`，
  扫描证据参考 `Rir_scan_pattern.csv`。准备可写目录，核对实际开关和路径。

## 6. BuildScanPattern 链接错误

### 6.1 集成方先确认

`BuildScanPattern` 是库内部扫描函数，当前仓库定义在内部头
`src/common/radar/ScanScheduleRuntime.h`，属于 `oneq::common::radar`，为 inline 实现。
正常的 `1q.lib` 使用方不需要包含该内部头，也不需要自行实现该函数。

1. 保存完整 LNK2019/LNK2001 错误，包括函数签名、引用位置和 `xxx.lib(xxx.obj)`。
2. 在“链接器 → 命令行 → 其他选项”临时增加 `/VERBOSE:LIB`，重新链接，
   确认真正使用的 `1q.lib` 绝对路径。清理旧库搜索路径及重复配置。
3. 确认公共头、主库、配套库来自同批交付，并确认静态库宏、架构、配置和 CRT。
4. 清理宿主中间产物/PCH 后重新生成，保留完整链接输出。

| 错误来源或现象 | 后续处理 |
| --- | --- |
| `1q.lib(...obj)` 引用 `BuildScanPattern` 未解析 | 核对实际库后，由库提供方检查内部符号定义和归档完整性，重建交付 |
| 宿主 `.obj` 直接引用内部 `BuildScanPattern` | 检查宿主是否引入了内部声明、私有头或复制的实现；恢复公开会话接入边界 |
| `__imp_...RirSession...` 未解析 | 优先核对 `ONEQ_STATIC_DEFINE` 是否在头文件/PCH 编译时生效 |
| `sqlite3_*` 或 zlib 符号未解析 | 检查配套依赖库是否链接、名称和配置是否正确 |
| LNK2038 或平台冲突 | 检查 CRT、迭代器调试配置及 x64/x86，不强制忽略冲突 |

### 6.2 库提供方处理

若已确认由正确的 `1q.lib` 内部对象引用该函数，库提供方应：

1. 核对编译相关对象时真正命中的 `ScanScheduleRuntime.h`，确认有完整函数体，
   命名空间和参数一致，没有旧声明或同名头污染。
2. 检查相关对象及符号是否被完整装入最终库，排除增量构建和跨版本产物混用。
3. 在干净构建目录重建，并用仅依赖交付公共头、主库和配套库的外部工程验证
   `RirSession` 构造和 `StepWithResult` 调用的链接。
4. 重新交付一致的库、头文件和构建记录，由集成方替换后再次验证。

**静态库归档成功不等于最终可执行文件能链接。** 必须包含外部消费端的链接验证。
消费端补一份内部扫描实现、给函数加空实现或删除扫描调用，都不能作为交付修复。
当前尚无完整错误行和实际交付库验证记录，不能将某个内部构建原因认定为已确认根因。

## 7. 运行验收与问题提交

按以下顺序验收：

1. 最小控制台程序编译、链接、启动通过。
2. 使用真实配置执行 `StepWithResult`，状态和诊断符合输入。
3. 按 [站点识别场景说明](../../examples/scenes/rir_ground_site_recognition/rir_ground_site_recognition.md)
   核对扫描、检测、航迹和识别结果，不能以空输入能调用作为功能通过。
4. 连续多周期验证状态保持、指定任务、运行补丁和电源状态。
5. 核对数据库、日志以及下游消费顺序。
6. 在仅含交付文件的部署目录复测，排除开发机路径污染。

报错时提供：完整错误和链接输出、实际 `1q.lib` 路径、交付版本或文件哈希、
VS/工具集版本、平台/配置/CRT、配套库清单。运行问题还需配置、输入周期和 `result.issues`。

本指南按当前仓库公开接口与构建规则编写；具体 ABI 和可用字段以同批次交付头及库为准。
本文不是实际交付二进制已通过消费端验证的证明。
