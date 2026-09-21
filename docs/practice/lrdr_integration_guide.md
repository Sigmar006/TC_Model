# LRDR 集成指南

Status: active  
Last-reviewed: 2026-09-21  
Scope: Windows C++ 集成、构建依赖、调用流程与链接排障。

LRDR 在本仓库对应 `remote_identification_radar`（RIR），入口是
`remote_identification_radar::session::RirSession`。接口以当前公共头为准，构建以当前 CMake 配置为准。

## 1. 选择集成方式

| 方式 | 所需材料 | 宿主职责 |
| --- | --- | --- |
| 静态库集成 | 同批次公共头、生成头、主库、配套依赖库和运行数据 | 调用公开会话 API |
| 完整仓库构建 | 完整源码、工具链、依赖、代码生成工具 | 通过仓库 CMake 构建统一产品库 |
| 手工移植模块 | 模块源码、递归依赖、生成代码和构建规则 | 自行维护全部编译和链接依赖 |

优先使用同批次交付包或完整仓库构建。`rir_engine`、`rir_core` 是内部 OBJECT 组件，最终装配进统一库；不要假设有独立的 `lrdr.lib`。

**`新建文件夹/` 与 `code_line_statistics_split/` 是代码统计快照，不是完整 SDK。**
其配套 `shared` 目录缺少 `common/radar/ScanScheduleRuntime.h`。只复制这些目录不能保证独立构建。

## 2. 静态库接入

### 交付物

- 完整 `include/1q/`，包含同次构建生成并安装的导出头。
- 与配置匹配的主库：默认 Release 为 `1q.lib`，Debug 为 `1qd.lib`，以交付记录为准。
- 同批次 SQLite、zlib 等依赖库。历史 vendor 包名称为 `sqlite3_vendor.lib`、`zlibstatic.lib`；Conan 包名称可能不同。
- 配置与识别特征数据库。参考 [基础配置](../../examples/basic_config/remote_identification_radar.json) 和 [场景配置](../../examples/scenes/rir_ground_site_recognition/rir_ground_site_recognition.json)。
- 编译器、工具集、架构、CRT、配置、源码版本及编译期开关记录。

### Visual Studio 设置

| 属性 | 设置 |
| --- | --- |
| C/C++ → 附加包含目录 | SDK 的 `include`，其下直接包含 `1q` |
| C/C++ → 预处理器定义 | 静态库消费者定义 `ONEQ_STATIC_DEFINE` |
| 链接器 → 附加库目录 | 主库与配套依赖目录 |
| 链接器 → 附加依赖项 | 主库及交付包要求的 SQLite、zlib 等库 |
| 平台 | 与交付一致；当前 Windows 主线为 x64 |
| 运行库 | 与交付一致；仓库默认 Release `/MD`、Debug `/MDd` |
| 工具集与配置 | 不混用不同批次头文件和库，不混用 Debug/Release |

仅使用公开会话 API 时，不应通过增加仓库 `src` 包含目录解决链接错误。需要源码调试时使用同批次 RelWithDebInfo 产物和符号；当前 Release 不生成调试符号。

### CMake 消费

仓库内别名为 `1q::core`，安装包导出目标为 `1q::1q`。具备完整包配置及依赖发现环境时：

```cmake
find_package(1q CONFIG REQUIRED)
target_link_libraries(your_app PRIVATE 1q::1q)
```

将安装前缀加入 `CMAKE_PREFIX_PATH`。安装树不自动打包所有第三方依赖，消费端仍需提供对应工具链或依赖包。若提示 `SQLite::SQLite3` 等目标不存在，应检查实际 `1qConfig.cmake` 与依赖目标映射；不能只补一个裸库路径就忽略配置错误。

## 3. 完整仓库构建

当前 `VisualStudio.15.0-amd64` 预设实际是 VS2026 generator + v141 工具集、C++17，名字不代表 VS2015。VS2015 交付兼容性需单独验证。

在仓库根目录使用 Git Bash / WSL 入口，环境准备见 [Windows 构建说明](windows_local_build.md)：

```bash
scripts/1q.sh configure VisualStudio.15.0-amd64
scripts/1q.sh build VisualStudio.15.0-amd64-release
scripts/1q.sh test VisualStudio.15.0-amd64-release -R 'unit::remote_identification_radar'
scripts/1q.sh test VisualStudio.15.0-amd64-release -R 'unit::common'
```

依赖版本与获取流程遵循 `conanfile.py` 和仓库脚本。确认 CTest 实际选中测试，“没有找到测试”不是通过。交付还需安装产物并在独立消费工程验证，详见 [构建与测试治理](build_and_test_governance.md)。

### 手工源码集成检查表

| 内容 | 权威入口或用途 |
| --- | --- |
| 模块源文件 | `src/remote_identification_radar/CMakeLists.txt` 中的 `RIR_ENGINE_SOURCES`、`RIR_CORE_SOURCES` |
| 公共 API 与跨域类型 | 完整 `include/1q/`，保留层级 |
| 公共实现 | `src/common/` 及模块引用的其他实现，沿 include 和符号依赖递归检查 |
| 扫描内核 | `src/common/radar/ScanScheduleRuntime.h`，保留完整 inline 函数体 |
| 扫描直接依赖 | `include/1q/foundation/scan_schedule_types.h`、`src/common/radar/AntennaPatternRuntime.h` |
| 天线内核依赖 | `src/common/numerics/Constants.h` 及其递归依赖 |
| 回放生成代码 | `rir_replay.fbs`、`rir_session_replay.fbs` 及递归 schema，按仓库 FlatBuffers 流程生成 |
| 第三方与链接闭包 | 模块 CMake、`src/CMakeLists.txt` 和 `conanfile.py` |

源码编译需要仓库 `include`、`src`、生成目录和第三方包含目录。不要展平目录、只复制函数声明，或删除报错源文件来凑出可链接版本。上述表格是检查入口，不是独立 LRDR 包的完整文件清单。

## 4. 会话调用与数据接线

1. 将配置映射到 `config::RirSessionConfig`。JSON 加载属于宿主/示例层，`RirSession` 不直接接受 JSON 路径。
2. 调用 `config::ValidateRirSessionConfig(cfg)` 并处理诊断，然后创建长期持有的会话。`Create` 不校验；`CreateWithDiagnostics` 即使产生问题也会返回会话，构造成功不代表配置有效。
3. 多实例用 `SetSensorEntityId` 设置设备标识。
4. 每周期更新输入并调用一次 `StepWithResult`。不要每帧重建会话，否则丢失航迹和识别积累状态。
5. 运行期调整使用 `TryApplyRuntimeConfig`：true 表示已暂存，在下一个成功执行周期边界提交；false 也可能表示没有变更。

| `RirCycleInput` 字段 | 宿主输入 |
| --- | --- |
| `input_cycle_index` | 非零周期号，由宿主管理周期顺序 |
| `dt_sec` | 有限且大于零的步长，秒 |
| `sim_time_sec` | 仿真绝对时刻，秒，与步长保持一致 |
| `platform_position` | 雷达站点/平台 ECEF 位置，米，必填 |
| `scene_targets` | 站点局部 ENU 目标、运动事实和识别特征，具体字段见 `RirSceneTypes.h` |
| `rf_scene` | 外部 RF 场景，可空，不含本机发射 |

不要将 ECEF 目标坐标直接作为 ENU，也不要混用度/弧度、米/千米。完整适配参考 [RIR 示例组件](../../examples/components/rir_sensor_component.cpp)。

以下是配置校验、输入装配完成后的调用片段，不是独立可运行的探测示例：

```cpp
#include "1q/remote_identification_radar/session/RirSession.h"

namespace rir = remote_identification_radar;

// 初始化时执行一次；cfg 为已校验配置。
auto radar = rir::session::RirSession::Create(cfg);

// 周期循环中；input 为宿主装配的 RirCycleInput。
const auto result = radar.StepWithResult(input);
if (result.status == rir::session::RirCycleStatus::kCompleted) {
  // 消费 result.output_frame；完成周期也可能没有检测/识别结果。
} else {
  // 记录 result.status、result.issues、result.abort_reason。
  // 不将上周期输出冒充本周期结果。
}
```

接入融合时先执行雷达周期，再取得当前量测并推进融合。量测适配器 `AdaptRirFeatureMeasurementsToDetectionRecords` 的接线见 [甲方集成验证清单](customer_integration_verification.md)。先跑通单会话，再接融合、推演和威胁评估。

## 5. BuildScanPattern 无法解析：专项排查

定义位于 `src/common/radar/ScanScheduleRuntime.h`，命名空间为 `oneq::common::radar`，是头文件内的 **inline 函数，没有对应的 `ScanScheduleRuntime.cpp`**。当前 `RirSession.cpp` 已包含该头。

| 错误现象 | 判断与处理 |
| --- | --- |
| C1083，找不到头文件 | 编译阶段缺文件或包含目录；补齐头及递归依赖 |
| LNK2019 / LNK2001，无法解析函数 | 链接存在引用但无匹配实现；检查实际头文件、声明/定义、旧对象文件和实际链接库 |
| “不是内部或外部命令” | shell/构建脚本误把函数名当命令；检查报错脚本行 |

缺少头通常先导致编译错误，**不能仅凭 LNK2019 断定是少复制了头文件**。拆分快照缺文件是已确认的问题，实际链接原因仍需完整错误核实。

排查顺序：

1. 保存完整错误编号、函数签名、引用它的 `.obj` / `.lib` 和构建配置。
2. 给引用源文件临时启用 MSVC `/showIncludes`，确认实际命中的 `ScanScheduleRuntime.h` 路径，排除旧同名文件。
3. 打开实际命中的头，确认有完整 inline 函数体，命名空间及参数与调用一致；不能只补 `extern` 声明。
4. 如曾把函数体移到 `.cpp`，核对定义与声明一致、源文件参与当前配置编译；不要让调用方只见声明，而定义仅以 inline 形式留在另一个 `.cpp`。优先恢复仓库原始头文件实现。
5. 清理当前配置中间产物和 PCH，重建主库及宿主，核对实际链接库路径。
6. 静态库集成由库构建方修复并重编相应对象，不要求宿主调用内部扫描函数。

仍失败时提供完整错误、包含路径、编译/链接命令、工具集/架构/配置及源码或 SDK 批次。不要通过删除扫描调用绕过问题。

## 6. 日志与运行数据

- 构建库时启用 `ONEQ_ENABLE_RIR_ACCEPTANCE_LOG` 才有相应验收日志能力；仅给宿主加宏不能改变已编译库。
- 当前配置有 `acceptance_log_enabled`、`acceptance_log_output_directory`，需与库版本匹配。验收日志与 `ONEQ_ENABLE_FILE_LOG` 内部调试日志不是同一开关。
- 数据库必须在运行机器可访问，相对路径受工作目录影响。数据库不可用时先查配置和执行诊断。
- 功能日志参考 `RadarOutput.log`、`RadarInternalInterface.log`，扫描证据参考 `Rir_scan_pattern.csv`；输出以实际构建和配置为准。

## 7. 集成验收

1. 编译链接：公共头、生成头、依赖库齐全，无未解析符号。
2. 会话冒烟：配置诊断可解释，周期状态符合输入，非法输入返回诊断。
3. 场景复现：按 [站点识别场景说明](../../examples/scenes/rir_ground_site_recognition/rir_ground_site_recognition.md) 核对扫描、检测、航迹、识别与日志；空输入能调用不代表功能通过。
4. 状态连续性：连续多周期保持同一会话，核对指定任务、补丁生效及关机状态。
5. 宿主接线：坐标、时间、目标标识一致，雷达先于融合步进。
6. 独立部署：仅用交付目录验证，排除开发机旧库、源码头和数据库路径污染。

本文根据当前源码与构建配置核对编写，不代表已在集成方工程复现链接错误，也不替代具体交付批次的编译、链接和场景验收记录。
