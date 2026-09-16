# examples/CMakeLists.txt
# 示例程序构建入口（主示例本体 + 每场景可执行 + 共享便利层）：
#   core/        组件基类 / 实体 / 世界 / 信号（Boost.Signals2，header-only，
#                零自定义分发层）+ 场景共享状态
#   components/  每个仿真模块一个组件（飞行 / AR / ESR / ECM / EOS / SBIRS /
#                SAR / RIR / 融合 / 推演 / 威胁评估），挂载到实体后按挂载序步进
#   logger/      集成端日志设施（双后端：spdlog 平台直用；Windows 走
#                std::ofstream 文件后端，CA_LOG_BACKEND_SPDLOG=0，仿库内
#                ProjectFileLog 口径，全平台均可构建）
#   app/         装配层：RunScene 运行体 + 可视化 CSV 落盘 + 指令路由 + 通用
#                runner（component_attachment_demo，--scene 跑任意场景）
#   scenes/      场景可执行集——每场景目录自带 main.cpp（薄入口，场景 JSON
#                由 ONEQ_SCENE_JSON 钉死）+ <name>.json + <name>.md 期望表；
#                场景加载代码（scene_data/scene_script/area_division）同住
#   basic_config/六域基础 session 配置模板 + RIR 识别库资产
#   common/      examples 与 tests 共享便利层（JSON 解析 / CSV / 配置加载器 /
#                可视化查看器），不属于 oneq 库的 public surface（库内部不
#                消费 JSON；见 OQ-7 收口记录）
# 精度评估场景（scenes/sbirs_triple_sat_fix_messages/）走 ECS：三星卫星实体 +
# 消息机制地面站融合组件（内挂评估会话）；兼容目标 precision_evaluation_demo 与之同源。

if(NOT TARGET ${PROJECT_ALIAS})
    message(FATAL_ERROR "${PROJECT_ALIAS} target not found. Ensure src/CMakeLists.txt is processed first")
endif()

# 共享变量：内联使用，无需函数传递。
set(ONEQ_EXAMPLE_COMMON_DIR "${CMAKE_CURRENT_SOURCE_DIR}/common")
set(ONEQ_EXAMPLE_COMMON_SOURCES
    "${ONEQ_EXAMPLE_COMMON_DIR}/json_reader.cpp")

# ---------------------------------------------------------------------------
# examples_core：场景应用共享静态库（core ECS + 11 组件 + 集成端日志 + 场景
# 加载 + 装配层）。通用 runner 与全部 ECS 场景可执行链它；单测分区
# （tests/cmake/partitions/Unit.cmake）仍直编同一批源码，不依赖本 target
# （测试构建不要求 ENABLE_EXAMPLES）。
add_library(examples_core STATIC
    app/outputs.cpp
    app/command_routing.cpp
    app/runner.cpp
    scenes/scene_data.cpp
    scenes/area_division.cpp
    scenes/ballistic_trajectory.cpp
    scenes/scene_script.cpp
    components/flight_component.cpp
    components/esr_sensor_component.cpp
    components/ecm_sensor_component.cpp
    components/ar_sensor_component.cpp
    components/eos_sensor_component.cpp
    components/sbirs_sensor_component.cpp
    components/sar_sensor_component.cpp
    components/rir_sensor_component.cpp
    components/fusion_component.cpp
    components/ground_station_fusion_component.cpp
    components/inference_component.cpp
    components/threat_component.cpp
    logger/logger.cpp
    ${ONEQ_EXAMPLE_COMMON_SOURCES})

# Boost.Signals2 事件库：与 tests 分区同源解析（tests/cmake/TestSupport.cmake）。
# conan 路径走 find_package（包配置文件由 conan 生成）；none 路径下
# VendorPackages.cmake 已直接定义 Boost::boost INTERFACE 目标，无需再 find。
if(NOT TARGET Boost::boost)
    find_package(Boost CONFIG REQUIRED)
endif()
target_include_directories(examples_core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${ONEQ_EXAMPLE_COMMON_DIR}"
    PRIVATE
    "${CMAKE_SOURCE_DIR}/src")
# Eigen header-only 链入：oneq_apply_target_configuration 的 PCH（Eigen/Core）需要
# include 路径（与 oneq_add_component 各组件同法）。
if(TARGET Eigen3::Eigen)
    target_link_libraries(examples_core PUBLIC Eigen3::Eigen)
endif()
target_link_libraries(examples_core PUBLIC
    ${PROJECT_ALIAS}
    Boost::boost)
# spdlog：集成端日志后端（logger.h 的 CA_LOG_EVENT / CA_LOG_VIEW），仅平台启用
# 时链接；Windows 文件后端分支只依赖标准库。示例源含中文 UTF-8 串字面量，
# 依赖源文件 UTF-8 BOM；编译/链接选项与库目标一样走
# oneq_apply_target_configuration（无 /utf-8）。
if(PROJECT_ENABLE_SPDLOG)
    target_link_libraries(examples_core PUBLIC ${PROJECT_SPDLOG_TARGET})
endif()

# 日志后端开关 + 基础配置/识别库路径 + 默认输出根（仿库内
# oneq_add_component 的 PROJECT_LOG_BACKEND_* 先例，见 ProjectTargets.cmake）。
target_compile_definitions(examples_core PRIVATE
    CA_LOG_BACKEND_SPDLOG=$<BOOL:${PROJECT_ENABLE_SPDLOG}>
    CA_LIBRARY_FILE_LOG=$<BOOL:${ONEQ_ENABLE_FILE_LOG}>
    SCENE_CONFIG_DIR="${CMAKE_SOURCE_DIR}/examples/basic_config"
    CA_DEFAULT_OUTPUT_DIR="${CMAKE_SOURCE_DIR}/examples/log"
    CA_RIR_DATABASE_PATH="${CMAKE_SOURCE_DIR}/examples/basic_config/remote_identification_radar/target_feature_database_v1.1.db")

# 集成端日志模式（编译期宏，见 logger/logger_modes.h）：
# CA_VIEW_LOG_MODE —— summary（每周期摘要，默认）| nonnominal | delta
#                    落盘频率用 --view-every / 场景 view_log_every_cycles，不靠换模式
# CA_EVENT_LOG_MODE —— all（逐条全量）| key（只记关键，默认，同 logger_modes.h）|
#                      aggregate（周期聚合）
# 不传（空）时用 logger_modes.h 源码默认；显式传值则覆盖源码默认。
set(CA_VIEW_LOG_MODE "" CACHE STRING
    "component_attachment 视图日志模式: summary|nonnominal|delta (空=用 logger_modes.h 默认)")
set(CA_EVENT_LOG_MODE "" CACHE STRING
    "component_attachment 事件日志模式: all|key|aggregate (空=用 logger_modes.h 默认)")

if(CA_VIEW_LOG_MODE)
    if(CA_VIEW_LOG_MODE STREQUAL "summary")
        target_compile_definitions(examples_core PRIVATE CA_VIEW_LOG_MODE_SUMMARY)
    elseif(CA_VIEW_LOG_MODE STREQUAL "nonnominal")
        target_compile_definitions(examples_core PRIVATE CA_VIEW_LOG_MODE_NONNOMINAL)
    elseif(CA_VIEW_LOG_MODE STREQUAL "delta")
        target_compile_definitions(examples_core PRIVATE CA_VIEW_LOG_MODE_DELTA)
    else()
        message(FATAL_ERROR "CA_VIEW_LOG_MODE 非法值 '${CA_VIEW_LOG_MODE}'（可选 summary|nonnominal|delta）")
    endif()
endif()

if(CA_EVENT_LOG_MODE)
    if(CA_EVENT_LOG_MODE STREQUAL "all")
        target_compile_definitions(examples_core PRIVATE CA_EVENT_LOG_MODE_ALL)
    elseif(CA_EVENT_LOG_MODE STREQUAL "key")
        target_compile_definitions(examples_core PRIVATE CA_EVENT_LOG_MODE_KEY)
    elseif(CA_EVENT_LOG_MODE STREQUAL "aggregate")
        target_compile_definitions(examples_core PRIVATE CA_EVENT_LOG_MODE_AGGREGATE)
    else()
        message(FATAL_ERROR "CA_EVENT_LOG_MODE 非法值 '${CA_EVENT_LOG_MODE}'（可选 all|key|aggregate）")
    endif()
endif()

# 飞行组件：ONEQ_ENABLE_FLIGHT_DYNAMIC 开启时接入 JSBSim 真实飞行仿真
# （运动学回退路径始终可用）。
if(ONEQ_ENABLE_FLIGHT_DYNAMIC)
    if(NOT DEFINED ONEQ_JSBSIM_DATA_ROOT_DIR)
        set(ONEQ_JSBSIM_DATA_ROOT_DIR "${CMAKE_SOURCE_DIR}/third_party/jsbsim")
    endif()
    target_compile_definitions(examples_core PRIVATE
        ONEQ_CA_FLIGHT_DYNAMIC_ENABLED=1
        FD_JSBSIM_ROOT_DIR="${ONEQ_JSBSIM_DATA_ROOT_DIR}")
    target_link_libraries(examples_core PUBLIC JSBSim::JSBSim)
endif()

# ---------------------------------------------------------------------------
# 通用 runner（对外 target 名保持不变）：--scene 跑任意场景 JSON；装配与执行
# 与场景可执行共用 examples_core 内的 RunScene。
# 验收日志通道提示：OPIR 宽窄视场验收量写入 opir_acceptance.log，开关为库编译期选项。
if(NOT ONEQ_ENABLE_OPIR_ACCEPTANCE_LOG)
    message(STATUS "examples: ONEQ_ENABLE_OPIR_ACCEPTANCE_LOG=OFF -> no "
        "opir_acceptance.log; reconfigure with -DONEQ_ENABLE_OPIR_ACCEPTANCE_LOG=ON")
endif()

add_executable(component_attachment_demo app/main.cpp)
target_link_libraries(component_attachment_demo PRIVATE examples_core)
target_compile_definitions(component_attachment_demo PRIVATE
    CA_SCENE_DIR="${CMAKE_SOURCE_DIR}/examples/scenes")
set(_oneq_example_targets examples_core component_attachment_demo)

# ---------------------------------------------------------------------------
# 每场景可执行（主开发路径）：scenes/<name>/main.cpp 薄入口自动发现——新建
# 场景 = 在 scenes/ 下建目录放 main.cpp + <name>.json + <name>.md 即被收录。
# ONEQ_EXAMPLE_SCENES 选建：all（默认）| none | 分号分隔场景名。
file(GLOB _oneq_scene_mains CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/scenes/*/main.cpp")
set(_oneq_scene_names "")
foreach(_main IN LISTS _oneq_scene_mains)
    get_filename_component(_scene_dir "${_main}" DIRECTORY)
    get_filename_component(_scene "${_scene_dir}" NAME)
    list(APPEND _oneq_scene_names "${_scene}")
endforeach()
list(SORT _oneq_scene_names)

set(ONEQ_EXAMPLE_SCENES "all" CACHE STRING
    "要构建的场景可执行：all | none | 分号分隔的 scenes/ 目录名")

if(ONEQ_EXAMPLE_SCENES STREQUAL "all")
    set(_oneq_selected_scenes ${_oneq_scene_names})
elseif(ONEQ_EXAMPLE_SCENES STREQUAL "none")
    set(_oneq_selected_scenes "")
else()
    set(_oneq_selected_scenes ${ONEQ_EXAMPLE_SCENES})
    foreach(_want IN LISTS _oneq_selected_scenes)
        if(NOT _want IN_LIST _oneq_scene_names)
            message(FATAL_ERROR
                "ONEQ_EXAMPLE_SCENES 含未知场景 '${_want}'（可选：${_oneq_scene_names}）")
        endif()
    endforeach()
endif()

foreach(_scene IN LISTS _oneq_selected_scenes)
    if(_scene STREQUAL "sbirs_triple_sat_fix_messages")
        continue()  # 精度评估场景走下方接线（链 examples_core，地面站融合组件）
    endif()
    if(_scene STREQUAL "sbirs_inframe_dual_target")
        continue()  # 同框验证场景共用精度评估 main，走下方同款接线（验收日志宏）
    endif()
    add_executable(${_scene} scenes/${_scene}/main.cpp)
    target_link_libraries(${_scene} PRIVATE examples_core)
    target_compile_definitions(${_scene} PRIVATE
        ONEQ_SCENE_JSON="${CMAKE_CURRENT_SOURCE_DIR}/scenes/${_scene}/${_scene}.json")
    list(APPEND _oneq_example_targets ${_scene})
endforeach()

# 精度评估场景可执行 + 兼容目标 precision_evaluation_demo（同源同行为）：
# 三星 SBIRS + 消息投递地面站融合 + 推演的真值对照五项误差 + AHP 综合评分；
# 验收文件钉到场景输出目录（默认 examples/log/<场景名>/）。
if(sbirs_triple_sat_fix_messages IN_LIST _oneq_selected_scenes)
    add_executable(sbirs_triple_sat_fix_messages scenes/sbirs_triple_sat_fix_messages/main.cpp)
    target_link_libraries(sbirs_triple_sat_fix_messages PRIVATE examples_core)
    target_compile_definitions(sbirs_triple_sat_fix_messages PRIVATE
        ONEQ_SCENE_JSON="${CMAKE_CURRENT_SOURCE_DIR}/scenes/sbirs_triple_sat_fix_messages/sbirs_triple_sat_fix_messages.json"
        PE_DEFAULT_OUTPUT_DIR="${CMAKE_SOURCE_DIR}/examples/log"
        PE_ACCEPTANCE_LOG_ENABLED=$<BOOL:${ONEQ_ENABLE_PRECISION_EVALUATION_LOG}>)
    list(APPEND _oneq_example_targets sbirs_triple_sat_fix_messages)
endif()

# 同框双目标验证场景（2026-09-02 单镜筒化）：共用精度评估 main 的派生场景
#（json 换成同框几何），接线与三星座同款（验收文件钉 examples/log/<场景名>/）。
if(sbirs_inframe_dual_target IN_LIST _oneq_selected_scenes)
    add_executable(sbirs_inframe_dual_target scenes/sbirs_inframe_dual_target/main.cpp)
    target_link_libraries(sbirs_inframe_dual_target PRIVATE examples_core)
    target_compile_definitions(sbirs_inframe_dual_target PRIVATE
        ONEQ_SCENE_JSON="${CMAKE_CURRENT_SOURCE_DIR}/scenes/sbirs_inframe_dual_target/sbirs_inframe_dual_target.json"
        PE_DEFAULT_OUTPUT_DIR="${CMAKE_SOURCE_DIR}/examples/log"
        PE_ACCEPTANCE_LOG_ENABLED=$<BOOL:${ONEQ_ENABLE_PRECISION_EVALUATION_LOG}>)
    list(APPEND _oneq_example_targets sbirs_inframe_dual_target)
endif()

add_executable(precision_evaluation_demo scenes/sbirs_triple_sat_fix_messages/main.cpp)
target_link_libraries(precision_evaluation_demo PRIVATE examples_core)
target_compile_definitions(precision_evaluation_demo PRIVATE
    ONEQ_SCENE_JSON="${CMAKE_CURRENT_SOURCE_DIR}/scenes/sbirs_triple_sat_fix_messages/sbirs_triple_sat_fix_messages.json"
    PE_DEFAULT_OUTPUT_DIR="${CMAKE_SOURCE_DIR}/examples/log"
    PE_ACCEPTANCE_LOG_ENABLED=$<BOOL:${ONEQ_ENABLE_PRECISION_EVALUATION_LOG}>)
list(APPEND _oneq_example_targets precision_evaluation_demo)
oneq_apply_target_configuration(TARGETS ${_oneq_example_targets})
unset(_oneq_example_targets)

if(NOT ONEQ_ENABLE_PRECISION_EVALUATION_LOG)
    message(STATUS "precision_evaluation: ONEQ_ENABLE_PRECISION_EVALUATION_LOG=OFF -> no "
        "precision_acceptance.log; reconfigure with -DONEQ_ENABLE_PRECISION_EVALUATION_LOG=ON")
endif()
if(NOT ONEQ_ENABLE_OPIR_ACCEPTANCE_LOG OR NOT ONEQ_ENABLE_FUSION_ACCEPTANCE_LOG
    OR NOT ONEQ_ENABLE_INFERENCE_ACCEPTANCE_LOG)
    message(STATUS "precision_evaluation: also enable "
        "ONEQ_ENABLE_OPIR_ACCEPTANCE_LOG / FUSION / INFERENCE for "
        "opir/fusion/inference_acceptance.log under the scene output dir")
endif()

# 示例冒烟不进 ctest（示例是集成参考而非测试门禁，release 全量跑 ~25 s、
# debug 下 400 周期 FD 全链可到 2-4 分钟，与单测时间预期不匹配）。手动运行：
#   ./build/<preset>/bin/<scene_name>                       （场景可执行，场景钉死）
#   ./build/<preset>/bin/component_attachment_demo
#       [--scene <path>] [--cycles <n>] [--output-dir <dir>]（通用 runner）
#   python3 examples/common/viz/build_viewer.py --check <dir>（可视化契约回归）
