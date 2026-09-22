# Unit-type test partitions: one executable per (type=unit, domain).
#
# Phase 2: splits the legacy 1q_unit_tests aggregate executable into per-domain
# 1q_<domain>_unit_tests partitions. The legacy 1q_unit_tests name becomes a
# custom aggregate target depending on every enabled unit partition (set up by
# the caller after including this file).
#
# Source discovery: each partition globs exactly its own type/domain directory
# (tests/unit/<domain>/*_test.cpp). This lets test authors drop a new file in
# without editing a central source list. The registry finalize step proves
# every _test.cpp is owned by exactly one partition.
#
# Note on replay overlap: trace session adapter, replay session and trace
# writer/compression sources currently live under tests/unit/<domain>/ and are
# also compiled into 1q_replay_fast_tests until Phase 3 moves them to
# replay/<domain>/ and removes the duplication. Codec roundtrip sources remain
# excluded from unit partitions, matching the legacy 1q_unit_tests membership.
# The registry's ONEQ_TEST_OVERLAP_ALLOWLIST documents this transition state.

function(_oneq_exclude_legacy_replay_codec_sources out_var)
    set(_sources ${ARGN})
    list(FILTER _sources EXCLUDE REGEX "_replay_codec_roundtrip_test\\.cpp$")
    set(${out_var} "${_sources}" PARENT_SCOPE)
endfunction()

# common: coordinate/estimation/numeric/atmosphere + replay trace writer/compression.
file(GLOB _oneq_unit_common CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/common/*_test.cpp")
if(_oneq_unit_common)
    set(_oneq_common_compile_defs "")
    if(ONEQ_ENABLE_FILE_LOG)
        # 文件日志后端测试（file_log_test.cpp）的 #if 守卫与库编译保持一致。
        list(APPEND _oneq_common_compile_defs PROJECT_LOG_BACKEND_FILE=1)
    endif()
    oneq_add_test_partition(
        TYPE unit DOMAIN common
        SOURCES ${_oneq_unit_common}
        TIMEOUT 60
        COMPILE_DEFS ${_oneq_common_compile_defs})
endif()

# examples: JsonReader test links examples/common/json_reader.cpp implementation.
file(GLOB _oneq_unit_examples CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/examples/*_test.cpp")
if(_oneq_unit_examples)
    # component_attachment 组件实现（ecs_component_runtime_test 测组件运行时
    # 修改接口；传感器→融合适配已上移库内 fusion/SensorAdapters.h）。
    # logger.cpp：组件源文件内 CA_LOG_EVENT / CA_LOG_VIEW 宏引用符号（单测
    # 不初始化 → 未初始化静默跳过路径）。集成端日志双后端（注入
    # CA_LOG_BACKEND_SPDLOG，与示例 target 同款开关）：组件源与 logger 全平台
    # 统一编译，spdlog 仅在平台启用时链接；Windows 走标准库文件后端（此前
    # 因 spdlog 排除的 ecs/scene_data/scene_script 测试随双后端一并恢复）。
    set(_oneq_examples_extra
        "${CMAKE_SOURCE_DIR}/examples/common/json_reader.cpp"
        "${CMAKE_SOURCE_DIR}/examples/scenes/area_division.cpp"
        "${CMAKE_SOURCE_DIR}/examples/scenes/ballistic_trajectory.cpp"
        "${CMAKE_SOURCE_DIR}/examples/scenes/scene_data.cpp"
        "${CMAKE_SOURCE_DIR}/examples/scenes/scene_script.cpp"
        "${CMAKE_SOURCE_DIR}/examples/components/ar_sensor_component.cpp"
        "${CMAKE_SOURCE_DIR}/examples/components/esr_sensor_component.cpp"
        "${CMAKE_SOURCE_DIR}/examples/components/eos_sensor_component.cpp"
        "${CMAKE_SOURCE_DIR}/examples/components/sbirs_sensor_component.cpp"
        "${CMAKE_SOURCE_DIR}/examples/components/sar_sensor_component.cpp"
        "${CMAKE_SOURCE_DIR}/examples/components/flight_component.cpp"
        "${CMAKE_SOURCE_DIR}/examples/logger/logger.cpp")
    set(_oneq_examples_link_libs "")
    if(PROJECT_ENABLE_SPDLOG)
        list(APPEND _oneq_examples_link_libs ${PROJECT_SPDLOG_TARGET})
    endif()
    oneq_add_test_partition(
        TYPE unit DOMAIN examples
        SOURCES ${_oneq_unit_examples}
        TIMEOUT 60
        INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/examples/common"
                     "${CMAKE_SOURCE_DIR}/tests/consumer/batch_validation"
                     "${CMAKE_SOURCE_DIR}/examples"
        EXTRA_SOURCES ${_oneq_examples_extra}
        COMPILE_DEFS "CA_LOG_BACKEND_SPDLOG=$<BOOL:${PROJECT_ENABLE_SPDLOG}>"
                     "CA_LIBRARY_FILE_LOG=$<BOOL:${ONEQ_ENABLE_FILE_LOG}>"
        LINK_LIBS ${_oneq_examples_link_libs})
    # scenes/scene_data.cpp（session_config 加载 + RIR 识别库路径解析）需要
    # examples/basic_config 路径宏，与 examples_core 目标同源注入。
    target_compile_definitions(${PROJECT_NAME}_examples_unit_tests PRIVATE
        SCENE_CONFIG_DIR="${CMAKE_SOURCE_DIR}/examples/basic_config"
        CA_RIR_DATABASE_PATH="${CMAKE_SOURCE_DIR}/examples/basic_config/remote_identification_radar/target_feature_database_v1.1.db")
    if(ONEQ_ENABLE_FLIGHT_DYNAMIC)
        # 飞行组件 FD 路径（与 examples/CMakeLists.txt 对称）：
        # 静态库不传递依赖，需显式链接 JSBSim；c172x 数据根注入。
        if(DEFINED ONEQ_JSBSIM_DATA_ROOT_DIR AND NOT ONEQ_JSBSIM_DATA_ROOT_DIR STREQUAL "")
            set(_oneq_examples_fd_root "${ONEQ_JSBSIM_DATA_ROOT_DIR}")
        else()
            set(_oneq_examples_fd_root "${CMAKE_SOURCE_DIR}/third_party/jsbsim")
        endif()
        target_compile_definitions(${PROJECT_NAME}_examples_unit_tests PRIVATE
            ONEQ_CA_FLIGHT_DYNAMIC_ENABLED=1
            FD_JSBSIM_ROOT_DIR="${_oneq_examples_fd_root}")
        if(TARGET JSBSim::JSBSim)
            target_link_libraries(${PROJECT_NAME}_examples_unit_tests PRIVATE JSBSim::JSBSim)
        endif()
    endif()
endif()

# airborne_radar (AR).
file(GLOB _oneq_unit_airborne_radar CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/airborne_radar/*_test.cpp")
_oneq_exclude_legacy_replay_codec_sources(_oneq_unit_airborne_radar
    ${_oneq_unit_airborne_radar})
if(_oneq_unit_airborne_radar)
    oneq_add_test_partition(
        TYPE unit DOMAIN airborne_radar
        SOURCES ${_oneq_unit_airborne_radar}
        TIMEOUT 60
        INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/unit/airborne_radar")
endif()

# remote_identification_radar (RIR).
file(GLOB _oneq_unit_remote_identification_radar CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/remote_identification_radar/*_test.cpp")
if(_oneq_unit_remote_identification_radar)
    oneq_add_test_partition(
        TYPE unit DOMAIN remote_identification_radar
        SOURCES ${_oneq_unit_remote_identification_radar}
        TIMEOUT 60
        LINK_LIBS SQLite::SQLite3
        INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/unit/remote_identification_radar"
                     "${CMAKE_CURRENT_BINARY_DIR}/generated")
endif()

# electronic_surveillance_radar (ESR).
file(GLOB _oneq_unit_electronic_surveillance_radar CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/electronic_surveillance_radar/*_test.cpp")
_oneq_exclude_legacy_replay_codec_sources(_oneq_unit_electronic_surveillance_radar
    ${_oneq_unit_electronic_surveillance_radar})
if(_oneq_unit_electronic_surveillance_radar)
    oneq_add_test_partition(
        TYPE unit DOMAIN electronic_surveillance_radar
        SOURCES ${_oneq_unit_electronic_surveillance_radar}
        TIMEOUT 60)
endif()

# electronic_countermeasure (ECM).
file(GLOB _oneq_unit_electronic_countermeasure CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/electronic_countermeasure/*_test.cpp")
if(_oneq_unit_electronic_countermeasure)
    oneq_add_test_partition(
        TYPE unit DOMAIN electronic_countermeasure
        SOURCES ${_oneq_unit_electronic_countermeasure}
        TIMEOUT 60)
endif()

# navigation.
file(GLOB _oneq_unit_navigation CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/navigation/*_test.cpp")
if(_oneq_unit_navigation)
    oneq_add_test_partition(
        TYPE unit DOMAIN navigation
        SOURCES ${_oneq_unit_navigation}
        TIMEOUT 60)
endif()

# fusion.
file(GLOB _oneq_unit_fusion CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/fusion/*_test.cpp")
if(_oneq_unit_fusion)
    oneq_add_test_partition(
        TYPE unit DOMAIN fusion
        SOURCES ${_oneq_unit_fusion}
        TIMEOUT 60)
endif()

# threat_assessment.
file(GLOB _oneq_unit_threat_assessment CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/threat_assessment/*_test.cpp")
if(_oneq_unit_threat_assessment)
    oneq_add_test_partition(
        TYPE unit DOMAIN threat_assessment
        SOURCES ${_oneq_unit_threat_assessment}
        TIMEOUT 60)
endif()

# target_inference.
file(GLOB _oneq_unit_target_inference CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/target_inference/*_test.cpp")
if(_oneq_unit_target_inference)
    oneq_add_test_partition(
        TYPE unit DOMAIN target_inference
        SOURCES ${_oneq_unit_target_inference}
        TIMEOUT 60)
endif()

# precision_evaluation.
file(GLOB _oneq_unit_precision_evaluation CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/precision_evaluation/*_test.cpp")
if(_oneq_unit_precision_evaluation)
    oneq_add_test_partition(
        TYPE unit DOMAIN precision_evaluation
        SOURCES ${_oneq_unit_precision_evaluation}
        TIMEOUT 60)
endif()

# electro_optical_sensor (EOS).
file(GLOB _oneq_unit_electro_optical_sensor CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/electro_optical_sensor/*_test.cpp")
_oneq_exclude_legacy_replay_codec_sources(_oneq_unit_electro_optical_sensor
    ${_oneq_unit_electro_optical_sensor})
if(_oneq_unit_electro_optical_sensor)
    oneq_add_test_partition(
        TYPE unit DOMAIN electro_optical_sensor
        SOURCES ${_oneq_unit_electro_optical_sensor}
        TIMEOUT 60)
endif()

# sbirs_sensor (SBIRS).
file(GLOB _oneq_unit_sbirs_sensor CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/sbirs_sensor/*_test.cpp")
_oneq_exclude_legacy_replay_codec_sources(_oneq_unit_sbirs_sensor
    ${_oneq_unit_sbirs_sensor})
if(_oneq_unit_sbirs_sensor)
    oneq_add_test_partition(
        TYPE unit DOMAIN sbirs_sensor
        SOURCES ${_oneq_unit_sbirs_sensor}
        COMPILE_DEFS ONEQ_ENABLE_OPIR_ACCEPTANCE_LOG=$<BOOL:${ONEQ_ENABLE_OPIR_ACCEPTANCE_LOG}>
        TIMEOUT 60)
endif()

# sar.
file(GLOB _oneq_unit_sar CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/sar/*_test.cpp")
_oneq_exclude_legacy_replay_codec_sources(_oneq_unit_sar
    ${_oneq_unit_sar})
if(_oneq_unit_sar)
    oneq_add_test_partition(
        TYPE unit DOMAIN sar
        SOURCES ${_oneq_unit_sar}
        TIMEOUT 60
        LABELS ci_required)
endif()
