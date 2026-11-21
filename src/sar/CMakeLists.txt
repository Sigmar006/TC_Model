# sar 模块源文件与安装清单

# -- engine: 信号处理、几何、回波、成像与输出内核 -----------------------------
# 注：CSA 已标注【未进行设计需求，不再扩展】，相关源文件不再纳入构建；
# 如需恢复，参见 git 历史。
# SarFocusingSelector（内部确定性聚焦算法建议器）、SarRadiometricCalibration
# （内部图像响应辐射定标）与 Omega-K 部件链（Stolt 插值 / 真值验收 / SHA-256 摘要等）
# 已解冻并纳入构建。
include("${CMAKE_CURRENT_LIST_DIR}/SarSources.cmake")

# SarSources.cmake is also consumed by standalone contract scripts and keeps
# paths relative to src/. Convert only this module's build lists to local paths.
set(_sar_sources_root "${CMAKE_CURRENT_LIST_DIR}/..")
list(TRANSFORM SAR_ENGINE_SOURCES PREPEND "${_sar_sources_root}/")
list(TRANSFORM SAR_CORE_SOURCES PREPEND "${_sar_sources_root}/")

oneq_add_component(sar_core SOURCES ${SAR_CORE_SOURCES})
# Three SAR session sources use the same file-local raw-IQ predicate. Keep two
# standalone so this remains safe even when callers increase the Unity batch size.
set_source_files_properties(
    "${_sar_sources_root}/sar/session/SarReplayFlatbufferCodec.cpp"
    "${_sar_sources_root}/sar/session/SarRecordingSession.cpp"
    PROPERTIES SKIP_UNITY_BUILD_INCLUSION ON)
target_link_libraries(sar_core PRIVATE flatbuffers::flatbuffers)
if(SAR_ENGINE_SOURCES)
    oneq_add_component(sar_engine SOURCES ${SAR_ENGINE_SOURCES})
    target_link_libraries(sar_engine PRIVATE Eigen3::Eigen)
    if(ONEQ_ENABLE_HDF5_OUTPUT)
        # 用 imported target（HighFive::HighFive）；裸名 HighFive 会被当作 -lHighFive。
        target_link_libraries(sar_engine PRIVATE ${ONEQ_HIGHFIVE_TARGET})
        target_compile_definitions(sar_engine PRIVATE ONEQ_ENABLE_HDF5_OUTPUT)
    endif()
endif()
oneq_add_flatbuffers_schemas(
    TARGET sar_core
    MODULE sar
    SCHEMAS sar_replay.fbs sar_session_replay.fbs)

if(MSVC AND MSVC_VERSION EQUAL 1900)
    add_executable(sar_legacy_toolchain_smoke
        ${_sar_sources_root}/sar/smoke/SarLegacyToolchainSmoke.cpp
        ${_sar_sources_root}/sar/imaging/SarPgaSupportGradientTruth.cpp
        ${_sar_sources_root}/sar/imaging/SarPgaPhaseGradientEstimator.cpp
        ${_sar_sources_root}/sar/imaging/SarPgaGradientTruthComparison.cpp
    )
    target_include_directories(sar_legacy_toolchain_smoke PRIVATE
        "${_sar_sources_root}"
    )
    set_target_properties(sar_legacy_toolchain_smoke PROPERTIES
        CXX_STANDARD 11
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
    )
    # 该 target 是最小化 legacy 工具链兼容性探针（C++11 + VS2015），故意不挂
    # apply_msvc_options()（不带 /W4、优化档、栈大小等），以免改变探针语义。
    # 但 /utf-8 是编译正确性前提而非可选优化：PGA 头经 #include "sar/signal/SarFft.h"
    # 拉入含中文注释的 SarFft.h，VS2015 若按系统代码页(936/GBK)解析会触发
    # C4819→C2059 级联。此处仅补 /utf-8，与 2026f570 修复 VS2015 全链路同一根因。
    if(MSVC_VERSION GREATER_EQUAL 1900)
        target_compile_options(sar_legacy_toolchain_smoke PRIVATE /utf-8)
    endif()
endif()
unset(_sar_sources_root)
