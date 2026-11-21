# dependencies/JsbsimProvider.cmake
# JSBSim 依赖解析：flight_dynamic 专属，仅在 ONEQ_ENABLE_FLIGHT_DYNAMIC=ON 时执行。
# JSBSim 不再耦合进 core/common；关闭 flight_dynamic 即完全不依赖 JSBSim。

if(NOT ONEQ_ENABLE_FLIGHT_DYNAMIC)
    return()
endif()

option(ONEQ_JSBSIM_FROM_SOURCE "Build JSBSim from third_party source (enables source-level debugging)" OFF)
set(ONEQ_JSBSIM_PREBUILT_ROOT_DIR "" CACHE PATH
    "Use a prebuilt JSBSim tree instead of building/finding JSBSim")

# 策略一：使用预编译目录（lib/libJSBSim），适合已构建好的 JSBSim 树。
if(ONEQ_JSBSIM_PREBUILT_ROOT_DIR)
    set(ONEQ_JSBSIM_BINARY_SOURCE "prebuilt:${ONEQ_JSBSIM_PREBUILT_ROOT_DIR}")
    find_library(ONEQ_JSBSIM_PREBUILT_LIBRARY
        NAMES JSBSim libJSBSim
        PATHS "${ONEQ_JSBSIM_PREBUILT_ROOT_DIR}/lib"
        NO_DEFAULT_PATH)
    if(NOT ONEQ_JSBSIM_PREBUILT_LIBRARY)
        message(FATAL_ERROR
            "ONEQ_JSBSIM_PREBUILT_ROOT_DIR does not contain lib/libJSBSim: "
            "${ONEQ_JSBSIM_PREBUILT_ROOT_DIR}")
    endif()
    add_library(JSBSim_prebuilt SHARED IMPORTED)
    set_target_properties(JSBSim_prebuilt PROPERTIES
        IMPORTED_LOCATION "${ONEQ_JSBSIM_PREBUILT_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/third_party/jsbsim/src")
    add_library(JSBSim::JSBSim ALIAS JSBSim_prebuilt)
# 策略二：从 third_party/jsbsim 源码构建，支持源码级调试。
elseif(ONEQ_JSBSIM_FROM_SOURCE)
    set(ONEQ_JSBSIM_BINARY_SOURCE "vendor:third_party/jsbsim")
    # 保存并强制覆盖 BUILD_SHARED_LIBS：JSBSim 需以共享库构建，避免与主项目静态策略冲突。
    set(_oneq_prev_build_shared_libs ${BUILD_SHARED_LIBS})
    set(BUILD_SHARED_LIBS ON CACHE BOOL "Build JSBSim as shared library" FORCE)
    # 关闭 JSBSim 自带的文档构建，减少不必要的依赖。
    set(BUILD_DOCS OFF CACHE BOOL "" FORCE)

    add_subdirectory(${CMAKE_SOURCE_DIR}/third_party/jsbsim ${CMAKE_BINARY_DIR}/third_party/jsbsim)

    # 恢复主项目原有的共享/静态策略。
    set(BUILD_SHARED_LIBS ${_oneq_prev_build_shared_libs} CACHE BOOL "" FORCE)
    unset(_oneq_prev_build_shared_libs)

    # 用 INTERFACE 库包装 libJSBSim，统一以 JSBSim::JSBSim 别名消费。
    add_library(JSBSim_interface INTERFACE)
    target_link_libraries(JSBSim_interface INTERFACE libJSBSim)
    target_include_directories(JSBSim_interface INTERFACE
        ${CMAKE_SOURCE_DIR}/third_party/jsbsim/src)
    add_library(JSBSim::JSBSim ALIAS JSBSim_interface)
# 策略三（默认）：由 conan 提供 jsbsim 预编译包。
else()
    set(ONEQ_JSBSIM_BINARY_SOURCE "conan:jsbsim/1.3.1")
    # Conan 包要求一条：与 conanfile.py 的 enable_flight_dynamic 选项保持一致。
    # 打开 ONEQ_ENABLE_FLIGHT_DYNAMIC 却没装包时，直接给出可执行的修复命令，
    # 而不是留给 find_package 抛晦涩的 package not found。
    find_package(jsbsim CONFIG QUIET)
    if(NOT (TARGET jsbsim::jsbsim OR TARGET JSBSim OR TARGET JSBSim::JSBSim))
        message(FATAL_ERROR
            "ONEQ_ENABLE_FLIGHT_DYNAMIC=ON 但未找到 JSBSim 包。\n"
            "请先用同一开关重跑依赖安装：\n"
            "  scripts/bootstrap_conan.sh <preset>   # 或\n"
            "  conan install . -o \"&:enable_flight_dynamic=True\" --build=missing\n"
            "或改用 -DONEQ_JSBSIM_PREBUILT_ROOT_DIR=<预编译树> / -DONEQ_JSBSIM_FROM_SOURCE=ON。")
    endif()
    # ConanCenter 的 jsbsim 配方导出名随版本变化（jsbsim::jsbsim / JSBSim::JSBSim / 裸 JSBSim），
    # 逐个探测后再统一到 JSBSim::JSBSim，避免 "target jsbsim::jsbsim not found"。
    if(TARGET jsbsim::jsbsim)
        add_library(JSBSim::JSBSim ALIAS jsbsim::jsbsim)
    elseif(TARGET JSBSim)
        add_library(JSBSim::JSBSim ALIAS JSBSim)
    endif()
endif()

# 解析结果写回 CACHE INTERNAL，供 ProjectDependencies 提升到调用方作用域。
set(ONEQ_JSBSIM_BINARY_SOURCE "${ONEQ_JSBSIM_BINARY_SOURCE}"
    CACHE INTERNAL "Resolved JSBSim binary source")
set(ONEQ_JSBSIM_DATA_ROOT_DIR "${CMAKE_SOURCE_DIR}/third_party/jsbsim"
    CACHE INTERNAL "Resolved JSBSim aircraft data root")
set(ONEQ_JSBSIM_DATA_SOURCE "vendor:third_party/jsbsim"
    CACHE INTERNAL "Resolved JSBSim aircraft data source")
message(STATUS "JSBSim binary source: ${ONEQ_JSBSIM_BINARY_SOURCE}")
message(STATUS "JSBSim data source: ${ONEQ_JSBSIM_DATA_SOURCE} (${ONEQ_JSBSIM_DATA_ROOT_DIR})")
