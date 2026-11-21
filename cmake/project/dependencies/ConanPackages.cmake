# dependencies/ConanPackages.cmake
# Conan 包发现与项目级依赖列表导出

# PACKAGE_MANAGER 已在 ProjectOptions.cmake 校验为 conan，此处直接解析包。
# spdlog 在 Windows 上存在已知的编译/链接问题，仅 UNIX 下启用；
# Windows 上由库内内置文件日志后端（ONEQ_ENABLE_FILE_LOG 门控）替代。
if(WIN32)
    set(PROJECT_ENABLE_SPDLOG OFF)
else()
    set(PROJECT_ENABLE_SPDLOG ON)
endif()
# 核心第三方库：特征值、Boost 头文件、nanoflann 近邻搜索、FlatBuffers 运行时。
find_package(Eigen3 CONFIG REQUIRED)
find_package(Boost CONFIG REQUIRED)
find_package(nanoflann CONFIG REQUIRED)
find_package(flatbuffers CONFIG REQUIRED)
# Boost 的 target 名随 CMakeDeps/recipe 版本不同：新版导出 Boost::headers，旧版导出
# Boost::boost。统一解析成 Boost::boost，避免下游 target_link_libraries(Boost::boost)
# 在国产化平台的新 conan 缓存上因 target 不存在而在生成阶段失败。
if(NOT TARGET Boost::boost)
    if(TARGET Boost::headers)
        add_library(Boost::boost ALIAS Boost::headers)
    else()
        message(FATAL_ERROR "Boost target is not available (expected Boost::boost or Boost::headers)")
    endif()
endif()

if(PROJECT_ENABLE_SPDLOG)
    find_package(spdlog CONFIG REQUIRED)
    # 仅接受编译型 spdlog::spdlog；header-only 形态在本项目中不被允许，
    # 因日志宏需绑定编译期后端实现而非纯头文件展开。
    if(TARGET spdlog::spdlog)
        set(PROJECT_SPDLOG_TARGET spdlog::spdlog)
    elseif(TARGET spdlog::spdlog_header_only)
        message(FATAL_ERROR "spdlog header-only target is not allowed in this configuration")
    else()
        message(FATAL_ERROR "spdlog target is not available")
    endif()
endif()
# zlib：必须优先用 conan 生成的 CONFIG 包。裸 find_package(ZLIB REQUIRED) 会先命中
# CMake 内置的 MODULE 模式 FindZLIB，从而绕开 conan 缓存去依赖系统 libz 开发包——
# 麒麟 V10 国防版最小安装常缺 zlib1g-dev，且系统 libz 与 conan zlib/1.3.1 混链会引入
# 版本不一致。故先 CONFIG，未命中再回退 MODULE（仅告警）。
find_package(ZLIB CONFIG QUIET)
if(NOT TARGET ZLIB::ZLIB)
    find_package(ZLIB REQUIRED)
    message(WARNING "ZLIB: 回退到系统 zlib（未命中 conan zlib CONFIG 包），请确认目标机已装 zlib 开发包")
endif()

# sqlite3（airborne_radar 识别特征库加载；库内部 PRIVATE 依赖，不进 ONEQ_LINK_DEPENDENCIES）。
# conan-center 的 sqlite3 recipe 经 CMakeDeps 生成包名 SQLite3、target SQLite::SQLite3。
find_package(SQLite3 CONFIG REQUIRED)

# HDF5 输出能力依赖 HighFive，其要求 C++17；低标准构建时静默关闭。
# Windows 始终关闭：conanfile.py 仅对非 Windows 安装 highfive（Windows 的 hdf5/highfive
# 需源码构建且对老 MSVC 工具链兼容性差），此分支仅 macOS/Linux 启用。
# 国产化平台（麒麟 aarch64）上 ConanCenter 没有 highfive/hdf5 预编译包，源码构建 HDF5
# 代价高且易失败，故提供 ONEQ_ENABLE_HDF5 显式开关，现场可 -DONEQ_ENABLE_HDF5=OFF 关闭。
if(PROJECT_CXX_STANDARD GREATER_EQUAL 17 AND NOT WIN32 AND ONEQ_ENABLE_HDF5)
    find_package(HighFive CONFIG REQUIRED)
    # CMakeDeps 导出的规范 target 名是 HighFive::HighFive；个别 provider/旧版包只导出
    # 裸名 HighFive。统一解析成 ONEQ_HIGHFIVE_TARGET，禁止把裸名直接写进链接项——裸名
    # 会被链接器当成库名解析为 -lHighFive，导致 Linux 上所有可执行目标链接失败。
    if(TARGET HighFive::HighFive)
        set(ONEQ_HIGHFIVE_TARGET HighFive::HighFive)
    elseif(TARGET HighFive)
        set(ONEQ_HIGHFIVE_TARGET HighFive)
    else()
        message(FATAL_ERROR
            "HighFive target is not available (expected HighFive::HighFive); "
            "check the conan highfive package or configure with -DONEQ_ENABLE_HDF5=OFF")
    endif()
    set(ONEQ_ENABLE_HDF5_OUTPUT ON)
    message(STATUS "SAR HDF5 output: ENABLED (${ONEQ_HIGHFIVE_TARGET})")
else()
    set(ONEQ_ENABLE_HDF5_OUTPUT OFF)
    set(ONEQ_HIGHFIVE_TARGET "")
    message(STATUS "SAR HDF5 output: disabled (requires C++17 + ONEQ_ENABLE_HDF5 on non-Windows, current: C++${PROJECT_CXX_STANDARD})")
endif()

# 标记 zlib 可用；导出公共库 target 需链接的依赖列表。
set(ONEQ_HAVE_ZLIB ON)
set(ONEQ_LINK_DEPENDENCIES
    Eigen3::Eigen
    Boost::boost
    nanoflann::nanoflann
    flatbuffers::flatbuffers)
