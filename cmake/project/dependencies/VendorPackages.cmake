# dependencies/VendorPackages.cmake
# none provider：不走 Conan 的第三方依赖发现与项目级依赖列表导出。
#
# PACKAGE_MANAGER 已在 ProjectOptions.cmake 校验为 none。本 provider 不调用
# conan、不读 conan data 文件、不假设 conan toolchain。第三方依赖来自工作空间
# third_party/ 目录，由 scripts/win/fetch_third_party.bat 把各库源码 git clone 到位
# （版本与 conanfile.py 对齐）。本文件从该目录消费：
#   - header-only 库（eigen、nanoflann、flatbuffers 运行时头、boost）：只暴露 include 路径，
#     以 INTERFACE IMPORTED target 形式提供，不触发其各自 add_subdirectory 的构建逻辑。
#   - zlib：add_subdirectory 编译静态库（gzopen 运行时 API 必须链接二进制）。
#   - flatc 工具：add_subdirectory(flatbuffers) 构建可执行 target。
#
# 对外契约与 ConanPackages.cmake 完全一致——产出相同的 imported target 名
# （Eigen3::Eigen、Boost::boost、nanoflann::nanoflann、flatbuffers::flatbuffers、
# ZLIB::ZLIB）与 ONEQ_LINK_DEPENDENCIES / ONEQ_HAVE_ZLIB / PROJECT_ENABLE_SPDLOG /
# ONEQ_ENABLE_HDF5_OUTPUT 等输出变量，故 src/、tests/、install 等下游消费点不感知
# provider 切换。

# third_party/ 根目录约定（与 scripts/win/fetch_third_party.bat、.gitignore 一致）。
set(ONEQ_THIRD_PARTY_DIR "${CMAKE_SOURCE_DIR}/third_party")

# 校验 third_party/ 存在；缺失时给出明确的引导提示而非晦涩的下游报错。
# 注意：third_party/ 被 .gitignore，checkout 后不存在是正常状态，需用户先跑拉取脚本。
if(NOT EXISTS "${ONEQ_THIRD_PARTY_DIR}/eigen")
    message(FATAL_ERROR
        "PACKAGE_MANAGER=none 要求 third_party/ 已就绪，但缺失：${ONEQ_THIRD_PARTY_DIR}/eigen\n"
        "请先在仓库根目录运行：scripts\\win\\fetch_third_party.bat\n"
        "（该脚本拉取 eigen/nanoflann/flatbuffers/zlib 并下载 boost 源码包到 third_party/）")
endif()

# spdlog 在 Windows 上存在已知的编译/链接问题，仅 UNIX 下启用；
# Windows 上由库内内置文件日志后端（ONEQ_ENABLE_FILE_LOG 门控）替代。
# none 模式当前面向 Windows VS2015，spdlog 本就 OFF；保留判定与 conan provider 一致。
if(WIN32)
    set(PROJECT_ENABLE_SPDLOG OFF)
else()
    set(PROJECT_ENABLE_SPDLOG ON)
endif()

# ---------------------------------------------------------------------------
# eigen 3.4.0（header-only）：仓库根目录即 Eigen/ 父目录。
# ---------------------------------------------------------------------------
if(NOT TARGET Eigen3::Eigen)
    set(_oneq_eigen_include_dir "${ONEQ_THIRD_PARTY_DIR}/eigen")
    if(NOT EXISTS "${_oneq_eigen_include_dir}/Eigen/Core"
       AND EXISTS "${ONEQ_THIRD_PARTY_DIR}/eigen/Core")
        # 某些离线包把 Eigen/ 目录内容直接铺在根下。生成仅供构建消费的标准
        # Eigen/<header> + Eigen/src 布局，避免修改 third_party 原始依赖。
        set(_oneq_eigen_flat_root "${ONEQ_THIRD_PARTY_DIR}/eigen")
        set(_oneq_eigen_include_dir "${CMAKE_BINARY_DIR}/third_party/eigen_compat_layout")
        set(_oneq_eigen_compat_root "${_oneq_eigen_include_dir}/Eigen")
        file(MAKE_DIRECTORY "${_oneq_eigen_compat_root}")
        file(GLOB _oneq_eigen_root_headers LIST_DIRECTORIES false
            "${_oneq_eigen_flat_root}/*")
        foreach(_oneq_eigen_header IN LISTS _oneq_eigen_root_headers)
            file(COPY "${_oneq_eigen_header}" DESTINATION "${_oneq_eigen_compat_root}")
        endforeach()
        file(COPY "${_oneq_eigen_flat_root}/src"
            DESTINATION "${_oneq_eigen_compat_root}")
        unset(_oneq_eigen_header)
        unset(_oneq_eigen_root_headers)
        unset(_oneq_eigen_compat_root)
        unset(_oneq_eigen_flat_root)
    endif()
    add_library(eigen_vendor INTERFACE IMPORTED)
    set_target_properties(eigen_vendor PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${_oneq_eigen_include_dir}")
    add_library(Eigen3::Eigen ALIAS eigen_vendor)
    message(STATUS "eigen 3.4.0: vendored header-only @ ${_oneq_eigen_include_dir}")
    unset(_oneq_eigen_include_dir)
endif()

# ---------------------------------------------------------------------------
# nanoflann v1.3.2（header-only）：单头 include/nanoflann.hpp。
# ---------------------------------------------------------------------------
if(NOT TARGET nanoflann::nanoflann)
    set(_oneq_nanoflann_include_dir "${ONEQ_THIRD_PARTY_DIR}/nanoflann/include")
    if(NOT EXISTS "${_oneq_nanoflann_include_dir}/nanoflann.hpp"
       AND EXISTS "${ONEQ_THIRD_PARTY_DIR}/nanoflann/nanoflann.hpp")
        # 兼容仅分发单头文件、未保留上游 include/ 目录的离线依赖包。
        set(_oneq_nanoflann_include_dir "${ONEQ_THIRD_PARTY_DIR}/nanoflann")
    endif()
    add_library(nanoflann_vendor INTERFACE IMPORTED)
    set_target_properties(nanoflann_vendor PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${_oneq_nanoflann_include_dir}")
    add_library(nanoflann::nanoflann ALIAS nanoflann_vendor)
    message(STATUS "nanoflann v1.3.2: vendored header-only @ ${_oneq_nanoflann_include_dir}")
    unset(_oneq_nanoflann_include_dir)
endif()

# ---------------------------------------------------------------------------
# flatbuffers v1.12.0：运行时头是 header-only（include/flatbuffers/）。
# flatc 工具的发现由 codegen/FlatBuffers.cmake 的 setup_flatc() 统一处理：
#   1) flatbuffers::flatc target；2) FLATBUFFERS_FLATC_EXECUTABLE；3) find_program(flatc) 兜底。
# 本 provider 额外暴露 flatbuffers::flatc imported target，指向 add_subdirectory 构建出的
# flatc，使 setup_flatc() 优先命中 target 路径（与 conan provider 行为一致）。
# ---------------------------------------------------------------------------
if(NOT TARGET flatbuffers::flatbuffers)
    add_library(flatbuffers_runtime_vendor INTERFACE IMPORTED)
    set_target_properties(flatbuffers_runtime_vendor PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${ONEQ_THIRD_PARTY_DIR}/flatbuffers/include")
    add_library(flatbuffers::flatbuffers ALIAS flatbuffers_runtime_vendor)
    message(STATUS "flatbuffers v1.12.0: vendored runtime headers @ ${ONEQ_THIRD_PARTY_DIR}/flatbuffers/include")
endif()

# flatc 工具：从 third_party/flatbuffers 源码构建。flatbuffers v1.12.0 的 add_subdirectory
# 在开启 FLATBUFFERS_BUILD_FLATC 时产出 flatc 可执行 target（注意是 executable，非 library，
# 故不能用 add_library ALIAS）。为隔离其选项污染，保存/恢复关键 cache。
# 构建输出走 EXCLUDE_FROM_ALL 的独立 binary 目录，避免 flatbuffers 自带 tests/examples 进主构建。
if(NOT TARGET flatc)
    set(_oneq_fb_prev_build_tests ${FLATBUFFERS_BUILD_TESTS})
    set(FLATBUFFERS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(FLATBUFFERS_BUILD_FLATC ON CACHE BOOL "" FORCE)
    set(FLATBUFFERS_INSTALL OFF CACHE BOOL "" FORCE)
    # flatbuffers v1.12.0 的 CMakeLists 声明 cmake_minimum_required(2.8.x)，现代 CMake
    # (3.27+/4.x) 会拒绝。CMAKE_POLICY_VERSION_MINIMUM 放宽兼容下限，让旧依赖能配置。
    set(_oneq_prev_policy_min ${CMAKE_POLICY_VERSION_MINIMUM})
    set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
    add_subdirectory("${ONEQ_THIRD_PARTY_DIR}/flatbuffers"
                     "${CMAKE_BINARY_DIR}/third_party/flatbuffers" EXCLUDE_FROM_ALL)
    if(_oneq_prev_policy_min STREQUAL "")
        unset(CMAKE_POLICY_VERSION_MINIMUM)
    else()
        set(CMAKE_POLICY_VERSION_MINIMUM ${_oneq_prev_policy_min})
    endif()
    unset(_oneq_prev_policy_min)
    set(FLATBUFFERS_BUILD_TESTS ${_oneq_fb_prev_build_tests} CACHE BOOL "" FORCE)
    unset(_oneq_fb_prev_build_tests)
    if(TARGET flatc)
        # 通过 FLATBUFFERS_FLATC_EXECUTABLE 变量把构建出的 flatc 路径交给 setup_flatc()
        # （codegen/FlatBuffers.cmake 的第二个解析分支）。用生成器表达式 $<TARGET_FILE:flatc>
        # 才能在多配置生成器下按 CONFIG 取到正确二进制，并自动建立 schema 生成对 flatc 的依赖。
        # 注意：该变量在配置期是字面生成器表达式，setup_flatc() 会跳过配置期版本探测。
        set(FLATBUFFERS_FLATC_EXECUTABLE "$<TARGET_FILE:flatc>" CACHE INTERNAL
            "Vendored flatc built from third_party/flatbuffers")
        message(STATUS "flatc: built from third_party/flatbuffers (v1.12.0)")
    else()
        message(WARNING
            "flatc target 未由 third_party/flatbuffers 产出；将回退到 find_program(flatc)。")
    endif()
endif()

# ---------------------------------------------------------------------------
# zlib v1.3.1：需编译静态库（ReplayTrace/Maneuver 使用 gzopen 运行时 API）。
# zlib 的 CMakeLists 默认产出 zlib（共享）与 zlibstatic（静态）。
#
# 关键：ZLIB::ZLIB 必须是 IMPORTED target 而非指向 zlibstatic 的 ALIAS。
# 因为 1q_lib 链接 ZLIB::ZLIB，install(EXPORT 1qTargets) 会枚举其链接闭包：
# 若 ZLIB::ZLIB 是 ALIAS→普通 target zlibstatic，zlibstatic 会被卷入导出集，
# 触发 "target zlibstatic is not in any export set" 错误。IMPORTED target 视作
# 外部依赖（与 conan provider 的 ZLIB::ZLIB 行为一致），不进导出集。
# 链接关系通过 IMPORTED 的 INTERFACE_LINK_LIBRARIES 以生成器表达式指向 zlibstatic，
# 既保留正确链接，又不污染导出闭包。
# ---------------------------------------------------------------------------
if(NOT TARGET ZLIB::ZLIB)
    set(_oneq_zlib_prev_shared ${BUILD_SHARED_LIBS})
    set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
    # zlib v1.3.1 的 CMakeLists 同样声明旧版 cmake_minimum_required，需放宽兼容下限。
    set(_oneq_prev_policy_min ${CMAKE_POLICY_VERSION_MINIMUM})
    set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
    add_subdirectory("${ONEQ_THIRD_PARTY_DIR}/zlib"
                     "${CMAKE_BINARY_DIR}/third_party/zlib" EXCLUDE_FROM_ALL)
    if(_oneq_prev_policy_min STREQUAL "")
        unset(CMAKE_POLICY_VERSION_MINIMUM)
    else()
        set(CMAKE_POLICY_VERSION_MINIMUM ${_oneq_prev_policy_min})
    endif()
    unset(_oneq_prev_policy_min)
    set(BUILD_SHARED_LIBS ${_oneq_zlib_prev_shared} CACHE BOOL "" FORCE)
    unset(_oneq_zlib_prev_shared)
    # zlibstatic 是 zlib CMake 的静态产物 target；优先用之，回退到 zlib。
    if(TARGET zlibstatic)
        set(_oneq_zlib_real_target zlibstatic)
    elseif(TARGET zlib)
        set(_oneq_zlib_real_target zlib)
    else()
        message(FATAL_ERROR
            "zlib add_subdirectory 未产出 zlibstatic/zlib target；请检查 third_party/zlib。")
    endif()
    # IMPORTED INTERFACE target：链接真实 zlib target，但自身对外是导入目标（不进导出集）。
    add_library(ZLIB::ZLIB INTERFACE IMPORTED)
    target_link_libraries(ZLIB::ZLIB INTERFACE ${_oneq_zlib_real_target})
    # zlib 头文件路径：third_party/zlib 根目录含 zlib.h。
    set_target_properties(ZLIB::ZLIB PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${ONEQ_THIRD_PARTY_DIR}/zlib")
    message(STATUS "zlib v1.3.1: vendored static build @ ${ONEQ_THIRD_PARTY_DIR}/zlib")
    unset(_oneq_zlib_real_target)
endif()

# ---------------------------------------------------------------------------
# boost 1.85.0（header-only）：airborne_radar 使用 boost/math/special_functions/gamma.hpp
# 与 boost/pool/object_pool.hpp。boost 是 modular superproject，源码包（由
# scripts/win/fetch_third_party.bat 下载解压）自带预合并的 boost/ 头目录，无需 b2 headers。
# 故 third_party/boost 即 boost/ 头父目录。
# ---------------------------------------------------------------------------
if(NOT TARGET Boost::boost)
    if(NOT EXISTS "${ONEQ_THIRD_PARTY_DIR}/boost/boost")
        message(FATAL_ERROR
            "PACKAGE_MANAGER=none 缺失 boost 头：${ONEQ_THIRD_PARTY_DIR}/boost/boost\n"
            "请重跑 scripts\\win\\fetch_third_party.bat（含 boost 源码包下载步骤）。")
    endif()
    add_library(boost_vendor INTERFACE IMPORTED)
    set_target_properties(boost_vendor PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${ONEQ_THIRD_PARTY_DIR}/boost")
    add_library(Boost::boost ALIAS boost_vendor)
    message(STATUS "boost 1.85.0: vendored header-only @ ${ONEQ_THIRD_PARTY_DIR}/boost")
endif()

# ---------------------------------------------------------------------------
# sqlite3 3.53.4：需编译静态库（airborne_radar 识别特征库加载）。
# amalgamation（sqlite3.c/sqlite3.h 单文件）由 scripts/win/fetch_third_party.bat 从
# sqlite.org 发布包下载解压至 third_party/sqlite/（GitHub sqlite 镜像只含 src/*.c
# 拆分源，不含 amalgamation）。直接单文件编译，不引入官方 CMakeLists（其 target
# 命名/示例构建与项目约定不一致）。
# 与 zlib 同构：对外暴露 IMPORTED INTERFACE SQLite::SQLite3（与 conan provider 的
# CMakeDeps target 名一致），链接真实静态 target；IMPORTED 视作外部依赖，
# 不参与 install(EXPORT) 导出闭包。
# 本路径面向 Windows no-Conan（PACKAGE_MANAGER=none），在 macOS/Linux 开发机上不执行。
# ---------------------------------------------------------------------------
if(NOT TARGET SQLite::SQLite3)
    if(NOT EXISTS "${ONEQ_THIRD_PARTY_DIR}/sqlite/sqlite3.c")
        message(FATAL_ERROR
            "PACKAGE_MANAGER=none 缺失 sqlite3 amalgamation：${ONEQ_THIRD_PARTY_DIR}/sqlite/sqlite3.c\n"
            "请重跑 scripts\\win\\fetch_third_party.bat。")
    endif()
    add_library(sqlite3_vendor STATIC
        "${ONEQ_THIRD_PARTY_DIR}/sqlite/sqlite3.c")
    target_include_directories(sqlite3_vendor PUBLIC
        "${ONEQ_THIRD_PARTY_DIR}/sqlite")
    add_library(SQLite::SQLite3 INTERFACE IMPORTED)
    target_link_libraries(SQLite::SQLite3 INTERFACE sqlite3_vendor)
    message(STATUS "sqlite3 3.53.4: vendored amalgamation @ ${ONEQ_THIRD_PARTY_DIR}/sqlite")
endif()

# HDF5 输出能力依赖 HighFive，其要求 C++17；低标准构建时静默关闭。
# none 模式面向 VS2015（C++14），HighFive 本就不启用。
if(PROJECT_CXX_STANDARD GREATER_EQUAL 17)
    message(FATAL_ERROR
        "PACKAGE_MANAGER=none 当前未 vendoring HighFive；C++17 构建请改用 conan provider。")
else()
    set(ONEQ_ENABLE_HDF5_OUTPUT OFF)
    message(STATUS "SAR HDF5 output: disabled (none provider, requires C++17 + HighFive)")
endif()

# 标记 zlib 可用；导出公共库 target 需链接的依赖列表（与 ConanPackages.cmake 完全一致）。
set(ONEQ_HAVE_ZLIB ON)
set(ONEQ_LINK_DEPENDENCIES
    Eigen3::Eigen
    Boost::boost
    nanoflann::nanoflann
    flatbuffers::flatbuffers)
