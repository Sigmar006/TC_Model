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
# VS2013 (MSVC 1800) 兼容补丁：Eigen 3.4.0 的 internal::aligned_storage 用
# `__declspec(align(<模板非类型参数>))` 强制对齐，而 MSVC 2013 无法解析依赖的
# 对齐参数（C2059），且没有 alignas 可用。这里仅在 MSVC < 1900 时把它替换为按
# 字面量对齐值偏特化的等价实现（对齐集合覆盖 Eigen 实际使用的 8/16/32/64）。
# 仅 SparseRef 使用该结构；本库未用 Eigen Sparse，故语义影响受限。
# 幂等：补丁写入后不再含原始特征串，重复 configure 不会重复改写。
# ---------------------------------------------------------------------------
function(oneq_patch_eigen_vs2013 eigen_include_dir)
    set(_meta "${eigen_include_dir}/Eigen/src/Core/util/Meta.h")
    if(NOT EXISTS "${_meta}")
        return()
    endif()
    file(READ "${_meta}" _meta_text)
    # checkout 后该文件为 CRLF（core.autocrlf），统一转 LF 后再匹配/写回，避免漏配。
    string(REPLACE "\r\n" "\n" _meta_text "${_meta_text}")
    set(_orig [=[
template<unsigned Len, unsigned Align>
struct aligned_storage {
  struct type {
    EIGEN_ALIGN_TO_BOUNDARY(Align) unsigned char data[Len];
  };
};]=])
    string(FIND "${_meta_text}" "${_orig}" _hit)
    # 幂等：补丁后文件含 aligned_storage_impl 标记；已打过则跳过，否则 #else 分支里
    # 保留的原文会被反复匹配、层层嵌套。
    if(NOT _meta_text MATCHES "aligned_storage_impl" AND NOT _hit EQUAL -1)
    set(_repl [=[
#if defined(_MSC_VER) && _MSC_VER < 1900
template<unsigned Len, unsigned Align, unsigned Dummy = 0>
struct aligned_storage_impl {
  struct type { unsigned char data[Len]; };
};
template<unsigned Len>
struct aligned_storage_impl<Len, 8> {
  struct type { __declspec(align(8)) unsigned char data[Len]; };
};
template<unsigned Len>
struct aligned_storage_impl<Len, 16> {
  struct type { __declspec(align(16)) unsigned char data[Len]; };
};
template<unsigned Len>
struct aligned_storage_impl<Len, 32> {
  struct type { __declspec(align(32)) unsigned char data[Len]; };
};
template<unsigned Len>
struct aligned_storage_impl<Len, 64> {
  struct type { __declspec(align(64)) unsigned char data[Len]; };
};
template<unsigned Len, unsigned Align>
struct aligned_storage : aligned_storage_impl<Len, Align> {};
#else
template<unsigned Len, unsigned Align>
struct aligned_storage {
  struct type {
    EIGEN_ALIGN_TO_BOUNDARY(Align) unsigned char data[Len];
  };
};
#endif]=])
    string(REPLACE "${_orig}" "${_repl}" _meta_patched "${_meta_text}")
    file(WRITE "${_meta}" "${_meta_patched}")
    message(STATUS "eigen vs2013 compat: patched aligned_storage in ${_meta}")
    endif()

    # GenericPacketMathFunctions.h：局部缓冲用 __declspec(align(sizeof(Packet)))，
    # sizeof(Packet) 在 VS2013 下同为不可解析的依赖表达式。这些缓冲只用
    # pstoreu/ploadu（非对齐读写），故用足够大的字面量 64 对齐替换即可。
    set(_gpmf "${eigen_include_dir}/Eigen/src/Core/arch/Default/GenericPacketMathFunctions.h")
    if(EXISTS "${_gpmf}")
        file(READ "${_gpmf}" _gpmf_text)
        string(REPLACE "\r\n" "\n" _gpmf_text "${_gpmf_text}")
        string(REPLACE "EIGEN_ALIGN_TO_BOUNDARY(sizeof(Packet))"
                       "EIGEN_ALIGN_TO_BOUNDARY(64)"
                       _gpmf_patched "${_gpmf_text}")
        if(NOT _gpmf_patched STREQUAL _gpmf_text)
            file(WRITE "${_gpmf}" "${_gpmf_patched}")
            message(STATUS "eigen vs2013 compat: patched GenericPacketMathFunctions in ${_gpmf}")
        endif()
    endif()
endfunction()

# ---------------------------------------------------------------------------
# eigen（header-only）：仓库根目录即 Eigen/ 父目录。
# VS2013（MSVC<1900）用 3.3.9：Eigen 3.4 在 MSVC 1800 上连基本的矩阵/向量乘积都会
# 触发编译器内部错误（fatal C1001），3.3.9 是最后正式支持 VS2013 的版本，无需补丁。
# 3.4 路径若仍面向 MSVC<1900，则走 oneq_patch_eigen_vs2013 补丁（保留以备选用）。
# ---------------------------------------------------------------------------
if(NOT TARGET Eigen3::Eigen)
    if(MSVC AND MSVC_VERSION LESS 1900)
        set(_oneq_eigen_root "${ONEQ_THIRD_PARTY_DIR}/eigen_vs2013")
        set(_oneq_eigen_version "3.3.9")
    else()
        set(_oneq_eigen_root "${ONEQ_THIRD_PARTY_DIR}/eigen")
        set(_oneq_eigen_version "3.4.0")
    endif()
    set(_oneq_eigen_include_dir "${_oneq_eigen_root}")
    if(NOT EXISTS "${_oneq_eigen_include_dir}/Eigen/Core"
       AND EXISTS "${_oneq_eigen_root}/Core")
        # 某些离线包把 Eigen/ 目录内容直接铺在根下。生成仅供构建消费的标准
        # Eigen/<header> + Eigen/src 布局，避免修改 third_party 原始依赖。
        set(_oneq_eigen_flat_root "${_oneq_eigen_root}")
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
    if(MSVC AND MSVC_VERSION LESS 1900 AND _oneq_eigen_version STREQUAL "3.4.0")
        oneq_patch_eigen_vs2013("${_oneq_eigen_include_dir}")
    endif()
    add_library(eigen_vendor INTERFACE IMPORTED)
    set_target_properties(eigen_vendor PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${_oneq_eigen_include_dir}")
    add_library(Eigen3::Eigen ALIAS eigen_vendor)
    message(STATUS "eigen ${_oneq_eigen_version}: vendored header-only @ ${_oneq_eigen_include_dir}")
    unset(_oneq_eigen_include_dir)
    unset(_oneq_eigen_root)
    unset(_oneq_eigen_version)
endif()

# ---------------------------------------------------------------------------
# nanoflann（header-only）：单头 include/nanoflann.hpp。
# VS2013（MSVC<1900）用 v1.2.3：1.3.x 引入的表达式 SFINAE（decltype + declval）
# 在 MSVC 2013 上不通过（C2228）。v1.2.3 要求数据集提供 kdtree_distance，两个
# adaptor 已补齐该接口；其余 API（SearchParams.sorted / radiusSearch(pair)）兼容。
# ---------------------------------------------------------------------------
if(NOT TARGET nanoflann::nanoflann)
    if(MSVC AND MSVC_VERSION LESS 1900)
        set(_oneq_nanoflann_root "${ONEQ_THIRD_PARTY_DIR}/nanoflann_vs2013")
        set(_oneq_nanoflann_version "v1.2.3")
    else()
        set(_oneq_nanoflann_root "${ONEQ_THIRD_PARTY_DIR}/nanoflann")
        set(_oneq_nanoflann_version "v1.3.2")
    endif()
    set(_oneq_nanoflann_include_dir "${_oneq_nanoflann_root}/include")
    if(NOT EXISTS "${_oneq_nanoflann_include_dir}/nanoflann.hpp"
       AND EXISTS "${_oneq_nanoflann_root}/nanoflann.hpp")
        # 兼容仅分发单头文件、未保留上游 include/ 目录的离线依赖包。
        set(_oneq_nanoflann_include_dir "${_oneq_nanoflann_root}")
    endif()
    add_library(nanoflann_vendor INTERFACE IMPORTED)
    set_target_properties(nanoflann_vendor PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${_oneq_nanoflann_include_dir}")
    add_library(nanoflann::nanoflann ALIAS nanoflann_vendor)
    message(STATUS "nanoflann ${_oneq_nanoflann_version}: vendored header-only @ ${_oneq_nanoflann_include_dir}")
    unset(_oneq_nanoflann_include_dir)
    unset(_oneq_nanoflann_root)
    unset(_oneq_nanoflann_version)
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
# boost（header-only）：airborne_radar 使用 boost/math/special_functions/gamma.hpp
# 与 boost/pool/object_pool.hpp，ESR 使用 boost/multiprecision/cpp_int.hpp。boost 是
# modular superproject，源码包（由 scripts/win/fetch_third_party.bat 下载解压）自带
# 预合并的 boost/ 头目录，无需 b2 headers。
# VS2013（MSVC<1900）用 1.68.0：1.85 的 math/multiprecision 要求 C++14（noexcept、
# constexpr），MSVC 2013 无法编译。故 third_party/boost_vs2013 即该版本 boost/ 头父目录。
# ---------------------------------------------------------------------------
if(NOT TARGET Boost::boost)
    if(MSVC AND MSVC_VERSION LESS 1900)
        set(_oneq_boost_root "${ONEQ_THIRD_PARTY_DIR}/boost_vs2013")
        set(_oneq_boost_version "1.68.0")
    else()
        set(_oneq_boost_root "${ONEQ_THIRD_PARTY_DIR}/boost")
        set(_oneq_boost_version "1.85.0")
    endif()
    if(NOT EXISTS "${_oneq_boost_root}/boost")
        message(FATAL_ERROR
            "PACKAGE_MANAGER=none 缺失 boost 头：${_oneq_boost_root}/boost\n"
            "请重跑 scripts\\win\\fetch_third_party.bat（含 boost 源码包下载步骤）。")
    endif()
    add_library(boost_vendor INTERFACE IMPORTED)
    set_target_properties(boost_vendor PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${_oneq_boost_root}")
    add_library(Boost::boost ALIAS boost_vendor)
    message(STATUS "boost ${_oneq_boost_version}: vendored header-only @ ${_oneq_boost_root}")
    unset(_oneq_boost_root)
    unset(_oneq_boost_version)
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
