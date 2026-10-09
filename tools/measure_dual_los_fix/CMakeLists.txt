# 开发期度量工具的独立构建入口（不属于主构建树，避免污染 examples/tests 布局）。
#
# 用法（先完成主库构建与安装，见 README）：
#   cmake -S tools/measure_dual_los_fix -B build/tools/measure_dual_los_fix ^
#         -G "Visual Studio 18 2026" -A x64 -T v141 ^
#         -DCMAKE_PREFIX_PATH="<repo>/build/install/VisualStudio.15.0-amd64;<repo>/build/VisualStudio.15.0-amd64/build/generators"
#   cmake --build build/tools/measure_dual_los_fix --config Release
#
# 说明：本工具链接已安装的 1q 静态库；包含 src/ 下的 SBIRS 误差模型私有头
# （开发期白盒度量，与 tests 的 src/ 包含惯例一致）。
cmake_minimum_required(VERSION 3.25)
project(measure_dual_los_fix LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(1q CONFIG REQUIRED)

# 1q 的导出接口引用第三方依赖目标（INTERFACE_LINK_LIBRARIES 里的
# Eigen3::Eigen / Boost::boost / nanoflann::nanoflann / flatbuffers::flatbuffers /
# ZLIB::ZLIB / SQLite::SQLite3）。这些目标由依赖自身的 CMake 包配置定义，
# 使用者须先 find_package，否则 CMake 会把名字当作裸库名交给链接器（LNK1104）。
# Conan 路径下这些配置文件由 CMakeDeps 生成，位于 <preset>/build/generators。
find_package(Eigen3 CONFIG REQUIRED)
find_package(Boost CONFIG REQUIRED)
find_package(nanoflann CONFIG REQUIRED)
find_package(flatbuffers CONFIG REQUIRED)
find_package(ZLIB CONFIG REQUIRED)
find_package(SQLite3 CONFIG REQUIRED)

# 构建树别名是 1q::core；已安装/导出的目标是 1q::1q（EXPORT_NAME = 项目小写名）。
if(TARGET 1q::1q)
    set(ONEQ_LINK_TARGET 1q::1q)
elseif(TARGET 1q::core)
    set(ONEQ_LINK_TARGET 1q::core)
else()
    message(FATAL_ERROR "未找到 1q 库目标（1q::1q / 1q::core）；请检查 CMAKE_PREFIX_PATH。")
endif()
message(STATUS "measure_dual_los_fix: 链接目标 = ${ONEQ_LINK_TARGET}")

add_executable(measure_dual_los_library measure_dual_los_library.cpp)

# 私有头（src/sbirs_sensor/foundation/SbirsErrorModel.h）：库只装 include/1q，
# 故白盒度量需显式指向源码树。
target_include_directories(measure_dual_los_library PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../src"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../include")

target_link_libraries(measure_dual_los_library PRIVATE ${ONEQ_LINK_TARGET})

if(MSVC)
    target_compile_options(measure_dual_los_library PRIVATE /W4)
    target_compile_definitions(measure_dual_los_library PRIVATE
        _USE_MATH_DEFINES
        _CRT_SECURE_NO_WARNINGS
        ONEQ_STATIC_DEFINE)   # 静态库语义：ONEQ_API 展开为空，消除 C4273 dll 链接不一致
else()
    target_compile_options(measure_dual_los_library PRIVATE -Wall -Wextra)
endif()
