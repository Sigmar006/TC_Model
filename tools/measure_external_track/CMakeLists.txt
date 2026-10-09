# 开发期度量：外部（双星交会）位置驱动 RIR 的端到端验证（不属于主构建树）。
#
# 用法（先完成主库构建与安装）：
#   cmake -S tools/measure_external_track -B build/tools/measure_external_track ^
#         -G "Visual Studio 18 2026" -A x64 -T v141 ^
#         -DCMAKE_PREFIX_PATH="<repo>/build/install/VisualStudio.15.0-amd64;<repo>/build/VisualStudio.15.0-amd64/build/generators"
#   cmake --build build/tools/measure_external_track --config Release
#   build/tools/measure_external_track/Release/measure_external_track.exe ^
#         --db <repo>/examples/basic_config/remote_identification_radar/target_feature_database_v1.1.db
cmake_minimum_required(VERSION 3.25)
project(measure_external_track LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(1q CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(Boost CONFIG REQUIRED)
find_package(nanoflann CONFIG REQUIRED)
find_package(flatbuffers CONFIG REQUIRED)
find_package(ZLIB CONFIG REQUIRED)
find_package(SQLite3 CONFIG REQUIRED)

if(TARGET 1q::1q)
    set(ONEQ_LINK_TARGET 1q::1q)
elseif(TARGET 1q::core)
    set(ONEQ_LINK_TARGET 1q::core)
else()
    message(FATAL_ERROR "未找到 1q 库目标（1q::1q / 1q::core）；请检查 CMAKE_PREFIX_PATH。")
endif()

add_executable(measure_external_track measure_external_track.cpp)
target_link_libraries(measure_external_track PRIVATE ${ONEQ_LINK_TARGET})

if(MSVC)
    target_compile_options(measure_external_track PRIVATE /W4)
    target_compile_definitions(measure_external_track PRIVATE
        _USE_MATH_DEFINES
        _CRT_SECURE_NO_WARNINGS
        ONEQ_STATIC_DEFINE)
else()
    target_compile_options(measure_external_track PRIVATE -Wall -Wextra)
endif()
