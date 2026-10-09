# 开发期度量：外部（双星交会）位置驱动 RIR 的端到端验证（不属于主构建树）。
#
# 用法（先完成主库构建与安装）：
#   cmake -S tools/measure_rir_from_fix -B build/tools/measure_rir_from_fix ^
#         -G "Visual Studio 18 2026" -A x64 -T v141 ^
#         -DCMAKE_PREFIX_PATH="<repo>/build/install/VisualStudio.15.0-amd64;<repo>/build/VisualStudio.15.0-amd64/build/generators"
#   cmake --build build/tools/measure_rir_from_fix --config Release
#   build/tools/measure_rir_from_fix/Release/measure_rir_from_external_fix.exe ^
#         --db <repo>/examples/basic_config/remote_identification_radar/target_feature_database_v1.1.db
cmake_minimum_required(VERSION 3.25)
project(measure_rir_from_fix LANGUAGES CXX)

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

add_executable(measure_rir_from_external_fix measure_rir_from_external_fix.cpp)
target_link_libraries(measure_rir_from_external_fix PRIVATE ${ONEQ_LINK_TARGET})

if(MSVC)
    target_compile_options(measure_rir_from_external_fix PRIVATE /W4)
    target_compile_definitions(measure_rir_from_external_fix PRIVATE
        _USE_MATH_DEFINES
        _CRT_SECURE_NO_WARNINGS
        ONEQ_STATIC_DEFINE)
else()
    target_compile_options(measure_rir_from_external_fix PRIVATE -Wall -Wextra)
endif()
