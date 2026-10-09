# 开发期度量：双星红外观测下融合 UKF 的速度估计精度（不属于主构建树）。
#
# 用法（先完成主库构建与安装）：
#   cmake -S tools/measure_fusion_velocity -B build/tools/measure_fusion_velocity ^
#         -G "Visual Studio 18 2026" -A x64 -T v141 ^
#         -DCMAKE_PREFIX_PATH="<repo>/build/install/VisualStudio.15.0-amd64;<repo>/build/VisualStudio.15.0-amd64/build/generators"
#   cmake --build build/tools/measure_fusion_velocity --config Release
#   build/tools/measure_fusion_velocity/Release/measure_fusion_velocity.exe ^
#         --scene-dir <场景输出目录> --target 1
cmake_minimum_required(VERSION 3.25)
project(measure_fusion_velocity LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(1q CONFIG REQUIRED)
# 1q 的导出接口引用第三方依赖目标，使用者须先 find_package（否则裸库名交给链接器报 LNK1104）。
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

add_executable(measure_fusion_velocity measure_fusion_velocity.cpp)
target_link_libraries(measure_fusion_velocity PRIVATE ${ONEQ_LINK_TARGET})

if(MSVC)
    target_compile_options(measure_fusion_velocity PRIVATE /W4)
    target_compile_definitions(measure_fusion_velocity PRIVATE
        _USE_MATH_DEFINES
        _CRT_SECURE_NO_WARNINGS
        ONEQ_STATIC_DEFINE)
else()
    target_compile_options(measure_fusion_velocity PRIVATE -Wall -Wextra)
endif()
