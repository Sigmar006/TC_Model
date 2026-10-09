# 开发期探针：识别特征提取器直调（不属于主构建树）。
cmake_minimum_required(VERSION 3.25)
project(probe_recognition_features LANGUAGES CXX)

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
else()
    set(ONEQ_LINK_TARGET 1q::core)
endif()

add_executable(probe_recognition_features probe_recognition_features.cpp)

# 识别提取器是 src/ 私有头，须显式指向源码树。
target_include_directories(probe_recognition_features PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../src"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../include")

target_link_libraries(probe_recognition_features PRIVATE ${ONEQ_LINK_TARGET})

if(MSVC)
    target_compile_options(probe_recognition_features PRIVATE /W4)
    target_compile_definitions(probe_recognition_features PRIVATE
        _USE_MATH_DEFINES
        _CRT_SECURE_NO_WARNINGS
        ONEQ_STATIC_DEFINE)
endif()
