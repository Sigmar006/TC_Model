# Product assembly only. Each module owns its sources, component targets,
# direct dependencies and replay schemas in its own subdirectory.

add_library(${PROJECT_CORE_TARGET})
add_library(${PROJECT_ALIAS} ALIAS ${PROJECT_CORE_TARGET})

include(GenerateExportHeader)
set(PROJECT_GENERATED_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/include")
set(PROJECT_GENERATED_EXPORT_HEADER
    "${PROJECT_GENERATED_INCLUDE_DIR}/1q/oneq_export.h")
file(MAKE_DIRECTORY "${PROJECT_GENERATED_INCLUDE_DIR}/1q")
generate_export_header(${PROJECT_CORE_TARGET}
    BASE_NAME ONEQ
    EXPORT_FILE_NAME "${PROJECT_GENERATED_EXPORT_HEADER}"
    STATIC_DEFINE ONEQ_STATIC_DEFINE)
set(PROJECT_GENERATED_INCLUDE_DIR "${PROJECT_GENERATED_INCLUDE_DIR}" PARENT_SCOPE)
set(PROJECT_GENERATED_EXPORT_HEADER "${PROJECT_GENERATED_EXPORT_HEADER}" PARENT_SCOPE)

target_include_directories(${PROJECT_CORE_TARGET}
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}"
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_GENERATED_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_CORE_TARGET} PUBLIC "cxx_std_${PROJECT_CXX_STANDARD}")

add_subdirectory(common)
add_subdirectory(airborne_radar)
add_subdirectory(electronic_surveillance_radar)
add_subdirectory(electronic_countermeasure)
add_subdirectory(electro_optical_sensor)
add_subdirectory(sbirs_sensor)
add_subdirectory(satellite_orbit)
add_subdirectory(sar)
add_subdirectory(remote_identification_radar)
add_subdirectory(navigation)
add_subdirectory(fusion)
add_subdirectory(threat_assessment)
add_subdirectory(target_inference)
add_subdirectory(precision_evaluation)
if(ONEQ_ENABLE_FLIGHT_DYNAMIC)
    add_subdirectory(flight_dynamic)
endif()

oneq_get_component_targets(ONEQ_OBJECT_TARGETS)
if(NOT ONEQ_OBJECT_TARGETS)
    message(FATAL_ERROR "No 1q component targets were registered")
endif()
foreach(ONEQ_OBJECT_TARGET IN LISTS ONEQ_OBJECT_TARGETS)
    target_sources(${PROJECT_CORE_TARGET} PRIVATE $<TARGET_OBJECTS:${ONEQ_OBJECT_TARGET}>)
endforeach()
unset(ONEQ_OBJECT_TARGET)

# Component targets own compilation dependencies. The public static library
# owns their final packaging link closure, without exporting implementation
# object targets as part of 1qTargets.cmake.
target_link_libraries(${PROJECT_CORE_TARGET} PRIVATE
    ${ONEQ_LINK_DEPENDENCIES})
if(PROJECT_ENABLE_SPDLOG)
    target_link_libraries(${PROJECT_CORE_TARGET} PRIVATE ${PROJECT_SPDLOG_TARGET})
endif()
if(ONEQ_HAVE_ZLIB)
    target_link_libraries(${PROJECT_CORE_TARGET} PRIVATE ZLIB::ZLIB)
endif()
# SQLite（airborne_radar 识别特征库加载；组件 PRIVATE 依赖需在此显式纳入链接闭包，
# 与 ZLIB 模式一致。两 provider（conan/vendored）均产出 SQLite::SQLite3）。
target_link_libraries(${PROJECT_CORE_TARGET} PRIVATE SQLite::SQLite3)
if(ONEQ_ENABLE_HDF5_OUTPUT)
    # 必须用 imported target 名（HighFive::HighFive）。裸名 HighFive 会被链接器当成库名
    # 解析为 -lHighFive，在 Linux/aarch64 上链接必失败（cannot find -lHighFive）。
    target_link_libraries(${PROJECT_CORE_TARGET} PRIVATE ${ONEQ_HIGHFIVE_TARGET})
endif()

oneq_apply_target_configuration(
    TARGETS ${PROJECT_CORE_TARGET}
    LINK_TARGETS ${PROJECT_CORE_TARGET})
oneq_configure_public_library(${PROJECT_CORE_TARGET})
oneq_finalize_flatbuffers()
