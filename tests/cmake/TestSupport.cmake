# Dependency discovery, source collection and the common 1q gtest harness.

if(MSVC AND MSVC_VERSION LESS 1910)
    set(CMAKE_MAP_IMPORTED_CONFIG_DEBUG Release)
    set(CMAKE_MAP_IMPORTED_CONFIG_RELWITHDEBINFO Release)
    set(CMAKE_MAP_IMPORTED_CONFIG_MINSIZEREL Release)
endif()

if(PACKAGE_MANAGER STREQUAL "conan")
    find_package(GTest CONFIG REQUIRED)
    find_package(rapidcheck CONFIG REQUIRED)
    find_package(Boost CONFIG REQUIRED)
    find_package(Eigen3 CONFIG REQUIRED)
    find_package(flatbuffers CONFIG REQUIRED)
    find_package(ZLIB QUIET)
endif()
if(NOT DEFINED flatbuffers_INCLUDE_DIRS_RELEASE)
    file(GLOB _oneq_flatbuffers_data_candidates
        "${CMAKE_BINARY_DIR}/build/generators/flatbuffers-*-data.cmake")
    foreach(_oneq_flatbuffers_data IN LISTS _oneq_flatbuffers_data_candidates)
        if(NOT _oneq_flatbuffers_data MATCHES "module-flatbuffers")
            include("${_oneq_flatbuffers_data}")
            break()
        endif()
    endforeach()
    unset(_oneq_flatbuffers_data)
    unset(_oneq_flatbuffers_data_candidates)
endif()

file(GLOB_RECURSE UNIT_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/*.cpp")
file(GLOB_RECURSE INTEGRATION_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/integration/*.cpp")
file(GLOB_RECURSE CONTRACT_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/contract/*.cpp")
file(GLOB_RECURSE PERFORMANCE_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/performance/*.cpp")

set(TEST_SOURCES
    ${UNIT_TEST_SOURCES}
    ${INTEGRATION_TEST_SOURCES}
    ${CONTRACT_TEST_SOURCES})

include(GoogleTest)
function(add_1q_gtest target_name suite_label discovery_timeout)
    if(NOT ARGN)
        return()
    endif()

    add_executable(${target_name} ${ARGN})
    target_link_libraries(${target_name} PRIVATE
        GTest::gtest_main
        GTest::gmock
        Boost::boost
        Eigen3::Eigen
        ${PROJECT_ALIAS})
    if(TARGET flatbuffers::flatbuffers)
        target_link_libraries(${target_name} PRIVATE flatbuffers::flatbuffers)
    endif()
    # 属性测试库只在 conan 且 enable_testing 时存在；no-Conan 的 Windows
    # 预设（VisualStudio.14.0-amd64-none）仍须能 configure。
    if(TARGET rapidcheck::rapidcheck)
        target_link_libraries(${target_name} PRIVATE rapidcheck::rapidcheck)
    endif()
    target_include_directories(${target_name} PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR})
    if(DEFINED ONEQ_FLATBUFFERS_GENERATED_DIR)
        target_include_directories(${target_name} PRIVATE
            ${ONEQ_FLATBUFFERS_GENERATED_DIR})
    endif()
    if(DEFINED flatbuffers_INCLUDE_DIRS_RELEASE)
        target_include_directories(${target_name} PRIVATE
            ${flatbuffers_INCLUDE_DIRS_RELEASE})
    endif()
    if(WIN32)
        target_compile_definitions(${target_name} PRIVATE
            FLATBUFFERS_LOCALE_INDEPENDENT=0)
    endif()
    if(DEFINED ONEQ_TEST_TEMP_DIR)
        target_compile_definitions(${target_name} PRIVATE
            ONEQ_TEST_TEMP_DIR="${ONEQ_TEST_TEMP_DIR}")
    endif()

    if(TARGET ZLIB::ZLIB)
        target_compile_definitions(${target_name} PRIVATE ONEQ_HAVE_ZLIB=1)
        if(DEFINED zlib_INCLUDE_DIRS_RELEASE)
            target_include_directories(${target_name} PRIVATE ${zlib_INCLUDE_DIRS_RELEASE})
        endif()
    else()
        target_compile_definitions(${target_name} PRIVATE ONEQ_HAVE_ZLIB=0)
    endif()

    oneq_apply_target_configuration(TARGETS ${target_name})

    if(WIN32 AND BUILD_SHARED_LIBS)
        add_custom_command(TARGET ${target_name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:${PROJECT_CORE_TARGET}>
                $<TARGET_FILE_DIR:${target_name}>
            COMMENT "Copying ${PROJECT_CORE_TARGET} DLL to test directory")
    endif()

    add_test(NAME "${suite_label}::${target_name}" COMMAND ${target_name})
    set_tests_properties("${suite_label}::${target_name}" PROPERTIES LABELS "${suite_label}")
endfunction()

# --------------------------------------------------------------------------
# Phase 2+: type × domain partition registration API.
#
# oneq_add_test_partition(
#   TYPE <unit|integration|replay|contract|performance|compatibility>
#   DOMAIN <common|airborne_radar|...>
#   SOURCES <source...>
#   [TIMEOUT <seconds>]
#   [LABELS <strategy-label...>]   # e.g. ci_required, ci_advisory, fast, slow, known_limit
#   [LINK_LIBS <lib...>]
#   [INCLUDE_DIRS <dir...>]
#   [COMPILE_DEFS <def...>]
#   [DEPENDS <target...>]
#   [EXTRA_SOURCES <non-test-source...>]   # e.g. json_reader.cpp helper
# )
#
# Produces:
#   - executable target  1q_<domain>_<type>_tests
#   - CTest entry        <type>::<domain>
#   - labels             <type>;<domain>;<strategy-labels...>
#   - registered sources in ONEQ_TEST_REGISTRY (via oneq_register_test_sources)
function(oneq_add_test_partition)
    cmake_parse_arguments(_oneq_part
        "GATED" "TYPE;DOMAIN;TIMEOUT" "SOURCES;LABELS;LINK_LIBS;INCLUDE_DIRS;COMPILE_DEFS;DEPENDS;EXTRA_SOURCES"
        ${ARGN})
    if(NOT _oneq_part_SOURCES)
        return()
    endif()

    set(_target "${PROJECT_NAME}_${_oneq_part_DOMAIN}_${_oneq_part_TYPE}_tests")
    set(_ctest "${_oneq_part_TYPE}::${_oneq_part_DOMAIN}")

    add_executable(${_target} ${_oneq_part_SOURCES} ${_oneq_part_EXTRA_SOURCES})
    target_link_libraries(${_target} PRIVATE
        GTest::gtest_main
        GTest::gmock
        Boost::boost
        Eigen3::Eigen
        ${PROJECT_ALIAS})
    if(TARGET flatbuffers::flatbuffers)
        target_link_libraries(${_target} PRIVATE flatbuffers::flatbuffers)
    endif()
    if(TARGET rapidcheck::rapidcheck)
        target_link_libraries(${_target} PRIVATE rapidcheck::rapidcheck)
    endif()
    if(_oneq_part_LINK_LIBS)
        target_link_libraries(${_target} PRIVATE ${_oneq_part_LINK_LIBS})
    endif()
    target_include_directories(${_target} PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR})
    if(DEFINED ONEQ_FLATBUFFERS_GENERATED_DIR)
        target_include_directories(${_target} PRIVATE ${ONEQ_FLATBUFFERS_GENERATED_DIR})
    endif()
    if(DEFINED flatbuffers_INCLUDE_DIRS_RELEASE)
        target_include_directories(${_target} PRIVATE ${flatbuffers_INCLUDE_DIRS_RELEASE})
    endif()
    if(_oneq_part_INCLUDE_DIRS)
        target_include_directories(${_target} PRIVATE ${_oneq_part_INCLUDE_DIRS})
    endif()
    if(WIN32)
        target_compile_definitions(${_target} PRIVATE FLATBUFFERS_LOCALE_INDEPENDENT=0)
    endif()
    if(DEFINED ONEQ_TEST_TEMP_DIR)
        target_compile_definitions(${_target} PRIVATE ONEQ_TEST_TEMP_DIR="${ONEQ_TEST_TEMP_DIR}")
    endif()
    if(TARGET ZLIB::ZLIB)
        target_compile_definitions(${_target} PRIVATE ONEQ_HAVE_ZLIB=1)
        if(DEFINED zlib_INCLUDE_DIRS_RELEASE)
            target_include_directories(${_target} PRIVATE ${zlib_INCLUDE_DIRS_RELEASE})
        endif()
    else()
        target_compile_definitions(${_target} PRIVATE ONEQ_HAVE_ZLIB=0)
    endif()
    if(_oneq_part_COMPILE_DEFS)
        target_compile_definitions(${_target} PRIVATE ${_oneq_part_COMPILE_DEFS})
    endif()
    oneq_apply_target_configuration(TARGETS ${_target})
    if(WIN32 AND BUILD_SHARED_LIBS)
        add_custom_command(TARGET ${_target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:${PROJECT_CORE_TARGET}>
                $<TARGET_FILE_DIR:${_target}>
            COMMENT "Copying ${PROJECT_CORE_TARGET} DLL to test directory")
    endif()
    if(_oneq_part_DEPENDS)
        add_dependencies(${_target} ${_oneq_part_DEPENDS})
    endif()

    set(_labels "${_oneq_part_TYPE};${_oneq_part_DOMAIN}")
    if(_oneq_part_LABELS)
        list(APPEND _labels ${_oneq_part_LABELS})
    endif()
    add_test(NAME "${_ctest}" COMMAND ${_target})
    set_tests_properties("${_ctest}" PROPERTIES LABELS "${_labels}")
    if(_oneq_part_TIMEOUT)
        set_tests_properties("${_ctest}" PROPERTIES TIMEOUT "${_oneq_part_TIMEOUT}")
    endif()

    # Record test sources (not EXTRA_SOURCES) in the registry.
    oneq_register_test_sources("${_oneq_part_TYPE}.${_oneq_part_DOMAIN}" ${_oneq_part_SOURCES})
endfunction()
