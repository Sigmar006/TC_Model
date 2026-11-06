# Test layout guard: enforce the type × domain source layout. Duplicate source
# ownership is enforced by TestRegistry.cmake during configure.

cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED SOURCE_DIR)
    message(FATAL_ERROR "SOURCE_DIR must be defined (-DSOURCE_DIR=...).")
endif()
# 规范化为绝对路径：GLOB ... RELATIVE 在 -P 脚本模式下，当 base 为相对路径时
# 返回空列表（静默放行违规布局）。对绝对输入幂等，仅兜底手动调用。
get_filename_component(SOURCE_DIR "${SOURCE_DIR}" ABSOLUTE)

set(_test_root "${SOURCE_DIR}/tests")

# No _test.cpp may live directly under tests/.
file(GLOB _root_tests "${_test_root}/*_test.cpp")
if(_root_tests)
    list(JOIN _root_tests "\n  " _root_list)
    message(FATAL_ERROR
        "test_layout_guard: _test.cpp must not live directly under tests/. Found:\n  ${_root_list}\n"
        "Place them under a type directory: tests/{unit,integration,contract,performance,replay,compatibility,consumer}/<domain>/")
endif()

# Ensure the type roots and their domain subdirectories match the registration
# model. This rejects a misplaced new source before it can be silently omitted
# from a partition.
set(_allowed_type_roots unit integration replay contract performance compatibility consumer)
set(_allowed_infra_dirs cmake support)
file(GLOB _entries RELATIVE "${_test_root}" "${_test_root}/*")
foreach(_e IN LISTS _entries)
    if(IS_DIRECTORY "${_test_root}/${_e}")
        list(FIND _allowed_type_roots "${_e}" _type_idx)
        list(FIND _allowed_infra_dirs "${_e}" _infra_idx)
        if(_type_idx EQUAL -1 AND _infra_idx EQUAL -1)
            message(FATAL_ERROR
                "test_layout_guard: unknown directory tests/${_e}/. "
                "Allowed type roots: ${_allowed_type_roots}; "
                "infrastructure: ${_allowed_infra_dirs}. "
                "All listed type roots are valid test categories.")
        endif()
    endif()
endforeach()

set(_unit_domains
    common examples airborne_radar electronic_surveillance_radar electronic_countermeasure
    electro_optical_sensor sbirs_sensor sar navigation fusion threat_assessment target_inference flight_dynamic
    remote_identification_radar precision_evaluation satellite_orbit)
set(_integration_domains
    airborne_radar electro_optical_sensor electronic_surveillance_radar
    sbirs_sensor cross_domain remote_identification_radar)
set(_replay_domains
    common airborne_radar electro_optical_sensor electronic_surveillance_radar electronic_countermeasure
    sar sbirs_sensor remote_identification_radar)
set(_contract_domains
    public_api airborne_radar electro_optical_sensor electronic_surveillance_radar
    sar sbirs_sensor remote_identification_radar)
set(_performance_domains sar cross_domain)
set(_compatibility_domains public_api sar)

foreach(_type IN ITEMS unit integration replay contract performance compatibility)
    file(GLOB _flat_sources "${_test_root}/${_type}/*_test.cpp")
    if(_flat_sources)
        list(JOIN _flat_sources "\n  " _flat_list)
        message(FATAL_ERROR
            "test_layout_guard: _test.cpp must live under tests/${_type}/<domain>/. Found:\n  ${_flat_list}")
    endif()

    file(GLOB _domain_entries RELATIVE "${_test_root}/${_type}" "${_test_root}/${_type}/*")
    foreach(_domain IN LISTS _domain_entries)
        if(NOT IS_DIRECTORY "${_test_root}/${_type}/${_domain}")
            continue()
        endif()
        list(FIND _${_type}_domains "${_domain}" _domain_idx)
        if(_domain_idx EQUAL -1)
            message(FATAL_ERROR
                "test_layout_guard: unknown domain tests/${_type}/${_domain}/. "
                "Allowed domains for ${_type}: ${_${_type}_domains}")
        endif()
    endforeach()
endforeach()

# The source-level partitions intentionally replace suite/case filters. Do not
# reintroduce gtest filters in test registration CMake files.
file(GLOB_RECURSE _test_cmake_files
    "${_test_root}/CMakeLists.txt"
    "${_test_root}/cmake/*.cmake")
foreach(_cmake_file IN LISTS _test_cmake_files)
    file(READ "${_cmake_file}" _cmake_content)
    string(REGEX MATCH "(--gtest_filter|gtest_filter)" _gtest_filter_match "${_cmake_content}")
    if(_gtest_filter_match)
        message(FATAL_ERROR
            "test_layout_guard: ${_cmake_file} reintroduces a GoogleTest filter. "
            "Register source-level type × domain partitions instead.")
    endif()
endforeach()

message(STATUS "test_layout_guard: type × domain layout check passed.")
