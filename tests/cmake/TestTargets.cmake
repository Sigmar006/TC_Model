# Test executable source ownership and aggregate build targets.
#
# Phase 2: the legacy 1q_unit_tests executable is replaced by per-domain unit
# partitions defined in partitions/Unit.cmake. 1q_unit_tests becomes a custom
# aggregate target. The FD executable (1q_fd_tests) is replaced by the
# 1q_flight_dynamic_unit_tests partition. Phase 3 converts replay-fast and
# integration, compiled contract and performance to per-domain partitions.

# --- unit partitions -------------------------------------------------------
include(${CMAKE_CURRENT_LIST_DIR}/partitions/Unit.cmake)

# 1q_unit_tests: aggregate target depending on all enabled unit partitions.
add_custom_target(${PROJECT_NAME}_unit_tests)
set(_oneq_unit_partition_targets
    ${PROJECT_NAME}_common_unit_tests
    ${PROJECT_NAME}_examples_unit_tests
    ${PROJECT_NAME}_airborne_radar_unit_tests
    ${PROJECT_NAME}_remote_identification_radar_unit_tests
    ${PROJECT_NAME}_electronic_surveillance_radar_unit_tests
    ${PROJECT_NAME}_electronic_countermeasure_unit_tests
    ${PROJECT_NAME}_electro_optical_sensor_unit_tests
    ${PROJECT_NAME}_sbirs_sensor_unit_tests
    ${PROJECT_NAME}_sar_unit_tests
    ${PROJECT_NAME}_navigation_unit_tests
    ${PROJECT_NAME}_fusion_unit_tests
    ${PROJECT_NAME}_threat_assessment_unit_tests
    ${PROJECT_NAME}_satellite_orbit_unit_tests
    ${PROJECT_NAME}_flight_dynamic_unit_tests)
foreach(_p IN LISTS _oneq_unit_partition_targets)
    if(TARGET ${_p})
        add_dependencies(${PROJECT_NAME}_unit_tests ${_p})
    endif()
endforeach()
unset(_oneq_unit_partition_targets)

# 1q_fd_tests: alias kept for legacy callers; points at the FD unit partition.
if(TARGET ${PROJECT_NAME}_flight_dynamic_unit_tests AND NOT TARGET ${PROJECT_NAME}_fd_tests)
    add_custom_target(${PROJECT_NAME}_fd_tests)
    add_dependencies(${PROJECT_NAME}_fd_tests ${PROJECT_NAME}_flight_dynamic_unit_tests)
    if(TARGET ${PROJECT_NAME}_flight_dynamic_known_limit_tests)
        add_dependencies(${PROJECT_NAME}_fd_tests ${PROJECT_NAME}_flight_dynamic_known_limit_tests)
    endif()
endif()

# --- performance partitions ------------------------------------------------
include(${CMAKE_CURRENT_LIST_DIR}/partitions/Performance.cmake)

add_custom_target(${PROJECT_NAME}_performance_tests)
if(TARGET ${PROJECT_NAME}_sar_performance_tests)
    add_dependencies(${PROJECT_NAME}_performance_tests ${PROJECT_NAME}_sar_performance_tests)
endif()
if(TARGET ${PROJECT_NAME}_cross_domain_performance_tests)
    add_dependencies(${PROJECT_NAME}_performance_tests ${PROJECT_NAME}_cross_domain_performance_tests)
endif()

# --- integration partitions ------------------------------------------------
include(${CMAKE_CURRENT_LIST_DIR}/partitions/Integration.cmake)

add_custom_target(${PROJECT_NAME}_integration_tests)
set(_oneq_integration_partition_targets
    ${PROJECT_NAME}_airborne_radar_integration_tests
    ${PROJECT_NAME}_remote_identification_radar_integration_tests
    ${PROJECT_NAME}_electro_optical_sensor_integration_tests
    ${PROJECT_NAME}_electronic_surveillance_radar_integration_tests
    ${PROJECT_NAME}_sbirs_sensor_integration_tests
    ${PROJECT_NAME}_cross_domain_integration_tests)
foreach(_p IN LISTS _oneq_integration_partition_targets)
    if(TARGET ${_p})
        add_dependencies(${PROJECT_NAME}_integration_tests ${_p})
    endif()
endforeach()
unset(_oneq_integration_partition_targets)

# --- compiled contract partitions -----------------------------------------
include(${CMAKE_CURRENT_LIST_DIR}/partitions/Contract.cmake)

add_custom_target(${PROJECT_NAME}_contract_tests)
set(_oneq_contract_partition_targets
    ${PROJECT_NAME}_public_api_contract_tests
    ${PROJECT_NAME}_airborne_radar_contract_tests
    ${PROJECT_NAME}_remote_identification_radar_contract_tests
    ${PROJECT_NAME}_electro_optical_sensor_contract_tests
    ${PROJECT_NAME}_electronic_surveillance_radar_contract_tests
    ${PROJECT_NAME}_sar_contract_tests
    ${PROJECT_NAME}_sbirs_sensor_contract_tests)
foreach(_p IN LISTS _oneq_contract_partition_targets)
    if(TARGET ${_p})
        add_dependencies(${PROJECT_NAME}_contract_tests ${_p})
    endif()
endforeach()
unset(_oneq_contract_partition_targets)

# --- replay partitions -----------------------------------------------------
include(${CMAKE_CURRENT_LIST_DIR}/partitions/Replay.cmake)

# Keep the legacy build target name as an aggregate; CTest selection remains
# compatible through the replay_fast label set on each partition.
add_custom_target(${PROJECT_NAME}_replay_fast_tests)
set(_oneq_replay_partition_targets
    ${PROJECT_NAME}_common_replay_tests
    ${PROJECT_NAME}_airborne_radar_replay_tests
    ${PROJECT_NAME}_remote_identification_radar_replay_tests
    ${PROJECT_NAME}_electro_optical_sensor_replay_tests
    ${PROJECT_NAME}_electronic_surveillance_radar_replay_tests
    ${PROJECT_NAME}_electronic_countermeasure_replay_tests
    ${PROJECT_NAME}_sar_replay_tests
    ${PROJECT_NAME}_sbirs_sensor_replay_tests)
foreach(_p IN LISTS _oneq_replay_partition_targets)
    if(TARGET ${_p})
        add_dependencies(${PROJECT_NAME}_replay_fast_tests ${_p})
    endif()
endforeach()
unset(_oneq_replay_partition_targets)

# --- aggregate build targets ----------------------------------------------
add_custom_target(${PROJECT_NAME}_tests)
set(_oneq_test_deps
    ${PROJECT_NAME}_unit_tests
    ${PROJECT_NAME}_performance_tests
    ${PROJECT_NAME}_integration_tests
    ${PROJECT_NAME}_contract_tests)
add_dependencies(${PROJECT_NAME}_tests ${_oneq_test_deps})
unset(_oneq_test_deps)

add_custom_target(${PROJECT_NAME}_tests_fast)
add_dependencies(${PROJECT_NAME}_tests_fast ${PROJECT_NAME}_replay_fast_tests)
