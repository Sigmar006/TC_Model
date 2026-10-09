if(NOT DEFINED SOURCE_DIR OR SOURCE_DIR STREQUAL "")
    message(FATAL_ERROR "SOURCE_DIR is required")
endif()
# 规范化为绝对路径：file(GLOB ... RELATIVE <base>) 在 -P 脚本模式下，当 base 为
# 相对路径时返回空列表（误报 whitelist 全部 drift）。对绝对输入幂等，仅兜底手动调用。
get_filename_component(SOURCE_DIR "${SOURCE_DIR}" ABSOLUTE)

set(PUBLIC_INCLUDE_DIR "${SOURCE_DIR}/include/1q")

# 本守护校验 public header whitelist 与磁盘头逐字一致(HARD 阻断)。
# whitelist 按 stable_api / deprecated_compat_api 两层组织(见下方
# EXPECTED_DEPRECATED_HEADERS 及其守护),便于分批收口。两层的完整语义
# 与迁移策略参见 docs/common/contract.md 的 Public API 边界章节。

# ── AR 推荐公开主路径（条件五域 + 会话 + 统一入口） ──────────
set(AR_PUBLIC_PRIMARY_HEADERS
    "airborne_radar/airborne_radar.hpp"
    "airborne_radar/config/ArHardwareConfig.h"
    "airborne_radar/config/ArMissionConfig.h"
    "airborne_radar/config/ArPolicyConfig.h"
    "airborne_radar/config/ArEnvironmentConfig.h"
    "airborne_radar/config/ArSessionConfig.h"
    "airborne_radar/config/ArRuntimeConfigPatch.h"
    "airborne_radar/config/ArProfileConstants.h"
    "airborne_radar/config/ArSessionConfigValidation.h"
    "airborne_radar/config/ArOrientationConfig.h"
    "airborne_radar/config/airborne_radar_config.hpp"
)

# ── AR 会话域
set(AR_SESSION_HEADERS
    "airborne_radar/session/ArCycleInput.h"
    "airborne_radar/session/ArCycleOutputAdapter.h"
    "airborne_radar/session/ArCycleResult.h"
    "airborne_radar/session/ArExclusionCauseRecorder.h"
    "airborne_radar/session/ArPlatformInput.h"
    "airborne_radar/session/ArRadarFrameTransform.h"
    "airborne_radar/session/ArExternalOutputAdapter.h"
    "airborne_radar/session/ArInputValidation.h"
    "airborne_radar/session/ArIssueCodes.h"
    "airborne_radar/session/ArInterferenceObservation.h"
    "airborne_radar/session/ArOutputTypes.h"
    "airborne_radar/session/ArSceneTypes.h"
    "airborne_radar/session/ArReplaySession.h"
    "airborne_radar/session/ArTrackOutput.h"
    "airborne_radar/session/ArTrackLifecycleRecorder.h"
    "airborne_radar/session/ArTrackOutputDebugView.h"
    "airborne_radar/session/ArSession.h"
    "airborne_radar/session/ArRecordingSession.h"
    "airborne_radar/session/ArCommand.h"
    "airborne_radar/session/ArControlProfile.h"
    "airborne_radar/session/DecisionControlTypes.h"
    "airborne_radar/session/TrackStateSnapshot.h"
)

set(ROOT_HEADER
    "api.hpp"
)

set(EOS_MODULE_ENTRY_HEADERS
    "electro_optical_sensor/electro_optical_sensor.hpp"
)

set(EOS_CONFIG_HEADERS
    "electro_optical_sensor/config/EosEnvironmentConfig.h"
    "electro_optical_sensor/config/EosHardwareConfig.h"
    "electro_optical_sensor/config/EosMissionConfig.h"
    "electro_optical_sensor/config/EosPolicyConfig.h"
    "electro_optical_sensor/config/EosRuntimeConfigPatch.h"
    "electro_optical_sensor/config/EosProfileConstants.h"
    "electro_optical_sensor/config/EosSessionConfig.h"
    "electro_optical_sensor/config/EosSessionConfigValidation.h"
    "electro_optical_sensor/config/electro_optical_sensor_config.hpp"
)

set(EOS_ENVIRONMENT_HEADERS)
set(EOS_EXTENSION_HEADERS)
set(EOS_FOUNDATION_HEADERS)

set(EOS_SESSION_HEADERS
    "electro_optical_sensor/session/EosCycleInput.h"
    "electro_optical_sensor/session/EosCycleOutputAdapter.h"
    "electro_optical_sensor/session/EosCycleResult.h"
    "electro_optical_sensor/session/EosDetectionLifecycleRecorder.h"
    "electro_optical_sensor/session/EosExclusionCauseRecorder.h"
    "electro_optical_sensor/session/EosExternalOutputAdapter.h"
    "electro_optical_sensor/session/EosInputValidation.h"
    "electro_optical_sensor/session/EosIssueCodes.h"
    "electro_optical_sensor/session/EosOutputDebugView.h"
    "electro_optical_sensor/session/EosOutputTypes.h"
    "electro_optical_sensor/session/EosPlatformEcefPose.h"
    "electro_optical_sensor/session/EosSceneTypes.h"
    "electro_optical_sensor/session/EosSession.h"
    "electro_optical_sensor/session/EosRecordingSession.h"
    "electro_optical_sensor/session/EosReplaySession.h"
)

set(SBIRS_MODULE_ENTRY_HEADERS
    "sbirs_sensor/sbirs_sensor.hpp"
)

set(SBIRS_CONFIG_HEADERS
    "sbirs_sensor/config/SbirsEnvironmentConfig.h"
    "sbirs_sensor/config/SbirsHardwareConfig.h"
    "sbirs_sensor/config/SbirsMissionConfig.h"
    "sbirs_sensor/config/SbirsOrientationConfig.h"
    "sbirs_sensor/config/SbirsPolicyConfig.h"
    "sbirs_sensor/config/SbirsRuntimeConfigPatch.h"
    "sbirs_sensor/config/SbirsSessionConfig.h"
    "sbirs_sensor/config/SbirsSessionConfigValidation.h"
    "sbirs_sensor/config/sbirs_sensor_config.hpp"
)

set(SBIRS_SESSION_HEADERS
    "sbirs_sensor/session/SbirsCycleInput.h"
    "sbirs_sensor/session/SbirsCycleInputAdapter.h"
    "sbirs_sensor/session/SbirsCycleOutputAdapter.h"
    "sbirs_sensor/session/SbirsCycleResult.h"
    "sbirs_sensor/session/SbirsDetectionLifecycleRecorder.h"
    "sbirs_sensor/session/SbirsExclusionCauseRecorder.h"
    "sbirs_sensor/session/SbirsExternalCue.h"
    "sbirs_sensor/session/SbirsExternalInputAdapter.h"
    "sbirs_sensor/session/SbirsExternalOutputAdapter.h"
    "sbirs_sensor/session/SbirsInputValidation.h"
    "sbirs_sensor/session/SbirsIssueCodes.h"
    "sbirs_sensor/session/SbirsOutputDebugView.h"
    "sbirs_sensor/session/SbirsOutputTypes.h"
    "sbirs_sensor/session/SbirsReplaySession.h"
    "sbirs_sensor/session/SbirsSceneTypes.h"
    "sbirs_sensor/session/SbirsSession.h"
    "sbirs_sensor/session/SbirsRecordingSession.h"
)

set(ESR_MODULE_ENTRY_HEADERS
    "electronic_surveillance_radar/electronic_surveillance_radar.hpp"
)

set(ESR_CONFIG_HEADERS
    "electronic_surveillance_radar/config/EsrEnvironmentConfig.h"
    "electronic_surveillance_radar/config/EsrHardwareConfig.h"
    "electronic_surveillance_radar/config/EsrMissionConfig.h"
    "electronic_surveillance_radar/config/EsrOrientationConfig.h"
    "electronic_surveillance_radar/config/EsrPolicyConfig.h"
    "electronic_surveillance_radar/config/EsrRuntimeConfigPatch.h"
    "electronic_surveillance_radar/config/EsrProfileConstants.h"
    "electronic_surveillance_radar/config/EsrSessionConfig.h"
    "electronic_surveillance_radar/config/EsrSessionConfigValidation.h"
    "electronic_surveillance_radar/config/electronic_surveillance_radar_config.hpp"
)

set(ESR_ENVIRONMENT_HEADERS)
set(ESR_EXTENSION_HEADERS)

set(ESR_SESSION_HEADERS
    "electronic_surveillance_radar/session/EmitterHypothesis.h"
    "electronic_surveillance_radar/session/EmitterObservation.h"
    "electronic_surveillance_radar/session/EsrCycleInput.h"
    "electronic_surveillance_radar/session/EsrCycleResult.h"
    "electronic_surveillance_radar/session/EsrExclusionCauseRecorder.h"
    "electronic_surveillance_radar/session/EsrInputValidation.h"
    "electronic_surveillance_radar/session/EsrIssueCodes.h"
    "electronic_surveillance_radar/session/EsrOutputTypes.h"
    "electronic_surveillance_radar/session/EsrSession.h"
    "electronic_surveillance_radar/session/EsrRecordingSession.h"
    "electronic_surveillance_radar/session/EsrReplaySession.h"
)

set(FD_HEADERS
    "flight_dynamic/FlightManager.h"
    "flight_dynamic/autopilot/Autopilot.h"
    "flight_dynamic/config/FlightDynamicConfig.h"
    "flight_dynamic/guidance/Maneuver.h"
    "flight_dynamic/guidance/Waypoint.h"
    "flight_dynamic/guidance/WaypointManager.h"
    "flight_dynamic/guidance/WaypointSequencingEvent.h"
    "flight_dynamic/model/VehicleState.h"
)

set(NAVIGATION_HEADERS
    "navigation/AreaCoveragePlanner.h"
    "navigation/CoverageArea.h"
    "navigation/CoveragePlanConfig.h"
    "navigation/RoutePoint.h"
    "navigation/navigation.hpp"
)

set(FUSION_HEADERS
    "fusion/DetectionRecord.h"
    "fusion/FusedTarget.h"
    "fusion/FusionConfig.h"
    "fusion/FusionEngine.h"
    "fusion/SensorAdapters.h"
    "fusion/fusion.hpp"
)

set(THREAT_ASSESSMENT_HEADERS
    "threat_assessment/ThreatEvaluationInput.h"
    "threat_assessment/ThreatEvaluator.h"
    "threat_assessment/ThreatEvaluatorConfig.h"
    "threat_assessment/ThreatResult.h"
    "threat_assessment/threat_assessment.hpp"
)

set(TARGET_INFERENCE_HEADERS
    "target_inference/InferenceResult.h"
    "target_inference/InferenceTrackState.h"
    "target_inference/TargetInferenceConfig.h"
    "target_inference/TargetInferenceEngine.h"
    "target_inference/target_inference.hpp"
)

set(PRECISION_EVALUATION_HEADERS
    "precision_evaluation/AhpEvaluator.h"
    "precision_evaluation/DualLosFix.h"
    "precision_evaluation/PrecisionEvaluationConfig.h"
    "precision_evaluation/PrecisionEvaluationMetrics.h"
    "precision_evaluation/PrecisionEvaluationSession.h"
    "precision_evaluation/PrecisionEvaluationTypes.h"
    "precision_evaluation/SbirsBearingAdapter.h"
    "precision_evaluation/precision_evaluation.hpp"
)

set(ENVIRONMENT_HEADERS
    "environment/AtmosphericState.h"
    "environment/AtmosphericTypes.h"
    "environment/IAtmosphereProvider.h"
    "environment/PropagationPhysics.h"
)

set(ELECTROMAGNETICS_HEADERS
    "electromagnetics/RfLinkBudget.h"
    "electromagnetics/RfScene.h"
)

set(ECM_HEADERS
    "electronic_countermeasure/EcmEsrAdapter.h"
    "electronic_countermeasure/EcmReplaySession.h"
    "electronic_countermeasure/EcmSession.h"
    "electronic_countermeasure/EcmRecordingSession.h"
    "electronic_countermeasure/EcmTypes.h"
)

set(SAR_MODULE_ENTRY_HEADERS
    "sar/sar.hpp"
)

set(SAR_CONFIG_HEADERS
    "sar/config/SarEnvironmentConfig.h"
    "sar/config/SarHardwareConfig.h"
    "sar/config/SarMissionConfig.h"
    "sar/config/SarPolicyConfig.h"
    "sar/config/SarRuntimeConfigPatch.h"
    "sar/config/SarProfileConstants.h"
    "sar/config/SarSessionConfig.h"
    "sar/config/SarSessionConfigValidation.h"
    "sar/config/sar_config.hpp"
)

set(SAR_SESSION_HEADERS
    "sar/session/SarCycleInput.h"
    "sar/session/SarCycleResult.h"
    "sar/session/SarInputValidation.h"
    "sar/session/SarIssueCodes.h"
    "sar/session/SarProductDebugView.h"
    "sar/session/SarProductLifecycleRecorder.h"
    "sar/session/SarReplaySession.h"
    "sar/session/SarSession.h"
    "sar/session/SarRecordingSession.h"
)

# ── 远程识别雷达（RIR）推荐公开主路径（五域 + 会话 + 统一入口） ──
set(RIR_PUBLIC_PRIMARY_HEADERS
    "remote_identification_radar/remote_identification_radar.hpp"
    "remote_identification_radar/config/RirEnvironmentConfig.h"
    "remote_identification_radar/config/RirHardwareConfig.h"
    "remote_identification_radar/config/RirMissionConfig.h"
    "remote_identification_radar/config/RirOrientationConfig.h"
    "remote_identification_radar/config/RirPolicyConfig.h"
    "remote_identification_radar/config/RirProfileConstants.h"
    "remote_identification_radar/config/RirRuntimeConfigPatch.h"
    "remote_identification_radar/config/RirSessionConfig.h"
    "remote_identification_radar/config/RirSessionConfigValidation.h"
    "remote_identification_radar/config/remote_identification_radar_config.hpp"
)

# ── RIR 会话域
set(RIR_SESSION_HEADERS
    "remote_identification_radar/session/RirCycleInput.h"
    "remote_identification_radar/session/RirCycleResult.h"
    "remote_identification_radar/session/RirExclusionCauseRecorder.h"
    "remote_identification_radar/session/RirFeatureMeasurementTypes.h"
    "remote_identification_radar/session/RirInputValidation.h"
    "remote_identification_radar/session/RirIssueCodes.h"
    "remote_identification_radar/session/RirOutputDebugView.h"
    "remote_identification_radar/session/RirOutputTypes.h"
    "remote_identification_radar/session/RirRecognitionResult.h"
    "remote_identification_radar/session/RirRecordingSession.h"
    "remote_identification_radar/session/RirReplaySession.h"
    "remote_identification_radar/session/RirSceneTypes.h"
    "remote_identification_radar/session/RirSession.h"
    "remote_identification_radar/session/RirTrackLifecycleRecorder.h"
    "remote_identification_radar/session/RirTrackRequest.h"
)

set(COORDINATE_HEADERS
    "coordinate/attitude_transform.h"
    "coordinate/inertial_transform.h"
    "coordinate/position_transform.h"
    "coordinate/scene_transform.h"
    "coordinate/types.h"
    "coordinate/velocity_transform.h"
)

set(FOUNDATION_HEADERS
    "foundation/RuntimeCycleExecutor.h"
    "foundation/scan_schedule_types.h"
    "foundation/SensorContract.h"
    "foundation/validation_types.h"
    "replay/ReplayTrace.h"
)

# 共享域：外部测试日志打印器（135d52fd「feat(logging): add external test log printer」
# 引入时漏登白名单，守护自 2026-09-28 起一直报 Missing from boundary）。
set(LOGGING_HEADERS
    "logging/ExternalTestLogPrinter.h"
)

set(EXPECTED_PUBLIC_HEADERS
    ${AR_PUBLIC_PRIMARY_HEADERS}
    ${AR_SESSION_HEADERS}
    ${ROOT_HEADER}
    ${EOS_MODULE_ENTRY_HEADERS}
    ${EOS_CONFIG_HEADERS}
    ${EOS_ENVIRONMENT_HEADERS}
    ${EOS_EXTENSION_HEADERS}
    ${EOS_FOUNDATION_HEADERS}
    ${EOS_SESSION_HEADERS}
    ${SBIRS_MODULE_ENTRY_HEADERS}
    ${SBIRS_CONFIG_HEADERS}
    ${SBIRS_SESSION_HEADERS}
    ${ESR_MODULE_ENTRY_HEADERS}
    ${ESR_CONFIG_HEADERS}
    ${ESR_ENVIRONMENT_HEADERS}
    ${ESR_EXTENSION_HEADERS}
    ${ESR_SESSION_HEADERS}
    ${FD_HEADERS}
    ${NAVIGATION_HEADERS}
    ${FUSION_HEADERS}
    ${THREAT_ASSESSMENT_HEADERS}
    ${TARGET_INFERENCE_HEADERS}
    ${PRECISION_EVALUATION_HEADERS}
    ${SAR_MODULE_ENTRY_HEADERS}
    ${SAR_CONFIG_HEADERS}
    ${SAR_SESSION_HEADERS}
    ${RIR_PUBLIC_PRIMARY_HEADERS}
    ${RIR_SESSION_HEADERS}
    ${COORDINATE_HEADERS}
    ${ELECTROMAGNETICS_HEADERS}
    ${ECM_HEADERS}
    ${ENVIRONMENT_HEADERS}
    ${FOUNDATION_HEADERS}
    ${LOGGING_HEADERS}
)

set(EXPECTED_DEPRECATED_HEADERS "")

file(READ "${CMAKE_CURRENT_LIST_FILE}" _boundary_script_source)
foreach(_deprecated_header IN LISTS EXPECTED_DEPRECATED_HEADERS)
    string(FIND "${_boundary_script_source}" "DEPRECATED_ENTRY:${_deprecated_header}" _idx)
    if(_idx EQUAL -1)
        message(FATAL_ERROR
            "deprecated_compat_api 条目缺少移除计划注释:\n"
            "  ${_deprecated_header}\n"
            "每个 deprecated 条目必须在本文件内以如下形式标注:\n"
            "  # DEPRECATED_ENTRY:<header> <批次/替代方案>\n"
            "无注释的 deprecated 条目不被允许,以避免静默滞留。")
    endif()
endforeach()

list(LENGTH EXPECTED_PUBLIC_HEADERS _stable_count)
list(LENGTH EXPECTED_DEPRECATED_HEADERS _deprecated_count)
math(EXPR _stable_api_count "${_stable_count} - ${_deprecated_count}")
message(STATUS
    "[public-api-boundary] stable_api=${_stable_api_count} "
    "deprecated_compat_api=${_deprecated_count} (total=${_stable_count})")

file(GLOB_RECURSE ACTUAL_PUBLIC_HEADERS
     RELATIVE "${PUBLIC_INCLUDE_DIR}"
     "${PUBLIC_INCLUDE_DIR}/*.h"
     "${PUBLIC_INCLUDE_DIR}/*.hpp")

execute_process(
    COMMAND find "${PUBLIC_INCLUDE_DIR}" -type f
            ! -name "*.h" ! -name "*.hpp" ! -name "README.md"
    OUTPUT_VARIABLE PUBLIC_INCLUDE_NON_HEADER_FILES
    OUTPUT_STRIP_TRAILING_WHITESPACE)

if(NOT "${PUBLIC_INCLUDE_NON_HEADER_FILES}" STREQUAL "")
  message(FATAL_ERROR
          "Non-header artifacts are not allowed under include/1q:\n"
          "${PUBLIC_INCLUDE_NON_HEADER_FILES}")
endif()

list(SORT EXPECTED_PUBLIC_HEADERS)
list(SORT ACTUAL_PUBLIC_HEADERS)

if(NOT ACTUAL_PUBLIC_HEADERS STREQUAL EXPECTED_PUBLIC_HEADERS)
  message(FATAL_ERROR
          "Public header whitelist drifted.\n"
          "Expected: ${EXPECTED_PUBLIC_HEADERS}\n"
          "Actual: ${ACTUAL_PUBLIC_HEADERS}")
endif()

set(MODULE_ENTRY_HEADERS_WITH_EXPLICIT_TOOLING
    "airborne_radar/airborne_radar.hpp"
    "electro_optical_sensor/electro_optical_sensor.hpp"
    "electronic_surveillance_radar/electronic_surveillance_radar.hpp"
    "sar/sar.hpp"
    "remote_identification_radar/remote_identification_radar.hpp")

foreach(HEADER IN LISTS MODULE_ENTRY_HEADERS_WITH_EXPLICIT_TOOLING)
  file(READ "${PUBLIC_INCLUDE_DIR}/${HEADER}" MODULE_ENTRY_HEADER_CONTENT)
  if(MODULE_ENTRY_HEADER_CONTENT MATCHES "#[ \t]*include[ \t]*[\"<][^\n]*(RecordingSession|ReplaySession|DebugView|LifecycleRecorder)\\.h")
    message(FATAL_ERROR
            "Module entry header must not aggregate observability tooling: ${HEADER}")
  endif()
endforeach()

execute_process(
    COMMAND find "${PUBLIC_INCLUDE_DIR}" -type d -empty
    OUTPUT_VARIABLE EMPTY_DIRS
    OUTPUT_STRIP_TRAILING_WHITESPACE)

if(NOT "${EMPTY_DIRS}" STREQUAL "")
  message(FATAL_ERROR
          "Empty directories are not allowed under include/1q:\n${EMPTY_DIRS}")
endif()

foreach(HEADER IN LISTS ACTUAL_PUBLIC_HEADERS)
  file(READ "${PUBLIC_INCLUDE_DIR}/${HEADER}" HEADER_CONTENT)

  if(HEADER_CONTENT MATCHES "#[ \t]*include[ \t]*[<\"]boost/"
     OR HEADER_CONTENT MATCHES "boost::any")
    message(FATAL_ERROR
            "Public header must not expose Boost: ${HEADER}")
  endif()

  if(HEADER_CONTENT MATCHES "#[ \t]*include[ \t]*[<\"]Eigen/")
    message(FATAL_ERROR
            "Public header must not expose Eigen: ${HEADER}")
  endif()
endforeach()

set(AR_CONFIG_HEADERS_NO_EXPERT_NAMESPACE
    "airborne_radar/config/ArHardwareConfig.h"
    "airborne_radar/config/ArPolicyConfig.h")

foreach(HEADER IN LISTS AR_CONFIG_HEADERS_NO_EXPERT_NAMESPACE)
  file(READ "${PUBLIC_INCLUDE_DIR}/${HEADER}" AR_CONFIG_HEADER_CONTENT)
  if(AR_CONFIG_HEADER_CONTENT MATCHES "namespace[ \t]+expert[ \t]*\\{")
    message(FATAL_ERROR
            "Public AR config header must not expose namespace expert: ${HEADER}")
  endif()
endforeach()

# Config Builder 已删除：公开头不得再出现 *SessionConfigBuilder.h / *RuntimeConfigBuilder.h。
# 会话配置直接赋值 *SessionConfig；运行期热更新直接写 *RuntimeConfigPatch（含 has_*）。
file(GLOB_RECURSE FORBIDDEN_CONFIG_BUILDER_HEADERS
     RELATIVE "${PUBLIC_INCLUDE_DIR}"
     "${PUBLIC_INCLUDE_DIR}/*SessionConfigBuilder.h"
     "${PUBLIC_INCLUDE_DIR}/*RuntimeConfigBuilder.h")
if(FORBIDDEN_CONFIG_BUILDER_HEADERS)
  message(FATAL_ERROR
          "Forbidden config Builder public headers reintroduced:\n"
          "  ${FORBIDDEN_CONFIG_BUILDER_HEADERS}\n"
          "Use direct *SessionConfig / *RuntimeConfigPatch assignment (explicit has_*).")
endif()

set(AR_FORBIDDEN_INTERNAL_PATHS
    "${CMAKE_SOURCE_DIR}/src/airborne_radar/config/legacy"
    "${CMAKE_SOURCE_DIR}/src/airborne_radar/config/internal/SessionConfigPipelineMapper.h"
    "${CMAKE_SOURCE_DIR}/src/airborne_radar/config/internal/ExpertToEngineeringMapping.h")

foreach(FORBIDDEN_PATH IN LISTS AR_FORBIDDEN_INTERNAL_PATHS)
  if(EXISTS "${FORBIDDEN_PATH}")
    message(FATAL_ERROR
            "Forbidden internal legacy path reintroduced: ${FORBIDDEN_PATH}\n"
            "M7-F requires legacy config shells, SessionConfigPipelineMapper, and ExpertToEngineeringMapping to stay removed.")
  endif()
endforeach()
