if(NOT DEFINED SOURCE_DIR OR SOURCE_DIR STREQUAL "")
    message(FATAL_ERROR "SOURCE_DIR is required")
endif()
# 规范化为绝对路径：file(GLOB ... RELATIVE <base>) 在 -P 脚本模式下，当 base 为
# 相对路径时返回空列表（误报注册表全量漂移）。对绝对输入幂等，仅兜底手动调用。
get_filename_component(SOURCE_DIR "${SOURCE_DIR}" ABSOLUTE)

set(PUBLIC_INCLUDE_DIR "${SOURCE_DIR}/include/1q")

# 本守护校验「仿真真值标识符挂载」契约（session_contract.md Attribution 挂载表，
# HARD 阻断）：真值标识符（external_target_id / target_name / *Attribution*）在
# 公共头文件中只允许出现在下方三类注册表内——
#   1) TRUTH_INPUT_HEADERS          场景输入 / 调用方关联键 / 诊断码字符串（真值
#      注入侧与指令侧，契约规则允许）；
#   2) ENVELOPE_ATTRIBUTION_HEADERS 信封归属与观测层（*CycleResult 挂载的
#      *AttributionRecord，及 DebugView / LifecycleRecorder / ExclusionCauseRecorder
#      等结构化观测类型）；
#   3) AR_LEGACY_PRODUCT_TRUTH_HEADERS  AR 产品通道 deprecated 遗留（冻结注册，
#      不得新增条目）。
# 未注册的新文件出现真值标识符 → FATAL：新产品类型的真值归属只允许信封挂载。

set(TRUTH_INPUT_HEADERS
    "airborne_radar/config/ArRuntimeConfigPatch.h"   # designated_external_target_id（调用方指定键）
    "airborne_radar/session/ArIssueCodes.h"          # 诊断码字符串（非数据字段）
    "airborne_radar/session/ArSceneTypes.h"
    "electro_optical_sensor/session/EosSceneTypes.h"
    "remote_identification_radar/config/RirRuntimeConfigPatch.h"
    "remote_identification_radar/session/RirIssueCodes.h"
    "remote_identification_radar/session/RirSceneTypes.h"
    "sar/session/SarCycleInput.h"
    "sbirs_sensor/session/SbirsSceneTypes.h"
)

set(ENVELOPE_ATTRIBUTION_HEADERS
    "airborne_radar/session/ArCycleResult.h"
    "airborne_radar/session/ArExclusionCauseRecorder.h"
    "airborne_radar/session/ArOutputTypes.h"
    "airborne_radar/session/ArTrackLifecycleRecorder.h"
    "airborne_radar/session/ArTrackOutputDebugView.h"
    "electro_optical_sensor/session/EosCycleResult.h"
    "electro_optical_sensor/session/EosDetectionLifecycleRecorder.h"
    "electro_optical_sensor/session/EosExclusionCauseRecorder.h"
    "electro_optical_sensor/session/EosOutputDebugView.h"
    "electro_optical_sensor/session/EosOutputTypes.h"
    "precision_evaluation/PrecisionEvaluationInputs.h"
    "remote_identification_radar/session/RirCycleResult.h"
    "remote_identification_radar/session/RirExclusionCauseRecorder.h"
    "remote_identification_radar/session/RirOutputDebugView.h"
    "remote_identification_radar/session/RirOutputTypes.h"
    "remote_identification_radar/session/RirTrackLifecycleRecorder.h"
    "sar/session/SarProductDebugView.h"
    "sbirs_sensor/session/SbirsCycleResult.h"
    "sbirs_sensor/session/SbirsDetectionLifecycleRecorder.h"
    "sbirs_sensor/session/SbirsExclusionCauseRecorder.h"
    "sbirs_sensor/session/SbirsOutputDebugView.h"
    "sbirs_sensor/session/SbirsOutputTypes.h"
)

# 冻结注册：session_contract.md Attribution 挂载表的 AR 历史特例
# （TrackStateSnapshot 内嵌真值，deprecated / sim-only，回收由后续独立工作处理）。
# 不得新增条目：产品通道（product frame）再出现真值标识符一律拒绝，
# 归属走 *CycleResult 信封对照表。
set(AR_LEGACY_PRODUCT_TRUTH_HEADERS
    "airborne_radar/session/ArExternalOutputAdapter.h"
    "airborne_radar/session/ArTrackOutput.h"
    "airborne_radar/session/TrackStateSnapshot.h"
)

set(REGISTERED_TRUTH_HEADERS
    ${TRUTH_INPUT_HEADERS}
    ${ENVELOPE_ATTRIBUTION_HEADERS}
    ${AR_LEGACY_PRODUCT_TRUTH_HEADERS}
)

file(GLOB_RECURSE ALL_PUBLIC_HEADERS
     RELATIVE "${PUBLIC_INCLUDE_DIR}"
     "${PUBLIC_INCLUDE_DIR}/*.h"
     "${PUBLIC_INCLUDE_DIR}/*.hpp")

set(TRUTH_IDENTIFIER_PATTERN "external_target_id|target_name|Attribution")

# 1) 磁盘扫描：任何含真值标识符的公共头必须在注册表内（捕获未注册的新挂载）。
foreach(HEADER IN LISTS ALL_PUBLIC_HEADERS)
    file(READ "${PUBLIC_INCLUDE_DIR}/${HEADER}" HEADER_CONTENT)
    if(HEADER_CONTENT MATCHES "${TRUTH_IDENTIFIER_PATTERN}")
        list(FIND REGISTERED_TRUTH_HEADERS "${HEADER}" _registered_idx)
        if(_registered_idx EQUAL -1)
            message(FATAL_ERROR
                "公共头包含仿真真值标识符但未注册:\n"
                "  ${HEADER}\n"
                "真值归属只允许信封通道挂载(session_contract.md Attribution 挂载表)。\n"
                "如属场景输入/调用方关联键/诊断码/信封观测层,请在本守护注册表登记;\n"
                "产品通道(product frame)新增真值字段一律拒绝,归属走 *CycleResult 信封对照表。")
        endif()
    endif()
endforeach()

# 2) 注册表健康：条目必须存在且仍含真值标识符（防僵尸条目静默漂移）。
foreach(HEADER IN LISTS REGISTERED_TRUTH_HEADERS)
    if(NOT EXISTS "${PUBLIC_INCLUDE_DIR}/${HEADER}")
        message(FATAL_ERROR "attribution 注册条目对应的文件不存在:\n  ${HEADER}")
    endif()
    file(READ "${PUBLIC_INCLUDE_DIR}/${HEADER}" HEADER_CONTENT)
    if(NOT HEADER_CONTENT MATCHES "${TRUTH_IDENTIFIER_PATTERN}")
        message(FATAL_ERROR
            "attribution 注册条目不再包含真值标识符(僵尸条目,应移除):\n  ${HEADER}")
    endif()
endforeach()

list(LENGTH TRUTH_INPUT_HEADERS _input_count)
list(LENGTH ENVELOPE_ATTRIBUTION_HEADERS _envelope_count)
list(LENGTH AR_LEGACY_PRODUCT_TRUTH_HEADERS _legacy_count)
message(STATUS
    "[attribution-mounting] input=${_input_count} envelope=${_envelope_count} "
    "ar_legacy=${_legacy_count}(冻结) 产品通道真值字段新增一律拒绝")
