# remote_identification_radar 模块源文件与安装清单

# -- engine: 识别算法（特征提取/积累/匹配/数据库加载；依赖 SQLite） --------------
# dwell/ 检测链（阶段 2-M）、tracking/ 轻量跟踪子集（阶段 2-T，旁路新增）。
set(RIR_ENGINE_SOURCES
    internal/RirRadarEquations.cpp
    internal/RirPropagationModel.cpp
    internal/RirSurfaceClutterModel.cpp
    dwell/RirDetectionCellResolver.cpp
    dwell/RirEffectiveRcs.cpp
    dwell/RirEmissionFactory.cpp
    dwell/RirReceiverStateBuilder.cpp
    dwell/RirRfFrontEndResolver.cpp
    dwell/RirSignalDetector.cpp
    tracking/RirTrackFilter.cpp
    tracking/RirLapjvSolver.cpp
    tracking/RirTrackPool.cpp
    tracking/RirImmFilter.cpp
    tracking/RirTrackAssociator.cpp
    tracking/RirTrackLifecycle.cpp
    recognition/RcsFeatureExtractor.cpp
    recognition/MotionFeatureExtractor.cpp
    recognition/PolarizationFeatureExtractor.cpp
    recognition/RangeProfileFeatureExtractor.cpp
    recognition/RecognitionObservationBuilder.cpp
    recognition/RecognitionFeatureDatabase.cpp
    recognition/RecognitionMatcher.cpp
    recognition/RecognitionTracker.cpp
)

# -- core: 会话、配置、运行时（粘合层，依赖轻） ---------------------------------
set(RIR_CORE_SOURCES
    session/RirSessionConfigValidation.cpp
    session/RirInputValidation.cpp
    session/RirSession.cpp
    session/RirRecordingSession.cpp
    session/RirReplaySession.cpp
    session/RirReplayFlatbufferCodec.cpp
    session/RirOutputDebugViewBuilder.cpp
    session/RirTrackLifecycleRecorder.cpp
    session/RirExclusionCauseRecorder.cpp
    runtime/RirController.cpp
    runtime/RirAcceptanceRecords.cpp
    recognition/PolarizationStatsExtractor.cpp
)

oneq_add_component(rir_engine SOURCES ${RIR_ENGINE_SOURCES})
oneq_add_component(rir_core SOURCES ${RIR_CORE_SOURCES})
# RIR 验收信息日志（[RirAccept] 事件流）：检测链/跟踪/识别接线点在 rir_engine，
# 会话/波位/驻留统计接线点在 rir_core，两侧共用 RirAcceptanceLog.h。
# 同一开关启用 RIR 验收日志与 RadarInternalInterface.log 内部接口证据。
if(ONEQ_ENABLE_RIR_ACCEPTANCE_LOG)
    target_compile_definitions(rir_engine PRIVATE ONEQ_ENABLE_RIR_ACCEPTANCE_LOG=1)
    target_compile_definitions(rir_core PRIVATE ONEQ_ENABLE_RIR_ACCEPTANCE_LOG=1)
endif()
# 识别特征库加载（SQLite 只读读取器）；仅 remote_identification_radar 内部消费，
# PRIVATE 不进公共链接面。
target_link_libraries(rir_engine PRIVATE SQLite::SQLite3)
# 检测物理子集依赖 boost::math（header-only，含于公共链接依赖；阶段 2-M M1）。
target_link_libraries(rir_engine PRIVATE ${ONEQ_LINK_DEPENDENCIES})
target_link_libraries(rir_core PRIVATE ${ONEQ_LINK_DEPENDENCIES})
oneq_add_flatbuffers_schemas(
    TARGET rir_core
    MODULE remote_identification_radar
    SCHEMAS rir_replay.fbs rir_session_replay.fbs)
