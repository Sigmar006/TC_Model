# common 模块源文件与安装清单

set(ONEQ_COMMON_SOURCES
    atmosphere/AtmospherePhysics.cpp
    atmosphere/PropagationPhysicsImpl.cpp
    atmosphere/StandardAtmosphere.cpp
    electromagnetics/RfLinkBudget.cpp
    electromagnetics/RfScene.cpp
    geometry/BoresightChain.cpp
    geometry/EarthOccultation.cpp
    geometry/GeometryTransform.cpp
    coordinate/PositionTransform.cpp
    coordinate/VelocityTransform.cpp
    coordinate/AttitudeTransform.cpp
    coordinate/InertialTransform.cpp
    coordinate/SceneTransform.cpp
    numerics/SpectralNumerics.cpp
    optimization/LapjvSolver.cpp
    radar/RadarEquations.cpp
    radar/DetectionCellResolver.cpp
    radar/MtiMtdAcceptanceBank.cpp
    radar/StatisticalCfarDetector.cpp
    radar/VegetationClutterModel.cpp
    rcs/RcsPhysics.cpp
    replay/ReplayTrace.cpp
    timing/TimingRegimeModel.cpp
    logging/AcceptanceFileLog.cpp
    logging/ExternalTestLogPrinter.cpp
    estimation/EstimationInstantiations.cpp
)

# 内置文件日志后端（PROJECT_LOG_* 的 Windows 落盘实现）：总开关 OFF 时不编译。
if(ONEQ_ENABLE_FILE_LOG)
    list(APPEND ONEQ_COMMON_SOURCES logging/ProjectFileLog.cpp)
endif()

oneq_add_component(oneq_common SOURCES ${ONEQ_COMMON_SOURCES})
target_link_libraries(oneq_common PRIVATE
    ${ONEQ_LINK_DEPENDENCIES})
