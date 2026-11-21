#include "flight_dynamic/adapter/JsbsimAdapter.h"

#include <cstdlib>
#include <string>

#if defined(_WIN32)
#include <io.h>
#else
#include <unistd.h>
#endif

#include "1q/coordinate/position_transform.h"
#include "common/logging/LogDirectory.h"
#include "common/logging/ProjectLog.h"
#include "FGFDMExec.h"
#include "flight_dynamic/adapter/PropertyNames.h"
#include "flight_dynamic/model/VehicleStateMapper.h"
#include "initialization/FGInitialCondition.h"
#include "models/FGFCS.h"
#include "models/FGOutput.h"
#include "models/FGPropulsion.h"
#include "simgear/misc/sg_path.hxx"

namespace oneq {
namespace flight_dynamic {
namespace adapter {

namespace {

bool MarkInitializationFailure(JsbsimAdapter::InitDiagnostics* diagnostics,
                               const std::string& reason) {
  if (diagnostics != nullptr) {
    diagnostics->initialization_failed = true;
    diagnostics->failure_reason = reason;
  }
  // 中译：JsbsimAdapter 初始化失败：{}。
  // 标识：初始化失败——JSBSim 飞行模型不可用时飞行仿真不可用，
  //       排查飞机数据根目录与机型 XML。
  PROJECT_LOG_ERROR("JsbsimAdapter initialization failed: {}", reason);
  return false;
}

void SetPropertyIfPresent(JSBSim::FGFDMExec& fdm_exec, const std::string& name, double value) {
  if (fdm_exec.GetPropertyManager()->GetNode(name) != nullptr) {
    fdm_exec.SetPropertyValue(name, value);
  }
}

void RetractLandingGearIfModeled(JSBSim::FGFDMExec& fdm_exec) {
  if (fdm_exec.GetPropertyManager()->GetNode(property::kGearCmdNorm) == nullptr) {
    return;
  }
  fdm_exec.SetPropertyValue(property::kGearCmdNorm, 0.0);
  SetPropertyIfPresent(fdm_exec, property::kGearPosNorm, 0.0);
  SetPropertyIfPresent(fdm_exec, "/controls/gear/gear-down-cond", 0.0);
}

void ExtendLandingGearIfModeled(JSBSim::FGFDMExec& fdm_exec) {
  if (fdm_exec.GetPropertyManager()->GetNode(property::kGearCmdNorm) == nullptr) {
    return;
  }
  fdm_exec.SetPropertyValue(property::kGearCmdNorm, 1.0);
  SetPropertyIfPresent(fdm_exec, property::kGearPosNorm, 1.0);
  SetPropertyIfPresent(fdm_exec, "/controls/gear/gear-down-cond", 1.0);
}

void ResetThrottleState(JSBSim::FGFDMExec& fdm_exec, double value) {
  auto fcs = fdm_exec.GetFCS();
  auto propulsion = fdm_exec.GetPropulsion();
  if (!fcs || !propulsion) {
    return;
  }

  fcs->SetThrottleCmd(-1, value);
  fcs->SetThrottlePos(-1, value);
  for (unsigned int i = 0; i < propulsion->GetNumEngines(); ++i) {
    if (i < propulsion->in.ThrottleCmd.size()) {
      propulsion->in.ThrottleCmd[i] = value;
    }
    if (i < propulsion->in.ThrottlePos.size()) {
      propulsion->in.ThrottlePos[i] = value;
    }
  }
  auto* throttle_cmd_node = fdm_exec.GetPropertyManager()->GetNode(property::kThrottleCmd, true);
  if (throttle_cmd_node != nullptr) {
    throttle_cmd_node->setDoubleValue(value);
  }
  auto* throttle_pos_node = fdm_exec.GetPropertyManager()->GetNode(property::kThrottlePosNorm, true);
  if (throttle_pos_node != nullptr) {
    throttle_pos_node->setDoubleValue(value);
  }
}

void SetPropellerAdvanceState(JSBSim::FGFDMExec& fdm_exec, double value) {
  auto fcs = fdm_exec.GetFCS();
  auto propulsion = fdm_exec.GetPropulsion();
  if (!fcs || !propulsion) {
    return;
  }

  for (unsigned int i = 0; i < propulsion->GetNumEngines(); ++i) {
    fcs->SetPropAdvanceCmd(static_cast<int>(i), value);
    fcs->SetPropAdvance(static_cast<int>(i), value);
  }
}

void SetBrakeState(JSBSim::FGFDMExec& fdm_exec, double value) {
  SetPropertyIfPresent(fdm_exec, property::kLeftBrakeCmd, value);
  SetPropertyIfPresent(fdm_exec, property::kRightBrakeCmd, value);
  SetPropertyIfPresent(fdm_exec, property::kCenterBrakeCmd, value);
}

void SettleInitialGroundState(JSBSim::FGFDMExec& fdm_exec, double dt_sec) {
  const auto* agl_node = fdm_exec.GetPropertyManager()->GetNode("position/h-agl-ft");
  if (agl_node && agl_node->getDoubleValue() > 1.0) {
    return;  // air start: no ground settling needed
  }

  const double saved_dt = fdm_exec.GetDeltaT();
  fdm_exec.Setdt(dt_sec);

  // Phase 1: HoldDown — compute ground contact forces without moving the
  // aircraft, so the gear spring/damper state is correctly initialized.
  fdm_exec.SetHoldDown(true);
  ResetThrottleState(fdm_exec, 0.0);
  SetBrakeState(fdm_exec, 1.0);
  for (int i = 0; i < 5; ++i) {
    fdm_exec.Run();
    ResetThrottleState(fdm_exec, 0.0);
    SetBrakeState(fdm_exec, 1.0);
  }

  fdm_exec.SetHoldDown(false);
  fdm_exec.Setdt(saved_dt);
}

void DisableXmlOutput(JSBSim::FGFDMExec& fdm_exec) { fdm_exec.DisableOutput(); }

// 取可写的临时目录（不依赖 std::filesystem：GCC 7.3 的 libstdc++ 没有该头，
// 而麒麟 V10 自带的就是 7.3）。
std::string TempJsbsimOutputDirectory() {
#if defined(_WIN32)
  const char* const tmp = std::getenv("TEMP");
  std::string base = (tmp != nullptr && *tmp != '\0') ? std::string(tmp)
                                                      : std::string("C:/Windows/Temp");
#else
  const char* const tmp = std::getenv("TMPDIR");
  std::string base =
      (tmp != nullptr && *tmp != '\0') ? std::string(tmp) : std::string("/tmp");
#endif
  while (!base.empty() && (base[base.size() - 1] == '/' || base[base.size() - 1] == '\\')) {
    base.erase(base.size() - 1);
  }
  if (base.empty()) {
    base = ".";
  }
  return base + "/1q_jsbsim_output";
}

bool PathExists(const std::string& path) {
  if (path.empty()) {
    return false;
  }
#if defined(_WIN32)
  return _access(path.c_str(), 0) == 0;
#else
  return ::access(path.c_str(), F_OK) == 0;
#endif
}

void PrepareXmlOutputPath(JSBSim::FGFDMExec& fdm_exec) {
  // 输出目录只是写 XML 的副作用保障：建目录失败不影响飞机数据加载，故整段不抛
  // 异常（本工程禁止 C++ 异常），也不用 std::filesystem。
  // EnsureParentDirectory 接收"文件路径"，逐段创建其父目录，这里借一个占位文件名
  // 把目标目录本身建出来。
  const std::string output_dir = TempJsbsimOutputDirectory();
  const std::string sentinel = output_dir + "/.keep";
  oneq::logging::EnsureParentDirectory(sentinel.c_str());
  fdm_exec.SetOutputPath(SGPath(output_dir));
}

void RotateXmlOutputBeforeRepeatedRunIC(JSBSim::FGFDMExec& fdm_exec) {
  const auto output = fdm_exec.GetOutput();
  if (output) {
    output->SetStartNewOutput();
  }
}

void ResetControlStateAfterTrimFailure(JSBSim::FGFDMExec& fdm_exec) {
  const char* properties[] = {
      property::kAileronCmd,          property::kElevatorCmd,        property::kRudderCmd,
      property::kPitchTrimCmd,        property::kRollTrimCmd,        property::kYawTrimCmd,
      property::kPitchRateIntegrator, property::kRollRateIntegrator, property::kYawRateIntegrator,
      property::kElevatorPosNorm,     property::kElevatorPosRad,     property::kLeftAileronPosRad,
      property::kRightAileronPosRad,  property::kRudderPosRad,
  };
  for (const char* property : properties) {
    SetPropertyIfPresent(fdm_exec, property, 0.0);
  }

  // Reset ALL fcs/* intermediate properties. DoTrim corrupts FBW internal
  // state (integrators, LQR outputs, actuator positions). A targeted reset
  // of known properties misses intermediate nodes like fcs/el-pitch-cmd or
  // fcs/pitch-cmd-summer, which then produce non-zero elevator deflection
  // on the very first Run() after recovery (observed: elevator-act=0.585
  // causing f22 to flip in <1s).
  auto* pm = fdm_exec.GetPropertyManager().get();
  if (pm) {
    auto* fcs_node = pm->GetNode("fcs");
    if (fcs_node) {
      for (int i = 0; i < fcs_node->nChildren(); ++i) {
        auto* child = fcs_node->getChild(i);
        if (child && std::abs(child->getDoubleValue()) > 1e-15) {
          child->setDoubleValue(0.0);
        }
      }
    }
  }
}

double InitialAltitudeM(const coordinate::ExternalKinematics& kinematics) {
  if (kinematics.position_frame == coordinate::PositionFrame::kLla) {
    return kinematics.position_lla_deg_m.altitude_m;
  }
  coordinate::LlaPositionDegM lla;
  if (coordinate::TryEcefToLla(kinematics.position_ecef_m, &lla)) {
    return lla.altitude_m;
  }
  return 0.0;
}

}  // namespace

JsbsimAdapter::JsbsimAdapter(const config::FlightDynamicConfig& config) {
  fdm_exec_.reset(new JSBSim::FGFDMExec());
  if (config.silent_mode) {
    fdm_exec_->SetDebugLevel(0);
  }

  SetDeltaT(config.dt_sec);
  PrepareXmlOutputPath(*fdm_exec_);

  if (!LoadAircraft(config)) {
    MarkInitializationFailure(&init_diag_,
                              "failed to load aircraft: " + config.aircraft_model);
    return;
  }
  init_diag_.model_loaded = true;

  DisableXmlOutput(*fdm_exec_);

  // Load the aircraft's reset00.xml to obtain the correct ground-start
  // altitude.  Each aircraft's gear extends below the body reference point
  // by a different amount; the reset XML's <altitude> sets AGL so the gear
  // contact points are at runway level.  Without this, RunIC's dt=0
  // integration leaves the gear penetrating the ground, producing
  // catastrophic spring force on the first real Run() frame.
  // VehicleStateMapper will skip its altitude override when the config
  // altitude is 0, preserving this XML value.
  std::string reset_path = fdm_exec_->GetFullAircraftPath().utf8Str() + "/reset00.xml";
  if (PathExists(reset_path)) {
    fdm_exec_->GetIC()->Load(SGPath(reset_path), false);  // useAircraftPath=false, path is absolute
    init_diag_.reset_xml_loaded = true;

  }

  ConfigureIntegrators(config);

  model::VehicleStateMapper::ApplyInitialConditions(*fdm_exec_, config.initial_kinematics,
                                                    config.initial_velocity_frame);
  init_diag_.ic_applied = true;

  if (!RunIC()) {
    MarkInitializationFailure(&init_diag_, "RunIC() failed");
    return;
  }
  init_diag_.run_ic_ok = true;

  // Variable-pitch propellers need an advance command before InitRunning()
  // computes propulsion steady state, otherwise turboprops can settle at
  // near-zero prop RPM and never develop takeoff thrust.
  SetPropellerAdvanceState(*fdm_exec_, 1.0);

  // Start all engines so that JSBSim can trim longitudinal velocity (udot).
  fdm_exec_->GetPropulsion()->InitRunning(-1);
  init_diag_.engines_started = true;

  // InitRunning sets per-engine FCS and propulsion throttles to 1.0 while
  // converging engine steady state for trim. Clear both surfaces so the first
  // Run() starts from adapter/user commands instead of stale full-power input.
  ResetThrottleState(*fdm_exec_, 0.0);
  const bool starts_in_air = InitialAltitudeM(config.initial_kinematics) > 10.0;
  if (starts_in_air) {
    RetractLandingGearIfModeled(*fdm_exec_);
    init_diag_.gear_retracted = true;
  } else {
    ExtendLandingGearIfModeled(*fdm_exec_);
  }
  SettleInitialGroundState(*fdm_exec_, config.dt_sec);

  if (config.do_trim) {
    init_diag_.trim_attempted = true;
    try {
      fdm_exec_->DoTrim(0);
      init_diag_.trim_succeeded = true;
    } catch (...) {
      init_diag_.trim_succeeded = false;
      // 中译：JSBSim 配平（DoTrim）抛出异常，配平未成功；适配器将继续以未配平状态运行。
      // 标识：初始化降级路径——JSBSim 库边界异常已在最窄处捕获并转换为本标志位，
      //       trim_succeeded=false 供上层诊断读取；随后将复位 FCS 内部状态。
      PROJECT_LOG_WARN("JsbsimAdapter: DoTrim(0) threw an exception, proceeding anyway.");

      // Reset FCS component internal state (integrator accumulators, filter
      // past values, actuator positions). SetProperty() alone is insufficient
      // because FGFCSComponent stores Output in a member variable that is not
      // backed by the property tree. Without this, corrupted integrator state
      // causes immediate elevator deflection (observed 0.585 on f22).
      // Save throttle state: InitModel() clears ThrottleCmd/ThrottlePos to 0.
      auto fcs = fdm_exec_->GetFCS();
      const auto& throttle_pos = fcs->GetThrottlePos();
      std::vector<double> saved_throttle = throttle_pos;
      for (auto& t : saved_throttle) {
        if (t < 0.5) t = 0.5;
      }
      fcs->InitModel();
      for (unsigned i = 0; i < saved_throttle.size(); ++i) {
        fcs->SetThrottleCmd(i, saved_throttle[i]);
        fcs->SetThrottlePos(i, saved_throttle[i]);
      }

      model::VehicleStateMapper::ApplyInitialConditions(*fdm_exec_, config.initial_kinematics,
                                                        config.initial_velocity_frame);
      RotateXmlOutputBeforeRepeatedRunIC(*fdm_exec_);
      if (!RunIC()) {
        MarkInitializationFailure(&init_diag_, "RunIC() failed after trim recovery");
        return;
      }
      SetPropellerAdvanceState(*fdm_exec_, 1.0);
      fdm_exec_->GetPropulsion()->InitRunning(-1);
      ResetThrottleState(*fdm_exec_, 0.0);
      if (InitialAltitudeM(config.initial_kinematics) > 10.0) {
        RetractLandingGearIfModeled(*fdm_exec_);
      } else {
        ExtendLandingGearIfModeled(*fdm_exec_);
      }
      ResetControlStateAfterTrimFailure(*fdm_exec_);
      init_diag_.trim_recovery_applied = true;
    }
  }
}

JsbsimAdapter::~JsbsimAdapter() = default;

bool JsbsimAdapter::Run() {
  if (!IsValid()) {
    return false;
  }
  return fdm_exec_->Run();
}

bool JsbsimAdapter::RunIC() {
  if (fdm_exec_ == nullptr) {
    return false;
  }
  return fdm_exec_->RunIC();
}

void JsbsimAdapter::SetDeltaT(double dt_sec) {
  if (fdm_exec_ != nullptr) {
    fdm_exec_->Setdt(dt_sec);
  }
}

double JsbsimAdapter::GetDeltaT() const { return fdm_exec_ != nullptr ? fdm_exec_->GetDeltaT() : 0.0; }

double JsbsimAdapter::GetProperty(const std::string& name) const {
  return fdm_exec_->GetPropertyValue(name);
}

void JsbsimAdapter::SetProperty(const std::string& name, double value) {
  fdm_exec_->SetPropertyValue(name, value);
}

bool JsbsimAdapter::HasProperty(const std::string& name) const {
  return fdm_exec_->GetPropertyManager()->GetNode(name) != nullptr;
}

JSBSim::FGPropagate& JsbsimAdapter::GetPropagate() { return *fdm_exec_->GetPropagate(); }

const JSBSim::FGPropagate& JsbsimAdapter::GetPropagate() const {
  return *fdm_exec_->GetPropagate();
}

JSBSim::FGAccelerations& JsbsimAdapter::GetAccelerations() {
  return *fdm_exec_->GetAccelerations();
}

const JSBSim::FGAccelerations& JsbsimAdapter::GetAccelerations() const {
  return *fdm_exec_->GetAccelerations();
}

JSBSim::FGFDMExec& JsbsimAdapter::GetFdmExec() { return *fdm_exec_; }

const JSBSim::FGFDMExec& JsbsimAdapter::GetFdmExec() const { return *fdm_exec_; }

bool JsbsimAdapter::LoadAircraft(const config::FlightDynamicConfig& config) {
  if (!config.aircraft_root_dir.empty()) {
    SGPath root(config.aircraft_root_dir);
    fdm_exec_->SetRootDir(root);
    fdm_exec_->SetAircraftPath(SGPath("aircraft"));
    fdm_exec_->SetEnginePath(SGPath("engine"));
    fdm_exec_->SetSystemsPath(SGPath("systems"));
  }
  return fdm_exec_->LoadModel(config.aircraft_model, true);
}

void JsbsimAdapter::ConfigureIntegrators(const config::FlightDynamicConfig& config) {
  SetProperty(property::kIntegratorRateRot, static_cast<double>(config.integrator_rate_rotational));
  SetProperty(property::kIntegratorRateTrans,
              static_cast<double>(config.integrator_rate_translational));
  SetProperty(property::kIntegratorPosRot, static_cast<double>(config.integrator_pos_rotational));
  SetProperty(property::kIntegratorPosTrans,
              static_cast<double>(config.integrator_pos_translational));
  SetProperty(property::kGravityModel, static_cast<double>(config.gravity_model));
  if (fdm_exec_->GetPropertyManager()->GetNode(property::kGuidanceRollAngleLimit) == nullptr) {
    SetProperty(property::kGuidanceRollAngleLimit, 0.785);  // 45 deg
  }
  if (fdm_exec_->GetPropertyManager()->GetNode(property::kGuidanceRollRateLimit) == nullptr) {
    SetProperty(property::kGuidanceRollRateLimit, 1.5);  // ~86 deg/s
  }
}

}  // namespace adapter
}  // namespace flight_dynamic
}  // namespace oneq
