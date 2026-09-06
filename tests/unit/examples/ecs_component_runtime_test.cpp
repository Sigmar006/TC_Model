/**
 * @file ecs_component_runtime_test.cpp
 * @brief component_attachment 组件运行时修改接口单元测试。
 *
 * 覆盖六个组件的运行时修改入口（未来外部调用入口，薄包装库 API）：
 * - ArSensorComponent::TryApplyRuntimeConfig —— AR 事务性提交（补丁先暂存、
 *   下个周期边界生效；与现有配置冲突的非法补丁入口即原子拒绝）。
 * - EsrSensorComponent::TryApplyRuntimeConfig / ApplyRuntimeConfigWithResult
 *   —— ESR 立即提交（调用即生效）；结构化结果携带拒绝原因枚举。
 * - EosSensorComponent::TryApplyRuntimeConfig —— EOS 立即提交（原子校验）。
 * - SbirsSensorComponent::TryApplyRuntimeConfig —— SBIRS 立即提交（原子校验）。
 * - SarSensorComponent::TryApplyRuntimeConfig —— SAR 立即提交（原子校验）。
 * - FlightComponent::PushManeuver / ClearManeuvers / Abort —— FD 命令式
 *   入口；FD 未启用/初始化失败（运动学回退）时返回 false。
 *
 * 只调用组件接口本身（不驱动 Step），验证转发语义与库一致；合法/非法
 * patch 取值沿用各模块 resolver 单测的已知用例。Fusion 无运行时修改设计
 * （库内无 RuntimeConfigPatch），不在覆盖范围（组件注释已注明）。
 */

#include <limits>

#include <gtest/gtest.h>

#include "1q/airborne_radar/config/ArProfileConstants.h"
#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/airborne_radar/session/ArTrackOutputDebugView.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/types.h"
#include "1q/electro_optical_sensor/config/EosRuntimeConfigPatch.h"
#include "1q/electro_optical_sensor/session/EosOutputDebugView.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigPatch.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "1q/electromagnetics/RfScene.h"
#include "1q/flight_dynamic/FlightManager.h"
#include "1q/fusion/SensorAdapters.h"
#include "1q/navigation/RoutePoint.h"
#include "1q/sar/config/SarRuntimeConfigPatch.h"
#include "1q/sar/session/SarSession.h"
#include "1q/sbirs_sensor/config/SbirsRuntimeConfigPatch.h"
#include "1q/sbirs_sensor/session/SbirsOutputDebugView.h"
#include "1q/sbirs_sensor/session/SbirsSceneTypes.h"
#include "1q/sbirs_sensor/session/SbirsSession.h"
#include "components/ar_sensor_component.h"
#include "components/eos_sensor_component.h"
#include "components/esr_sensor_component.h"
#include "components/flight_component.h"
#include "components/sar_sensor_component.h"
#include "components/sbirs_sensor_component.h"
#include "core/scene_types.h"
#include "core/world.h"
#include "core/rf_world_broker.h"

namespace ca = component_attachment;

namespace {

/// AR 最小合法会话配置（与 ar_rf_session_test 同源：policy 三 profile）。
airborne_radar::config::ArSessionConfig MakeArConfig() {
  airborne_radar::config::ArSessionConfig cfg;
  cfg.policy.detection = airborne_radar::config::profiles::kDetectionPriorityDetection;
  cfg.policy.tracking = airborne_radar::config::profiles::kFastAssociationTracking;
  cfg.policy.lifecycle = airborne_radar::config::profiles::kFastConfirmLifecycle;
  return cfg;
}

/// 机场位置（度制 LLA，FD 模式下视为地面）。
oneq::coordinate::LlaPositionDegM MakeAirfield() {
  oneq::coordinate::LlaPositionDegM airfield;
  airfield.latitude_deg = 30.0;
  airfield.longitude_deg = 120.0;
  airfield.altitude_m = 0.0;
  return airfield;
}

}  // namespace

// =============================================================================
// ArSensorComponent：AR 事务性提交（暂存语义，入口原子拒绝）
// =============================================================================

TEST(ArSensorComponentRuntimeTest, ValidWorkModePatchAccepted) {
  ca::ArSensorComponent component(airborne_radar::session::ArSession::Create(MakeArConfig()),
                                                /*platform_entity_id=*/1U,
                                                /*transmitter_equipment_id=*/1U);

  airborne_radar::config::ArRuntimeConfigPatch patch;
  patch.has_work_mode = true;
  patch.work_mode = airborne_radar::config::ArWorkMode::kStt;

  EXPECT_TRUE(component.TryApplyRuntimeConfig(patch));
}

TEST(ArSensorComponentRuntimeTest, NonPositiveBeamwidthRejected) {
  ca::ArSensorComponent component(airborne_radar::session::ArSession::Create(MakeArConfig()),
                                                /*platform_entity_id=*/1U,
                                                /*transmitter_equipment_id=*/1U);

  // 非正指令波束宽度（沿用 ar_runtime_patch_mapper_test 已知非法用例）。
  airborne_radar::config::ArRuntimeConfigPatch patch;
  patch.has_commanded_beamwidth_enabled = true;
  patch.commanded_beamwidth_enabled = true;
  patch.has_commanded_beamwidth_deg = true;
  patch.commanded_beamwidth_deg.commanded_az_beamwidth_deg = 0.0f;

  EXPECT_FALSE(component.TryApplyRuntimeConfig(patch));
}

// =============================================================================
// EsrSensorComponent：ESR 立即提交 + 结构化结果
// =============================================================================

TEST(EsrSensorComponentRuntimeTest, ValidScanRatePatchAppliesImmediately) {
  ca::EsrSensorComponent component(electronic_surveillance_radar::session::EsrSession::Create());

  electronic_surveillance_radar::config::EsrRuntimeConfigPatch patch;
  patch.has_scan_rate_hz = true;
  patch.scan_rate_hz = 4.0f;  // 合法扫描率（沿用 resolver 单测取值）

  EXPECT_TRUE(component.TryApplyRuntimeConfig(patch));

  // 结构化结果入口：同值语义下仍为已应用（resolver 校验值域而非变化）。
  const auto result = component.ApplyRuntimeConfigWithResult(patch);
  EXPECT_EQ(result.status,
            electronic_surveillance_radar::session::EsrRuntimeConfigApplyStatus::kApplied);
  EXPECT_TRUE(result.applied);
}

TEST(EsrSensorComponentRuntimeTest, InfiniteScanRateRejectedWithStructuredStatus) {
  ca::EsrSensorComponent component(electronic_surveillance_radar::session::EsrSession::Create());

  // 非有限扫描率（沿用 resolver 单测 kRejectedInvalidScanRate 用例）。
  electronic_surveillance_radar::config::EsrRuntimeConfigPatch patch;
  patch.has_scan_rate_hz = true;
  patch.scan_rate_hz = std::numeric_limits<float>::infinity();

  EXPECT_FALSE(component.TryApplyRuntimeConfig(patch));

  const auto result = component.ApplyRuntimeConfigWithResult(patch);
  EXPECT_EQ(
      result.status,
      electronic_surveillance_radar::session::EsrRuntimeConfigApplyStatus::kRejectedInvalidScanRate);
  EXPECT_FALSE(result.applied);
}

// =============================================================================
// EosSensorComponent：EOS 立即提交（原子校验）
// =============================================================================

TEST(EosSensorComponentRuntimeTest, ValidScanRatePatchAppliesImmediately) {
  ca::EosSensorComponent component(electro_optical_sensor::session::EosSession::Create());

  electro_optical_sensor::config::EosRuntimeConfigPatch patch;
  patch.has_scan_rate_deg_per_sec = true;
  patch.scan_rate_deg_per_sec = 60.0f;  // 合法扫描角速度（沿用 resolver 单测取值）

  EXPECT_TRUE(component.TryApplyRuntimeConfig(patch));
}

TEST(EosSensorComponentRuntimeTest, ZeroFrameRateRejectsWholePatch) {
  ca::EosSensorComponent component(electro_optical_sensor::session::EosSession::Create());

  // 非正帧率（沿用 resolver 单测 InvalidFieldRejectsWholePatch 用例）。
  electro_optical_sensor::config::EosRuntimeConfigPatch patch;
  patch.has_scan_rate_deg_per_sec = true;
  patch.scan_rate_deg_per_sec = 60.0f;
  patch.has_frame_rate_hz = true;
  patch.frame_rate_hz = 0.0f;

  EXPECT_FALSE(component.TryApplyRuntimeConfig(patch));
}

// =============================================================================
// SbirsSensorComponent：SBIRS 立即提交（校验原子拒绝）
// =============================================================================

TEST(SbirsSensorComponentRuntimeTest, ValidScanRatePatchAppliesImmediately) {
  ca::SbirsSensorComponent component(sbirs_sensor::session::SbirsSession::Create());

  sbirs_sensor::config::SbirsRuntimeConfigPatch patch;
  patch.has_scan_rate_deg_per_sec = true;
  patch.scan_rate_deg_per_sec = 25.0f;  // 合法扫描速率（沿用 resolver 单测取值）

  EXPECT_TRUE(component.TryApplyRuntimeConfig(patch));
}

TEST(SbirsSensorComponentRuntimeTest, NegativeScanRateRejectsWholePatch) {
  ca::SbirsSensorComponent component(sbirs_sensor::session::SbirsSession::Create());

  // 负扫描速率（沿用 ValidateSbirsSessionConfig 的 scan_rate >= 0 规则）。
  sbirs_sensor::config::SbirsRuntimeConfigPatch patch;
  patch.has_scan_rate_deg_per_sec = true;
  patch.scan_rate_deg_per_sec = -1.0f;

  EXPECT_FALSE(component.TryApplyRuntimeConfig(patch));
}

// =============================================================================
// SarSensorComponent：SAR 立即提交（校验原子拒绝）
// =============================================================================

TEST(SarSensorComponentRuntimeTest, DisableRawEchoAppliesImmediately) {
  ca::SarSensorComponent component(sar::session::SarSession::Create());

  // 关闭 raw echo 生成（合法开关翻转，沿用 session 层单测取值）。
  sar::config::SarRuntimeConfigPatch patch;
  patch.has_enable_raw_echo_generation = true;
  patch.enable_raw_echo_generation = false;

  EXPECT_TRUE(component.TryApplyRuntimeConfig(patch));
}

TEST(SarSensorComponentRuntimeTest, L1ImagingWithoutRawEchoRejected) {
  ca::SarSensorComponent component(sar::session::SarSession::Create());

  // L1 成像依赖 raw echo 生成（沿用 SarSessionRuntimeConfigTest
  // 的 L1ImagingWithoutRawEchoRejected 用例）。
  sar::config::SarRuntimeConfigPatch patch;
  patch.has_enable_raw_echo_generation = true;
  patch.enable_raw_echo_generation = false;
  ASSERT_TRUE(component.TryApplyRuntimeConfig(patch));

  sar::config::SarRuntimeConfigPatch l1_patch;
  l1_patch.has_enable_l1_rda_imaging = true;
  l1_patch.enable_l1_rda_imaging = true;

  EXPECT_FALSE(component.TryApplyRuntimeConfig(l1_patch));
}

// =============================================================================
// FlightComponent：FD 命令式入口（FD 不可用返回 false）
// =============================================================================

TEST(FlightComponentRuntimeTest, ManeuverInterfacesReportFlightDynamicsAvailability) {
  ca::FlightComponent component(MakeAirfield(), /*initial_heading_deg=*/90.0,
                                /*initial_speed_mps=*/50.0, /*cruise_altitude_m=*/400.0,
                                {});

  oneq::flight_dynamic::ManeuverCommand cmd;
  cmd.type = oneq::flight_dynamic::guidance::ManeuverType::kFlyToWaypoint;
  cmd.target.latitude_rad = 0.52;  // ≈ 30°（正东 3 航点首点量级）
  cmd.target.longitude_rad = 2.10;
  cmd.target.altitude_m = 400.0;
  cmd.target.radius_m = 500.0;
  cmd.target.speed_mps = 50.0;

  const bool pushed = component.PushManeuver(cmd);
  const bool cleared = component.ClearManeuvers();
  const bool aborted = component.Abort();

#if defined(ONEQ_CA_FLIGHT_DYNAMIC_ENABLED)
  // FD 启用且 JSBSim c172x 数据可用（third_party/jsbsim，与 fd 测试分区同
  // 前提）：三个入口均成功转发到 FlightManager。
  EXPECT_TRUE(pushed);
  EXPECT_TRUE(cleared);
  EXPECT_TRUE(aborted);
#else
  // FD 未启用：运动学回退路径，指令接口返回 false（不崩溃）。
  EXPECT_FALSE(pushed);
  EXPECT_FALSE(cleared);
  EXPECT_FALSE(aborted);
#endif
}

TEST(FlightComponentRuntimeTest, PatrolLoopWrapsRouteIndex) {
  // 循环巡逻（loop_route=true）：航路耗尽后回绕首个航点继续。两航点
  // 正东约 963 m 间距（0.01° @ 30°N），运动学寻的路径下每轮循环
  // 产生 2 条 waypoint_reached 事件、索引回绕 1 次。
  ca::AppSceneState scene;
  ca::World world(scene);
  ca::Entity& platform = world.CreateEntity("platform");

  std::vector<navigation::RoutePoint> route;
  navigation::RoutePoint wp0;
  wp0.position.latitude_deg = 30.0;
  wp0.position.longitude_deg = 120.01;
  wp0.position.altitude_m = 400.0;
  wp0.speed_mps = 50.0;
  wp0.radius_m = 200.0;
  navigation::RoutePoint wp1 = wp0;
  wp1.position.longitude_deg = 120.02;
  route.push_back(wp0);
  route.push_back(wp1);

  platform.Attach(std::make_unique<ca::FlightComponent>(
      MakeAirfield(), /*initial_heading_deg=*/90.0, /*initial_speed_mps=*/50.0,
      /*cruise_altitude_m=*/400.0, route, /*loop_route=*/true));
  auto* flight = platform.Find<ca::FlightComponent>();

  int reached_count = 0;
  boost::signals2::scoped_connection conn = world.signals().on_waypoint_reached.connect(
      [&](const ca::WaypointReachedEvent&) { ++reached_count; });

  std::size_t wraps = 0U;
  std::size_t prev_index = 0U;
  for (std::uint64_t cycle = 1U; cycle <= 400U; ++cycle) {
    scene.cycle = cycle;
    scene.t_sec = static_cast<double>(cycle);
    world.Step(1.0);
    const std::size_t index = flight->next_waypoint_index();
    if (index < prev_index) {
      ++wraps;
    }
    prev_index = index;
  }
  // 运动学回退：每轮约 40 s（2 × 963 m / 50 m/s），400 周期内完成多轮循环；
  // FD 路径含起飞段（~157 s），至少完成 1 轮回绕。
#if defined(ONEQ_CA_FLIGHT_DYNAMIC_ENABLED)
  EXPECT_GE(wraps, 1U);
  EXPECT_GE(reached_count, 3);
#else
  EXPECT_GE(wraps, 2U);
  EXPECT_GE(reached_count, 4);
#endif
}

// =============================================================================
// 传感器查询 getter：开关机 + 当前扫描方位（外置查询需求，实体选定后按名/
// ID 拉取最新快照的数据源）
// =============================================================================

namespace {

/// EOS 最小合法会话配置：周期 dt=1s 要求 dt ≤ 10/frame_rate_hz（默认帧率
/// 30 → 上限 0.33 s 会拒绝 1 s 周期），覆写为 10 Hz 与 demo 对齐。
electro_optical_sensor::config::EosSessionConfig MakeEosConfig() {
  electro_optical_sensor::config::EosSessionConfig cfg;
  cfg.mission.frame_rate_hz = 10.0f;
  return cfg;
}

/// 组件查询测试场景：World + 平台实体（Flight 组件）+ 四传感器最小目标真值。
/// world.Step 按挂载序驱动全部组件；每周期 dt=1 s。场景真值各注入一个
/// 正东 12 km 的最小目标（同一物理目标），保证各周期正常完成（角度断言
/// 隐含 kCompleted：非完成周期输出帧为空帧 → 方位 0，断言即失败）。
class SensorQueryScene {
 public:
  SensorQueryScene() {
    platform_ = &world_.CreateEntity("platform");
    platform_->Attach(std::make_unique<ca::FlightComponent>(
        MakeAirfield(), /*initial_heading_deg=*/90.0, /*initial_speed_mps=*/50.0,
        /*cruise_altitude_m=*/400.0, std::vector<navigation::RoutePoint>{}));
    FillTruths();
  }

  ca::World& world() { return world_; }
  ca::Entity& platform() { return *platform_; }

  /// 按窗口起点重建脉冲列波形（首脉冲 = 窗口起点，PRI 1ms×200 脉冲 ≈ 0.2 s
  /// 覆盖 1 s 周期窗口），并把重建后的辐射源写回 scene_.emitters[0]。
  void RebuildEmitterWaveforms(double window_start_time_s) {
    if (!oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
            window_start_time_s, 9.5e9, 2.0e6, 5.0e7, 1.0e-6, 1.0e-3, 200U, 0.0, 42U, 1U,
            &emitter_template_.waveform)) {
      return;
    }
    if (!scene_.emitters.empty()) {
      scene_.emitters[0] = emitter_template_;
    }
  }

  /// 推进一个周期（周期号/时间注入共享场景状态后 Step）。
  /// RF-WORLD 每周期初重建（脚本辐射源 + 上周期装备发射），与 demo 主循环同源；
  /// e62b6824 起 ESR/AR 组件消费 scene.rf_world 而非 scene.emitters。
  void DriveCycle(std::uint64_t cycle) {
    scene_.cycle = cycle;
    scene_.t_sec = static_cast<double>(cycle);
    RebuildEmitterWaveforms(scene_.t_sec);
    ca::BeginRfWorldCycle(&scene_, 1.0);
    world_.Step(1.0);
  }

 private:
  /// 四传感器各一个最小目标（构造镜像 component_attachment_demo 脚本）。
  void FillTruths() {
    const oneq::coordinate::LlaPositionDegM origin = MakeAirfield();
    oneq::coordinate::EnuPositionM offset;
    offset.east_m = 12000.0;
    oneq::coordinate::EcefPositionM target_ecef;
    oneq::coordinate::TryEnuToEcef(offset, origin, &target_ecef);

    // AR/EOS：同一世界 ECEF 真值（EOS 经平台锚点转 ENU；外观字段供光学通道）。
    ca::app::TargetEcefState ar_world_target;
    ar_world_target.id = 1001U;
    ar_world_target.position = target_ecef;
    ar_world_target.rcs = 2.0f;
    ar_world_target.temperature_k = 520.0f;
    ar_world_target.projected_area_m2 = 18.0f;
    scene_.world_targets.push_back(ar_world_target);

    // ESR：一个辐射源（脉冲列波形，10 GHz 级）。脉冲列自窗口起点铺开——
    // 与 demo 的 MakeEmitterTruths 同源语义：RF 活动必须与本周期窗口时频重叠，
    // 固定 t=0 起点的波形在 cycle>=2 会被 RF v2 接收机拒绝（零重叠整周期拒绝）。
    // 波形参数在 DriveCycle 按当前窗口起点重建（见 RebuildEmitterWaveforms）。
    emitter_template_ = oneq::electromagnetics::RfSceneEmission{};
    emitter_template_.identity.platform_id = 1001U;
    emitter_template_.identity.equipment_id = 1U;
    emitter_template_.identity.emission_id = 1U;
    emitter_template_.position_ecef_m = target_ecef;
    emitter_template_.antenna.peak_gain_dbi = 30.0;
    RebuildEmitterWaveforms(0.0);
    scene_.emitters.push_back(emitter_template_);

    // SBIRS：一个红外目标 + 天基平台（目标群中心正上方 +500 km，凝视模式）。
    sbirs_sensor::session::SbirsSceneTarget ir;
    ir.target_id = 1001U;
    ir.target_name = "ir_target_1001";
    ir.position_ecef_m.x = target_ecef.x_m;
    ir.position_ecef_m.y = target_ecef.y_m;
    ir.position_ecef_m.z = target_ecef.z_m;
    ir.radiant_intensity_w_per_sr = 3819.864;
    ir.active = true;
    scene_.sbirs_targets.push_back(ir);
    scene_.sbirs_satellite_position_ecef_m.x = target_ecef.x_m;
    scene_.sbirs_satellite_position_ecef_m.y = target_ecef.y_m;
    scene_.sbirs_satellite_position_ecef_m.z = target_ecef.z_m + 500000.0;
  }

  ca::AppSceneState scene_;
  ca::World world_{scene_};
  oneq::electromagnetics::RfSceneEmission emitter_template_{};  ///< 脉冲列参数模板（波形每周期按窗口重建）
  ca::Entity* platform_{nullptr};
};

}  // namespace

TEST(SensorQueryGettersTest, CompletedCyclesReportPoweredOnAndScanAzimuth) {
  SensorQueryScene scene;
  // Demo mount order: Flight → ESR → [ECM] → AR → … so AR RF publish does not
  // land in rf_world before ESR consumes the script emitters for this cycle.
  scene.platform().Attach(std::make_unique<ca::EsrSensorComponent>(
      electronic_surveillance_radar::session::EsrSession::Create()));
  scene.platform().Attach(std::make_unique<ca::ArSensorComponent>(
      airborne_radar::session::ArSession::Create(MakeArConfig()), 1U, 1U));
  scene.platform().Attach(std::make_unique<ca::EosSensorComponent>(
      electro_optical_sensor::session::EosSession::Create(MakeEosConfig())));
  scene.platform().Attach(std::make_unique<ca::SbirsSensorComponent>(
      sbirs_sensor::session::SbirsSession::Create()));
  scene.platform().Attach(std::make_unique<ca::SarSensorComponent>(
      sar::session::SarSession::Create()));

  scene.DriveCycle(1U);

  // 开关机：完成周期全部开机（AR 补丁暂存语义不影响状态查询）。
  EXPECT_TRUE(scene.platform().Find<ca::ArSensorComponent>()->powered_on());
  EXPECT_TRUE(scene.platform().Find<ca::EsrSensorComponent>()->powered_on());
  EXPECT_TRUE(scene.platform().Find<ca::EosSensorComponent>()->powered_on());
  EXPECT_TRUE(scene.platform().Find<ca::SbirsSensorComponent>()->powered_on());
  EXPECT_TRUE(scene.platform().Find<ca::SarSensorComponent>()->powered_on());

  // 扫描方位：首周期波束中心方位（各库"先推进相位再填充"或"先索引再推进"
  // 的既有语义，期望值按默认配置推导：ESR 相位 0 → 首波束 -60°；EOS
  // start -60° + 速率 20°/s×1s = -40°；SBIRS start 300°（-60° 等价折入
  // [0,360)，ECI 约定）+ 速率 10°/s×1s = 310°——组件把库内弧度转度显示）。
  EXPECT_FLOAT_EQ(scene.platform().Find<ca::EsrSensorComponent>()->scan_azimuth_deg(), -60.0f);
  EXPECT_FLOAT_EQ(scene.platform().Find<ca::EosSensorComponent>()->scan_azimuth_deg(), -40.0f);
  EXPECT_FLOAT_EQ(scene.platform().Find<ca::SbirsSensorComponent>()->scan_azimuth_deg(), 310.0f);
}

TEST(SensorQueryGettersTest, PowerOffPatchFlipsPoweredStateAndClearsAzimuth) {
  SensorQueryScene scene;
  // Demo mount order: Flight → ESR → [ECM] → AR → … so AR RF publish does not
  // land in rf_world before ESR consumes the script emitters for this cycle.
  scene.platform().Attach(std::make_unique<ca::EsrSensorComponent>(
      electronic_surveillance_radar::session::EsrSession::Create()));
  scene.platform().Attach(std::make_unique<ca::ArSensorComponent>(
      airborne_radar::session::ArSession::Create(MakeArConfig()), 1U, 1U));
  scene.platform().Attach(std::make_unique<ca::EosSensorComponent>(
      electro_optical_sensor::session::EosSession::Create(MakeEosConfig())));
  scene.platform().Attach(std::make_unique<ca::SbirsSensorComponent>(
      sbirs_sensor::session::SbirsSession::Create()));
  scene.platform().Attach(std::make_unique<ca::SarSensorComponent>(
      sar::session::SarSession::Create()));

  scene.DriveCycle(1U);  // 开机周期：查询状态就绪

  // 五传感器全部下电（各模块 sensor_enabled 补丁；AR 为事务性
  // 暂存，下个周期边界生效，其余立即生效）。
  {
    airborne_radar::config::ArRuntimeConfigPatch ar_off;
    ar_off.has_sensor_enabled = true;
    ar_off.sensor_enabled = false;
    EXPECT_TRUE(scene.platform().Find<ca::ArSensorComponent>()->TryApplyRuntimeConfig(ar_off));
  }
  {
    electronic_surveillance_radar::config::EsrRuntimeConfigPatch esr_off;
    esr_off.has_sensor_enabled = true;
    esr_off.sensor_enabled = false;
    EXPECT_TRUE(scene.platform().Find<ca::EsrSensorComponent>()->TryApplyRuntimeConfig(esr_off));
  }
  {
    electro_optical_sensor::config::EosRuntimeConfigPatch eos_off;
    eos_off.has_sensor_enabled = true;
    eos_off.sensor_enabled = false;
    EXPECT_TRUE(scene.platform().Find<ca::EosSensorComponent>()->TryApplyRuntimeConfig(eos_off));
  }
  {
    sbirs_sensor::config::SbirsRuntimeConfigPatch sbirs_off;
    sbirs_off.has_sensor_enabled = true;
    sbirs_off.sensor_enabled = false;
    EXPECT_TRUE(scene.platform().Find<ca::SbirsSensorComponent>()->TryApplyRuntimeConfig(sbirs_off));
  }
  {
    sar::config::SarRuntimeConfigPatch sar_off;
    sar_off.has_sensor_enabled = true;
    sar_off.sensor_enabled = false;
    EXPECT_TRUE(scene.platform().Find<ca::SarSensorComponent>()->TryApplyRuntimeConfig(sar_off));
  }

  scene.DriveCycle(2U);

  // 开关机：下电周期全部关机。
  EXPECT_FALSE(scene.platform().Find<ca::ArSensorComponent>()->powered_on());
  EXPECT_FALSE(scene.platform().Find<ca::EsrSensorComponent>()->powered_on());
  EXPECT_FALSE(scene.platform().Find<ca::EosSensorComponent>()->powered_on());
  EXPECT_FALSE(scene.platform().Find<ca::SbirsSensorComponent>()->powered_on());
  EXPECT_FALSE(scene.platform().Find<ca::SarSensorComponent>()->powered_on());

  // 扫描方位：关机时组件不驱动会话、角度主动清零（不残留上周期方位）。
  EXPECT_FLOAT_EQ(scene.platform().Find<ca::EsrSensorComponent>()->scan_azimuth_deg(), 0.0f);
  EXPECT_FLOAT_EQ(scene.platform().Find<ca::EosSensorComponent>()->scan_azimuth_deg(), 0.0f);
  EXPECT_FLOAT_EQ(scene.platform().Find<ca::SbirsSensorComponent>()->scan_azimuth_deg(), 0.0f);

  // 重新上电：SAR 组件恢复驱动（状态翻转回 true，会话可继续工作）。
  {
    sar::config::SarRuntimeConfigPatch sar_on;
    sar_on.has_sensor_enabled = true;
    sar_on.sensor_enabled = true;
    EXPECT_TRUE(scene.platform().Find<ca::SarSensorComponent>()->TryApplyRuntimeConfig(sar_on));
  }
  scene.DriveCycle(3U);
  EXPECT_TRUE(scene.platform().Find<ca::SarSensorComponent>()->powered_on());
}

TEST(SensorQueryGettersTest, PowerOffFreezesSessionUntilReenabled) {
  // 组件层电源门控：关机期间不驱动会话，会话扫描相位冻结；重新上电后
  // 从冻结处继续推进（SBIRS 每周期推进 10°/s × 1 s，可用角度差验证）。
  SensorQueryScene scene;
  scene.platform().Attach(std::make_unique<ca::SbirsSensorComponent>(
      sbirs_sensor::session::SbirsSession::Create()));

  // 开机首周期：start 300°（-60° 等价）+ 推进 10° = 310°。
  scene.DriveCycle(1U);
  EXPECT_TRUE(scene.platform().Find<ca::SbirsSensorComponent>()->powered_on());
  EXPECT_FLOAT_EQ(scene.platform().Find<ca::SbirsSensorComponent>()->scan_azimuth_deg(), 310.0f);

  // 下电：关机两个周期，组件短路（不驱动会话），角度清零。
  {
    sbirs_sensor::config::SbirsRuntimeConfigPatch sbirs_off;
    sbirs_off.has_sensor_enabled = true;
    sbirs_off.sensor_enabled = false;
    EXPECT_TRUE(scene.platform().Find<ca::SbirsSensorComponent>()->TryApplyRuntimeConfig(sbirs_off));
  }
  scene.DriveCycle(2U);
  scene.DriveCycle(3U);
  EXPECT_FALSE(scene.platform().Find<ca::SbirsSensorComponent>()->powered_on());
  EXPECT_FLOAT_EQ(scene.platform().Find<ca::SbirsSensorComponent>()->scan_azimuth_deg(), 0.0f);

  // 重新上电：相位从关机前冻结处继续（310° 基础上仅推进一个周期 → 320°）；
  // 若关机期间被驱动，相位会多推进两个周期（→ 340°），断言即失败。
  {
    sbirs_sensor::config::SbirsRuntimeConfigPatch sbirs_on;
    sbirs_on.has_sensor_enabled = true;
    sbirs_on.sensor_enabled = true;
    EXPECT_TRUE(scene.platform().Find<ca::SbirsSensorComponent>()->TryApplyRuntimeConfig(sbirs_on));
  }
  scene.DriveCycle(4U);
  EXPECT_TRUE(scene.platform().Find<ca::SbirsSensorComponent>()->powered_on());
  // 310° + 10° = 320°。
  EXPECT_FLOAT_EQ(scene.platform().Find<ca::SbirsSensorComponent>()->scan_azimuth_deg(), 320.0f);
}

TEST(SensorQueryGettersTest, SbirsLastDebugViewCarriesPerTargetStateAndExclusionDiagnostics) {
  // 规则 12/13b 组件级可见性：LastDebugView() 每周期由 Step 经
  // SbirsOutputDebugViewBuilder 回填（per-target 状态 + kInfo 排除诊断），
  // 供调用方结构化持久化；组件另直写人读摘要行到集成端日志。
  SensorQueryScene scene;
  scene.platform().Attach(std::make_unique<ca::SbirsSensorComponent>(
      sbirs_sensor::session::SbirsSession::Create()));
  scene.DriveCycle(1U);

  const auto& view = scene.platform().Find<ca::SbirsSensorComponent>()->LastDebugView();
  EXPECT_EQ(view.input_cycle_index, 1U);
  EXPECT_TRUE(view.executed_this_cycle);
  ASSERT_EQ(view.targets.size(), 1U);
  EXPECT_EQ(view.targets[0].target_id, 1001U);
  // 默认配置下目标（星下点 el≈-90°、az≈0°）落在 WFOV（el 中心 0°、az 扫描
  // 中心 -50°、20°×20°）之外 → kNotInOutput + sbirs.target_out_of_wfov
  // kInfo 诊断（规则 13b，message 含 target_id）。
  EXPECT_EQ(view.targets[0].status,
            sbirs_sensor::session::SbirsDebugTargetStatus::kNotInOutput);
  bool found_wfov = false;
  for (const auto& issue : view.issues) {
    if (issue.code == "sbirs.target_out_of_wfov") {
      found_wfov = true;
      EXPECT_EQ(issue.severity, sbirs_sensor::session::SbirsIssueSeverity::kInfo);
      EXPECT_NE(issue.message.find("target_id=1001"), std::string::npos);
    }
  }
  EXPECT_TRUE(found_wfov);
}

TEST(SensorQueryGettersTest, ArLastDebugViewCarriesPerTargetState) {
  // 规则 12 组件级可见性（AR 通道）：LastDebugView() 每周期由 Step 经
  // ArTrackOutputDebugViewBuilder 回填（input 目标逐条列出，含外部目标标识
  // 与输入存在标志），供调用方结构化持久化。
  SensorQueryScene scene;
  scene.platform().Attach(std::make_unique<ca::ArSensorComponent>(
      airborne_radar::session::ArSession::Create(MakeArConfig()), 1U, 1U));
  scene.DriveCycle(1U);

  const auto& view = scene.platform().Find<ca::ArSensorComponent>()->LastDebugView();
  EXPECT_EQ(view.world_cycle_index, 1U);
  EXPECT_TRUE(view.completed_this_cycle);
  ASSERT_EQ(view.tracks.size(), 1U);
  EXPECT_EQ(view.tracks[0].external_target_id, 1001U);
  EXPECT_TRUE(view.tracks[0].present_in_input);
}

TEST(SensorQueryGettersTest, EosLastDebugViewCarriesPerTargetStateAndExclusionDiagnostics) {
  // 规则 12/13b 组件级可见性（EOS 通道）：LastDebugView() 每周期由 Step 经
  // EosOutputDebugViewBuilder 回填（per-target 状态 + kInfo 排除诊断），
  // 供调用方结构化持久化。
  SensorQueryScene scene;
  scene.platform().Attach(std::make_unique<ca::EosSensorComponent>(
      electro_optical_sensor::session::EosSession::Create(MakeEosConfig())));
  scene.DriveCycle(1U);

  const auto& view = scene.platform().Find<ca::EosSensorComponent>()->LastDebugView();
  EXPECT_EQ(view.input_cycle_index, 1U);
  EXPECT_TRUE(view.executed_this_cycle);
  ASSERT_EQ(view.targets.size(), 1U);
  EXPECT_EQ(view.targets[0].target_id, 1001U);
  // 默认配置下目标在平台视轴（俯仰中心 -48° 下视）外 → kNotInOutput +
  // eos.target_out_of_fov kInfo 诊断（规则 13b，message 含 target_id）。
  EXPECT_EQ(view.targets[0].status,
            electro_optical_sensor::session::EosDebugTargetStatus::kNotInOutput);
  bool found_fov = false;
  for (const auto& issue : view.issues) {
    if (issue.code == "eos.target_out_of_fov") {
      found_fov = true;
      EXPECT_EQ(issue.severity, electro_optical_sensor::session::EosIssueSeverity::kInfo);
      EXPECT_NE(issue.message.find("target_id=1001"), std::string::npos);
    }
  }
  EXPECT_TRUE(found_fov);
}

TEST(SensorQueryGettersTest, SarLastDebugViewCarriesProductState) {
  // 规则 12 组件级可见性（SAR 通道）：LastDebugView() 每周期由 Step 经
  // SarProductDebugViewBuilder 回填（阶段型视图：执行状态/完成阶段/L1/L3
  // 成像标志/SNR/点目标/问题列表），供调用方结构化持久化。单测场景未注入
  // SAR 点目标且默认配置侧视几何不保证成立（squint 门控属执行期拒绝）：
  // 只断言视图被回填且与周期结果一致，不断言产品产出。
  SensorQueryScene scene;
  scene.platform().Attach(std::make_unique<ca::SarSensorComponent>(
      sar::session::SarSession::Create()));
  scene.DriveCycle(1U);

  const auto& view = scene.platform().Find<ca::SarSensorComponent>()->LastDebugView();
  // 执行期门控的拒绝仍回填周期号（validation 拒绝才会留 0；本场景输入合法）。
  EXPECT_EQ(view.input_cycle_index, 1U);
  // 点目标列表镜像本周期输入（无论是否成像）。
  EXPECT_TRUE(view.point_targets.empty());
  // 执行标志与中止原因由构建器从周期结果回填：未执行周期必带中止原因码。
  if (!view.executed_this_cycle) {
    EXPECT_FALSE(view.abort_reason.empty());
  }
}

TEST(SensorQueryGettersTest, LastDebugViewClearedOnPowerOff) {
  // 关机：组件不驱动会话，调试视图清零（无有效周期，与扫描方位清零同语义；
  // 与 SBIRS 组件行为一致，AR/EOS/SAR 对齐后同样不残留上周期快照）。
  SensorQueryScene scene;
  scene.platform().Attach(std::make_unique<ca::ArSensorComponent>(
      airborne_radar::session::ArSession::Create(MakeArConfig()), 1U, 1U));
  scene.platform().Attach(std::make_unique<ca::EosSensorComponent>(
      electro_optical_sensor::session::EosSession::Create(MakeEosConfig())));
  scene.platform().Attach(std::make_unique<ca::SbirsSensorComponent>(
      sbirs_sensor::session::SbirsSession::Create()));
  scene.platform().Attach(std::make_unique<ca::SarSensorComponent>(
      sar::session::SarSession::Create()));

  scene.DriveCycle(1U);  // 开机周期：视图已回填
  EXPECT_EQ(scene.platform().Find<ca::ArSensorComponent>()->LastDebugView().tracks.size(), 1U);
  EXPECT_EQ(scene.platform().Find<ca::EosSensorComponent>()->LastDebugView().targets.size(), 1U);
  EXPECT_EQ(scene.platform().Find<ca::SbirsSensorComponent>()->LastDebugView().targets.size(), 1U);
  EXPECT_EQ(scene.platform().Find<ca::SarSensorComponent>()->LastDebugView().input_cycle_index, 1U);

  {
    airborne_radar::config::ArRuntimeConfigPatch ar_off;
    ar_off.has_sensor_enabled = true;
    ar_off.sensor_enabled = false;
    EXPECT_TRUE(scene.platform().Find<ca::ArSensorComponent>()->TryApplyRuntimeConfig(ar_off));
  }
  {
    electro_optical_sensor::config::EosRuntimeConfigPatch eos_off;
    eos_off.has_sensor_enabled = true;
    eos_off.sensor_enabled = false;
    EXPECT_TRUE(scene.platform().Find<ca::EosSensorComponent>()->TryApplyRuntimeConfig(eos_off));
  }
  {
    sbirs_sensor::config::SbirsRuntimeConfigPatch sbirs_off;
    sbirs_off.has_sensor_enabled = true;
    sbirs_off.sensor_enabled = false;
    EXPECT_TRUE(scene.platform().Find<ca::SbirsSensorComponent>()->TryApplyRuntimeConfig(sbirs_off));
  }
  {
    sar::config::SarRuntimeConfigPatch sar_off;
    sar_off.has_sensor_enabled = true;
    sar_off.sensor_enabled = false;
    EXPECT_TRUE(scene.platform().Find<ca::SarSensorComponent>()->TryApplyRuntimeConfig(sar_off));
  }
  scene.DriveCycle(2U);

  // 关机周期：四通道调试视图均为默认清零快照（无残留目标行/周期号）。
  EXPECT_EQ(scene.platform().Find<ca::ArSensorComponent>()->LastDebugView().tracks.size(), 0U);
  EXPECT_EQ(scene.platform().Find<ca::EosSensorComponent>()->LastDebugView().targets.size(), 0U);
  EXPECT_EQ(scene.platform().Find<ca::SbirsSensorComponent>()->LastDebugView().targets.size(), 0U);
  EXPECT_EQ(scene.platform().Find<ca::SarSensorComponent>()->LastDebugView().input_cycle_index, 0U);
  EXPECT_FALSE(scene.platform().Find<ca::ArSensorComponent>()->LastDebugView().completed_this_cycle);
  EXPECT_FALSE(scene.platform().Find<ca::EosSensorComponent>()->LastDebugView().executed_this_cycle);
  EXPECT_FALSE(scene.platform().Find<ca::SbirsSensorComponent>()->LastDebugView().executed_this_cycle);
  EXPECT_FALSE(scene.platform().Find<ca::SarSensorComponent>()->LastDebugView().executed_this_cycle);
}

TEST(SbirsSensorComponentRuntimeTest, SingleStationAttachFallsBackToSbirsSourceId) {
  ca::SceneState scene_state;
  ca::World world(scene_state);
  ca::Entity& platform = world.CreateEntity("platform");
  EXPECT_GT(platform.id(), 0U);

  platform.Attach(std::make_unique<ca::SbirsSensorComponent>(
      sbirs_sensor::session::SbirsSession::Create()));

  auto* comp = platform.Find<ca::SbirsSensorComponent>();
  ASSERT_NE(comp, nullptr);
  EXPECT_EQ(comp->device_id(), fusion::kSbirsSourceId);
}

TEST(SbirsSensorComponentRuntimeTest, SingleStationAttachUsesInjectedDeviceId) {
  ca::SceneState scene_state;
  ca::World world(scene_state);
  ca::Entity& platform = world.CreateEntity("platform");

  platform.Attach(std::make_unique<ca::SbirsSensorComponent>(
      sbirs_sensor::session::SbirsSession::Create(), 42U));

  auto* comp = platform.Find<ca::SbirsSensorComponent>();
  ASSERT_NE(comp, nullptr);
  EXPECT_EQ(comp->device_id(), 42U);
}

TEST(SensorComponentDeviceIdTest, ComponentsDefaultAndInjectedDeviceIds) {
  ca::ArSensorComponent ar(airborne_radar::session::ArSession::Create(), 1U, 1U, 101U);
  EXPECT_EQ(ar.device_id(), 101U);

  ca::ArSensorComponent ar_def(airborne_radar::session::ArSession::Create(), 1U, 1U);
  EXPECT_EQ(ar_def.device_id(), fusion::kArSourceId);

  ca::EsrSensorComponent esr(electronic_surveillance_radar::session::EsrSession::Create(), 102U);
  EXPECT_EQ(esr.device_id(), 102U);

  ca::EsrSensorComponent esr_def(electronic_surveillance_radar::session::EsrSession::Create());
  EXPECT_EQ(esr_def.device_id(), fusion::kEsrSourceId);

  ca::EosSensorComponent eos(electro_optical_sensor::session::EosSession::Create(), 103U);
  EXPECT_EQ(eos.device_id(), 103U);

  ca::EosSensorComponent eos_def(electro_optical_sensor::session::EosSession::Create());
  EXPECT_EQ(eos_def.device_id(), fusion::kEosSourceId);
}
