/**
 * @file runner.cpp
 * @brief 场景装配与执行入口（见 runner.h）。
 *
 * RunScene 自原 app/main.cpp 主程序体迁入（行为逐行保持）：加载场景 JSON →
 * 集成端日志初始化 → 实体/组件装配（挂载序 = 步进序）→ 周期循环（真值注入
 * → 指令派发 → World::Step → 落盘）→ 摘要与冒烟断言。通用 runner
 * （app/main.cpp）与每场景可执行（scenes/<name>/main.cpp）共用本入口。
 */
#include "app/runner.h"

#include "app/output_dir.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "app/fs_compat.h"

#include "1q/airborne_radar/session/ArSession.h"
#include "1q/coordinate/types.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "1q/fusion/FusionEngine.h"
#include "1q/fusion/FusedTarget.h"
#include "1q/sar/session/SarSession.h"
#include "1q/sbirs_sensor/session/SbirsSession.h"

#include "components/ar_sensor_component.h"
#include "components/ecm_sensor_component.h"
#include "logger/logger.h"
#include "components/eos_sensor_component.h"
#include "components/esr_sensor_component.h"
#include "components/flight_component.h"
#include "components/fusion_component.h"
#include "components/inference_component.h"
#include "components/rir_sensor_component.h"
#include "components/sar_sensor_component.h"
#include "components/sbirs_sensor_component.h"
#include "core/rf_world_broker.h"
#include "core/scene_types.h"
#include "components/threat_component.h"
#include "core/world.h"
#include "logger/acceptance_timing.h"
#include "app/command_routing.h"
#include "app/outputs.h"
#include "scenes/scene_data.h"
#include "scenes/scene_script.h"

namespace ca = component_attachment;

namespace component_attachment {
namespace app {

namespace {

/// 脚本指令 → 人读指令名（command_issued 事件行与 DecisionListener 文案同形）。
const char* ScriptedCommandName(ca::CommandKind kind) {
  switch (kind) {
    case ca::CommandKind::kDesignateTarget:
      return "DESIGNATE_TARGET";
    case ca::CommandKind::kEngageHighThreat:
      return "ENGAGE_HIGH_THREAT";
    case ca::CommandKind::kClearDesignation:
      return "CLEAR_DESIGNATION";
    case ca::CommandKind::kEnableAntiFalseTarget:
      return "ENABLE_ANTI_FALSE_TARGET_DISCRIMINATION";
  }
  return "UNKNOWN";
}

}  // namespace

/// 默认输出根目录（CMake 注入 CA_DEFAULT_OUTPUT_DIR：仓库内 examples/log/，
/// 运行时产物不入版本控制，见 .gitignore；无宏时的回退值供独立编译）。
#ifdef CA_DEFAULT_OUTPUT_DIR
constexpr char kDefaultOutputDir[] = CA_DEFAULT_OUTPUT_DIR;
#else
constexpr char kDefaultOutputDir[] = "examples/log";
#endif

void PrintUsage(const char* program) {
  std::cout << "Usage: " << program
            << " [--scene <path>] [--cycles <n>] [--view-every <n>] [--output-dir <dir>]\n"
            << "  --scene <path>      场景描述文件（默认 <场景目录>/baseline_takeoff_east.json）\n"
            << "  --cycles <n>        仿真周期数（覆盖场景文件，默认场景文件值）\n"
            << "  --view-every <n>    视图摘要间隔（周期求余，覆盖场景 view_log_every_cycles）\n"
            << "  --output-dir <dir>  日志+CSV 目录（默认 " << kDefaultOutputDir
            << "/<场景名>/）\n";
}

int RunScene(const std::string& scene_path, const RunOptions& options) {
  const int cycles_override = options.cycles_override;
  const int view_every_override = options.view_every_override;
  std::string output_dir = options.output_dir;
  const bool output_dir_overridden = !output_dir.empty();
  if (!output_dir_overridden) {
    output_dir = std::string(kDefaultOutputDir);
  }

  // 场景加载（场景层几何/真值/挂载/指令/冒烟 + session_config 自持会话
  // 配置：挂载即全量，缺挂载子块/未挂载携带 → 报错退出）。
  app::SceneData scene_data;
  app::SceneSessionConfigs configs;
  std::string scene_error;
  if (!app::LoadSceneData(scene_path.c_str(), &scene_data, &configs, &scene_error)) {
    std::cerr << "Failed to load scene '" << scene_path << "': " << scene_error << "\n";
    return 1;
  }
  const std::uint32_t num_cycles =
      cycles_override > 0 ? static_cast<std::uint32_t>(cycles_override) : scene_data.cycles;
  if (num_cycles == 0U) {
    std::cerr << "Invalid cycle count: must be > 0 (scene '" << scene_path
              << "' or --cycles)\n";
    return 1;
  }
  const std::uint32_t view_every =
      view_every_override > 0 ? static_cast<std::uint32_t>(view_every_override)
                              : scene_data.view_log_every_cycles;
  if (view_every == 0U) {
    std::cerr << "Invalid --view-every / view_log_every_cycles: must be > 0\n";
    return 1;
  }
  app::SetViewLogEveryCycles(view_every);
  if (!output_dir_overridden) {
    // 场景自带日志落点：log_dir（场景 JSON 必填，解析基点 examples/log/）。
    output_dir = std::string(kDefaultOutputDir) + "/" + scene_data.log_dir;
  }
  if (app::IsInsideTempArea(output_dir)) {
    std::cerr << "refusing output dir \"" << output_dir
              << "\": 场景产物禁止落到系统临时目录（强制约束；输出位置由场景 "
                 "JSON 的 log_dir 声明，--output-dir 仅允许非临时目录覆盖）\n";
    return 1;
  }
  app_fs::create_directories(output_dir);

  // 集成端日志初始化（三个日志文件：库日志 1q_library.log + 集成端事件行
  // integration_events.log + 视图行 integration_views.log；见 logger/logger.h）。
  // 须在会话创建之前调用：库内 PROJECT_LOG_* 走 spdlog 默认 logger，装配后库
  // 日志即入文件而非 stdout。
  app::InitIntegrationLog(output_dir);

  // 装配：共享场景状态 → World → 平台实体 + 8 组件（挂载序 = 步进序）。
  ca::AppSceneState scene;
  ca::World world(scene);

  if (scene_data.ecm_enabled) {
    // 同平台 ECM 发射链与 AR 接收链 co-site 隔离（演示层与跨域集成测试同量级）；
    // 发射机设备号取自场景 session_config.ecm（单源：JSON）。
    configs.ar.hardware.receiver.co_site_paths.push_back(
        {configs.ecm.transmitter_equipment_id, configs.ar.hardware.receiver.equipment_id,
         100.0});
  }

  // RIR 地基识别雷达站点（可选，场景 rir.enabled）：独立实体（固定站点，
  // S 波段识别雷达的物理摆放——非机载），会话配置来自场景
  // session_config.rir（识别库路径由 CMake 注入 CA_RIR_DATABASE_PATH 解析）。
  // 实体按创建序步进：站点先于平台
  // 创建，其本周期特征量测在平台融合组件 Step 时已就绪（同周期聚合，无跨
  // 周期滞后）。
  ca::Entity* rir_site = nullptr;
  if (scene_data.rir_enabled) {
    rir_site = &world.CreateEntity(ca::kRirSiteEntityName);
    const std::chrono::steady_clock::time_point rir_create_begin =
        std::chrono::steady_clock::now();
    remote_identification_radar::session::RirSession rir_session =
        remote_identification_radar::session::RirSession::Create(configs.rir);
    const double rir_create_ms = app::SteadyElapsedMs(rir_create_begin);
    app::LogAcceptanceMs(0, 0.0, "初始化时间性能测试", "RIR", rir_create_ms);
    app::LogAcceptanceMs(0, 0.0, "单个模型加载时间性能测试", "RIR",
                          rir_session.LastRecognitionDatabaseLoadMs());
    rir_site->Attach(std::make_unique<ca::RirSensorComponent>(
        std::move(rir_session), scene_data.rir_site_origin, configs.rir.sensor_platform_id,
        configs.rir.mission.recognition_dwell_sec, scene_data.device_ids.rir));
  }

  ca::Entity& platform = world.CreateEntity("platform");

  // 平台初始状态来自场景文件：机场地面（alt 0，六自由度机动从起飞开始）→
  // 巡航高度 → 沿场景航路巡航；区域巡逻场景（coverage 块）航路来自
  // AreaCoveragePlanner 规划，循环巡逻（loop_route）。
  const oneq::coordinate::LlaPositionDegM platform_origin = scene_data.platform_origin;
  platform.Attach(std::make_unique<ca::FlightComponent>(
      platform_origin, scene_data.initial_heading_deg, scene_data.cruise_speed_mps,
      scene_data.cruise_altitude_m, scene_data.waypoints,
      /*loop_route=*/scene_data.coverage.planned));

  // RF 链挂载序：Flight → ESR → [ECM] → AR → …。场景 sensors.* = false 则不挂，
  // 该通道不写视图/排除原因。
  if (scene_data.esr_enabled) {
    platform.Attach(std::make_unique<ca::EsrSensorComponent>(
        electronic_surveillance_radar::session::EsrSession::Create(configs.esr),
        scene_data.device_ids.esr));
  }
  if (scene_data.ecm_enabled && scene_data.esr_enabled) {
    platform.Attach(std::make_unique<ca::EcmSensorComponent>(
        electronic_countermeasure::session::EcmSession::Create(configs.ecm)));
  }
  if (scene_data.ar_enabled) {
    platform.Attach(std::make_unique<ca::ArSensorComponent>(
        airborne_radar::session::ArSession::Create(configs.ar), ca::kPlatformEntityId,
        configs.ar.hardware.transmitter.equipment_id, scene_data.device_ids.ar));
  }
  if (scene_data.eos_enabled) {
    platform.Attach(std::make_unique<ca::EosSensorComponent>(
        electro_optical_sensor::session::EosSession::Create(configs.eos),
        scene_data.device_ids.eos));
  }
  if (scene_data.sbirs_enabled) {
    const std::chrono::steady_clock::time_point sbirs_create_begin =
        std::chrono::steady_clock::now();
    sbirs_sensor::session::SbirsSession sbirs_session =
        sbirs_sensor::session::SbirsSession::Create(configs.sbirs);
    app::LogAcceptanceMs(0, 0.0, "初始化时间性能测试", "OPIR",
                          app::SteadyElapsedMs(sbirs_create_begin));
    platform.Attach(std::make_unique<ca::SbirsSensorComponent>(
        std::move(sbirs_session), scene_data.device_ids.sbirs));
  }
  if (scene_data.sar_enabled) {
    platform.Attach(std::make_unique<ca::SarSensorComponent>(
        sar::session::SarSession::Create(configs.sar)));
  }

  // 融合配置来自场景 session_config.fusion（空间门限/方位相干门限/特征门/窗口/失跟周期/源权重；
  // 基线场景放宽方位相干门限到 8°：ESR 假设方位含平滑滞差、EOS 探测含扫描
  // 中心残差，同物理目标的跨源方位差实测可达 4-6°——业务层调参，非库内标准）。
  // 逐航迹滤波库内默认开启（AR 位置通道有运动学估计；SBIRS 角度-only 无量测
  // 原点时仍走关联通道）。
  platform.Attach(std::make_unique<ca::FusionComponent>(
      std::make_unique<fusion::FusionEngine>(configs.fusion)));

  // 目标推演组件（P3 链路）：读融合运动学估计，输出轨迹/发射点/落点/类型概率
  // （含误差预算）。挂载序在 Fusion 之后、Threat 之前。
  platform.Attach(std::make_unique<ca::InferenceComponent>());

  // 威胁评估配置来自场景 session_config.threat（缺省 = 库内默认权重/断点/阈值）。
  // 挂载序在 Fusion 之后：威胁组件每周期读融合输出与 AR 调试视图组装输入。
  platform.Attach(std::make_unique<ca::ThreatComponent>(configs.threat));

  // 验收行（多个模型并行加载）：初始化结束时显示动态库（模块）加载完毕——
  // 按实际挂载的模块列清单，写入 integration_events.log。
  {
    std::vector<const char*> loaded_models;
    if (scene_data.rir_enabled) {
      loaded_models.push_back("RIR");
    }
    if (scene_data.esr_enabled) {
      loaded_models.push_back("ESR");
    }
    if (scene_data.ecm_enabled && scene_data.esr_enabled) {
      loaded_models.push_back("ECM");
    }
    if (scene_data.ar_enabled) {
      loaded_models.push_back("AR");
    }
    if (scene_data.eos_enabled) {
      loaded_models.push_back("EOS");
    }
    if (scene_data.sbirs_enabled) {
      loaded_models.push_back("OPIR");
    }
    if (scene_data.sar_enabled) {
      loaded_models.push_back("SAR");
    }
    loaded_models.push_back("Fusion");
    loaded_models.push_back("Inference");
    loaded_models.push_back("Threat");
    std::string model_names;
    for (const char* name : loaded_models) {
      if (!model_names.empty()) {
        model_names += ",";
      }
      model_names += name;
    }
    // 验收判定标准 第53项：多个模型并行加载——只写加载状态（模块清单）；
    // 装配墙钟与第 50–52 项计时口径重复，不写。
    app::LogAcceptanceText(
        0, 0.0, "多个模型并行加载性能测试",
        CA_FMT_FORMAT("动态库加载完毕 模块数={} 模块=[{}]",
                      loaded_models.size(), model_names));
  }

  // 事件接线：决策监听器（订阅融合信号，门限来自场景）+ 指令路由器（订阅
  // 指令信号 → 键解析 → 传感器运行期补丁：指定/锁定经事件驱动，不再是任务
  // 开始配置）+ 周期落盘输出（统一契约 v2：多机平台轨迹/目标真值/航路/巡逻
  // 区域 CSV；集成端日志已由 InitIntegrationLog 装配，组件在 Step 内直写
  // 视图与事件）。
  app::DecisionListener decision(world, scene_data.high_threat_confidence);
  app::CommandRouter command_router(
      world, scene_data.ar_enabled ? platform.Find<ca::ArSensorComponent>() : nullptr,
      rir_site != nullptr ? rir_site->Find<ca::RirSensorComponent>() : nullptr,
      scene_data.targets);
  app::AppOutputs outputs(output_dir);
  ca::FusionComponent* fusion_component = platform.Find<ca::FusionComponent>();

  // 多机编队：主平台（platform 块，挂传感器/融合）+ 从机（platforms[] 数组，
  // 纯飞行）。每架飞行器各自的航路/区域任务 = "不同指令"；aircraft_id =
  // 1（主） + 2..N（按数组序）。FlightComponent 自包含（每实例一个
  // FlightManager），多机无结构改动。
  std::vector<ca::Entity*> wingmen;
  for (const auto& sp : scene_data.platforms) {
    ca::Entity& wing = world.CreateEntity(sp.name);
    wing.Attach(std::make_unique<ca::FlightComponent>(
        sp.origin, sp.initial_heading_deg, sp.cruise_speed_mps, sp.cruise_altitude_m,
        sp.waypoints, /*loop_route=*/sp.coverage.planned));
    wingmen.push_back(&wing);
  }
  const std::uint32_t aircraft_count =
      1U + static_cast<std::uint32_t>(scene_data.platforms.size());

  // 装配后写一次航路与巡逻区域（航路来自场景 waypoints/coverage 规划，
  // FlightComponent 构造时已就绪；zones 仅巡逻场景有区域）。
  outputs.RecordRoute(1U, scene_data.waypoints);
  if (scene_data.coverage.planned) {
    outputs.RecordZones("patrol_area", scene_data.coverage.area);
  }
  for (std::size_t i = 0U; i < scene_data.platforms.size(); ++i) {
    const auto& sp = scene_data.platforms[i];
    outputs.RecordRoute(static_cast<std::uint32_t>(i + 2U), sp.waypoints);
    if (sp.coverage.planned) {
      outputs.RecordZones(sp.name, sp.coverage.area);
    }
  }

  // RIR 站点几何落盘（一次；仅 RIR 场景）：站点 LLA + 扫描体积 + 最大作用距离，
  // 供 rir_scan_viewer 画扫描扇区。逐周期目标状态在循环内落 rir_targets.csv。
  ca::RirSensorComponent* rir_viz_src =
      rir_site != nullptr ? rir_site->Find<ca::RirSensorComponent>() : nullptr;
  if (rir_viz_src != nullptr) {
    outputs.RecordRirSite(scene_data.rir_site_origin, configs.rir);
  }

  std::vector<app::TargetEcefState> target_states =
      app::MakeTargetStates(scene_data.targets, platform_origin);
  // start_cycle>1：主循环从绝对周期 N 起算，此前真值按脚本快进到 N−1（弹道
  // 用 t=(cycle·dt) 解析求值，不逐步累积）。
  for (std::uint32_t warmup = 1U; warmup < scene_data.start_cycle; ++warmup) {
    app::AdvanceTargetStates(target_states, warmup, scene_data.dt_sec, platform_origin);
  }

  std::size_t max_fused_targets = 0U;
  // 天基平台（卫星）位置：凝视模式，固定于目标群质心正上方 + 场景高度
  // （ECEF z 轴），目标恒位于星下点附近（SBIRS az/el 为 ECI 极坐标——
  // 输入仍为 ECEF，库内按 GMST 旋转到 ECI，全向扫描 span 360° + 下视
  // el −90° 覆盖，GMST 平移不影响探测；消费方每周期注入世界模型）。
  // 无目标时保持上一周期位置（初始零向量 = 场景占位）。
  const std::uint32_t end_cycle = scene_data.start_cycle + num_cycles - 1U;
  for (std::uint32_t cycle = scene_data.start_cycle; cycle <= end_cycle; ++cycle) {
    app::BeginViewLogCycle(cycle);
    // 消费方每周期注入共享场景状态（周期号/时间/四通道世界真值）。
    scene.cycle = cycle;
    scene.t_sec = static_cast<double>(cycle) * scene_data.dt_sec;
    scene.world_targets = target_states;
    scene.emitters = app::MakeEmitterTruths(target_states, scene_data.esr, scene.t_sec);
    scene.sbirs_targets = app::MakeSbirsTargetInputs(target_states);
    scene.sbirs_utc_julian_day = scene_data.sbirs_utc_julian_day;  // SBIRS ECI 输出参考系（UTC 儒略日）
    scene.sar_point_targets = app::MakeSarPointTargets(target_states);
    // 探测池黑板周期边界：本周期各源组件 Step 内写入、融合组件步进时聚合读。
    scene.detection_pool.clear();
    scene.sbirs_ground_station_inbox.clear();
    if (rir_site != nullptr) {
      // RIR 场景目标：世界 ECEF → 站点局部 ENU（含识别特征真值铺样）。
      scene.rir_targets = app::MakeRirSceneTargets(target_states, scene_data.rir_site_origin);
    }
    BeginRfWorldCycle(&scene, scene_data.dt_sec);
    if (!target_states.empty()) {
      double centroid_x = 0.0;
      double centroid_y = 0.0;
      double centroid_z = 0.0;
      for (const auto& target : target_states) {
        centroid_x += target.position.x_m;
        centroid_y += target.position.y_m;
        centroid_z += target.position.z_m;
      }
      const double count = static_cast<double>(target_states.size());
      scene.sbirs_satellite_position_ecef_m.x = centroid_x / count;
      scene.sbirs_satellite_position_ecef_m.y = centroid_y / count;
      scene.sbirs_satellite_position_ecef_m.z =
          centroid_z / count + scene_data.sbirs_satellite_altitude_m;
    }

    // 运行期指令脚本派发（world.Step 前 = 当周期生效）：场景 commands[] 按
    // 周期下发外部指令——与决策侧 DecisionListener 走同一信号入口，经
    // CommandRouter 翻译键后下发传感器运行期补丁（指定/锁定的事件驱动路径）。
    for (const auto& scripted : scene_data.commands) {
      if (scripted.start_cycle != cycle) {
        continue;
      }
      ca::CommandIssuedEvent command;
      command.cycle = cycle;
      command.kind = scripted.kind;
      command.target_key = scripted.target_id;  // 脚本指令直接用外部目标 ID
      command.duration_cycles = scripted.duration_cycles;
      command.command = ScriptedCommandName(scripted.kind);
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string scripted_command_event_log =
          std::string("指令=") +
          (command.command.c_str()) +
          " 目标=" +
          std::to_string(static_cast<unsigned long long>(command.target_key)) +
          " 窗口=" +
          std::to_string(command.duration_cycles);
      CA_LOG_EVENT(world, "command_issued", "指令={} 目标={} 窗口={}",
                   command.command.c_str(),
                   static_cast<unsigned long long>(command.target_key),
                   command.duration_cycles);
      world.signals().on_command_issued(command);
    }

    if (fusion_component != nullptr) {
      fusion_component->BeginCycle(world, cycle);
    }
    world.Step(scene_data.dt_sec);  // 步进序：Flight → ESR → [ECM] → AR → EOS → … → Fusion

    // 平台轨迹/调试视图落盘（多机：主平台 + 每架从机一行，aircraft_id 区分；
    // 目标真值每目标一行，entity_type 透出空中/地面）。主循环不打印周期行，
    // 避免长跑场景（如 RIR 1560 周期）刷屏终端；结束摘要见下方 Summary。
    const auto* flight = platform.Find<ca::FlightComponent>();
    const auto* fusion = fusion_component;
    max_fused_targets = std::max(max_fused_targets, fusion->targets().size());
    outputs.RecordPlatformRow(cycle, scene.t_sec, 1U, *flight);
    for (std::size_t i = 0U; i < wingmen.size(); ++i) {
      const auto* wing_flight = wingmen[i]->Find<ca::FlightComponent>();
      outputs.RecordPlatformRow(cycle, scene.t_sec,
                                static_cast<std::uint32_t>(i + 2U), *wing_flight);
    }
    for (const auto& target : target_states) {
      outputs.RecordTruthRow(cycle, scene.t_sec, target);
    }
    // RIR 逐周期逐目标状态落盘（仅 RIR 场景）：取组件本周期调试视图快照，
    // 逐目标视线方位/俯仰、斜距、航迹状态、识别结论 → rir_targets.csv。
    if (rir_viz_src != nullptr) {
      outputs.RecordRirCycle(cycle, scene.t_sec, scene_data.rir_site_origin,
                             rir_viz_src->LastDebugView(),
                             rir_viz_src->LastMaxDetectedSlantRangeM());
    }
    // 调试视图落盘由各传感器组件在 Step 内直写（取视图 → 人读摘要行 → 集成
    // 端日志；见 logger/logger.h 的 CA_LOG_VIEW），主程序不再收拢。

    // 消费方世界模型推进（在 Step 之后；周期号用于应用变速机动表）。
    app::AdvanceTargetStates(target_states, cycle, scene_data.dt_sec, platform_origin);
  }

  // 验收判定标准 第54项：场景数/总仿真周期——库内会话无法在首周期知道总周期数，
  // 由示例层结束时回写真实值。
  app::LogAcceptanceText(
      0U, 0.0, "可支持连续运行次数性能测试",
      CA_FMT_FORMAT("场景数={} 总仿真周期={}", 1U, num_cycles));

  app::FlushIntegrationLog();
  outputs.Flush();
  // RIR 站点组件（未启用场景为 nullptr；摘要与冒烟按挂载与否条件化）。
  const ca::RirSensorComponent* rir_sensor =
      rir_site != nullptr ? rir_site->Find<ca::RirSensorComponent>() : nullptr;
  std::cout << "\n=== Component Attachment Summary ===\n"
            << "scene=" << scene_data.name
            << " cycles=" << num_cycles
            << (scene_data.start_cycle > 1U
                    ? " start_cycle=" + std::to_string(scene_data.start_cycle)
                    : "")
            << " view_every=" << view_every
            << " aircraft=" << aircraft_count
            << " entities=" << world.entity_count()
            << " components=" << platform.component_count()
            << " patrol=" << (scene_data.coverage.planned ? "planned" : "off")
            << (scene_data.coverage.planned ? " (loop, waypoints="
                                               + std::to_string(scene_data.waypoints.size()) + ")"
                                            : "")
            << " events=" << app::EventCount()
            << " sbirs_events=" << app::SbirsEventCount()
            << " sar_products=" << app::SarProductEventCount()
            << " command_issued=" << (decision.issued() ? "true" : "false")
            << " command_executed=" << command_router.executed_count()
            << " command_dropped=" << command_router.dropped_count()
            << " ar_views=" << app::ArViewCount()
            << " eos_views=" << app::EosViewCount()
            << " sbirs_views=" << app::SbirsViewCount()
            << " sar_views=" << app::SarViewCount()
            << " threat_views=" << app::ThreatViewCount()
            << (rir_sensor != nullptr
                    ? " rir_views=" + std::to_string(app::RirViewCount()) +
                          " rir_confirmed_cycles=" +
                          std::to_string(rir_sensor->confirmed_recognition_outputs())
                    : "")
            << "\n"
            << "log output -> " << output_dir
            << " (integration_events.log / integration_views.log / "
               "*_acceptance.log / rir_antenna_pattern.csv / rir_scan_pattern.csv / "
               "platform_track.csv / target_truth.csv / route_plan.csv / zones.csv"
            << (rir_viz_src != nullptr ? " / rir_site.csv / rir_targets.csv" : "") << ")\n";

  // 外置查询演示：按实体名/类型查找平台实体，读取各传感器开关机与当前扫描
  // 方位（查询逻辑 = 组件 const getter；外部系统选定实体后按名/ID 拉取
  // 最新快照即可）。AR/SAR 无扫描方位语义，不提供角度字段。
  const auto* ar = platform.Find<ca::ArSensorComponent>();
  const auto* esr = platform.Find<ca::EsrSensorComponent>();
  const auto* eos = platform.Find<ca::EosSensorComponent>();
  const auto* sbirs = platform.Find<ca::SbirsSensorComponent>();
  const auto* sar = platform.Find<ca::SarSensorComponent>();
  std::cout << "\n=== Platform Sensor States (entity query) ===\n";
  if (ar != nullptr) {
    std::cout << "  ar    powered=" << (ar->powered_on() ? "on" : "off") << "\n";
  }
  if (esr != nullptr) {
    std::cout << "  esr   powered=" << (esr->powered_on() ? "on" : "off")
              << " scan_az=" << esr->scan_azimuth_deg() << " deg\n";
  }
  if (eos != nullptr) {
    std::cout << "  eos   powered=" << (eos->powered_on() ? "on" : "off")
              << " scan_az=" << eos->scan_azimuth_deg() << " deg\n";
  }
  if (sbirs != nullptr) {
    std::cout << "  sbirs powered=" << (sbirs->powered_on() ? "on" : "off")
              << " scan_az=" << sbirs->scan_azimuth_deg() << " deg\n";
  }
  if (sar != nullptr) {
    std::cout << "  sar   powered=" << (sar->powered_on() ? "on" : "off") << "\n";
  }
  if (rir_sensor != nullptr) {
    std::cout << "  rir   powered=" << (rir_sensor->powered_on() ? "on" : "off")
              << " confirmed_cycles=" << rir_sensor->confirmed_recognition_outputs() << "\n";
  }

  // 冒烟：视图按摘要间隔求余后的拍数断言（默认每周期一行；--view-every N
  // 时为 cycles/N）。事件仍按场景 smoke 下限。
  const auto& smoke = scene_data.smoke;
  const std::uint32_t view_ticks = app::ViewLogExpectedTicks(num_cycles, view_every);
  const bool views_ok =
      (ar == nullptr || app::ArViewCount() >= view_ticks) &&
      (eos == nullptr || app::EosViewCount() >= view_ticks) &&
      (sbirs == nullptr || app::SbirsViewCount() >= view_ticks) &&
      (sar == nullptr || app::SarViewCount() == view_ticks) &&
      app::ThreatViewCount() >= view_ticks;
  const bool powered_ok =
      (ar == nullptr || ar->powered_on()) && (esr == nullptr || esr->powered_on()) &&
      (eos == nullptr || eos->powered_on()) && (sbirs == nullptr || sbirs->powered_on()) &&
      (sar == nullptr || sar->powered_on()) &&
      (rir_sensor == nullptr || rir_sensor->powered_on());
  if (app::EventCount() < smoke.min_key_events ||
      (sbirs != nullptr && app::SbirsEventCount() < smoke.min_sbirs_events) ||
      (sar != nullptr && app::SarProductEventCount() < smoke.min_sar_products) ||
      max_fused_targets < smoke.min_fused_targets ||
      (rir_sensor != nullptr &&
       rir_sensor->confirmed_recognition_outputs() < smoke.min_rir_recognition_outputs) ||
      outputs.platform_rows() != num_cycles * aircraft_count || !views_ok || !powered_ok) {
    std::cerr << "SMOKE FAILED: events=" << app::EventCount()
              << " (>=" << smoke.min_key_events << " required), sbirs_events="
              << app::SbirsEventCount() << " (>=" << smoke.min_sbirs_events
              << " required), sar_products=" << app::SarProductEventCount()
              << " (>=" << smoke.min_sar_products << " required), max_fused="
              << max_fused_targets << " (>=" << smoke.min_fused_targets
              << " required), rir_confirmed_cycles="
              << (rir_sensor != nullptr
                      ? std::to_string(rir_sensor->confirmed_recognition_outputs())
                      : "n/a")
              << " (>=" << smoke.min_rir_recognition_outputs << " required), platform_rows="
              << outputs.platform_rows() << " (== " << num_cycles << " × " << aircraft_count
              << " 机 required), "
              << "ar_views=" << app::ArViewCount()
              << " (>= " << view_ticks << " required), eos_views="
              << app::EosViewCount() << " (>= " << view_ticks << " required), sbirs_views="
              << app::SbirsViewCount() << " (>= " << view_ticks << " required), sar_views="
              << app::SarViewCount() << " (== " << view_ticks << " required), threat_views="
              << app::ThreatViewCount() << " (>= " << view_ticks << " required), sensor_powered="
              << (powered_ok ? "true" : "false") << " (mounted only)\n";
    return 1;
  }
  return 0;
}

int RunSceneWithArgs(const char* scene_json_path, int argc, char* argv[]) {
  // 场景可执行的参数面：仅调试覆盖（--cycles/--view-every/--output-dir）；
  // 场景由编译宏 ONEQ_SCENE_JSON 钉死，不提供 --scene（任意场景走通用
  // runner component_attachment_demo）。
  RunOptions options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--cycles" && i + 1 < argc) {
      options.cycles_override = std::atoi(argv[++i]);
    } else if (arg == "--view-every" && i + 1 < argc) {
      options.view_every_override = std::atoi(argv[++i]);
    } else if (arg == "--output-dir" && i + 1 < argc) {
      options.output_dir = argv[++i];
    } else {
      std::cerr << "Usage: " << argv[0]
                << " [--cycles <n>] [--view-every <n>] [--output-dir <dir>]\n"
                << "  本可执行运行钉死场景 " << scene_json_path
                << "；任意场景用通用 runner component_attachment_demo --scene <path>\n";
      return 1;
    }
  }
  return RunScene(scene_json_path, options);
}

}  // namespace app
}  // namespace component_attachment
