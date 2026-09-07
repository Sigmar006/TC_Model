/**
 * @file scenes/scene_data.h
 * @brief 自定义实体-组件示例：场景描述（scenes 目录的 JSON 场景文件 → SceneData）。
 *
 * 场景描述文件是"消费方世界模型 + 业务调参"的数据化载体：目标脚本、
 * 平台飞行脚本（原点/航向/巡航/航点或区域巡逻 coverage 块）、ESR 辐射源
 * 波形、天基平台位置、EOS 扫描与 SAR 任务几何/链路覆写、融合配置与冒烟
 * 断言下限。场景文件加载（LoadSceneData）遵循 examples/common/config_loaders
 * 惯例：缺省字段静默默认（成员初始化值 = 默认值，与历史代码覆写值一致）、
 * JSON 语法错误与缺必需块/几何字段经 error 字符串报出。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_SCENE_DATA_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_SCENE_DATA_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/airborne_radar/airborne_radar.hpp"
#include "1q/coordinate/types.h"
#include "1q/electro_optical_sensor/electro_optical_sensor.hpp"
#include "1q/electronic_countermeasure/EcmTypes.h"
#include "1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp"
#include "1q/fusion/FusionConfig.h"
#include "1q/fusion/SensorAdapters.h"
#include "1q/remote_identification_radar/remote_identification_radar.hpp"
#include "1q/sar/sar.hpp"
#include "1q/sbirs_sensor/sbirs_sensor.hpp"
#include "1q/threat_assessment/ThreatEvaluatorConfig.h"
#include "1q/navigation/CoverageArea.h"
#include "1q/navigation/CoveragePlanConfig.h"
#include "1q/navigation/RoutePoint.h"
#include "core/events.h"

namespace component_attachment {
namespace app {

/// 目标变速机动（场景文件 targets[].maneuvers[] 条目）：分段匀速语义——
/// 目标在 start_cycle 及之后使用新速度（绝对速度，非增量），直到下一条机动
/// 生效。供"目标大机动/逃逸"类场景验证 AR 失跟/重捕、SAR 几何破坏等。
struct TargetManeuver {
  std::uint32_t start_cycle{0U}; /**< 生效周期（含；相对周期 1 起） */
  double v_east_mps{0.0};        /**< 该周期起的新东向速度（m/s） */
  double v_north_mps{0.0};       /**< 该周期起的新北向速度（m/s） */
};

/// 距离向散射中心脚本（targets[].rir_scatterers[] 条目）：一维距离像特征
/// 真值的最小描述（位置 + 强度），极化/相位/起伏不配（缺省非相干叠加）。
struct RirScattererScript {
  double offset_m{0.0};  /**< 相对目标参考点的距离向位置（m） */
  double rcs_dbsm{0.0};  /**< 该散射中心 RCS（dBsm） */
};

/// 区域巡逻任务（场景文件可选 coverage 块）：覆盖区域 + 规划参数。
/// 加载时经 navigation::AreaCoveragePlanner 生成巡逻航路（填入 waypoints），
/// 平台按航路循环巡逻（见 FlightComponent::loop_route）。与显式
/// platform.waypoints 互斥（同时出现报错，避免航路来源歧义）。
struct CoverageTask {
  navigation::CoverageArea area{};                 /**< 覆盖区域（多边形 / 圆形） */
  navigation::CoveragePlanConfig config{};         /**< 覆盖规划参数 */
  bool planned{false};                             /**< 已由规划器生成航路（= 巡逻场景） */
};

/// 编队切分任务（顶层可选 mission_area 块）：主机收到的单个覆盖区域 + 规划
/// 参数。加载时经 area_division::DivideArea 自动切分为每架飞机（主机 +
/// platforms[] 从机）的子区域，再逐机经 AreaCoveragePlanner 生成巡逻航路
/// （分工覆盖：多边形 = 沿扫描航向等宽条带，圆形 = 同心环）。与各平台
/// coverage/waypoints 块互斥（区域来源歧义报错）；编队数 = 1 + platforms.size()。
struct FormationMissionArea {
  navigation::CoverageArea area{};                 /**< 覆盖区域（多边形 / 圆形） */
  navigation::CoveragePlanConfig config{};         /**< 覆盖规划参数 */
};

/// 运行期指令脚本（场景文件顶层可选 commands[] 条目）：外部指挥系统在指定
/// 周期下发的指令——指定/锁定目标不再走任务开始配置，而是经指令事件
/// （CommandIssuedEvent）→ CommandRouter → 组件运行期补丁的统一路径执行。
/// 主循环在每周期 world.Step 前按 start_cycle 派发（当周期生效）。
struct ScriptedCommand {
  std::uint32_t start_cycle{0U};   /**< 下发周期（含；相对周期 1 起，须 > 0） */
  CommandKind kind{CommandKind::kDesignateTarget}; /**< 指令类型 */
  std::uint64_t target_id{0U};     /**< 目标外部 ID（kClearDesignation 时忽略） */
  std::uint32_t duration_cycles{0U}; /**< 限时窗口（周期；0 = 无限期） */
};

/// 目标脚本（场景文件 targets[] 条目）：四通道共享同一物理目标
/// （方位/距离/高度 → ECEF 位置；东/北速度 → ECEF 速度；外观/RCS/辐射源
/// 中心频率随真值流转到各通道转换函数，转换函数不再按数组下标回查脚本）。
struct ScriptedTarget {
  std::uint32_t id{0U};                     /**< 外部目标标识（AR/ESR/EOS/SBIRS/SAR 共用） */
  std::string type{"air"};                  /**< 实体类型（"air" 空中 / "ground" 地面 /
                                                 "ballistic" 弹道；ground = 静止近地运动学点，
                                                 ballistic = 二体椭圆弹道弧线，见
                                                 scenes/ballistic_trajectory.h） */
  double azimuth_deg{0.0};                  /**< 真方位（北偏东，deg） */
  double range_m{0.0};                      /**< 斜距（m） */
  double altitude_m{0.0};                   /**< 目标高度（m；与平台巡航高度解耦） */
  double v_east_mps{0.0};                   /**< 局部东向速度（m/s） */
  double v_north_mps{0.0};                  /**< 局部北向速度（m/s） */
  double temperature_k{0.0};                /**< 等效温度（EOS 外观） */
  double rcs{0.0};                          /**< 雷达截面积（m²；SAR dBsm 换算源） */
  double projected_area_m2{0.0};            /**< 等效投影面积（EOS 外观，m²） */
  double radiant_intensity_w_per_sr{0.0};   /**< 目标辐射强度（SBIRS 外观，W/sr；已折算温度/发射率/投影面积） */
  double emitter_center_frequency_hz{0.0};  /**< ESR 辐射源中心频率（Hz；≤0 = 该目标不配辐射源） */
  std::vector<TargetManeuver> maneuvers{};  /**< 变速机动表（可选；start_cycle 严格递增） */

  // 弹道目标脚本（type == "ballistic" 专用；与 ENU 几何字段 azimuth_deg/
  // range_m/altitude_m/v_east_mps/v_north_mps/maneuvers 互斥，加载器校验）：
  // 起止 LLA + 顶高 + 顶高时刻三/四约束闭式解二体椭圆（客户句柄号作目标 id）。
  bool is_ballistic{false};                 /**< 是否弹道类型（type == "ballistic"） */
  oneq::coordinate::LlaPositionDegM start_lla{}; /**< 发射点 LLA（deg, deg, m；发射场坪） */
  oneq::coordinate::LlaPositionDegM end_lla{};   /**< 落点 LLA（deg, deg, m） */
  double max_alt_m{0.0};                    /**< 弹道顶高（m；数据 max_alt） */
  double max_alt_time_s{0.0};               /**< 数据顶高时刻（s；场景时间轴锚点） */

  // RIR 识别特征真值（可选块，has_rir_features=false 时该目标对 RIR 只供标量
  // RCS、无识别结论——四维特征中仅探测链可用）：标量值由组件铺均匀视角网格
  // /散射器（仿集成测试配方），识别库模板匹配源。极化须显式给值才铺样
  // （0 dBsm 是合法物理值，不能用零值当"未提供"判断）。
  bool has_rir_features{false};             /**< 是否携带 RIR 特征真值 */
  double rir_rcs_dbsm{0.0};                 /**< 视角 RCS 网格值（dBsm） */
  std::vector<remote_identification_radar::session::RirPolSMatrixSample>
      rir_pol_dictionary{}; /**< 极化散射字典（全量，开机一次性载入；架构 B） */
  double rir_pol_window_deg{5.0};           /**< 极化窗口跨度（deg，±窗口） */
  std::string rir_truth_model{};            /**< 真值型号名（人读 + 识别准确率统计） */
  std::vector<RirScattererScript> rir_scatterers{}; /**< 距离向散射中心（一维像真值） */
};

/// 平台描述（platform 块 / platforms[] 数组条目共用）：飞行器初始状态与
/// 航路/区域任务。主平台（platform 块）额外挂载传感器与融合；platforms[]
/// 数组条目为从机（纯飞行，各自航路/区域 = "不同指令"）。
struct ScenePlatform {
  std::string name{"platform"};             /**< 平台名（实体名 / aircraft_id 顺序） */
  oneq::coordinate::LlaPositionDegM origin{}; /**< 机场位置（度制 LLA；必填） */
  double initial_heading_deg{90.0};         /**< 起飞航向（deg，北偏东） */
  double cruise_altitude_m{400.0};          /**< 巡航高度（m） */
  double cruise_speed_mps{50.0};            /**< 巡航速度参考（m/s） */
  std::vector<navigation::RoutePoint> waypoints{}; /**< 巡航/巡逻航路（缺省空 = 直飞） */
  CoverageTask coverage{};                  /**< 区域巡逻任务（可选；与 waypoints 互斥） */
};

/// ESR 辐射源共享波形参数（场景文件 esr 块；中心频率在目标条目内）。
/// 默认值 = 历史基线：200 脉冲 @ 10 GHz 级，单周期积分脉冲数足够，
/// 统计检测概率趋近 1（pfa=1e-6 下多脉冲积分过检）。
struct EsrEmitterParams {
  double peak_gain_dbi{30.0};        /**< 天线峰值增益（dBi） */
  double bandwidth_hz{2.0e6};        /**< 带宽（Hz） */
  double peak_power_w{5.0e7};        /**< 峰值功率（W） */
  double pulse_width_s{1.0e-6};      /**< 脉宽（s） */
  double pri_s{1.0e-3};              /**< 脉冲重复间隔（s） */
  std::uint32_t pulse_count{200U};   /**< 单周期积分脉冲数 */
  std::uint32_t timing_seed{42U};    /**< 波形时序种子（确定性复现） */
};

/// 场景描述：scenes/*.json 的全部业务数据。缺省字段 = 成员初始化值
/// （数值与 demo_config 历史常量一致：kNumCycles/kDtSec 移入本结构后，
/// 周期数/步长按场景文件控制）；平台与目标几何字段为必填（loader 校验）。
struct SceneData {
  std::string name{"unnamed_scene"};        /**< 场景名（摘要打印） */
  std::string log_dir{};                    /**< 本场景日志输出目录（相对
                                                  examples/log/ 的路径；四参加载
                                                  必填——场景自带日志落点，禁止
                                                  漂到运行目录或临时目录） */
  std::uint32_t cycles{400U};               /**< 周期数（缺省 = demo_config kNumCycles） */
  std::uint32_t start_cycle{1U};            /**< 首周期仿真序号（缺省 1；>1 时跳过前段空窗，
                                                  弹道/指令/落盘仍用绝对周期号） */
  std::uint32_t view_log_every_cycles{1U};  /**< 视图摘要间隔：周期 % 此值 == 0 才写；1 = 每周期 */
  double dt_sec{1.0};                       /**< 步长（s，缺省 = demo_config kDtSec） */

  oneq::coordinate::LlaPositionDegM platform_origin{}; /**< 机场位置（度制 LLA；必填） */
  double initial_heading_deg{90.0};         /**< 起飞航向（deg，北偏东） */
  double cruise_altitude_m{400.0};          /**< 巡航高度（m；c172x 低空巡航量级，原 kCruiseAltitudeM） */
  double cruise_speed_mps{50.0};            /**< 巡航速度参考（m/s，原 kCruiseSpeedMps） */
  std::vector<navigation::RoutePoint> waypoints{}; /**< 巡航/巡逻航路（缺省空 = 直飞；
                                              coverage 块存在时为规划器输出的巡逻航路） */
  CoverageTask coverage{};                  /**< 区域巡逻任务（可选；planned=true 时
                                              waypoints 为规划结果，平台循环巡逻） */

  /// 从机（platforms[] 数组，可选）：各自航路/区域任务，纯飞行（不挂传感器）。
  /// 主平台（platform 块） + 从机共同构成多机编队；aircraft_id = 1（主）+
  /// 2..N（按数组序）。
  std::vector<ScenePlatform> platforms{};

  std::vector<ScriptedTarget> targets{};    /**< 目标脚本（四通道共享；几何字段必填） */
  EsrEmitterParams esr{};                   /**< ESR 辐射源波形参数 */

  double sbirs_satellite_altitude_m{500000.0}; /**< 天基平台高度（m；凝视目标群质心正上方） */
  // SBIRS 输出参考系为 ECI（2026-08 正式变更），本字段提供 UTC 儒略日（JD_UTC，
  // 缺省 = 2024-01-01 00:00 UTC）。SBIRS 全向扫描（span 360°）+ 下视覆盖，
  // GMST 引起的 az 平移不影响探测；示例日志中的方位角随之显示为 ECI 参考。
  double sbirs_utc_julian_day{2460310.5}; /**< 天基通道 UTC 儒略日（JD_UTC，必填给库） */

  // 机载传感器默认挂载。场景 sensors.{ar,esr,eos,sbirs,sar}=false 则不挂，
  // 不写该通道视图/排除原因事件。RIR/ECM 仍用各自 enabled。
  bool ar_enabled{true};
  bool esr_enabled{true};
  bool eos_enabled{true};
  bool sbirs_enabled{true};
  bool sar_enabled{true};

  // RIR 地基识别雷达站点块（可选，enabled=false 时整机不挂载——现有场景行为
  // 不变）：站点为固定 LLA（雷达局部 ENU 原点 + 特征量测 sensor_origin）；
  // 指定识别任务经运行期指令（commands[] 或威胁闭环）下发，不再随场景配置。
  bool rir_enabled{false};                          /**< 是否挂载 RIR 地基站点组件 */
  oneq::coordinate::LlaPositionDegM rir_site_origin{
      30.0, 120.0, 0.0};                            /**< 站点位置（LLA；enabled 时必填语义） */

  // 运行期指令脚本（顶层可选 commands[]，缺省空）：按周期派发的指定/锁定/
  // 清除指令——"外部控制"的确定性载体（见 ScriptedCommand）。
  std::vector<ScriptedCommand> commands{};

  // ECM 电子对抗块（可选，enabled=true 时挂载 EcmSensorComponent；须 ESR→ECM→AR 挂载序）。
  bool ecm_enabled{false};                          /**< 是否挂载 ECM 组件 */

  double high_threat_confidence{3.0};       /**< 决策门限：融合置信度达到该值视为高置信威胁
                                                 （示例业务策略，原 demo_config kHighThreatConfidence） */

  /// 统一传感器设备实体标识（可选 top-level device_ids 块，缺省各自 fusion::k*SourceId）。
  struct SceneDeviceIds {
    std::uint32_t ar{fusion::kArSourceId};
    std::uint32_t esr{fusion::kEsrSourceId};
    std::uint32_t eos{fusion::kEosSourceId};
    std::uint32_t sbirs{fusion::kSbirsSourceId};
    std::uint32_t rir{fusion::kRirSourceId};
    std::uint32_t ecm{101U};
  };
  SceneDeviceIds device_ids{};

  /// 冒烟断言下限（场景文件 smoke 块；"无目标"等零产出场景显式置 0）。
  struct SmokeExpectations {
    std::uint32_t min_key_events{1U};       /**< 关键事件数下限 */
    std::uint32_t min_sbirs_events{1U};     /**< SBIRS 关键探测事件数下限 */
    std::uint32_t min_sar_products{1U};     /**< SAR 关键产品事件数下限 */
    std::uint32_t min_fused_targets{1U};    /**< 融合目标数（峰值）下限 */
    std::uint32_t min_rir_recognition_outputs{0U}; /**< RIR 识别结论输出周期数下限（未挂载时 0） */
  };
  SmokeExpectations smoke{};
};

/// 场景自持的会话配置集（场景文件 session_config{} 块 → 六域传感器 + ECM +
/// 融合 + 威胁的库 SessionConfig）。挂载即全量：挂载的通道必带对应子块
/// （自 examples/basic_config/<域>.json 模板整份拷贝后按场景改），未挂载
/// 通道禁止携带（loader 校验，"有配置 = 有挂载"一一对应）。
struct SceneSessionConfigs {
  fusion::FusionConfig fusion{};            /**< 融合（恒挂载；字段集见 loader 映射） */
  threat_assessment::ThreatEvaluatorConfig threat{}; /**< 威胁评估（恒挂载） */
  airborne_radar::config::ArSessionConfig ar{};      /**< 机载雷达（sensors.ar） */
  electronic_surveillance_radar::config::EsrSessionConfig esr{}; /**< ESR（sensors.esr） */
  electro_optical_sensor::config::EosSessionConfig eos{};        /**< EOS（sensors.eos） */
  sbirs_sensor::config::SbirsSessionConfig sbirs{};  /**< SBIRS（sensors.sbirs） */
  sar::config::SarSessionConfig sar{};               /**< SAR（sensors.sar） */
  remote_identification_radar::config::RirSessionConfig rir{};   /**< RIR（rir.enabled） */
  electronic_countermeasure::config::EcmSessionConfig ecm{};     /**< ECM（ecm.enabled） */
};

/// 场景文件 → SceneData（仅场景层：几何/真值/挂载开关/指令/冒烟；session_config
/// 块被忽略）。返回 false 并置 error（JSON 语法错误/根节点非对象/缺必需块
/// （platform/targets）/几何字段缺失）；可缺省字段按成员默认值填充。
/// 场景可执行与通用 runner 走四参重载（含 session_config 挂载校验）。
bool LoadSceneData(const char* path, SceneData* scene, std::string* error);

/// 场景文件 → SceneData + SceneSessionConfigs（session_config 挂载即全量：
/// 挂载通道缺子块 / 未挂载通道带子块 → error；RIR 识别库路径由编译宏
/// CA_RIR_DATABASE_PATH 钉定，相对值按 SCENE_CONFIG_DIR 解析）。
bool LoadSceneData(const char* path, SceneData* scene, SceneSessionConfigs* configs,
                   std::string* error);

}  // namespace app
}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_SCENE_DATA_H_
