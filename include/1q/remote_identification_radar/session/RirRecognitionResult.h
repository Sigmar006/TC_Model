/**
 * @file RirRecognitionResult.h
 * @brief 远程目标识别结果与周期摘要类型。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_RECOGNITION_RESULT_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_RECOGNITION_RESULT_H_

#include <cstdint>
#include <string>

#include "1q/api.hpp"

namespace remote_identification_radar {
namespace session {

/**
 * @brief RirRecognitionState 识别状态机。
 */
enum class ONEQ_API RirRecognitionState : std::uint8_t {
  kDisabled = 0,          /**< 识别未启用或数据库未加载。 */
  kAccumulating = 1,      /**< 正在积累特征，尚未达到输出条件。 */
  kCategoryConfirmed = 2, /**< 大类已确认，型号待定。 */
  kModelConfirmed = 3,    /**< 型号已确认。 */
  kUnknown = 4,           /**< 已积累足够观测但无候选满足门限。 */
  kStale = 5              /**< 退出模式或特征缺失超时，结论已过期。 */
};

/**
 * @brief RirRecognitionCategory 识别目标大类。
 */
enum class ONEQ_API RirRecognitionCategory : std::uint8_t {
  kBallistic = 0, /**< 弹道目标。 */
  kNearSpace = 1, /**< 临近空间目标。 */
  kOther = 2,     /**< 其它。 */
  kUnknown = 3,   /**< 未知。 */
  kFighter = 4,   /**< 战斗机。 */
  kBomber = 5,    /**< 轰炸机。 */
  kMissile = 6,   /**< 导弹。 */
  // 中译：**弹道体系穿透辅助目标**（2026-11-07）——识别库已含 DECOY/DEBRIS/BOOSTER 三类模板，
  //   但公共枚举此前只有七类，导致这三类模板命中后只能落成 kUnknown（现场"全部未知"的根因）。
  //   追加三个大类（保留既有编号不变），由 CategoryToPublic 从库的 category_id 映射。
  // 标识：穿透辅助目标大类（2026-11-07）。
  kDecoy = 7,     /**< 诱饵（再入段假目标）。 */
  kDebris = 8,    /**< 弹片/碎片。 */
  kBooster = 9    /**< 剥离的助推火箭。 */
};

/**
 * @brief RirBallisticSubclass 弹道目标细分类型（与 RirRecognitionCategory 平行，
 *        不改七粗类枚举）。
 * @note 判决判据未冻结（2026-09-03 契约：等甲方原件），当前无生产者、恒 kUnknown。
 */
enum class ONEQ_API RirBallisticSubclass : std::uint8_t {
  kUnknown = 0,       /**< 未判决。 */
  kReentryVehicle = 1, /**< 弹头（再入飞行器）。 */
  kHeavyDecoy = 2,    /**< 重诱饵。 */
  kLightDecoy = 3,    /**< 轻诱饵。 */
  kDebris = 4         /**< 碎片。 */
};

/**
 * @brief RirRecognitionFeatureDimension 特征维度位掩码。
 */
enum class ONEQ_API RirRecognitionFeatureDimension : std::uint8_t {
  kRcs = 1 << 0,          /**< RCS（0x01）。 */
  kMotion = 1 << 1,       /**< 运动（0x02）。 */
  kPolarization = 1 << 2, /**< 极化（0x04）。 */
  kRangeProfile = 1 << 3  /**< 距离像（0x08）。 */
};

/**
 * @brief RirRecognitionFeatureScores 四类特征的相似度与质量。
 * @note 所有分量 ∈ [0, 1]；质量 0 表示该维度未参与融合。
 */
struct ONEQ_API RirRecognitionFeatureScores {
  float rcs_similarity{0.0f};           /**< RCS 相似度。 */
  float rcs_quality{0.0f};              /**< RCS 质量。 */
  float motion_similarity{0.0f};        /**< 运动相似度。 */
  float motion_quality{0.0f};           /**< 运动质量。 */
  float polarization_similarity{0.0f};  /**< 极化相似度。 */
  float polarization_quality{0.0f};     /**< 极化质量。 */
  float range_profile_similarity{0.0f}; /**< 距离像相似度。 */
  float range_profile_quality{0.0f};    /**< 距离像质量。 */
};

/**
 * @brief RirRecognitionCandidate 单个识别候选（型号 + 大类 + 得分）。
 * @note 仅用于诊断（"为什么判成这一类"需要看到前几名与各自得分）；
 *       固定长度字符数组，避免在公共结构里引入动态分配。
 */
struct ONEQ_API RirRecognitionCandidate {
  char model_id[32]{};   /**< 型号 ID（超长截断，NUL 结尾）。 */
  char category_id[20]{};/**< 大类 ID（库内字符串，超长截断）。 */
  float score{0.0f};     /**< 先验加权得分（未归一化）。 */
};

/**
 * @brief RirRecognitionResult 单条航迹的识别结论。
 */
struct ONEQ_API RirRecognitionResult {
  RirRecognitionState state{RirRecognitionState::kDisabled};
  RirRecognitionCategory target_category{RirRecognitionCategory::kUnknown};
  RirBallisticSubclass ballistic_subclass{RirBallisticSubclass::kUnknown}; /**< 弹道细分类型（平行字段，判据未冻结恒 kUnknown）。 */
  std::string target_model{};  /**< 最可能型号；未确认时为空字符串。 */
  float confidence{0.0f};      /**< 第一候选归一化综合置信度，[0, 1]。 */
  float best_score{0.0f};      /**< 第一候选得分，[0, 1]。 */
  float runner_up_score{0.0f}; /**< 第二候选得分，[0, 1]。 */
  RirRecognitionFeatureScores feature_scores{};
  std::uint8_t valid_feature_mask{0U};  /**< 本周期参与融合的特征维度位掩码。 */
  std::uint32_t observation_count{0U};  /**< 证据积累量（有效观测数）。 */
  float accumulation_sec{0.0f};         /**< 证据积累时长（s）。 */
  std::string database_version{};       /**< 输出所使用的特征库版本。 */
  std::uint32_t source_cycle_index{0U}; /**< 产生此结论的 cycle_index。 */
  std::uint64_t source_batch_id{0U};    /**< 产生此结论的 batch_id。 */
  // 中译：**前三候选透出**（2026-11-08）——此前只有 best_model/runner_up_score，
  //   看不出"第二名是谁、差多少"，现场"装了诱饵特征却判成弹道/未知"无从定位。
  //   现在把前 3 名的（型号、大类、得分）一并带出，宿主诊断行可直接打印。
  // 标识：识别前三候选透出（2026-11-08）。
  RirRecognitionCandidate top_candidates[3]{}; /**< 前 3 个候选（降序，空位其余字段为 0）。 */
  std::uint8_t top_candidate_count{0U};        /**< 有效候选数，<= 3。 */
};

/**
 * @brief RirDwellBudgetSummary 单周期识别驻留预算摘要（阶段 2-S S3）。
 * @note 识别驻留预算由自持链路按内部确认航迹数申请；实际执行数以检测通过
 *       且成功积累的目标数为准（未识别优先 + 斜距次近，D-A5）。
 */
struct ONEQ_API RirDwellBudgetSummary {
  std::uint32_t scheduled_dwell_count{0U}; /**< 本周期计划驻留条目数（TAS 逐驻留计划）。 */
  std::uint32_t executed_dwell_count{0U};  /**< 本周期实际执行驻留条目数（库内逐驻留执行恒等于计划数）。 */
  float dwell_budget_sec{0.0f};            /**< 本周期驻留时间预算（s）。 */
  float dwell_consumed_sec{0.0f};          /**< 本周期实际消耗驻留时间（s）。 */
};

/**
 * @brief RirRecognitionCycleSummary 单周期识别效能摘要（嵌入 RirCycleResult）。
 */
struct ONEQ_API RirRecognitionCycleSummary {
  std::uint32_t participating_track_count{0U}; /**< 参与识别的航迹数。 */
  std::uint32_t category_confirmed_count{0U};  /**< 大类确认数。 */
  std::uint32_t model_confirmed_count{0U};     /**< 型号确认数。 */
  std::uint32_t unknown_count{0U};             /**< 未知数。 */
  std::uint32_t disabled_count{0U};            /**< 未启用/无特征数。 */
  float rcs_availability_rate{0.0f};           /**< RCS 特征可用率（质量>0 比例），[0, 1]。 */
  float motion_availability_rate{0.0f};        /**< 运动特征可用率，[0, 1]。 */
  float polarization_availability_rate{0.0f};  /**< 极化特征可用率，[0, 1]。 */
  float range_profile_availability_rate{0.0f}; /**< 距离像特征可用率，[0, 1]。 */
  float mean_confidence{0.0f};                 /**< 已确认型号的平均置信度，[0, 1]。 */
  float mean_first_confirmation_sec{0.0f};     /**< 平均首次确认时间（s）。 */
  bool has_ground_truth{false};                /**< 本周期是否有真值可用于正确率统计。 */
  float category_accuracy{0.0f};               /**< 大类正确率，仅 has_ground_truth 时有效。 */
  float model_accuracy{0.0f};                  /**< 型号正确率，仅 has_ground_truth 时有效。 */

  RirDwellBudgetSummary dwell_budget{}; /**< 本周期驻留预算摘要（自持链路）。 */
};

}  // namespace session
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_RECOGNITION_RESULT_H_
