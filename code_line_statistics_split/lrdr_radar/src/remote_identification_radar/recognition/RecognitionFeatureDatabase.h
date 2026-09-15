/**
 * @file RecognitionFeatureDatabase.h
 * @brief 目标特征数据库（SQLite 加载 + 结构校验，库内部）。
 *
 * 每个数据库文件是完整、只读、自描述的识别基线（SQLite schema v1.1）：meta 承载
 * database_id/version/created_utc/通道约定，units 表声明量纲，display_name 与
 * aspect 适用区间随数据入库（加载校验、当前不参与匹配）。加载为全量原子替换：
 * 新文件通过模式、单位、数值与交叉引用校验后才生效，加载失败时保持原库不变
 * （调用方持有旧实例）。SQLite 是加载期读取器——加载完成后连接关闭，
 * 运行期只读内存结构。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_RECOGNITION_RECOGNITION_FEATURE_DATABASE_H_
#define REMOTE_IDENTIFICATION_RADAR_RECOGNITION_RECOGNITION_FEATURE_DATABASE_H_

#include <string>
#include <vector>

namespace remote_identification_radar {
namespace recognition {

/** @brief 单连续特征模板：均值 + 标准差（截断高斯相似度）。 */
struct RirFeatureTemplate {
  float mean{0.0f};
  float std{1.0f};
};

/** @brief 型号 profile 的 RCS 模板（dBsm 域）。 */
struct RirRcsTemplate {
  float mean_dbsm{0.0f};
  float std_db{1.0f};
  float azimuth_variation_db{0.0f};
  float elevation_variation_db{0.0f};
  float minimum_aspect_coverage_deg{0.0f};
};

/** @brief 型号 profile 的运动模板。 */
struct RirMotionTemplate {
  RirFeatureTemplate speed_mps{};
  RirFeatureTemplate altitude_m{};
  RirFeatureTemplate acceleration_mps2{};
  RirFeatureTemplate turn_radius_log10{};
};

/** @brief 型号 profile 的极化模板（dB 域）。 */
struct RirPolarizationTemplate {
  RirFeatureTemplate energy_difference_db{};
  RirFeatureTemplate relative_difference_db{};
  RirFeatureTemplate energy_sum_db{};
};

/** @brief 型号 profile 的距离像模板。 */
struct RirRangeProfileTemplate {
  RirFeatureTemplate length_m{};
  RirFeatureTemplate peak_count{};
  RirFeatureTemplate peak_energy_concentration{};
  float minimum_bandwidth_hz{0.0f};
};

/** @brief 型号 profile：模板 + 适用条件。 */
struct RirModelProfile {
  std::string profile_id{};
  float min_snr_db{0.0f};
  float max_range_resolution_m{0.0f}; /**< 0 表示不限。 */
  // 适用姿态区间（度）；缺省全范围。仅承载与校验，当前不参与匹配（D10 边界）。
  float aspect_az_min_deg{-180.0f};
  float aspect_az_max_deg{180.0f};
  float aspect_el_min_deg{-90.0f};
  float aspect_el_max_deg{90.0f};
  RirRcsTemplate rcs{};
  RirMotionTemplate motion{};
  RirPolarizationTemplate polarization{};
  RirRangeProfileTemplate range_profile{};
};

/** @brief 可匹配型号：模板集合 + 先验。 */
struct RirModel {
  std::string model_id{};
  std::string category_id{};
  std::string display_name{};
  float prior{1.0f};
  std::vector<RirModelProfile> profiles{};
};

/** @brief 标准目标大类及先验。 */
struct RirCategoryEntry {
  std::string category_id{};
  std::string display_name{};
  float prior{1.0f};
};

/**
 * @brief RirFeatureDatabase 已加载且校验通过的目标特征数据库。
 *
 * Load 成功后才返回非空数据库；失败时 error 含具体路径与字段。
 * 数据库保存的是从观测提取出的特征统计模板，不保存场景侧真值列表。
 */
class RirFeatureDatabase {
 public:
  /**
   * @brief 从 SQLite 文件加载并校验数据库（只读打开，加载后连接关闭）。
   * @param[in] path SQLite 数据库文件路径。
   * @param[out] error 失败原因（含路径与表/字段上下文）；成功时为空。
   * @return 校验通过返回 true 且 IsLoaded()==true；失败返回 false，
   *         返回对象保持未加载状态（不 fallback 到默认库）。
   */
  static bool Load(const std::string& path, RirFeatureDatabase* database,
                   std::string* error);

  bool IsLoaded() const { return loaded_; }
  const std::string& database_id() const { return database_id_; }
  const std::string& version() const { return version_; }
  const std::vector<RirCategoryEntry>& categories() const { return categories_; }
  const std::vector<RirModel>& models() const { return models_; }

 private:
  bool loaded_{false};
  std::string database_id_{};
  std::string version_{};
  std::vector<RirCategoryEntry> categories_{};
  std::vector<RirModel> models_{};
};

}  // namespace recognition
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_RECOGNITION_RECOGNITION_FEATURE_DATABASE_H_
