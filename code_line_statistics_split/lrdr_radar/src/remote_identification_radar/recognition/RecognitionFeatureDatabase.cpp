/**
 * @file RecognitionFeatureDatabase.cpp
 * @brief 目标特征数据库加载与校验实现（SQLite schema v1.1）。
 *
 * SQLite 是加载期读取器：加载时只读打开 → 读 meta/units/表 → 校验 → 关闭连接，
 * 成功后数据库全量驻留内存，运行期不持有连接（Matcher/Tracker 只读消费
 * 内存结构）。失败语义与错误信息风格与 v1.0 一致（含路径与字段上下文）。
 *
 * v1.1 语义分组表：模板组（rcs/motion/polarization/range_profile）各一表，
 * 行存在 = 组存在；组内子模板 std 必填且 > 0，mean 可空（缺省 0.0f）。
 * 自描述元数据：units 表声明 7 个量纲（rcs 必须为 dBsm），meta 承载
 * created_utc/通道约定等。display_name 与 aspect 区间随数据入库（承载不消费）。
 */

#include "remote_identification_radar/recognition/RecognitionFeatureDatabase.h"

#include <cmath>
#include <cstring>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <sqlite3.h>

namespace remote_identification_radar {
namespace recognition {

namespace {

/** @brief 受支持的数据库 schema 版本。 */
constexpr const char* kSupportedSchemaVersion = "1.1";

/** @brief 模板组表名（错误信息与校验标签）。 */
constexpr const char* kRcsTable = "rcs_templates";
constexpr const char* kMotionTable = "motion_templates";
constexpr const char* kPolarizationTable = "polarization_templates";
constexpr const char* kRangeProfileTable = "range_profile_templates";

/** @brief units 表 7 个必填量纲（设计文档 §7.2/7.4 定义）。 */
constexpr const char* kRequiredUnitQuantities[] = {
    "rcs", "speed", "altitude", "acceleration", "turn_radius", "polarization", "range"};

/** @brief sqlite3 连接 RAII 关闭器（无异常；错误码转状态字符串）。 */
struct SqliteCloser {
  void operator()(sqlite3* db) const { sqlite3_close(db); }
};
using SqlitePtr = std::unique_ptr<sqlite3, SqliteCloser>;

/** @brief 以 sqlite3 错误消息填充 error（含路径与步骤），返回 false。 */
bool FailWithSqliteError(const std::string& path, sqlite3* db, const char* step,
                         std::string* error) {
  if (error != nullptr) {
    *error = path + ": " + step + ": " + sqlite3_errmsg(db);
  }
  return false;
}

/** @brief 查找语句列索引；未找到返回 -1。 */
int FindColumn(sqlite3_stmt* stmt, const char* name) {
  const int count = sqlite3_column_count(stmt);
  for (int i = 0; i < count; ++i) {
    if (std::strcmp(sqlite3_column_name(stmt, i), name) == 0) {
      return i;
    }
  }
  return -1;
}

/** @brief 取列浮点值；NULL 列回退默认值。 */
float ColumnFloat(sqlite3_stmt* stmt, int column, float fallback) {
  if (column < 0 || sqlite3_column_type(stmt, column) == SQLITE_NULL) {
    return fallback;
  }
  return static_cast<float>(sqlite3_column_double(stmt, column));
}

/**
 * @brief 读单连续特征模板（mean/std 列对）。
 * 模板组行存在即子模板存在：std 必须非 NULL 且 > 0（CHECK 兜底，此处显式校验
 * 以给出字段上下文），mean 可空（缺省 0.0f）。
 * @param std_field_name 错误信息中的 std 字段名（RCS 为 "std_db"，其余 "std"）。
 */
bool ReadTemplateColumnPair(sqlite3_stmt* stmt, int mean_column, int std_column,
                            const std::string& path, RirFeatureTemplate* out,
                            std::string* error, const char* std_field_name = "std") {
  if (std_column < 0 || sqlite3_column_type(stmt, std_column) == SQLITE_NULL) {
    if (error != nullptr) {
      *error = path + "." + std_field_name + " must be > 0";
    }
    return false;
  }
  const double std_value = sqlite3_column_double(stmt, std_column);
  if (!(std_value > 0.0)) {
    if (error != nullptr) {
      *error = path + "." + std_field_name + " must be > 0";
    }
    return false;
  }
  out->std = static_cast<float>(std_value);
  out->mean = ColumnFloat(stmt, mean_column, 0.0f);
  return true;
}

/** @brief 按列名前缀读模板；suffix 附加在 "_mean"/"_std" 后（turn_radius 用 "_log10"）。 */
bool ReadNamedTemplate(sqlite3_stmt* stmt, const char* prefix, const char* suffix,
                       const std::string& path, RirFeatureTemplate* out,
                       std::string* error) {
  const std::string mean_column = std::string(prefix) + "_mean" + suffix;
  const std::string std_column = std::string(prefix) + "_std" + suffix;
  return ReadTemplateColumnPair(stmt, FindColumn(stmt, mean_column.c_str()),
                                FindColumn(stmt, std_column.c_str()), path, out, error);
}

/** @brief 读取 rcs 模板组（列名 mean_dbsm/std_db，含变化量列）。 */
bool ReadRcsTemplate(sqlite3_stmt* stmt, const std::string& path,
                     RirModelProfile* profile, std::string* error) {
  RirFeatureTemplate mean_std;
  if (!ReadTemplateColumnPair(stmt, FindColumn(stmt, "mean_dbsm"), FindColumn(stmt, "std_db"),
                              path + ".rcs", &mean_std, error, "std_db")) {
    return false;
  }
  profile->rcs.mean_dbsm = mean_std.mean;
  profile->rcs.std_db = mean_std.std;
  profile->rcs.azimuth_variation_db = ColumnFloat(stmt, FindColumn(stmt, "azimuth_variation_db"), 0.0f);
  profile->rcs.elevation_variation_db = ColumnFloat(stmt, FindColumn(stmt, "elevation_variation_db"), 0.0f);
  profile->rcs.minimum_aspect_coverage_deg =
      ColumnFloat(stmt, FindColumn(stmt, "minimum_aspect_coverage_deg"), 0.0f);
  return true;
}

/** @brief 读取 motion 模板组（speed/altitude/acceleration；turn_radius 用 log10 尺度）。 */
bool ReadMotionTemplate(sqlite3_stmt* stmt, const std::string& path,
                        RirModelProfile* profile, std::string* error) {
  if (!ReadNamedTemplate(stmt, "speed", "", path + ".motion.speed_mps",
                         &profile->motion.speed_mps, error)) {
    return false;
  }
  if (!ReadNamedTemplate(stmt, "altitude", "", path + ".motion.altitude_m",
                         &profile->motion.altitude_m, error)) {
    return false;
  }
  if (!ReadNamedTemplate(stmt, "acceleration", "", path + ".motion.acceleration_mps2",
                         &profile->motion.acceleration_mps2, error)) {
    return false;
  }
  // 转弯半径按对数尺度比对（§7.4：mean_log10/std_log10）。
  if (!ReadNamedTemplate(stmt, "turn_radius", "_log10", path + ".motion.turn_radius_m",
                         &profile->motion.turn_radius_log10, error)) {
    return false;
  }
  return true;
}

/** @brief 读取 polarization 模板组。 */
bool ReadPolarizationTemplate(sqlite3_stmt* stmt, const std::string& path,
                              RirModelProfile* profile, std::string* error) {
  if (!ReadNamedTemplate(stmt, "energy_difference", "", path + ".polarization.energy_difference_db",
                         &profile->polarization.energy_difference_db, error)) {
    return false;
  }
  if (!ReadNamedTemplate(stmt, "relative_difference", "", path + ".polarization.relative_difference_db",
                         &profile->polarization.relative_difference_db, error)) {
    return false;
  }
  if (!ReadNamedTemplate(stmt, "energy_sum", "", path + ".polarization.energy_sum_db",
                         &profile->polarization.energy_sum_db, error)) {
    return false;
  }
  return true;
}

/** @brief 读取 range_profile 模板组。 */
bool ReadRangeProfileTemplate(sqlite3_stmt* stmt, const std::string& path,
                              RirModelProfile* profile, std::string* error) {
  if (!ReadNamedTemplate(stmt, "length", "", path + ".range_profile.length_m",
                         &profile->range_profile.length_m, error)) {
    return false;
  }
  if (!ReadNamedTemplate(stmt, "peak_count", "", path + ".range_profile.peak_count",
                         &profile->range_profile.peak_count, error)) {
    return false;
  }
  if (!ReadNamedTemplate(stmt, "peak_energy_concentration", "",
                         path + ".range_profile.peak_energy_concentration",
                         &profile->range_profile.peak_energy_concentration, error)) {
    return false;
  }
  profile->range_profile.minimum_bandwidth_hz =
      ColumnFloat(stmt, FindColumn(stmt, "minimum_bandwidth_hz"), 0.0f);
  return true;
}

/** @brief 单行 profile 读取 + 校验（复合主键 (model_id, profile_id) 唯一性由 SQLite 保证）。 */
bool ReadProfileRow(sqlite3_stmt* stmt, RirModelProfile* out, const std::string& path,
                    std::string* error) {
  const int profile_id_column = FindColumn(stmt, "profile_id");
  const int model_id_column = FindColumn(stmt, "model_id");
  if (profile_id_column < 0 || model_id_column < 0) {
    if (error != nullptr) {
      *error = path + ": profiles table missing profile_id/model_id column";
    }
    return false;
  }
  const unsigned char* profile_id_text = sqlite3_column_text(stmt, profile_id_column);
  const unsigned char* model_id_text = sqlite3_column_text(stmt, model_id_column);
  if (profile_id_text == nullptr || model_id_text == nullptr ||
      profile_id_text[0] == '\0' || model_id_text[0] == '\0') {
    if (error != nullptr) {
      *error = path + ".profiles: profile_id/model_id must be non-empty strings";
    }
    return false;
  }
  out->profile_id = reinterpret_cast<const char*>(profile_id_text);
  const std::string profile_path = path + ".profiles[" + out->profile_id + "]";
  out->min_snr_db = ColumnFloat(stmt, FindColumn(stmt, "min_snr_db"), 0.0f);
  out->max_range_resolution_m = ColumnFloat(stmt, FindColumn(stmt, "max_range_resolution_m"), 0.0f);
  // aspect：NULL = 全范围缺省（结构体默认）；均非空时校验 min <= max（单边合法）。
  const int az_min_column = FindColumn(stmt, "aspect_az_min_deg");
  const int az_max_column = FindColumn(stmt, "aspect_az_max_deg");
  const int el_min_column = FindColumn(stmt, "aspect_el_min_deg");
  const int el_max_column = FindColumn(stmt, "aspect_el_max_deg");
  out->aspect_az_min_deg = ColumnFloat(stmt, az_min_column, out->aspect_az_min_deg);
  out->aspect_az_max_deg = ColumnFloat(stmt, az_max_column, out->aspect_az_max_deg);
  out->aspect_el_min_deg = ColumnFloat(stmt, el_min_column, out->aspect_el_min_deg);
  out->aspect_el_max_deg = ColumnFloat(stmt, el_max_column, out->aspect_el_max_deg);
  if (az_min_column >= 0 && az_max_column >= 0 &&
      sqlite3_column_type(stmt, az_min_column) != SQLITE_NULL &&
      sqlite3_column_type(stmt, az_max_column) != SQLITE_NULL &&
      out->aspect_az_min_deg > out->aspect_az_max_deg) {
    if (error != nullptr) {
      *error = profile_path + ".aspect_az_min_deg must be <= aspect_az_max_deg";
    }
    return false;
  }
  if (el_min_column >= 0 && el_max_column >= 0 &&
      sqlite3_column_type(stmt, el_min_column) != SQLITE_NULL &&
      sqlite3_column_type(stmt, el_max_column) != SQLITE_NULL &&
      out->aspect_el_min_deg > out->aspect_el_max_deg) {
    if (error != nullptr) {
      *error = profile_path + ".aspect_el_min_deg must be <= aspect_el_max_deg";
    }
    return false;
  }
  return true;
}

/** @brief (model_id, profile_id) → 挂载位置索引。 */
using ProfileIndex =
    std::map<std::pair<std::string, std::string>, std::pair<std::size_t, std::size_t>>;

}  // namespace

bool RirFeatureDatabase::Load(const std::string& path, RirFeatureDatabase* database,
                                      std::string* error) {
  if (database == nullptr) {
    if (error != nullptr) {
      *error = "null database output";
    }
    return false;
  }

  // 只读打开（识别基线是只读数据；加载完成后连接即关闭）。
  sqlite3* raw_db = nullptr;
  if (sqlite3_open_v2(path.c_str(), &raw_db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
    if (error != nullptr) {
      *error = path + ": cannot open sqlite database: " +
               (raw_db != nullptr ? sqlite3_errmsg(raw_db) : "unknown error");
    }
    if (raw_db != nullptr) {
      sqlite3_close(raw_db);
    }
    return false;
  }
  SqlitePtr db(raw_db);

  // 外键约束开启：models.category_id / profiles.model_id / 模板表 profile 引用由 SQLite 兜底。
  {
    char* sqlite_error = nullptr;
    if (sqlite3_exec(db.get(), "PRAGMA foreign_keys = ON", nullptr, nullptr,
                     &sqlite_error) != SQLITE_OK) {
      if (error != nullptr) {
        *error = path + ": enable foreign_keys: " +
                 (sqlite_error != nullptr ? sqlite_error : "unknown error");
      }
      sqlite3_free(sqlite_error);
      return false;
    }
  }

  RirFeatureDatabase candidate;

  // -- meta 表：自描述元数据（v1.1 六键必填） ----------------------------------
  {
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db.get(), "SELECT key, value FROM meta", -1, &stmt, nullptr) !=
        SQLITE_OK) {
      return FailWithSqliteError(path, db.get(), "missing or invalid meta table", error);
    }
    std::string schema_version;
    std::string database_id;
    std::string version;
    std::string created_utc;
    std::string polarization_channels;
    std::string polarization_energy_reference;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      const unsigned char* key = sqlite3_column_text(stmt, 0);
      const unsigned char* value = sqlite3_column_text(stmt, 1);
      if (key == nullptr || value == nullptr) {
        continue;
      }
      const std::string key_str = reinterpret_cast<const char*>(key);
      const std::string value_str = reinterpret_cast<const char*>(value);
      if (key_str == "schema_version") {
        schema_version = value_str;
      } else if (key_str == "database_id") {
        database_id = value_str;
      } else if (key_str == "version") {
        version = value_str;
      } else if (key_str == "created_utc") {
        created_utc = value_str;
      } else if (key_str == "polarization_channels") {
        polarization_channels = value_str;
      } else if (key_str == "polarization_energy_reference") {
        polarization_energy_reference = value_str;
      }
    }
    sqlite3_finalize(stmt);
    if (schema_version.empty()) {
      if (error != nullptr) {
        *error = path + ".schema_version must be a non-empty string";
      }
      return false;
    }
    if (schema_version != kSupportedSchemaVersion) {
      if (error != nullptr) {
        *error = path + ".schema_version: unsupported version '" + schema_version +
                 "' (supported: " + kSupportedSchemaVersion + ")";
      }
      return false;
    }
    // v1.1 自描述契约：以下键必填非空（建库工具保证；外部库缺失即拒绝）。
    if (database_id.empty()) {
      if (error != nullptr) {
        *error = path + ".database_id must be a non-empty string";
      }
      return false;
    }
    if (version.empty()) {
      if (error != nullptr) {
        *error = path + ".version must be a non-empty string";
      }
      return false;
    }
    if (created_utc.empty()) {
      if (error != nullptr) {
        *error = path + ".created_utc must be a non-empty string";
      }
      return false;
    }
    if (polarization_channels.empty()) {
      if (error != nullptr) {
        *error = path + ".polarization_channels must be a non-empty string";
      }
      return false;
    }
    if (polarization_energy_reference.empty()) {
      if (error != nullptr) {
        *error = path + ".polarization_energy_reference must be a non-empty string";
      }
      return false;
    }
    candidate.database_id_ = std::move(database_id);
    candidate.version_ = std::move(version);
  }

  // -- units 表：7 个必填量纲；rcs 必须为 dBsm（匹配数学域强校验） ------------
  {
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db.get(), "SELECT quantity, unit FROM units", -1, &stmt, nullptr) !=
        SQLITE_OK) {
      return FailWithSqliteError(path, db.get(), "missing or invalid units table", error);
    }
    std::set<std::string> present_quantities;
    std::string rcs_unit;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      const unsigned char* quantity_text = sqlite3_column_text(stmt, 0);
      const unsigned char* unit_text = sqlite3_column_text(stmt, 1);
      if (quantity_text == nullptr || quantity_text[0] == '\0') {
        if (error != nullptr) {
          *error = path + ".units: quantity must be a non-empty string";
        }
        sqlite3_finalize(stmt);
        return false;
      }
      const std::string quantity = reinterpret_cast<const char*>(quantity_text);
      if (unit_text == nullptr || unit_text[0] == '\0') {
        if (error != nullptr) {
          *error = path + ".units." + quantity + " must be a non-empty string";
        }
        sqlite3_finalize(stmt);
        return false;
      }
      present_quantities.insert(quantity);
      if (quantity == "rcs") {
        rcs_unit = reinterpret_cast<const char*>(unit_text);
      }
    }
    sqlite3_finalize(stmt);
    for (const char* quantity : kRequiredUnitQuantities) {
      if (present_quantities.find(quantity) == present_quantities.end()) {
        if (error != nullptr) {
          *error = path + ".units: missing quantity '" + std::string(quantity) + "'";
        }
        return false;
      }
    }
    if (rcs_unit != "dBsm") {
      if (error != nullptr) {
        *error = path + ".units.rcs must be 'dBsm' (found '" + rcs_unit + "')";
      }
      return false;
    }
  }

  // -- categories 表 -----------------------------------------------------------
  {
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db.get(), "SELECT category_id, display_name, prior FROM categories", -1,
                           &stmt, nullptr) != SQLITE_OK) {
      return FailWithSqliteError(path, db.get(), "missing or invalid categories table", error);
    }
    std::set<std::string> category_ids;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      const unsigned char* category_id_text = sqlite3_column_text(stmt, 0);
      if (category_id_text == nullptr || category_id_text[0] == '\0') {
        if (error != nullptr) {
          *error = path + ".categories: category_id must be a non-empty string";
        }
        sqlite3_finalize(stmt);
        return false;
      }
      RirCategoryEntry entry;
      entry.category_id = reinterpret_cast<const char*>(category_id_text);
      if (!category_ids.insert(entry.category_id).second) {
        if (error != nullptr) {
          *error = path + ".categories[" + entry.category_id + "].category_id must be unique";
        }
        sqlite3_finalize(stmt);
        return false;
      }
      // display_name 可空（文档性显示名）。
      const int display_name_column = FindColumn(stmt, "display_name");
      if (display_name_column >= 0 &&
          sqlite3_column_type(stmt, display_name_column) != SQLITE_NULL) {
        entry.display_name =
            reinterpret_cast<const char*>(sqlite3_column_text(stmt, display_name_column));
      }
      // v1.1：prior 必填且 > 0（CHECK 兜底；此处显式校验给出字段上下文）。
      const int prior_index = FindColumn(stmt, "prior");
      if (prior_index < 0 || sqlite3_column_type(stmt, prior_index) == SQLITE_NULL) {
        if (error != nullptr) {
          *error = path + ".categories[" + entry.category_id + "].prior must be > 0";
        }
        sqlite3_finalize(stmt);
        return false;
      }
      const double prior = sqlite3_column_double(stmt, prior_index);
      if (!(prior > 0.0)) {
        if (error != nullptr) {
          *error = path + ".categories[" + entry.category_id + "].prior must be > 0";
        }
        sqlite3_finalize(stmt);
        return false;
      }
      entry.prior = static_cast<float>(prior);
      candidate.categories_.push_back(std::move(entry));
    }
    sqlite3_finalize(stmt);
  }

  // -- models 表（category 引用校验） ------------------------------------------
  std::unordered_map<std::string, std::size_t> model_index_by_id;
  {
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db.get(),
                           "SELECT model_id, category_id, display_name, prior FROM models", -1,
                           &stmt, nullptr) != SQLITE_OK) {
      return FailWithSqliteError(path, db.get(), "missing or invalid models table", error);
    }
    std::set<std::string> model_ids;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      const unsigned char* model_id_text = sqlite3_column_text(stmt, 0);
      const unsigned char* category_id_text = sqlite3_column_text(stmt, 1);
      if (model_id_text == nullptr || model_id_text[0] == '\0') {
        if (error != nullptr) {
          *error = path + ".models: model_id must be a non-empty string";
        }
        sqlite3_finalize(stmt);
        return false;
      }
      if (category_id_text == nullptr || category_id_text[0] == '\0') {
        if (error != nullptr) {
          *error = path + ".models[" + reinterpret_cast<const char*>(model_id_text) +
                   "].category_id must be a non-empty string";
        }
        sqlite3_finalize(stmt);
        return false;
      }
      RirModel model;
      model.model_id = reinterpret_cast<const char*>(model_id_text);
      model.category_id = reinterpret_cast<const char*>(category_id_text);
      if (!model_ids.insert(model.model_id).second) {
        if (error != nullptr) {
          *error = path + ".models[" + model.model_id + "].model_id must be unique";
        }
        sqlite3_finalize(stmt);
        return false;
      }
      bool category_exists = false;
      for (const RirCategoryEntry& category : candidate.categories_) {
        if (category.category_id == model.category_id) {
          category_exists = true;
          break;
        }
      }
      if (!category_exists) {
        if (error != nullptr) {
          *error = path + ".models[" + model.model_id +
                   "].category_id references unknown category '" + model.category_id + "'";
        }
        sqlite3_finalize(stmt);
        return false;
      }
      const int display_name_column = FindColumn(stmt, "display_name");
      if (display_name_column >= 0 &&
          sqlite3_column_type(stmt, display_name_column) != SQLITE_NULL) {
        model.display_name =
            reinterpret_cast<const char*>(sqlite3_column_text(stmt, display_name_column));
      }
      const int prior_index = FindColumn(stmt, "prior");
      if (prior_index < 0 || sqlite3_column_type(stmt, prior_index) == SQLITE_NULL) {
        if (error != nullptr) {
          *error = path + ".models[" + model.model_id + "].prior must be > 0";
        }
        sqlite3_finalize(stmt);
        return false;
      }
      const double prior = sqlite3_column_double(stmt, prior_index);
      if (!(prior > 0.0)) {
        if (error != nullptr) {
          *error = path + ".models[" + model.model_id + "].prior must be > 0";
        }
        sqlite3_finalize(stmt);
        return false;
      }
      model.prior = static_cast<float>(prior);
      model_index_by_id.emplace(model.model_id, candidate.models_.size());
      candidate.models_.push_back(std::move(model));
    }
    sqlite3_finalize(stmt);
  }

  // -- profiles 表（按 model_id 分组挂载；aspect 校验） ------------------------
  ProfileIndex profile_index;
  {
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db.get(),
                           "SELECT profile_id, model_id, min_snr_db, max_range_resolution_m, "
                           "aspect_az_min_deg, aspect_az_max_deg, "
                           "aspect_el_min_deg, aspect_el_max_deg "
                           "FROM profiles",
                           -1, &stmt, nullptr) != SQLITE_OK) {
      return FailWithSqliteError(path, db.get(), "missing or invalid profiles table", error);
    }
    std::unordered_map<std::string, std::set<std::string>> profile_ids_by_model;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      RirModelProfile profile;
      if (!ReadProfileRow(stmt, &profile, path, error)) {
        sqlite3_finalize(stmt);
        return false;
      }
      const unsigned char* model_id_text = sqlite3_column_text(stmt, FindColumn(stmt, "model_id"));
      const std::string model_id = reinterpret_cast<const char*>(model_id_text);
      // (model_id, profile_id) 模型内唯一：显式校验（复合主键兜底）。
      std::set<std::string>& model_profile_ids = profile_ids_by_model[model_id];
      if (!model_profile_ids.insert(profile.profile_id).second) {
        if (error != nullptr) {
          *error = path + ".profiles[" + profile.profile_id +
                   "].profile_id must be unique within the model";
        }
        sqlite3_finalize(stmt);
        return false;
      }
      const auto model_it = model_index_by_id.find(model_id);
      if (model_it == model_index_by_id.end()) {
        if (error != nullptr) {
          *error = path + ".profiles[" + profile.profile_id +
                   "].model_id references unknown model '" + model_id + "'";
        }
        sqlite3_finalize(stmt);
        return false;
      }
      const std::size_t profile_index_in_model =
          candidate.models_[model_it->second].profiles.size();
      const std::string profile_id = profile.profile_id;  // move 前保留（索引键）。
      candidate.models_[model_it->second].profiles.push_back(std::move(profile));
      profile_index.emplace(std::make_pair(model_id, profile_id),
                            std::make_pair(model_it->second, profile_index_in_model));
    }
    sqlite3_finalize(stmt);
  }

  // -- 模板组表（行存在 = 组存在；std 必填 > 0） --------------------------------
  // lambda 置于 Load 内：需访问 candidate（私有成员）。
  const auto load_template_table =
      [&](const char* table, const char* select_sql,
          bool (*reader)(sqlite3_stmt*, const std::string&, RirModelProfile*,
                         std::string*)) -> bool {
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db.get(), select_sql, -1, &stmt, nullptr) != SQLITE_OK) {
      const std::string step = "missing or invalid " + std::string(table) + " table";
      return FailWithSqliteError(path, db.get(), step.c_str(), error);
    }
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      const unsigned char* profile_id_text = sqlite3_column_text(stmt, 0);
      const unsigned char* model_id_text = sqlite3_column_text(stmt, 1);
      if (profile_id_text == nullptr || profile_id_text[0] == '\0' ||
          model_id_text == nullptr || model_id_text[0] == '\0') {
        if (error != nullptr) {
          *error = path + "." + table + ": profile_id/model_id must be non-empty strings";
        }
        sqlite3_finalize(stmt);
        return false;
      }
      const std::string profile_id = reinterpret_cast<const char*>(profile_id_text);
      const std::string model_id = reinterpret_cast<const char*>(model_id_text);
      const auto it = profile_index.find(std::make_pair(model_id, profile_id));
      if (it == profile_index.end()) {
        if (error != nullptr) {
          *error = path + "." + table + "[" + profile_id +
                   "]: references unknown profile (model_id '" + model_id + "')";
        }
        sqlite3_finalize(stmt);
        return false;
      }
      RirModelProfile* profile =
          &candidate.models_[it->second.first].profiles[it->second.second];
      if (!reader(stmt, path + ".profiles[" + profile_id + "]", profile, error)) {
        sqlite3_finalize(stmt);
        return false;
      }
    }
    sqlite3_finalize(stmt);
    return true;
  };

  if (!load_template_table(kRcsTable,
                           "SELECT profile_id, model_id, mean_dbsm, std_db, "
                           "azimuth_variation_db, elevation_variation_db, "
                           "minimum_aspect_coverage_deg FROM rcs_templates",
                           ReadRcsTemplate)) {
    return false;
  }
  if (!load_template_table(kMotionTable,
                           "SELECT profile_id, model_id, "
                           "speed_mean, speed_std, altitude_mean, altitude_std, "
                           "acceleration_mean, acceleration_std, "
                           "turn_radius_mean_log10, turn_radius_std_log10 FROM motion_templates",
                           ReadMotionTemplate)) {
    return false;
  }
  if (!load_template_table(kPolarizationTable,
                           "SELECT profile_id, model_id, "
                           "energy_difference_mean, energy_difference_std, "
                           "relative_difference_mean, relative_difference_std, "
                           "energy_sum_mean, energy_sum_std FROM polarization_templates",
                           ReadPolarizationTemplate)) {
    return false;
  }
  if (!load_template_table(kRangeProfileTable,
                           "SELECT profile_id, model_id, "
                           "length_mean, length_std, peak_count_mean, peak_count_std, "
                           "peak_energy_concentration_mean, peak_energy_concentration_std, "
                           "minimum_bandwidth_hz FROM range_profile_templates",
                           ReadRangeProfileTemplate)) {
    return false;
  }

  // 全部校验通过后才提交候选数据库（全量原子替换语义）；连接随作用域关闭。
  candidate.loaded_ = true;
  *database = std::move(candidate);
  return true;
}

}  // namespace recognition
}  // namespace remote_identification_radar
