/**
 * @file IFeatureRepository.h
 * @brief 定义目标特征仓储抽象接口。
 */

#ifndef AIRBORNE_RADAR_ENVIRONMENT_DATABASE_I_FEATURE_REPOSITORY_H_
#define AIRBORNE_RADAR_ENVIRONMENT_DATABASE_I_FEATURE_REPOSITORY_H_

#include <initializer_list>
#include <map>
#include <string>

namespace airborne_radar {
namespace environment {

/**
 * @brief FeatureVector 表示用于分类匹配的输入特征向量。
 * @note 使用键值对存储，避免固定维度特征写死，便于后续扩展。
 */
struct FeatureVector {
  using ValueMap = std::map<std::string, float>;

  FeatureVector() = default;

  /**
   * @brief 使用初始化列表构造特征向量。
   * @param[in] init 初始特征键值对列表。
   */
  FeatureVector(std::initializer_list<std::pair<std::string, float>> init) {
    for (const auto& entry : init) {
      values[entry.first] = entry.second;
    }
  }

  /**
   * @brief 设置特征值。
   * @param[in] name 特征名称。
   * @param[in] value 特征值。
   */
  void Set(const std::string& name, float value) { values[name] = value; }

  /**
   * @brief 查询特征值是否存在。
   * @param[in] name 特征名称。
   * @return 存在指定特征值时返回 `true`。
   */
  bool Has(const std::string& name) const { return values.find(name) != values.end(); }

  /**
   * @brief 获取特征值，不存在时返回默认值。
   * @param[in] name 特征名称。
   * @param[in] default_value 默认值，默认为 `0.0f`。
   * @return 特征值或默认值。
   */
  float GetOrDefault(const std::string& name, float default_value = 0.0f) const {
    const auto it = values.find(name);
    return it == values.end() ? default_value : it->second;
  }
  ValueMap values; /**< 特征键值映射。 */
};

/**
 * @brief MatchResult 表示仓储匹配得到的最优结果。
 */
struct MatchResult {
  std::string target_type = "UNKNOWN"; /**< 最优匹配的目标类型标签。 */
  float probability{0.0f};            /**< 归一化匹配概率。 */
  float distance{0.0f};               /**< 最优匹配对应的特征距离。 */
};

/**
 * @brief 提供特征查询与仓储生命周期控制接口。
 */
class IFeatureRepository {
 public:
  virtual ~IFeatureRepository() = default;

  /**
   * @brief 连接外部数据源。
   * @param[in] connection_string 外部数据库连接串。
   * @return 连接配置有效时返回 `true`。
   */
  virtual bool ConnectDataSource(const std::string& connection_string) = 0;

  /**
   * @brief 拉取外部特征数据并更新内存仓储。
   * @return 拉取并刷新成功时返回 `true`。
   */
  virtual bool ReloadFromDataSource() = 0;

  /**
   * @brief 使用输入特征查询最优匹配。
   * @param[in] input 输入特征向量。
   * @param[out] result 输出匹配结果。
   * @return 查询成功时返回 `true`。
   */
  virtual bool QueryBestMatch(const FeatureVector& input, MatchResult& result) const = 0;
};

}  // namespace environment
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_ENVIRONMENT_DATABASE_I_FEATURE_REPOSITORY_H_
