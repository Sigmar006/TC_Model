/**
 * @file SignalComponentFactory.h
 * @brief 定义 SignalPipeline 私有组件工厂，负责配置映射与组件装配。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_SIGNAL_COMPONENT_FACTORY_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_SIGNAL_COMPONENT_FACTORY_H_

#include <Eigen/Core>
#include <memory>
#include <utility>
#include <vector>

#include "airborne_radar/config/SignalEngineeringConfig.h"
#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/pipeline/SignalPipelineExecutionConfig.h"
#include "airborne_radar/signal/pipeline/SignalPipelineRuntimeTypes.h"
#include "airborne_radar/signal/tracking/BoostTrackPool.h"
#include "airborne_radar/signal/tracking/IKalmanPredictor.h"
#include "airborne_radar/signal/tracking/IKalmanUpdater.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/KalmanUpdater.h"
#include "airborne_radar/signal/tracking/SynchronizedTrackPool.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"
#include "airborne_radar/signal/tracking/TrackLifecycleManager.h"
#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

using ExecutionConfig = ::airborne_radar::config::execution::InternalExecutionConfig;

/**
 * @brief Pipeline 自持有的核心组件装配产物。
 */
struct OwnedSignalComponents {
  // VS2013 不生成隐式移动操作（含移动-only 成员时退化为已删除拷贝），显式实现。
  OwnedSignalComponents() = default;
  OwnedSignalComponents(OwnedSignalComponents&& other) noexcept
      : association_config(std::move(other.association_config)),
        track_filter_config(std::move(other.track_filter_config)),
        kalman_predictor(std::move(other.kalman_predictor)),
        kalman_updater(std::move(other.kalman_updater)),
        signal_detector(std::move(other.signal_detector)) {}
  OwnedSignalComponents& operator=(OwnedSignalComponents&& other) noexcept {
    association_config = std::move(other.association_config);
    track_filter_config = std::move(other.track_filter_config);
    kalman_predictor = std::move(other.kalman_predictor);
    kalman_updater = std::move(other.kalman_updater);
    signal_detector = std::move(other.signal_detector);
    return *this;
  }
  association::DataAssociationConfig association_config{};     /**< 关联配置（由执行配置映射）。 */
  tracking::TrackFilterConfig track_filter_config{};           /**< 轨迹滤波器配置（由执行配置映射）。 */
  std::unique_ptr<tracking::IKalmanPredictor> kalman_predictor; /**< Kalman 预测器（启用时非空）。 */
  std::unique_ptr<tracking::IKalmanUpdater> kalman_updater;     /**< Kalman 更新器（启用时非空）。 */
  std::unique_ptr<detection::SignalDetector> signal_detector;   /**< 信号检测器（启用物理检测时非空）。 */
};

/**
 * @brief 生命周期装配产物，包含轨迹池、（IMM）预测器/更新器集合与生命周期管理器。
 */
struct LifecycleAssemblyArtifacts {
  // VS2013 不生成隐式移动操作（含移动-only 成员时退化为已删除拷贝），显式实现。
  LifecycleAssemblyArtifacts() = default;
  LifecycleAssemblyArtifacts(LifecycleAssemblyArtifacts&& other) noexcept
      : pool(std::move(other.pool)),
        pool_wrapper(std::move(other.pool_wrapper)),
        kalman_predictor(std::move(other.kalman_predictor)),
        kalman_updater(std::move(other.kalman_updater)),
        imm_predictors_owned(std::move(other.imm_predictors_owned)),
        imm_updaters_owned(std::move(other.imm_updaters_owned)),
        imm_predictors(std::move(other.imm_predictors)),
        imm_updaters(std::move(other.imm_updaters)),
        lifecycle_manager(std::move(other.lifecycle_manager)) {}
  LifecycleAssemblyArtifacts& operator=(LifecycleAssemblyArtifacts&& other) noexcept {
    pool = std::move(other.pool);
    pool_wrapper = std::move(other.pool_wrapper);
    kalman_predictor = std::move(other.kalman_predictor);
    kalman_updater = std::move(other.kalman_updater);
    imm_predictors_owned = std::move(other.imm_predictors_owned);
    imm_updaters_owned = std::move(other.imm_updaters_owned);
    imm_predictors = std::move(other.imm_predictors);
    imm_updaters = std::move(other.imm_updaters);
    lifecycle_manager = std::move(other.lifecycle_manager);
    return *this;
  }
  std::unique_ptr<tracking::BoostTrackPool> pool;                  /**< 底层轨迹池。 */
  std::unique_ptr<tracking::ITrackPool> pool_wrapper;              /**< 多线程模式下的同步包装器。 */
  std::unique_ptr<tracking::IKalmanPredictor> kalman_predictor;    /**< 单模型 Kalman 预测器。 */
  std::unique_ptr<tracking::IKalmanUpdater> kalman_updater;        /**< 单模型 Kalman 更新器。 */
  std::vector<std::unique_ptr<tracking::IKalmanPredictor>> imm_predictors_owned; /**< IMM 各模型预测器（自有）。 */
  std::vector<std::unique_ptr<tracking::IKalmanUpdater>> imm_updaters_owned;      /**< IMM 各模型更新器（自有）。 */
  std::vector<tracking::IKalmanPredictor*> imm_predictors;         /**< IMM 预测器裸指针视图。 */
  std::vector<tracking::IKalmanUpdater*> imm_updaters;              /**< IMM 更新器裸指针视图。 */
  std::unique_ptr<tracking::ITrackLifecycleManager> lifecycle_manager; /**< 生命周期管理器。 */
};

/**
 * @brief SignalPipeline 私有组件工厂，负责从执行配置映射子配置并装配各组件实例。
 */
class SignalComponentFactory final {
 public:
  /**
   * @brief 从执行配置构建生命周期配置。
   * @param config execution 运行配置。
   * @return 映射后的生命周期配置。
   */
  static tracking::LifecycleConfig BuildLifecycleConfig(const ExecutionConfig& config);

  /**
   * @brief 从执行配置构建轨迹滤波器配置。
   * @param config execution 运行配置。
   * @return 映射后的轨迹滤波器配置。
   */
  static tracking::TrackFilterConfig BuildTrackFilterConfig(const ExecutionConfig& config);

  /**
   * @brief 从执行配置构建数据关联配置。
   * @param config execution 运行配置。
   * @return 映射后的数据关联配置。
   */
  static association::DataAssociationConfig BuildAssociationConfig(const ExecutionConfig& config);

  /**
   * @brief 从执行配置装配 Pipeline 自持有的核心组件（关联/滤波配置、预测器、更新器、检测器）。
   * @param config execution 运行配置。
   * @return 装配产物；未启用的组件对应字段为空。
   */
  static OwnedSignalComponents BuildOwnedPipelineComponents(const ExecutionConfig& config);

  /**
   * @brief 从执行配置装配生命周期所需的全部组件（轨迹池、预测器/更新器、生命周期管理器）。
   * @param config execution 运行配置。
   * @return 装配产物；IMM 未启用时 imm_* 字段为空。
   */
  static LifecycleAssemblyArtifacts BuildLifecycleAssemblyArtifacts(const ExecutionConfig& config);

 private:
  static void LogLifecycleAssemblyConfigViolation(const char* message, float value);
  static void LogLifecycleAssemblyConfigViolation(const char* message, std::size_t value);
  static Eigen::MatrixXf BuildImmTransitionProbability(const ExecutionConfig& config,
                                                       std::size_t model_count);
  static Eigen::VectorXf BuildImmInitialWeights(const ExecutionConfig& config,
                                                std::size_t model_count);
};

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_SIGNAL_COMPONENT_FACTORY_H_
