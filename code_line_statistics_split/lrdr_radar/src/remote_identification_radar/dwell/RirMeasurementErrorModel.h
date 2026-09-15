/**
 * @file RirMeasurementErrorModel.h
 * @brief RIR 测量误差模型薄适配层（common 单源）。
 * @note 本文件仅供 RIR 模块内部使用，不作为公开 API；检测量测误差供
 *       阶段 2-T 轻量关联的波门定标消费（不出 public 面）。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_MEASUREMENT_ERROR_MODEL_H_
#define REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_MEASUREMENT_ERROR_MODEL_H_

#include "common/numerics/Constants.h"
#include "common/radar/MeasurementErrorModel.h"
#include "remote_identification_radar/dwell/RirBeamControl.h"

namespace remote_identification_radar {
namespace dwell {

/**
 * @brief RirMeasurementErrorState 表示当前探测的测量误差结果。
 * @note 口径（2026-08-30 拆分）：std 字段只含随机项；固定系统偏置（未标定残差）
 *       单独由 bias 字段承载，应施加在量测均值侧而非并入随机 std。
 */
struct RirMeasurementErrorState {
  float range_error_std_m{0.0f};   /**< 距离测量随机标准差（米，不含固定偏置）。 */
  float angle_error_std_rad{0.0f}; /**< 方位/俯仰合成的等效角度测量随机标准差（弧度，不含固定偏置）。 */
  float range_bias_m{0.0f};        /**< 距离测量固定系统偏置（米），施加在量测均值侧。 */
  float angle_bias_rad{0.0f};      /**< 方位/俯仰合成的等效角度固定系统偏置（弧度），施加在量测均值侧。 */
};

/**
 * @brief RirMeasurementErrorModel 负责计算当前探测的测量误差。
 */
class RirMeasurementErrorModel {
 public:
  /**
   * @brief 计算测量误差。
   * @param effective_snr_db 等效积累后的信噪比（dB）。
   * @param effective_beamwidth_deg 生效波束宽度（度）。
   * @param bandwidth_hz 发射机信号带宽（Hz）。
   * @return 测量误差结果（std 字段为纯随机项；bias 字段为固定系统偏置，
   *         与 common 单源同步拆出，施加在量测均值侧）。
   */
  static RirMeasurementErrorState Compute(float effective_snr_db,
                                          const RirEffectiveBeamwidthDeg& effective_beamwidth_deg,
                                          float bandwidth_hz) {
    const oneq::common::radar::MeasurementErrorState common_state =
        oneq::common::radar::ComputeMeasurementError(
            effective_snr_db, bandwidth_hz,
            oneq::common::numerics::DegToRad(effective_beamwidth_deg.az_beamwidth_deg),
            oneq::common::numerics::DegToRad(effective_beamwidth_deg.el_beamwidth_deg));
    RirMeasurementErrorState state;
    state.range_error_std_m = common_state.range_error_std_m;
    state.angle_error_std_rad = common_state.angle_error_std_rad;
    state.range_bias_m = common_state.range_bias_m;
    state.angle_bias_rad = common_state.angle_bias_rad;
    return state;
  }

 private:
  /**
   * @brief 根据有效方位/俯仰波束宽度计算等效角度测量随机标准差。
   * @param snr_db 等效积累后的信噪比（dB）。
   * @param beamwidth_deg 已解析的有效方位/俯仰波束宽度。
   * @return 横向各向同性量测模型使用的等效角度标准差（弧度，纯随机项，不含固定偏置）。
   */
  static float ComputeEquivalentAngleErrorStdDev(
      float snr_db, const RirEffectiveBeamwidthDeg& beamwidth_deg) {
    return oneq::common::radar::ComputeEquivalentAngleErrorStdDev(
        snr_db, oneq::common::numerics::DegToRad(beamwidth_deg.az_beamwidth_deg),
        oneq::common::numerics::DegToRad(beamwidth_deg.el_beamwidth_deg));
  }
};

}  // namespace dwell
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_MEASUREMENT_ERROR_MODEL_H_
