/**
 * @file SbirsEnvironmentModel.h
 * @brief SBIRS-inspired 气象衰减模型。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_ENVIRONMENT_SBIRS_ENVIRONMENT_MODEL_H_
#define ONEQ_SRC_SBIRS_SENSOR_ENVIRONMENT_SBIRS_ENVIRONMENT_MODEL_H_

#include "1q/sbirs_sensor/config/SbirsEnvironmentConfig.h"
#include "1q/sbirs_sensor/session/SbirsSceneTypes.h"

namespace sbirs_sensor {
namespace environment {

/** @brief 大气有效壳层厚度（壳顶高度），单位 m；球对称近似。 */
constexpr double kAtmosphereShellThicknessM = 100000.0;
/** @brief 气团因子上限：近地平掠射路径防发散（本值对应约 80 km 以上近地点）。 */
constexpr double kMaxShellAirmassFactor = 10.0;

/**
 * @brief 计算视线的大气壳段气团因子 X = 穿壳弦长 ÷ 垂直壳厚。
 * @param[in] satellite_position_eci_m 卫星位置（ECEF 或同原点惯性系分量），单位 m
 * @param[in] target_position_eci_m 目标位置（同系分量），单位 m
 * @return 气团因子，夹紧到 [0, kMaxShellAirmassFactor]；重合位置返回 0
 * @note 球对称壳层（半径 = 地球平均半径 + 壳厚）：视线段完全在壳外时 X=0（纯空间
 *       路径不穿大气）；地面目标正对天顶时 X=1（与垂直穿过大气的基准一致）。
 *       穿地路径由上游遮挡门先行排除，本函数不重复判遮挡。
 */
double ComputeShellAirmassFactor(const session::SbirsVector3M& satellite_position_eci_m,
                                 const session::SbirsVector3M& target_position_eci_m);
/**
 * @brief 计算几何修正后的路径透过率 τ_geo = τ_eff^X（X 为壳段气团因子）。
 * @param[in] environment 环境与气象配置（决定 τ_eff）
 * @param[in] satellite_position_eci_m 卫星位置（ECEF 或同原点惯性系分量），单位 m
 * @param[in] target_position_eci_m 目标位置（同系分量），单位 m
 * @return 路径透过率，X=0（纯空间路径）时恒为 1，X=1（地面天顶）时等于 τ_eff
 */
float ResolveGeometricTransmittance(const config::SbirsEnvironmentConfig& environment,
                                    const session::SbirsVector3M& satellite_position_eci_m,
                                    const session::SbirsVector3M& target_position_eci_m);

/**
 * @brief 计算气象衰减比例 `A_total`（独立项加权叠加 + 可选交互项）。
 * @param[in] environment 环境与气象配置
 * @return 衰减比例，夹紧到 [0, 1]（1 表示完全衰减）
 * @note 内部实现细节。交互项仅在对应权重 > 0 时启用，默认 0 保持向后兼容。
 */
float ResolveWeatherAttenuation(const config::SbirsEnvironmentConfig& environment);
/**
 * @brief 计算气象修正后的有效透过率 τ_eff = base × (1 − A_total)。
 * @param[in] environment 环境与气象配置
 * @return 有效透过率，夹紧到 [0, 1]
 */
float ResolveEffectiveTransmittance(const config::SbirsEnvironmentConfig& environment);

}  // namespace environment
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_ENVIRONMENT_SBIRS_ENVIRONMENT_MODEL_H_
