/**
 * @file types.h
 * @brief 定义坐标工具公共值类型。
 */

#ifndef ONEQ_COORDINATE_TYPES_H_
#define ONEQ_COORDINATE_TYPES_H_

#include "1q/api.hpp"

namespace oneq {
namespace coordinate {

/**
 * @brief WGS84 大地位置（纬度、经度、椭球高）。
 */
struct ONEQ_API LlaPositionDegM {
  double latitude_deg{0.0};  /**< 纬度（单位：deg，范围 [-90, 90]） */
  double longitude_deg{0.0}; /**< 经度（单位：deg，范围 [-180, 180]） */
  double altitude_m{0.0};    /**< 椭球高（单位：m） */
  LlaPositionDegM() = default;
  LlaPositionDegM(double latitude, double longitude, double altitude)
      : latitude_deg(latitude), longitude_deg(longitude), altitude_m(altitude) {}
};

/**
 * @brief 地心地固位置（ECEF）。
 */
struct ONEQ_API EcefPositionM {
  double x_m{0.0}; /**< ECEF X（单位：m） */
  double y_m{0.0}; /**< ECEF Y（单位：m） */
  double z_m{0.0}; /**< ECEF Z（单位：m） */
  EcefPositionM() = default;
  EcefPositionM(double x, double y, double z) : x_m(x), y_m(y), z_m(z) {}
};

/**
 * @brief 地心惯性位置（ECI，J2000 平赤道面，地球自转轴为 z 轴）。
 * @note 与 ECEF 的差异仅在于绕 z 轴旋转（旋转角 = GMST），赤道面与 z 轴方向一致；
 *       单位 m。用于需要惯性参考系的场景（如天基红外传感器输出方位/俯仰）。
 */
struct ONEQ_API EciPositionM {
  double x_m{0.0}; /**< ECI X（单位：m） */
  double y_m{0.0}; /**< ECI Y（单位：m） */
  double z_m{0.0}; /**< ECI Z（单位：m） */
  EciPositionM() = default;
  EciPositionM(double x, double y, double z) : x_m(x), y_m(y), z_m(z) {}
};

/**
 * @brief 地心惯性速度（ECI），单位 m/s。
 */
struct ONEQ_API EciVelocityMps {
  double x_mps{0.0}; /**< ECI X 方向速度（单位：m/s） */
  double y_mps{0.0}; /**< ECI Y 方向速度（单位：m/s） */
  double z_mps{0.0}; /**< ECI Z 方向速度（单位：m/s） */
  EciVelocityMps() = default;
  EciVelocityMps(double x, double y, double z) : x_mps(x), y_mps(y), z_mps(z) {}
};

/**
 * @brief 局部东-北-天位置。
 */
struct ONEQ_API EnuPositionM {
  double east_m{0.0};  /**< East 方向位置（单位：m） */
  double north_m{0.0}; /**< North 方向位置（单位：m） */
  double up_m{0.0};    /**< Up 方向位置（单位：m） */
  EnuPositionM() = default;
  EnuPositionM(double east, double north, double up) : east_m(east), north_m(north), up_m(up) {}
};

/**
 * @brief 局部北-东-地位置。
 */
struct ONEQ_API NedPositionM {
  double north_m{0.0}; /**< North 方向位置（单位：m） */
  double east_m{0.0};  /**< East 方向位置（单位：m） */
  double down_m{0.0};  /**< Down 方向位置（单位：m） */
  NedPositionM() = default;
  NedPositionM(double north, double east, double down) : north_m(north), east_m(east), down_m(down) {}
};

/**
 * @brief 局部北-天-东位置。
 */
struct ONEQ_API NuePositionM {
  double north_m{0.0}; /**< North 方向位置（单位：m） */
  double up_m{0.0};    /**< Up 方向位置（单位：m） */
  double east_m{0.0};  /**< East 方向位置（单位：m） */
  NuePositionM() = default;
  NuePositionM(double north, double up, double east) : north_m(north), up_m(up), east_m(east) {}
};

/**
 * @brief 地心地固速度（ECEF）。
 */
struct ONEQ_API EcefVelocityMps {
  double x_mps{0.0}; /**< ECEF X 方向速度（单位：m/s） */
  double y_mps{0.0}; /**< ECEF Y 方向速度（单位：m/s） */
  double z_mps{0.0}; /**< ECEF Z 方向速度（单位：m/s） */
  EcefVelocityMps() = default;
  EcefVelocityMps(double x, double y, double z) : x_mps(x), y_mps(y), z_mps(z) {}
};

/**
 * @brief 局部东-北-天速度。
 */
struct ONEQ_API EnuVelocityMps {
  double east_mps{0.0};  /**< East 方向速度（单位：m/s） */
  double north_mps{0.0}; /**< North 方向速度（单位：m/s） */
  double up_mps{0.0};    /**< Up 方向速度（单位：m/s） */
  EnuVelocityMps() = default;
  EnuVelocityMps(double east, double north, double up)
      : east_mps(east), north_mps(north), up_mps(up) {}
};

/**
 * @brief 局部北-东-地速度。
 */
struct ONEQ_API NedVelocityMps {
  double north_mps{0.0}; /**< North 方向速度（单位：m/s） */
  double east_mps{0.0};  /**< East 方向速度（单位：m/s） */
  double down_mps{0.0};  /**< Down 方向速度（单位：m/s） */
  NedVelocityMps() = default;
  NedVelocityMps(double north, double east, double down)
      : north_mps(north), east_mps(east), down_mps(down) {}
};

/**
 * @brief 局部北-天-东速度。
 */
struct ONEQ_API NueVelocityMps {
  double north_mps{0.0}; /**< North 方向速度（单位：m/s） */
  double up_mps{0.0};    /**< Up 方向速度（单位：m/s） */
  double east_mps{0.0};  /**< East 方向速度（单位：m/s） */
  NueVelocityMps() = default;
  NueVelocityMps(double north, double up, double east)
      : north_mps(north), up_mps(up), east_mps(east) {}
};

/**
 * @brief 欧拉角姿态（yaw/pitch/roll，单位：deg）。
 * @note 旋转顺序为 Z-Y-X；沿用仓库现有约定，正 pitch 表示正仰角。
 */
struct ONEQ_API EulerAnglesDeg {
  double yaw_deg{0.0};   /**< 偏航角（单位：deg） */
  double pitch_deg{0.0}; /**< 俯仰角（单位：deg） */
  double roll_deg{0.0};  /**< 横滚角（单位：deg） */
  EulerAnglesDeg() = default;
  EulerAnglesDeg(double yaw, double pitch, double roll)
      : yaw_deg(yaw), pitch_deg(pitch), roll_deg(roll) {}
};

/**
 * @brief 三维向量（double 精度）。
 * @note 用于坐标转换中间结果；三维向量（double 精度）。
 */
struct ONEQ_API Vector3d {
  double x{0.0}; /**< X 分量 */
  double y{0.0}; /**< Y 分量 */
  double z{0.0}; /**< Z 分量 */
  Vector3d() = default;
  Vector3d(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

/**
 * @brief 局部坐标系参考定义。
 * @note 各传感器模块（AR/EOS/ESR）直接复用本类型作为局部坐标参考系定义。
 */
struct ONEQ_API LocalFrameReference {
  LlaPositionDegM origin_lla{};        /**< 局部坐标参考原点 */
  EulerAnglesDeg frame_attitude_deg{}; /**< 局部坐标系相对 ENU 的姿态角 */
};

/**
 * @brief 外部位置参考系类型。
 */
enum class PositionFrame {
  kEcef = 0, /**< 位置由 ECEF 坐标给出 */
  kLla = 1   /**< 位置由 WGS84 LLA 坐标给出 */
};

/**
 * @brief 外部运动学输入结构体。
 * @note 仅 `position_frame` 对应的位置字段被读取：`kEcef` 时读取 `position_ecef_m`，
 *       `kLla` 时读取 `position_lla_deg_m`。调用方应只填写与 `position_frame` 匹配的
 *       位置字段，另一个位置字段的值会被忽略。
 * @note `velocity_mps` 始终是 ECEF 速度；即使 `position_frame==kLla`，也不要直接填
 *       ENU/NED 速度。若输入速度来自局部 ENU，请先用 `TryEnuToEcefVelocity()` 转换。
 */
struct ONEQ_API ExternalKinematics {
  PositionFrame position_frame{PositionFrame::kEcef}; /**< 位置参考系 */
  EcefPositionM position_ecef_m{};      /**< ECEF 位置（position_frame==kEcef 时使用） */
  LlaPositionDegM position_lla_deg_m{}; /**< LLA 位置（position_frame==kLla 时使用） */
  EcefVelocityMps velocity_mps{};       /**< 速度（ECEF，m/s） */
  EulerAnglesDeg attitude_deg{};        /**< 姿态角（Body->ENU，deg） */
};

}  // namespace coordinate
}  // namespace oneq

#endif  // ONEQ_COORDINATE_TYPES_H_
