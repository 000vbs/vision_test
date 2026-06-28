#ifndef AUTO_AIM__AIMER_HPP
#define AUTO_AIM__AIMER_HPP

#include <Eigen/Dense>

#include <memory>
#include <string>

#include "armor.hpp"
#include "spin_kalman.hpp"

namespace auto_aim
{

class Aimer
{
public:
  struct Target
  {
    double yaw;    // rad
    double pitch;  // rad
    Eigen::Vector3d xyz_in_gimbal;  // 单位: m
  };

  explicit Aimer(const std::string & config_path);

  /// 输入: 相机系下目标 3D 坐标 + 装甲板 ID + 时间戳 (s)
  /// 输出: 云台需要调整到的 yaw / pitch 角度
  Target aim(
    const Eigen::Vector3d & xyz_in_camera, ArmorName armor_name, double timestamp);

  /// 获取 Kalman 滤波器, 用于外部查询状态 (如 omega)
  const SpinKalman & kalman() const { return *kalman_; }

private:
  Eigen::Matrix3d R_camera2gimbal_;
  Eigen::Vector3d t_camera2gimbal_;

  double projectile_speed_;  ///< 弹丸速度 [m/s]

  // 简单差分退化 (作为 Kalman 的 fallback, 保留以兼容非步兵目标)
  bool diff_initialized_;
  double diff_last_timestamp_;
  Eigen::Vector3d diff_last_xyz_in_gimbal_;
  Eigen::Vector3d diff_velocity_in_gimbal_;

  std::unique_ptr<SpinKalman> kalman_;
  SpinKalmanConfig kalman_config_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP
