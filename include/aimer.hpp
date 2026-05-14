#ifndef AUTO_AIM__AIMER_HPP
#define AUTO_AIM__AIMER_HPP

#include <Eigen/Dense>
#include <string>

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

  // 输入: 相机系下目标 3D 坐标 + 时间戳 (s)
  // 输出: 云台需要调整到的 yaw / pitch 角度
  Target aim(const Eigen::Vector3d & xyz_in_camera, double timestamp);

private:
  Eigen::Matrix3d R_camera2gimbal_;
  Eigen::Vector3d t_camera2gimbal_;

  bool initialized_;
  double last_timestamp_;
  Eigen::Vector3d last_xyz_in_gimbal_;
  Eigen::Vector3d velocity_in_gimbal_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP
