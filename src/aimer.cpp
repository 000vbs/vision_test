#include "aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>

namespace auto_aim
{

Aimer::Aimer(const std::string & config_path)
: initialized_(false), last_timestamp_(0.0),
  last_xyz_in_gimbal_(Eigen::Vector3d::Zero()),
  velocity_in_gimbal_(Eigen::Vector3d::Zero())
{
  auto yaml = YAML::LoadFile(config_path);
  auto r = yaml["R_camera2gimbal"].as<std::vector<double>>();
  auto t = yaml["t_camera2gimbal"].as<std::vector<double>>();

  R_camera2gimbal_ << r[0], r[1], r[2],
                      r[3], r[4], r[5],
                      r[6], r[7], r[8];
  t_camera2gimbal_ << t[0], t[1], t[2];
}

Aimer::Target Aimer::aim(const Eigen::Vector3d & xyz_in_camera, double timestamp)
{
  // 相机系 -> 云台系 (X 向前, Y 向左, Z 向上)
  Eigen::Vector3d xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;

  // 一阶速度估计 (相邻帧差分), 用于线性外推预测
  Eigen::Vector3d predicted = xyz_in_gimbal;
  if (initialized_) {
    double dt = timestamp - last_timestamp_;
    if (dt > 1e-6 && dt < 0.1) {
      velocity_in_gimbal_ = (xyz_in_gimbal - last_xyz_in_gimbal_) / dt;
    }
    // 简单线性预测: 提前 fly_time 估计弹丸到达时目标的位置 (此处用固定 0 表示不预测)
    constexpr double fly_time = 0.0;
    predicted = xyz_in_gimbal + velocity_in_gimbal_ * fly_time;
  }
  last_xyz_in_gimbal_ = xyz_in_gimbal;
  last_timestamp_ = timestamp;
  initialized_ = true;

  Target target;
  target.xyz_in_gimbal = predicted;
  target.yaw = std::atan2(predicted.y(), predicted.x());
  target.pitch = std::atan2(predicted.z(), std::hypot(predicted.x(), predicted.y()));
  return target;
}

}  // namespace auto_aim
