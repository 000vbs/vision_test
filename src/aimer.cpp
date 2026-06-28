#include "aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <vector>

namespace auto_aim
{

namespace
{
/// 从 yaml node 读取 SpinKalman 配置, 未提供则使用默认值
SpinKalmanConfig load_kalman_config(const YAML::Node & yaml)
{
  SpinKalmanConfig cfg;

  if (yaml["spin_kalman"]) {
    auto sk = yaml["spin_kalman"];
    if (sk["q_pos"]) cfg.q_pos = sk["q_pos"].as<double>();
    if (sk["q_vel"]) cfg.q_vel = sk["q_vel"].as<double>();
    if (sk["q_theta"]) cfg.q_theta = sk["q_theta"].as<double>();
    if (sk["q_omega"]) cfg.q_omega = sk["q_omega"].as<double>();
    if (sk["r_pos"]) cfg.r_pos = sk["r_pos"].as<double>();
    if (sk["init_cov_pos"]) cfg.init_cov_pos = sk["init_cov_pos"].as<double>();
    if (sk["init_cov_vel"]) cfg.init_cov_vel = sk["init_cov_vel"].as<double>();
    if (sk["init_cov_theta"]) cfg.init_cov_theta = sk["init_cov_theta"].as<double>();
    if (sk["init_cov_omega"]) cfg.init_cov_omega = sk["init_cov_omega"].as<double>();
    if (sk["mahalanobis_threshold"]) cfg.mahalanobis_threshold = sk["mahalanobis_threshold"].as<double>();
    if (sk["max_omega"]) cfg.max_omega = sk["max_omega"].as<double>();
    if (sk["max_dt"]) cfg.max_dt = sk["max_dt"].as<double>();
  }

  // 读取装甲板几何参数
  if (yaml["armor_geometry"]) {
    for (const auto & kv : yaml["armor_geometry"]) {
      std::string name_str = kv.first.as<std::string>();
      ArmorName name = ArmorName::not_armor;
      if (name_str == "one") name = ArmorName::one;
      else if (name_str == "two") name = ArmorName::two;
      else if (name_str == "three") name = ArmorName::three;
      else if (name_str == "four") name = ArmorName::four;
      else if (name_str == "five") name = ArmorName::five;
      else if (name_str == "sentry") name = ArmorName::sentry;
      else if (name_str == "outpost") name = ArmorName::outpost;
      else if (name_str == "base") name = ArmorName::base;
      else continue;

      auto & geom = cfg.armor_geom[name];
      if (kv.second["radius"]) geom.radius = kv.second["radius"].as<double>();
      if (kv.second["angle_offset"])
        geom.angle_offset = kv.second["angle_offset"].as<double>();
      if (kv.second["height_offset"])
        geom.height_offset = kv.second["height_offset"].as<double>();
    }
  }

  return cfg;
}
}  // namespace

Aimer::Aimer(const std::string & config_path)
: diff_initialized_(false), diff_last_timestamp_(0.0),
  diff_last_xyz_in_gimbal_(Eigen::Vector3d::Zero()),
  diff_velocity_in_gimbal_(Eigen::Vector3d::Zero())
{
  auto yaml = YAML::LoadFile(config_path);
  auto r = yaml["R_camera2gimbal"].as<std::vector<double>>();
  auto t = yaml["t_camera2gimbal"].as<std::vector<double>>();

  R_camera2gimbal_ << r[0], r[1], r[2],
                      r[3], r[4], r[5],
                      r[6], r[7], r[8];
  t_camera2gimbal_ << t[0], t[1], t[2];

  // 弹丸速度 (默认 15.0 m/s, 适配 RoboMaster 17mm 弹丸)
  projectile_speed_ = yaml["projectile_speed"]
                        ? yaml["projectile_speed"].as<double>()
                        : 15.0;

  // 初始化 Kalman 滤波器
  kalman_config_ = load_kalman_config(yaml);
  kalman_ = std::make_unique<SpinKalman>(kalman_config_);
}

Aimer::Target Aimer::aim(
  const Eigen::Vector3d & xyz_in_camera, ArmorName armor_name, double timestamp)
{
  // 相机系 -> 云台系 (X 向前, Y 向左, Z 向上)
  Eigen::Vector3d xyz_in_gimbal =
    R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;

  // ===== Kalman 滤波 + 预测 =====
  kalman_->update(xyz_in_gimbal, armor_name, timestamp);

  // 计算弹丸飞行时间
  double distance = kalman_->vehicle_center().norm();
  double fly_time = distance / projectile_speed_;

  // 预测弹丸到达时刻的目标位置
  Eigen::Vector3d predicted = kalman_->predict(fly_time, armor_name);

  Target target;
  target.xyz_in_gimbal = predicted;
  target.yaw = std::atan2(predicted.y(), predicted.x());
  target.pitch =
    std::atan2(predicted.z(), std::hypot(predicted.x(), predicted.y()));
  return target;
}

}  // namespace auto_aim
