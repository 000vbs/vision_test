#include "spin_kalman.hpp"

#include <cmath>

namespace auto_aim
{

// ============================================================================
// SpinKalmanConfig 默认值
// ============================================================================

SpinKalmanConfig::SpinKalmanConfig()
: q_pos(1e-4)
, q_vel(1.0)
, q_theta(1e-2)
, q_omega(50.0)
, r_pos(1e-4)
, init_cov_pos(1e-2)
, init_cov_vel(1.0)
, init_cov_theta(1.0)
, init_cov_omega(100.0)
, mahalanobis_threshold(4.0)
, max_omega(30.0)
, max_dt(0.5)
{
  // 默认步兵装甲板几何参数 (角偏移从 x 正半轴开始, 逆时针)
  // one:   正前方偏右  ~45°
  // two:   左侧       ~135°
  // three: 正后方      ~225° (即 -135°)
  // four:  右侧       ~315° (即 -45°)

  armor_geom[ArmorName::one]   = {0.22,  0.7854,  0.01};   // r≈0.22m, yaw+45°, h+1cm
  armor_geom[ArmorName::two]   = {0.22,  2.3562,  0.01};   // yaw+135°
  armor_geom[ArmorName::three] = {0.22, -2.3562, -0.01};   // yaw-135°
  armor_geom[ArmorName::four]  = {0.22, -0.7854, -0.01};   // yaw-45°

  // sentry / outpost / base 直接用车辆中心 (无旋转偏移), 兼容非步兵目标
  armor_geom[ArmorName::sentry]  = {0.0, 0.0, 0.0};
  armor_geom[ArmorName::outpost] = {0.0, 0.0, 0.0};
  armor_geom[ArmorName::base]    = {0.0, 0.0, 0.0};
  armor_geom[ArmorName::five]    = {0.0, 0.0, 0.0};
  armor_geom[ArmorName::not_armor] = {0.0, 0.0, 0.0};
}

// ============================================================================
// SpinKalman
// ============================================================================

SpinKalman::SpinKalman(const SpinKalmanConfig & config)
: config_(config)
, state_(StateVec::Zero())
, cov_(StateCov::Identity())
, last_timestamp_(0.0)
, initialized_(false)
{
}

void SpinKalman::init(const Eigen::Vector3d & z, ArmorName armor_id, double timestamp)
{
  auto it = config_.armor_geom.find(armor_id);
  if (it == config_.armor_geom.end()) return;
  const auto & geom = it->second;

  // 反推车辆中心: 从观测位置减去装甲板偏移
  // 注意: 第一次无法知道 theta，我们将 theta 初始化为能使观测成立的相位
  double init_theta = 0.0;
  if (geom.radius > 1e-6) {
    init_theta = std::atan2(z.y(), z.x()) - geom.angle_offset;
    // 归一到 [-pi, pi]
    while (init_theta >  M_PI) init_theta -= 2.0 * M_PI;
    while (init_theta < -M_PI) init_theta += 2.0 * M_PI;
  }

  state_.setZero();
  state_(0) = z.x() - geom.radius * std::cos(init_theta + geom.angle_offset);
  state_(1) = z.y() - geom.radius * std::sin(init_theta + geom.angle_offset);
  state_(2) = z.z() - geom.height_offset;
  state_(3) = 0.0;  // vx
  state_(4) = 0.0;  // vy
  state_(5) = 0.0;  // vz
  state_(6) = init_theta;
  state_(7) = 0.0;  // omega

  // 初始协方差
  cov_.setIdentity();
  cov_(0, 0) = config_.init_cov_pos;
  cov_(1, 1) = config_.init_cov_pos;
  cov_(2, 2) = config_.init_cov_pos;
  cov_(3, 3) = config_.init_cov_vel;
  cov_(4, 4) = config_.init_cov_vel;
  cov_(5, 5) = config_.init_cov_vel;
  cov_(6, 6) = config_.init_cov_theta;
  cov_(7, 7) = config_.init_cov_omega;

  last_timestamp_ = timestamp;
  initialized_ = true;
}

void SpinKalman::update(const Eigen::Vector3d & z, ArmorName armor_id, double timestamp)
{
  if (!initialized_) {
    init(z, armor_id, timestamp);
    return;
  }

  auto it = config_.armor_geom.find(armor_id);
  if (it == config_.armor_geom.end()) return;
  const auto & geom = it->second;

  double dt = timestamp - last_timestamp_;

  // 超时重置
  if (dt <= 0.0 || dt > config_.max_dt) {
    init(z, armor_id, timestamp);
    return;
  }

  // ===== EKF Predict =====
  predict_state(dt);

  Eigen::Matrix<double, 8, 8> F = transition_jacobian(dt);
  Eigen::Matrix<double, 8, 8> Q = process_noise_cov(dt);
  cov_ = F * cov_ * F.transpose() + Q;

  // ===== 异常值检测 (马氏距离) =====
  Eigen::Vector3d z_pred = expected_observation(geom);
  Eigen::Vector3d innovation = z - z_pred;

  Eigen::Matrix<double, 3, 8> H = observation_jacobian(state_(6), geom);
  Eigen::Matrix<double, 3, 3> R = measurement_noise_cov();
  Eigen::Matrix<double, 3, 3> S = H * cov_ * H.transpose() + R;

  double mahalanobis = std::sqrt(
    (innovation.transpose() * S.inverse() * innovation)(0));

  // 角速度合理性检查
  if (std::abs(state_(7)) > config_.max_omega) {
    // 角速度异常, 可能是误检测或切换到其他目标, 软重置
    init(z, armor_id, timestamp);
    return;
  }

  // 如果是合理观测则执行更新; 异常观测仅做 predict (跳过 update)
  if (mahalanobis < config_.mahalanobis_threshold) {
    // ===== EKF Update =====
    Eigen::Matrix<double, 8, 3> K = cov_ * H.transpose() * S.inverse();
    state_ += K * innovation;

    Eigen::Matrix<double, 8, 8> I = Eigen::Matrix<double, 8, 8>::Identity();
    cov_ = (I - K * H) * cov_;
  }

  // 角度归一化
  while (state_(6) >  M_PI) state_(6) -= 2.0 * M_PI;
  while (state_(6) < -M_PI) state_(6) += 2.0 * M_PI;

  last_timestamp_ = timestamp;
}

Eigen::Vector3d SpinKalman::predict(double fly_time, ArmorName armor_id) const
{
  if (!initialized_) return state_.head<3>();

  auto it = config_.armor_geom.find(armor_id);
  if (it == config_.armor_geom.end()) return state_.head<3>();
  const auto & geom = it->second;

  // 预测车辆中心
  double cx_pred = state_(0) + state_(3) * fly_time;
  double cy_pred = state_(1) + state_(4) * fly_time;
  double cz_pred = state_(2) + state_(5) * fly_time;

  // 预测旋转角
  double theta_pred = state_(6) + state_(7) * fly_time;

  // 装甲板在预测时刻的位置 = 车辆中心 + 旋转偏移
  Eigen::Vector3d result;
  result.x() = cx_pred + geom.radius * std::cos(theta_pred + geom.angle_offset);
  result.y() = cy_pred + geom.radius * std::sin(theta_pred + geom.angle_offset);
  result.z() = cz_pred + geom.height_offset;

  return result;
}

// ============================================================================
// Private helpers
// ============================================================================

void SpinKalman::predict_state(double dt)
{
  // 位置更新
  state_(0) += state_(3) * dt;
  state_(1) += state_(4) * dt;
  state_(2) += state_(5) * dt;
  // 速度不变 (CV 模型)
  // 角度更新
  state_(6) += state_(7) * dt;
  // 角速度不变
}

Eigen::Matrix<double, 8, 8> SpinKalman::transition_jacobian(double dt)
{
  // F = I + ∂f/∂x * dt  (线性过程模型, F 就是解析形式)
  Eigen::Matrix<double, 8, 8> F = Eigen::Matrix<double, 8, 8>::Identity();
  // 位置对速度的偏导
  F(0, 3) = dt;
  F(1, 4) = dt;
  F(2, 5) = dt;
  // 角度对角速度的偏导
  F(6, 7) = dt;
  return F;
}

Eigen::Matrix<double, 3, 8> SpinKalman::observation_jacobian(
  double theta, const ArmorGeometry & geom)
{
  double r = geom.radius;
  double phi = geom.angle_offset;
  double angle = theta + phi;

  // H = ∂h/∂x, h = [cx + r*cos(θ+φ), cy + r*sin(θ+φ), cz + h_off]
  Eigen::Matrix<double, 3, 8> H = Eigen::Matrix<double, 3, 8>::Zero();

  // ∂x_obs / ∂cx = 1
  H(0, 0) = 1.0;
  // ∂x_obs / ∂θ = -r * sin(θ+φ)
  H(0, 6) = -r * std::sin(angle);

  // ∂y_obs / ∂cy = 1
  H(1, 1) = 1.0;
  // ∂y_obs / ∂θ =  r * cos(θ+φ)
  H(1, 6) = r * std::cos(angle);

  // ∂z_obs / ∂cz = 1
  H(2, 2) = 1.0;

  return H;
}

Eigen::Vector3d SpinKalman::expected_observation(const ArmorGeometry & geom) const
{
  double angle = state_(6) + geom.angle_offset;
  Eigen::Vector3d z_pred;
  z_pred.x() = state_(0) + geom.radius * std::cos(angle);
  z_pred.y() = state_(1) + geom.radius * std::sin(angle);
  z_pred.z() = state_(2) + geom.height_offset;
  return z_pred;
}

Eigen::Matrix<double, 8, 8> SpinKalman::process_noise_cov(double dt) const
{
  // 离散化过程噪声 (一阶近似)
  // Q_d ≈ G * Q_c * G^T * dt, 其中 G 是噪声映射矩阵
  //
  // 噪声驱动项:
  //   pos:   受速度噪声 + 直接位置扰动
  //   vel:   加速度白噪声
  //   theta: 受角速度噪声 + 直接角度扰动
  //   omega: 角加速度白噪声

  double dt2 = dt * dt;
  double dt3 = dt2 * dt;

  Eigen::Matrix<double, 8, 8> Q = Eigen::Matrix<double, 8, 8>::Zero();

  // 位置: x 受 vx 噪声的积分 + 直接扰动
  // Cov(cx) = q_vel * dt³/3 + q_pos * dt
  Q(0, 0) = config_.q_vel * dt3 / 3.0 + config_.q_pos * dt;
  Q(1, 1) = config_.q_vel * dt3 / 3.0 + config_.q_pos * dt;
  Q(2, 2) = config_.q_vel * dt3 / 3.0 + config_.q_pos * dt;

  // 速度: 加速度白噪声积分
  Q(3, 3) = config_.q_vel * dt;
  Q(4, 4) = config_.q_vel * dt;
  Q(5, 5) = config_.q_vel * dt;

  // pos-vel 交叉项
  Q(0, 3) = config_.q_vel * dt2 / 2.0;
  Q(3, 0) = Q(0, 3);
  Q(1, 4) = config_.q_vel * dt2 / 2.0;
  Q(4, 1) = Q(1, 4);
  Q(2, 5) = config_.q_vel * dt2 / 2.0;
  Q(5, 2) = Q(2, 5);

  // 角度
  Q(6, 6) = config_.q_omega * dt3 / 3.0 + config_.q_theta * dt;
  Q(7, 7) = config_.q_omega * dt;
  Q(6, 7) = config_.q_omega * dt2 / 2.0;
  Q(7, 6) = Q(6, 7);

  return Q;
}

Eigen::Matrix<double, 3, 3> SpinKalman::measurement_noise_cov() const
{
  return Eigen::Matrix<double, 3, 3>::Identity() * config_.r_pos;
}

}  // namespace auto_aim
