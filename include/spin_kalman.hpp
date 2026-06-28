#ifndef AUTO_AIM__SPIN_KALMAN_HPP
#define AUTO_AIM__SPIN_KALMAN_HPP

#include <Eigen/Dense>

#include <map>

#include "armor.hpp"

namespace auto_aim
{

/// 单个装甲板在车体坐标系下的几何参数
struct ArmorGeometry
{
  double radius;        ///< 装甲板到车辆中心的水平距离 (m)
  double angle_offset;  ///< 装甲板在车体上的角度偏移 (rad)
  double height_offset; ///< 装甲板相对车辆中心的垂直偏移 (m)
};

/// 小陀螺 EKF 的配置参数
struct SpinKalmanConfig
{
  // -- 过程噪声 (连续时间谱密度, 单位与状态量一致) --
  double q_pos;     ///< 位置过程噪声 (m²/s)
  double q_vel;     ///< 速度过程噪声 ((m/s)²/s) — 建模加速扰动
  double q_theta;   ///< 角度过程噪声 (rad²/s)
  double q_omega;   ///< 角速度过程噪声 ((rad/s)²/s) — 建模角加速扰动

  // -- 测量噪声 --
  double r_pos;     ///< 位置测量噪声方差 (m²)

  // -- 初始协方差 --
  double init_cov_pos;     ///< 位置初始方差
  double init_cov_vel;     ///< 速度初始方差
  double init_cov_theta;   ///< 角度初始方差
  double init_cov_omega;   ///< 角速度初始方差

  // -- 异常值检测 --
  double mahalanobis_threshold;  ///< 马氏距离阈值 (典型值 3.0 ~ 5.0)
  double max_omega;              ///< 最大合理角速度 (rad/s)，用于初始化和异常检测
  double max_dt;                 ///< 最大允许的时间间隔 (s)，超过则重置滤波器

  // -- 装甲板几何参数 --
  std::map<ArmorName, ArmorGeometry> armor_geom;

  SpinKalmanConfig();
};

/// 小陀螺整车观测扩展卡尔曼滤波器
///
/// 状态向量 (8D): [cx, cy, cz, vx, vy, vz, theta, omega]
///   - cx,cy,cz: 车辆中心在云台系下的坐标 (m)
///   - vx,vy,vz: 车辆速度 (m/s)
///   - theta:    车辆旋转相位角 (rad)
///   - omega:    车辆旋转角速度 (rad/s)
///
/// 观测模型 (非线性):
///   x_obs = cx + r_i * cos(theta + phi_i)
///   y_obs = cy + r_i * sin(theta + phi_i)
///   z_obs = cz + h_i
///
/// 使用解析雅可比的标准 EKF predict/update 循环。

class SpinKalman
{
public:
  /// 8 维状态向量类型
  using StateVec = Eigen::Matrix<double, 8, 1>;
  /// 8×8 协方差矩阵类型
  using StateCov = Eigen::Matrix<double, 8, 8>;

  explicit SpinKalman(const SpinKalmanConfig & config);

  /// 是否已初始化
  bool initialized() const { return initialized_; }

  /// 用第一帧观测初始化滤波器
  /// @param z        观测: 装甲板 3D 坐标 (云台系) [m]
  /// @param armor_id 哪块装甲板
  /// @param timestamp 观测时间戳 [s]
  void init(const Eigen::Vector3d & z, ArmorName armor_id, double timestamp);

  /// 投入一次观测, 执行完整的 EKF predict + update 循环
  /// @param z        观测: 装甲板 3D 坐标 (云台系) [m]
  /// @param armor_id 哪块装甲板
  /// @param timestamp 观测时间戳 [s]
  void update(const Eigen::Vector3d & z, ArmorName armor_id, double timestamp);

  /// 外推预测: 给定弹丸飞行时间, 返回该装甲板在预测时刻的 3D 位置 (云台系) [m]
  /// @param fly_time   弹丸飞行时间 [s]
  /// @param armor_id   目标装甲板
  Eigen::Vector3d predict(double fly_time, ArmorName armor_id) const;

  /// 获取当前状态向量
  const StateVec & state() const { return state_; }

  /// 获取当前车辆中心 (cx, cy, cz) [m]
  Eigen::Vector3d vehicle_center() const { return state_.head<3>(); }

  /// 获取当前估计的角速度 [rad/s]
  double omega() const { return state_(7); }

private:
  SpinKalmanConfig config_;
  StateVec state_;
  StateCov cov_;
  double last_timestamp_;
  bool initialized_;

  // ---- 内部辅助 ----

  /// 过程模型: 将状态向前传播 dt 秒 (直接修改 state_)
  void predict_state(double dt);

  /// 过程模型雅可比 (状态转移矩阵 F, 8×8)
  static Eigen::Matrix<double, 8, 8> transition_jacobian(double dt);

  /// 观测模型雅可比 (H, 3×8)
  /// @param theta  当前车辆旋转角
  /// @param geom   目标装甲板几何参数
  static Eigen::Matrix<double, 3, 8> observation_jacobian(
    double theta, const ArmorGeometry & geom);

  /// 观测模型: 从当前状态和装甲板几何计算期望观测
  Eigen::Vector3d expected_observation(const ArmorGeometry & geom) const;

  /// 过程噪声协方差 Q (离散化, 8×8)
  Eigen::Matrix<double, 8, 8> process_noise_cov(double dt) const;

  /// 测量噪声协方差 R (3×3)
  Eigen::Matrix<double, 3, 3> measurement_noise_cov() const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__SPIN_KALMAN_HPP
