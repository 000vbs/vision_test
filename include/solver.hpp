#ifndef AUTO_AIM__SOLVER_HPP
#define AUTO_AIM__SOLVER_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <string>

#include "armor.hpp"

namespace auto_aim
{
class Solver
{
public:
  explicit Solver(const std::string & config_path);

  // 核心解算函数，要求在此内部实现 PnP 算法，并将结果写入 armor 中（如 xyz_in_world 或 xyz_in_gimbal）
  void solve(Armor & armor) const;

private:
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  
  // TODO: 可以根据需要添加辅助函数，例如从 yaml 读取内参
};

}  // namespace auto_aim
#endif
