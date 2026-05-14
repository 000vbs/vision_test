#include "solver.hpp"

#include <yaml-cpp/yaml.h>

#include <opencv2/core/mat.hpp>

namespace auto_aim
{

Solver::Solver(const std::string & config_path)
{
  // 从 yaml 中读取相机内参与畸变系数
  auto yaml = YAML::LoadFile(config_path);
  auto k = yaml["camera_matrix"].as<std::vector<double>>();
  auto d = yaml["distort_coeffs"].as<std::vector<double>>();

  camera_matrix_ = (cv::Mat_<double>(3, 3) <<
    k[0], k[1], k[2],
    k[3], k[4], k[5],
    k[6], k[7], k[8]);

  distort_coeffs_ = cv::Mat(d, true).clone().reshape(1, 1);
}

void Solver::solve(Armor & armor) const
{
  // 装甲板物理尺寸 (单位: m)
  // 小装甲板: 135mm x 55mm; 大装甲板: 225mm x 55mm
  constexpr double small_w = 0.135, small_h = 0.055;
  constexpr double big_w = 0.225, big_h = 0.055;
  const double w = (armor.type == ArmorType::big) ? big_w : small_w;
  const double h = (armor.type == ArmorType::big) ? big_h : small_h;

  // 与 Armor 构造里 keypoints 的顺序保持一致: TL, TR, BR, BL
  const std::vector<cv::Point3f> object_points = {
    {static_cast<float>(-w / 2), static_cast<float>(-h / 2), 0.f},
    {static_cast<float>( w / 2), static_cast<float>(-h / 2), 0.f},
    {static_cast<float>( w / 2), static_cast<float>( h / 2), 0.f},
    {static_cast<float>(-w / 2), static_cast<float>( h / 2), 0.f},
  };

  cv::Mat rvec, tvec;
  bool success = cv::solvePnP(
    object_points, armor.points, camera_matrix_, distort_coeffs_,
    rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
  if (!success) return;

  armor.xyz_in_gimbal = Eigen::Vector3d(
    tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
}

}  // namespace auto_aim
