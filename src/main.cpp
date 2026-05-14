#include <iostream>
#include <opencv2/opencv.hpp>

#include "aimer.hpp"
#include "solver.hpp"
#include "yolov5.hpp"

using namespace auto_aim;
using namespace std;

int main(int argc, char** argv) {
  // 1. 初始化模型与求解器
  string model_path = "assets/0526.onnx";   // 或是0526.xml
  string config_path = "configs/infantry.yaml";
  string video_path = "assets/infantry.avi";

  // TODO: 初始化 YOLOv5 类
  YOLOV5 yolov5(config_path, false);
  // TODO: 初始化 Solver 类
  Solver solver(config_path);
  Aimer aimer(config_path);
  cv::VideoCapture cap(video_path);
  if (!cap.isOpened()) {
    cerr << "Failed to open video: " << video_path << endl;
    return -1;
  }

  cv::Mat frame;
  int frame_count = 0; // 修复2：补充detect函数必须的帧序号参数
  double fps = cap.get(cv::CAP_PROP_FPS);
  if (fps <= 1.0) fps = 30.0;
  while (cap.read(frame)) {
    double timestamp = frame_count / fps;
    // 2. 调用模型识别
    // TODO: 使用 yolov5 提取 armors
    auto armors = yolov5.detect(frame, frame_count);
    // 3. 将装甲板画出来并求解 PnP
    for (auto & armor : armors) {
      // TODO: 使用 solver.solve(armor) 进行位姿解算得到相对距离
      solver.solve(armor);
      auto target = aimer.aim(armor.xyz_in_gimbal, timestamp);

      // TODO: 在图像上绘制装甲板（或者利用 yolo 类自带的画图函数）并可将距离通过 cv::putText 写在图像上
      std::vector<cv::Point> poly;
      poly.reserve(armor.points.size());
      for (const auto & p : armor.points) poly.emplace_back(cv::Point(p));
      cv::polylines(frame, poly, true, cv::Scalar(0, 255, 0), 2);

      double distance = armor.xyz_in_gimbal.norm();
      auto info = cv::format(
        "%s %.2fm yaw=%.1f pitch=%.1f",
        ARMOR_NAMES[armor.name].c_str(), distance,
        target.yaw * 180.0 / CV_PI, target.pitch * 180.0 / CV_PI);
      cv::putText(frame, info, armor.center, cv::FONT_HERSHEY_SIMPLEX,
                  0.6, cv::Scalar(0, 255, 255), 2);
    }

    cv::imshow("Vision Assessment", frame);
    if (cv::waitKey(1) == 27) { // ESC 退出
      break;
    }
    frame_count++;
  }

  return 0;
}
