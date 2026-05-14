# 视觉组二轮考核自瞄部分

## 任务背景
在今年的视觉算法架构中，我们抛弃了传统视觉以及 ROS2 架构，采取了 **纯 C++ 框架 + 神经网络识别** 的体系以求性能上的极限以及观测的稳定。
考虑到你们对当前的软硬件工具可能还不熟悉，本次考核重点考察：
1. 阅读已有的 C++ 代码（了解其类的方法和参数要求）
2. 通过已有 YOLOv5 类获得识别出来的装甲板的 2D 坐标
3. 将获得的 2D 坐标使用 PnP 解算出当前识别到的装甲板在相机坐标系下的 3D 坐标并显示在视频上
4. （进阶）设计解算逻辑预测目标弹道

## 环境要求
- Ubuntu 环境（推荐双系统）
1. 安装依赖项：
   - [OpenVINO](https://docs.openvino.ai/2026/get-started/install-openvino.html?PACKAGE=OPENVINO_BASE&VERSION=v_2026_1_0&OP_SYSTEM=LINUX&DISTRIBUTION=APT)
   - 其余：
    ```bash
    sudo apt install -y \
        git \
        g++ \
        cmake \
        libopencv-dev \
        libfmt-dev \
        libeigen3-dev \
        libspdlog-dev \
        libyaml-cpp-dev \
    ```

2. 编译：
    ```bash
    cmake -B build
    make -C build/ -j`nproc`
    ```

## 基础任务 (必做)
本框架已经内置了上一代项目里稳定运行的 `YOLOv5` 包装类（见 `include/yolov5.hpp`）以及基础的 `Armor` 装甲板结构体。
1. **模型推理与视频读取**
   * 修改 `src/main.cpp` 中的 TODO 部分。
   * 使用 OpenCV 或合适的库加载视频 `assets/infantry.mp4`。
   * 你需要通过查阅 `yolov5.hpp` 函数签名，了解如何初始化此类以及如何调用该类的成员函数。
   * 把当前画面的每一帧提取出的装甲板通过**YOLO自带的方法**或者自己手写方法绘制出来（直接 `imshow`）。
2. **PnP 位姿解算 (2D -> 3D 转换)**
   * 阅读 `include/solver.hpp` 文件，补全 `src/solver.cpp` 里的函数实现。
   * 从 `configs/infantry.yaml` 读取相机内参 `camera_matrix` 和畸变系数 `distort_coeffs`。
   * 提取装甲板 2D 四点，并根据 `ArmorType`（大/小装甲板规格）来映射 3D 物理参考点的定义，最终使用 `cv::solvePnP` 求出平移向量 `tvec` 进而算出距离相机的三维坐标 (x,y,z)。

## 进阶任务 (选做)
1. **纯自研目标预测 (弹道解算 `aimer`)**
   * 在基础任务中你已经获得了当前帧中装甲板的三维坐标位置。但由于子弹飞行有时间，需要提前向目标未来出现的位置开火。
   * 本任务纯开放，没有预设的文件：你需要在此项目中自主设计或添加一个 `Aimer` 类。
   * 目标是通过该类输出目标的 yaw 与 pitch 角度，这个角度是实际发给下位机的需要调整的云台角度，因此推荐做坐标转换，也就是将相机坐标系下的三维坐标转换到云台坐标系下（云台坐标系与相机坐标系有一个固定的旋转和平移关系，这两个关系具体在 `configs/infantry.yaml` 中定义：R_camera2gimbal 和 t_camera2gimbal），然后根据云台坐标系下的目标位置计算出需要调整的 yaw 和 pitch 角度。规定的云台坐标系定义为X向前，Y向左，Z向上。
   * 这个类需要接收时间戳和三维坐标系作为输入信号，自己制定一套基于扩展卡尔曼滤波 (EKF) 或是其他纯数学一阶/二阶导数预测的方法。如果预测暂时做不到也可以直接用当前目标的状态输出目标角度。

## 注意事项
assets 文件夹里提供了测试视频 `infantry.avi` 和模型文件 `0526.xml` 等。
PnP 解算需要的相机内参和坐标转换关系已经在 `configs/infantry.yaml` 中提供。

如果题目仍有缺漏请联系我们。祝你好运！
