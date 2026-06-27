cmake_minimum_required(VERSION 3.10)
project(VisionAssessment)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# OpenCV 依赖
find_package(OpenCV REQUIRED)

# Eigen3 依赖
find_package(Eigen3 REQUIRED)

# OpenVINO 依赖
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")
find_package(OpenVINO REQUIRED COMPONENTS Runtime)

# fmt 依赖 — 强制使用系统版本 (与 spdlog 1.9.2 兼容)
# 必须在 find_package(spdlog) 之前设置，因为 spdlog 内部也依赖 fmt
set(fmt_DIR "/usr/lib/x86_64-linux-gnu/cmake/fmt" CACHE PATH "Path to system fmt (v8.x, compatible with system spdlog)")
find_package(fmt REQUIRED)

# spdlog 依赖
find_package(spdlog REQUIRED)

# yaml-cpp 依赖 — 强制使用系统版本 (避免引入 conda 的 include 路径)
set(yaml-cpp_DIR "/usr/lib/x86_64-linux-gnu/cmake/yaml-cpp" CACHE PATH "Path to system yaml-cpp")
find_package(yaml-cpp REQUIRED)

include_directories(
    include
    .
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenVINO_INCLUDE_DIRS}
)

file(GLOB SOURCES "src/*.cpp" "tools/*.cpp")

add_executable(assessment ${SOURCES})

target_link_libraries(assessment
    ${OpenCV_LIBS}
    Eigen3::Eigen
    openvino::runtime
    spdlog::spdlog
    fmt::fmt
    yaml-cpp
)
