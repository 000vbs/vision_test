cmake_minimum_required(VERSION 3.10)
project(VisionAssessment)

set(CMAKE_CXX_STANDARD 17)


# OpenCV 依赖
find_package(OpenCV REQUIRED)

# Eigen3 依赖
find_package(Eigen3 REQUIRED)

# OpenVINO 依赖
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")
find_package(OpenVINO REQUIRED COMPONENTS Runtime)

# fmt 依赖
find_package(fmt REQUIRED)

# yaml-cpp 依赖
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
    fmt::fmt
    yaml-cpp
)
