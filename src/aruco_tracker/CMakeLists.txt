cmake_minimum_required(VERSION 3.5)
project(aruco_tracker)

add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${Eigen3_INCLUDE_DIRS})

add_executable(aruco_tracker ArucoTracker.cpp)

ament_target_dependencies(aruco_tracker Eigen3 rclcpp px4_msgs sensor_msgs cv_bridge OpenCV)

target_compile_features(aruco_tracker PUBLIC c_std_99 cxx_std_17)

install(TARGETS aruco_tracker DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
