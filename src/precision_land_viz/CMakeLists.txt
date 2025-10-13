cmake_minimum_required(VERSION 3.5)
project(precision_land_viz)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Gazebo
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
set(GZ_TRANSPORT_VER 13)
set(GZ_MSGS_VER 10)

add_executable(tag_pose_visualizer
    tag_pose_visualizer.cpp
)

ament_target_dependencies(tag_pose_visualizer rclcpp geometry_msgs)

target_link_libraries(tag_pose_visualizer
  gz-transport${GZ_TRANSPORT_VER}::core
  gz-msgs${GZ_MSGS_VER}::core
)

target_compile_features(tag_pose_visualizer PUBLIC c_std_99 cxx_std_17)

install(TARGETS tag_pose_visualizer DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
