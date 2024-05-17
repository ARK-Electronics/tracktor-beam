cmake_minimum_required(VERSION 3.8)
project(tracktorbeam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# add_executable(tracktorbeam src/ArucoTracker.cpp)
# add_executable(objecttracker src/MovingObjectTracker.cpp)
# add_executable(objecttracker src/VehicleTracker.cpp)
add_executable(videoplayback src/VideoPlayback.cpp)
add_executable(objecttracker src/VehicleTrackerYOLO.cpp)

ament_target_dependencies(objecttracker
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

ament_target_dependencies(videoplayback
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# Include directories for each target
target_include_directories(objecttracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(videoplayback PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Set compile features for each target
target_compile_features(objecttracker PUBLIC c_std_99 cxx_std_17)
target_compile_features(videoplayback PUBLIC c_std_99 cxx_std_17)

install(TARGETS objecttracker videoplayback
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
