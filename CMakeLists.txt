cmake_minimum_required(VERSION 3.8)
project(omnitrack)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Define the executable
add_executable(omnitrack src/Tracker.cpp)

# Specify dependencies to link against
ament_target_dependencies(omnitrack
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

target_include_directories(omnitrack PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(omnitrack PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS omnitrack
  DESTINATION lib/${PROJECT_NAME})

# Testing and linting setup (optional, but recommended for quality code)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files, config files, etc. (if you have any)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
