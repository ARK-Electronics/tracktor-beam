#pragma once
#include <array>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class ArucoTrackerNode : public rclcpp::Node
{
public:
	// Constructor
	ArucoTrackerNode();

private:
	// Callbacks
	void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
	void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
	// Image processing
	void annotate_image(cv_bridge::CvImagePtr image);

	// ROS2 Subscribers and Publishers
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_sub;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_sub;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_pub;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_pub;

	// Data
	std::unique_ptr<cv::aruco::ArucoDetector> _detector;
	cv::Mat _camera_matrix;
	cv::Mat _dist_coeffs;

	// State
	std::array<double, 3> _target;
	double _marker_size = {0.0};

	// Filter
	int _num_detected = 60; // Number of frames to track for each marker
    int _min_prec_value = 90; // Minimum precision percentage to consider the marker valid
    std::map<int, std::vector<int>> _ids_hashmap; // Tracking detection history for each marker
	int aruco_id;
};

