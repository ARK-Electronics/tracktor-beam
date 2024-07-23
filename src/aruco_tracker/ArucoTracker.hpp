#include <array>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

class ArucoTrackerNode : public rclcpp::Node
{
public:
	// Constructor
	ArucoTrackerNode();

private:
	// Callbacks
	void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
	void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
	void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
	// Image processing
	void annotate_image(cv_bridge::CvImagePtr image);

	// ROS2 Subscribers and Publishers
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _vehicle_local_position_sub;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_sub;

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_pub;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_pub;

	// Data
	cv::Ptr<cv::aruco::Dictionary> _dictionary;
	cv::Mat _camera_matrix;
	cv::Mat _dist_coeffs;

	// State
	float _distance_to_ground = {NAN};
	float heading = {};
	std::array<double, 3> _target;
	double _marker_size = {0.0};
};

