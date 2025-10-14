#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/gimbal_manager_set_attitude.hpp>
#include <px4_msgs/msg/gimbal_device_attitude_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class GimbalControllerNode : public rclcpp::Node
{
public:
	GimbalControllerNode();

private:
	void loadParameters();

	// Callbacks
	void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void gimbal_status_callback(const px4_msgs::msg::GimbalDeviceAttitudeStatus::SharedPtr msg);

	// Control functions
	void controlLoop();
	void calculateGimbalAttitude(const Eigen::Vector3d& target_position);
	Eigen::Quaterniond targetPositionToGimbalQuaternion(const Eigen::Vector3d& target_position);
	void acquireGimbalControl();

	// Subscribers
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_sub;
	rclcpp::Subscription<px4_msgs::msg::GimbalDeviceAttitudeStatus>::SharedPtr _gimbal_status_sub;

	// Publishers
	rclcpp::Publisher<px4_msgs::msg::GimbalManagerSetAttitude>::SharedPtr _gimbal_cmd_pub;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_pub;

	// Timer for periodic updates
	rclcpp::TimerBase::SharedPtr _timer;

	// State
	Eigen::Vector3d _last_target_position;
	Eigen::Quaterniond _current_gimbal_attitude;
	bool _has_target {false};
	bool _has_control {false};
	uint64_t _last_target_time_us {0};
	rclcpp::Time _last_target_ros_time;

	// Parameters
	double _param_control_rate {20.0};          // Hz
	double _param_timeout_ms {1000.0};          // ms - target timeout
	double _param_pitch_offset_deg {0.0};       // deg - pitch offset
	double _param_yaw_offset_deg {0.0};         // deg - yaw offset
	uint8_t _param_gimbal_device_id {0};        // gimbal device ID
	bool _param_roll_lock {true};               // lock roll axis
	bool _param_pitch_lock {true};              // lock pitch axis
	bool _param_yaw_lock {false};               // lock yaw axis
};
