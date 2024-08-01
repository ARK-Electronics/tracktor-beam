#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.hpp>
#include <vector>

class PrecisionLand : public px4_ros2::ModeBase
{
public:
	explicit PrecisionLand(rclcpp::Node& node);

	void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);

	// Called when the mode is activated
	void onActivate() override;
	// Called when the mode is deactivated
	void onDeactivate() override;
	// Serves as the "main loop" and is called at regular intervals. Runs the state machine.
	void updateSetpoint(float dt_s) override;

private:
	bool positionReached(const Eigen::Vector3f& target) const;

	enum class State {
		Idle,		// Safety state
		Search, 	// Searches for target -- TODO: optionally perform a search pattern
		Approach, 	// Positioning over landing target while maintaining altitude
		Descend, 	// Stay over landing target while descending
		Finished
	};
	// ROS2 Subscriptions
	rclcpp::Node& _node;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _vehicle_land_detected_sub;

	// PX4_ROS2 Modules
	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
	std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

	// State machine handling
	void switchToState(State state);
	std::string stateName(State state);

	// Search pattern generation
	void generateSearchWaypoints();

	// Data
	State _state = State::Search;
	Eigen::Vector3f _target_position = { NAN, NAN, NAN};
	float _target_heading = {};
	float _approach_altitude = {};
	rclcpp::Time _last_target_timestamp;
	bool _search_allowed;
	std::vector<Eigen::Vector3f> _search_waypoints;
	int _search_waypoint_index = 0;
	bool _land_detected = false;
	bool _target_lost = false;
	bool _target_lost_prev = false;
};
