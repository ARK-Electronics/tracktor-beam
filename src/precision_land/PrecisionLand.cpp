/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "PrecisionLand.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

static const std::string kModeName = "PrecisionLandCustom";
static const bool kEnableDebugOutput = true;

using namespace px4_ros2::literals;

PrecisionLand::PrecisionLand(rclcpp::Node& node)
	: ModeBase(node, kModeName)
	, _node(node)
{

	// Publish TrajectorySetpoint
	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

	// Subscribe to VehicleLocalPosition
	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

	// Subscribe to VehicleAttitude
	_vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);

	// Subscribe to target_pose
	_target_pose_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose",
			   rclcpp::QoS(1).best_effort(), std::bind(&PrecisionLand::targetPoseCallback, this, std::placeholders::_1));

	// Subscribe to vehicle_land_detected
	_vehicle_land_detected = _node.create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected",
	rclcpp::QoS(1).best_effort(), [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
		_land_detected = msg->landed;
	}
											      );
}

void PrecisionLand::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{

	// Last Target timestamp
	_last_target_timestamp = _node.now();

	// Aruco pose in camera frame
	geometry_msgs::msg::Pose aruco_pose;
	aruco_pose.position.x = msg->pose.position.x;
	aruco_pose.position.y = msg->pose.position.y;
	aruco_pose.position.z = msg->pose.position.z;
	aruco_pose.orientation.w = msg->pose.orientation.w;
	aruco_pose.orientation.x = msg->pose.orientation.x;
	aruco_pose.orientation.y = msg->pose.orientation.y;
	aruco_pose.orientation.z = msg->pose.orientation.z;


	// Camera pose in drone frame
	geometry_msgs::msg::Pose camera_pose;
	Eigen::Matrix3f R;
	R << 0, -1, 0,
	1, 0, 0,
	0, 0, 1;
	Eigen::Quaternionf quat(R);
	camera_pose.position.x = 0.12;// camera case and camera position 0.12
	camera_pose.position.y = 0;// camera case and camera position
	camera_pose.position.z = 0;// camera case and camera position
	camera_pose.orientation.w = quat.w();
	camera_pose.orientation.x = quat.x();
	camera_pose.orientation.y = quat.y();
	camera_pose.orientation.z = quat.z();

	// Drone pose in world frame
	geometry_msgs::msg::Pose drone_pose;
	auto vehicle_q = _vehicle_attitude->attitude();
	drone_pose.position.x = _vehicle_local_position->positionNed().x();
	drone_pose.position.y = _vehicle_local_position->positionNed().y();
	drone_pose.position.z = _vehicle_local_position->positionNed().z();
	drone_pose.orientation.w = vehicle_q.w();
	drone_pose.orientation.x = vehicle_q.x();
	drone_pose.orientation.y = vehicle_q.y();
	drone_pose.orientation.z = vehicle_q.z();

	// Convert to KDL::Frame
	KDL::Frame frame_drone, frame_camera, frame_aruco;
	tf2::fromMsg(drone_pose, frame_drone);
	tf2::fromMsg(camera_pose, frame_camera);
	tf2::fromMsg(aruco_pose, frame_aruco);

	// Calculate the pose of the aruco in the world frame
	KDL::Frame frame_aruco_world = frame_drone * frame_camera * frame_aruco;

	// Convert to geometry_msgs::Pose
	geometry_msgs::msg::Pose pose_aruco_in_world = tf2::toMsg(frame_aruco_world);

	// Fetch the heading of the aruco in the world frame
	auto q = Eigen::Quaternionf(pose_aruco_in_world.orientation.w, pose_aruco_in_world.orientation.x, pose_aruco_in_world.orientation.y, pose_aruco_in_world.orientation.z);
	_target_heading = px4_ros2::quaternionToYaw(q);

	// Fetch the position of the aruco in the world frame
	auto target_position = Eigen::Vector3f(pose_aruco_in_world.position.x, pose_aruco_in_world.position.y, pose_aruco_in_world.position.z);
	_target_position = target_position;


}

void PrecisionLand::onActivate()
{
	generateSearchWaypoints();
	// Initialize _target_position with NaN values
	_target_position.setConstant(std::numeric_limits<float>::quiet_NaN());
	// State transition to Search
	RCLCPP_INFO(_node.get_logger(), "Switching to State::Search");
	_state = State::Search;
}

void PrecisionLand::onDeactivate()
{
	// TODO:
}

void PrecisionLand::updateSetpoint(float dt_s)
{
	// Checking target lost
	auto elapsed = (_node.now().nanoseconds() - _last_target_timestamp.nanoseconds())/1e9;
	// Based on the state, set the target lost flag
	if (_state == State::Approach && elapsed > 2.0) {
		RCLCPP_INFO(_node.get_logger(), "Target lost during approach");
		_target_lost = true;
	} else if (_state == State::Descend && elapsed > 0.5) {
		RCLCPP_INFO(_node.get_logger(), "Target lost during descend");
		_target_lost = true;
	} else {
		_target_lost = false;
	}

	switch (_state) {
	case State::Search: {

		// TODO: logic should use timestamp to detect stale data.

		// If the market has not been detected, search for it
		if (std::isnan(_target_position.x())) {
			// Get the next waypoint
			Eigen::Vector3f target_position = _search_waypoints[_search_waypoint_index];
			// Go to the next waypoint
			// Publisher for trajectory setpoint
			_trajectory_setpoint_msg.timestamp = _node.now().nanoseconds() / 1000;
			_trajectory_setpoint_msg.position = {target_position.x(), target_position.y(), target_position.z()};
			_trajectory_setpoint_msg.velocity = {NAN, NAN, NAN};
			_trajectory_setpoint_msg.acceleration = {NAN, NAN, NAN};
			_trajectory_setpoint_msg.jerk = {NAN, NAN, NAN};
			_trajectory_setpoint_msg.yaw = NAN;
			_trajectory_setpoint_msg.yawspeed = NAN;
			// Publish the trajectory setpoint
			_trajectory_setpoint->update(_trajectory_setpoint_msg);


			// Check if the drone has reached the target position
			if (positionReached(target_position)) {
				_search_waypoint_index++;

				// If we have searched all waypoints, start over
				if (_search_waypoint_index >= static_cast<int>(_search_waypoints.size())) {
					_search_waypoint_index = 0;
				}
			}
		}

		// -- Check if the marker has been detected --> State Transition
		else {
			RCLCPP_INFO(_node.get_logger(), "Switching to State::Approach");
			// Target posittion printed
			// RCLCPP_INFO(_node.get_logger(), "Target position: %f, %f, %f", double(_target_position.x()), double(_target_position.y()), double(_target_position.z()));
			_state = State::Approach;
		}

		break;
	}

	case State::Approach: {
		// Aproach the target position
		_approach_altitude = _vehicle_local_position->positionNed().z();

		auto position = Eigen::Vector3f(_target_position.x(), _target_position.y(), _approach_altitude);
		// Publisher for trajectory setpoint
		_trajectory_setpoint_msg.timestamp = _node.now().nanoseconds() / 1000;
		_trajectory_setpoint_msg.position = {position.x(), position.y(), _approach_altitude};
		_trajectory_setpoint_msg.velocity = {NAN, NAN, NAN};
		_trajectory_setpoint_msg.acceleration = {NAN, NAN, NAN};
		_trajectory_setpoint_msg.jerk = {NAN, NAN, NAN};
		// _trajectory_setpoint_msg.yaw = _target_heading;
		// _trajectory_setpoint_msg.yawspeed = NAN;
		// Calculate yawspeed for smooth heading adjustment
		float current_heading = _vehicle_local_position->heading();
		float heading_difference = _target_heading - current_heading;
		float max_yawspeed = 0.1; // Maximum yawspeed (rad/s), adjust for desired smoothness

		// Normalize the heading difference to be within [-pi, pi]
		heading_difference = atan2(sin(heading_difference), cos(heading_difference));

		// Calculate yawspeed to gradually reduce the heading difference
		float yawspeed = std::clamp(heading_difference, -max_yawspeed, max_yawspeed);

		_trajectory_setpoint_msg.yaw = NAN; // Set yaw to NAN to use yawspeed control
		_trajectory_setpoint_msg.yawspeed = yawspeed;

		// Publish the trajectory setpoint
		_trajectory_setpoint->update(_trajectory_setpoint_msg);


		// -- Check std::absf(Position - Target < Threshold) --> State Transition
		if (positionReached(position)) {
			// If target has been lost, switch to search
			if (_target_lost) {
				RCLCPP_INFO(_node.get_logger(), "Switching back to State::Search");
				_target_position.setConstant(std::numeric_limits<float>::quiet_NaN());
				_state = State::Search;
				break;
			}
			else {
				// If target is still detected, switch to descend
				RCLCPP_INFO(_node.get_logger(), "Switching to State::Descend");
				_state = State::Descend;
			}
		}

		break;
	}

	case State::Descend: {
		// TODO: check if distance to bottom is still valid
		// - if invalid stop
		// - start timeout, switch to failsafe if timed out : failsafe = normal land

		// TODO: while in failsafe (normal land) keep checking conditions to switch back into precision land


		auto distance_to_ground = _vehicle_local_position->distanceGround();
		auto target_z = _vehicle_local_position->positionNed().z() + distance_to_ground + 10;
		auto position = Eigen::Vector3f(_target_position.x(), _target_position.y(), target_z);
		auto heading = _target_heading;
		auto time_now = _node.now();

		// Log the current time and the last target timestamp
		auto elapsed = time_now - _last_target_timestamp;
		// Check if the last target timestamp is older than 0.2 second and we are close to the ground
		if (_target_lost || distance_to_ground < 0.3) {
			// If the target is lost or we are close to the ground go straight down
			_trajectory_setpoint_msg.timestamp = _node.now().nanoseconds() / 1000;
			_trajectory_setpoint_msg.position = {NAN, NAN, NAN};
			_trajectory_setpoint_msg.velocity = {0.0, 0.0, 0.4};
			_trajectory_setpoint_msg.acceleration = {0.0, 0.0, NAN};
			_trajectory_setpoint_msg.jerk = {NAN, NAN, NAN};
			_trajectory_setpoint_msg.yaw = heading;
			_trajectory_setpoint_msg.yawspeed = NAN;
			
	
		}
		else {
			// If the target is not lost and we are not close to the ground, descend to the target position
			// Publisher for trajectory setpoint
			_trajectory_setpoint_msg.timestamp = _node.now().nanoseconds() / 1000;
			_trajectory_setpoint_msg.position = {position.x(), position.y(), NAN};
			_trajectory_setpoint_msg.velocity = {NAN, NAN, 0.35};
			_trajectory_setpoint_msg.acceleration = {NAN, NAN, NAN};
			_trajectory_setpoint_msg.jerk = {NAN, NAN, NAN};
			_trajectory_setpoint_msg.yaw = heading;
			_trajectory_setpoint_msg.yawspeed = NAN;
		}

		// Publish the trajectory setpoint
		_trajectory_setpoint->update(_trajectory_setpoint_msg);

		// -- Check std::absf(Position - Target < Threshold) --> State Transition
		if (_land_detected) {
			RCLCPP_INFO(_node.get_logger(), "Switching to State::Finished");
			_state = State::Finished;
		}

		break;
	}

	case State::Finished: {
		ModeBase::completed(px4_ros2::Result::Success);
		break;
	}
	} // end switch/case
}

void PrecisionLand::generateSearchWaypoints()
{
	// Generate spiral search waypoints
	// The search waypoints are generated in the NED frame
	// Parameters for the search pattern
	double start_x = 0.0;
	double start_y = 0.0;
	double current_z = _vehicle_local_position->positionNed().z();
	auto min_z = -1.0;

	double max_radius = 2.0;
	double layer_spacing = 0.5;
	int points_per_layer = 16;
	std::vector<Eigen::Vector3f> waypoints;
	RCLCPP_INFO(_node.get_logger(), "current_z: %f", double(current_z));

	// Generate waypoints
	// Calculate the number of layers needed
	int num_layers = (static_cast<int>((min_z - current_z) / layer_spacing) / 2) < 1 ? 1 : (static_cast<int>((min_z - current_z) / layer_spacing) / 2);


	// Generate waypoints
	for (int layer = 0; layer < num_layers; ++layer) {
		std::vector<Eigen::Vector3f> layer_waypoints;

		// Spiral out to max radius
		double radius = 0.0;

		for (int point = 0; point < points_per_layer + 1; ++point) {
			double angle = 2.0 * M_PI * point / points_per_layer;
			double x = start_x + radius * cos(angle);
			double y = start_y + radius * sin(angle);
			double z = current_z;

			layer_waypoints.push_back(Eigen::Vector3f(x, y, z));
			radius += max_radius / points_per_layer;
		}

		// Push the spiral out waypoints to the main waypoints vector
		waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());

		// Decrease the altitude for the inward spiral
		current_z += layer_spacing;

		// Reverse the layer waypoints for spiral in
		std::reverse(layer_waypoints.begin(), layer_waypoints.end());

		// Adjust the z-coordinate for the inward spiral
		for (auto& waypoint : layer_waypoints) {
			waypoint.z() = current_z;
		}

		// Push the reversed waypoints to the main waypoints vector
		waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());

		// Decrease the altitude for the next outward spiral
		current_z += layer_spacing;
	}

	// Set the search waypoints
	_search_waypoints = waypoints;
}

bool PrecisionLand::positionReached(const Eigen::Vector3f& target) const
{
	// Parameters for delta_position and delta_velocitry
	static constexpr float kDeltaPosition = 0.25f;
	static constexpr float kDeltaVelocity = 0.25f;

	auto position = _vehicle_local_position->positionNed();
	auto velocity = _vehicle_local_position->velocityNed();

	const auto delta_pos = target - position;
	// NOTE: this does NOT handle a moving target!
	return (delta_pos.norm() < kDeltaPosition) && (velocity.norm() < kDeltaVelocity);
}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<PrecisionLand>>(kModeName, kEnableDebugOutput));
	rclcpp::shutdown();

	return 0;
}
