#include "GimbalController.hpp"
#include <cmath>

GimbalControllerNode::GimbalControllerNode()
	: Node("gimbal_controller_node")
{
	loadParameters();

	// QoS settings
	auto qos = rclcpp::QoS(10).best_effort();

	// Subscribers
	_target_pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
				   "/target_pose", qos,
				   std::bind(&GimbalControllerNode::target_pose_callback, this, std::placeholders::_1));

	_gimbal_status_sub = create_subscription<px4_msgs::msg::GimbalDeviceAttitudeStatus>(
				     "/fmu/out/gimbal_device_attitude_status", qos,
				     std::bind(&GimbalControllerNode::gimbal_status_callback, this, std::placeholders::_1));

	// Publishers
	_gimbal_cmd_pub = create_publisher<px4_msgs::msg::GimbalManagerSetAttitude>(
				  "/fmu/in/gimbal_manager_set_attitude", qos);

	_vehicle_command_pub = create_publisher<px4_msgs::msg::VehicleCommand>(
				       "/fmu/in/vehicle_command", qos);

	// Timer for periodic control updates
	auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / _param_control_rate));
	_timer = create_wall_timer(timer_period,
				    std::bind(&GimbalControllerNode::controlLoop, this));

	// Initialize last target time to current time to avoid time source mismatch
	_last_target_ros_time = this->now();

	RCLCPP_INFO(get_logger(), "Gimbal controller initialized");
	RCLCPP_INFO(get_logger(), "Control rate: %.1f Hz", _param_control_rate);
	RCLCPP_INFO(get_logger(), "Pitch offset: %.1f deg, Yaw offset: %.1f deg",
		    _param_pitch_offset_deg, _param_yaw_offset_deg);
}

void GimbalControllerNode::loadParameters()
{
	declare_parameter<double>("control_rate", 20.0);
	declare_parameter<double>("timeout_ms", 1000.0);
	declare_parameter<double>("pitch_offset_deg", 0.0);
	declare_parameter<double>("yaw_offset_deg", 0.0);
	declare_parameter<int>("gimbal_device_id", 0);
	declare_parameter<bool>("roll_lock", true);
	declare_parameter<bool>("pitch_lock", true);
	declare_parameter<bool>("yaw_lock", false);

	get_parameter("control_rate", _param_control_rate);
	get_parameter("timeout_ms", _param_timeout_ms);
	get_parameter("pitch_offset_deg", _param_pitch_offset_deg);
	get_parameter("yaw_offset_deg", _param_yaw_offset_deg);

	int gimbal_id = 0;
	get_parameter("gimbal_device_id", gimbal_id);
	_param_gimbal_device_id = static_cast<uint8_t>(gimbal_id);

	get_parameter("pitch_lock", _param_pitch_lock);
	get_parameter("yaw_lock", _param_yaw_lock);
	get_parameter("roll_lock", _param_roll_lock);
}

void GimbalControllerNode::target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	// Store target position from ArUco tracker
	_last_target_position.x() = msg->pose.position.x;
	_last_target_position.y() = msg->pose.position.y;
	_last_target_position.z() = msg->pose.position.z;

	bool was_tracking = _has_target;
	_has_target = true;
	_last_target_ros_time = this->now();  // Use node's current time, not message timestamp

	// Acquire control when we first detect a target, or when target comes back after timeout
	if (!was_tracking) {
		RCLCPP_INFO(get_logger(), "Target acquired, requesting gimbal control");
		acquireGimbalControl();
		_has_control = false; // Will be set true after we send the command
	}

	// RCLCPP_INFO(get_logger(), "Target position: [%.3f, %.3f, %.3f]",
	// 	    _last_target_position.x(), _last_target_position.y(), _last_target_position.z());
}

void GimbalControllerNode::gimbal_status_callback(const px4_msgs::msg::GimbalDeviceAttitudeStatus::SharedPtr msg)
{
	// Store current gimbal attitude for feedback
	_current_gimbal_attitude.w() = msg->q[0];
	_current_gimbal_attitude.x() = msg->q[1];
	_current_gimbal_attitude.y() = msg->q[2];
	_current_gimbal_attitude.z() = msg->q[3];

	// RCLCPP_INFO(get_logger(), "Gimbal attitude: [%.3f, %.3f, %.3f, %.3f]",
	// 	    _current_gimbal_attitude.w(), _current_gimbal_attitude.x(),
	// 	    _current_gimbal_attitude.y(), _current_gimbal_attitude.z());
}

Eigen::Quaterniond GimbalControllerNode::targetPositionToGimbalQuaternion(const Eigen::Vector3d& target_position)
{
	// Target position is in camera optical frame:
	// Camera: X=right, Y=down, Z=forward

	// Transform to body/gimbal frame:
	// Body: X=forward, Y=right, Z=down
	Eigen::Vector3d target_body;
	target_body.x() = target_position.z();   // forward = camera forward
	target_body.y() = target_position.x();   // right = camera right
	target_body.z() = target_position.y();   // down = camera down

	double range = target_body.norm();

	if (range < 0.01) {
		RCLCPP_WARN(get_logger(), "Target too close, using neutral attitude");
		return Eigen::Quaterniond::Identity();
	}

	// Calculate gimbal angles in body frame
	// Pitch: rotation around Y axis (right), positive = nose down
	// Yaw: rotation around Z axis (down), positive = nose right
	double pitch = std::atan2(target_body.z(), target_body.x());
	double yaw = std::atan2(target_body.y(), target_body.x());

	// Apply offsets
	pitch += _param_pitch_offset_deg * M_PI / 180.0;
	yaw += _param_yaw_offset_deg * M_PI / 180.0;

	// RCLCPP_INFO(get_logger(), "Target body frame: [%.3f, %.3f, %.3f], Pitch: %.2f deg, Yaw: %.2f deg",
	// 	    target_body.x(), target_body.y(), target_body.z(),
	// 	    pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);

	// Create quaternion using body frame axes
	// X: forward, Y: right, Z: down
	Eigen::AngleAxisd pitch_rotation(pitch, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd yaw_rotation(yaw, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd roll_rotation(0.0, Eigen::Vector3d::UnitX());

	// Combined rotation: yaw * pitch * roll (applied right to left)
	Eigen::Quaterniond q = yaw_rotation * pitch_rotation * roll_rotation;
	q.normalize();

	return q;
}

void GimbalControllerNode::controlLoop()
{
	// Call the main control function with the stored target position
	calculateGimbalAttitude(_last_target_position);
}

void GimbalControllerNode::calculateGimbalAttitude(const Eigen::Vector3d& target_position)
{
	// Check if we have a target first
	if (!_has_target) {
		return;
	}

	// Check if target is recent
	auto now = this->now();
	auto time_since_target = (now - _last_target_ros_time).seconds() * 1000.0; // ms

	if (time_since_target > _param_timeout_ms) {
		if (_has_target) {
			RCLCPP_INFO(get_logger(), "Target timeout (%.1f ms), stopping tracking", time_since_target);
			_has_target = false;
			_has_control = false; // Release our control state
		}
		return;
	}

	// Calculate desired gimbal attitude
	Eigen::Quaterniond desired_attitude = targetPositionToGimbalQuaternion(_last_target_position);

	// Prepare gimbal command message
	px4_msgs::msg::GimbalManagerSetAttitude msg;

	// Timestamp (microseconds)
	msg.timestamp = this->now().nanoseconds() / 1000;

	// Origin and target (can be left at defaults for simulation)
	msg.origin_sysid = 1;
	msg.origin_compid = 190;
	msg.target_system = 1;
	msg.target_component = 154; // MAV_COMP_ID_GIMBAL (154)

	// Gimbal device ID
	msg.gimbal_device_id = _param_gimbal_device_id;

	// Set flags
	uint32_t flags = 0;
	if (_param_roll_lock) {
		flags |= px4_msgs::msg::GimbalManagerSetAttitude::GIMBAL_MANAGER_FLAGS_ROLL_LOCK;
	}
	if (_param_pitch_lock) {
		flags |= px4_msgs::msg::GimbalManagerSetAttitude::GIMBAL_MANAGER_FLAGS_PITCH_LOCK;
	}
	if (_param_yaw_lock) {
		flags |= px4_msgs::msg::GimbalManagerSetAttitude::GIMBAL_MANAGER_FLAGS_YAW_LOCK;
	}
	msg.flags = flags;

	// Set quaternion (w, x, y, z)
	msg.q[0] = desired_attitude.w();
	msg.q[1] = desired_attitude.x();
	msg.q[2] = desired_attitude.y();
	msg.q[3] = desired_attitude.z();

	// Angular velocity (set to 0 for position control)
	msg.angular_velocity_x = 0.0f;
	msg.angular_velocity_y = 0.0f;
	msg.angular_velocity_z = 0.0f;

	// Publish command
	_gimbal_cmd_pub->publish(msg);

	// Log occasionally to monitor (every 20 commands = 1 second at 20Hz)
	static int pub_count = 0;
	if (++pub_count % 20 == 1) {
		RCLCPP_INFO(get_logger(), "Gimbal cmd: q=[%.3f, %.3f, %.3f, %.3f] (R:%.1f° P:%.1f° Y:%.1f°), origin=%d/%d",
			    msg.q[0], msg.q[1], msg.q[2], msg.q[3],
			    std::atan2(2.0*(msg.q[0]*msg.q[1] + msg.q[2]*msg.q[3]), 1.0 - 2.0*(msg.q[1]*msg.q[1] + msg.q[2]*msg.q[2])) * 180.0 / M_PI,
			    std::asin(2.0*(msg.q[0]*msg.q[2] - msg.q[3]*msg.q[1])) * 180.0 / M_PI,
			    std::atan2(2.0*(msg.q[0]*msg.q[3] + msg.q[1]*msg.q[2]), 1.0 - 2.0*(msg.q[2]*msg.q[2] + msg.q[3]*msg.q[3])) * 180.0 / M_PI,
			    msg.origin_sysid, msg.origin_compid);
	}
}

void GimbalControllerNode::acquireGimbalControl()
{
	// Send MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE to acquire primary control
	px4_msgs::msg::VehicleCommand cmd;

	cmd.timestamp = this->now().nanoseconds() / 1000;
	cmd.target_system = 1;
	cmd.target_component = 0; // 0 = broadcast to all components
	cmd.command = 1001; // MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE
	cmd.param1 = 1.0f; // sysid 1 (autopilot) as primary control
	cmd.param2 = 190.0f; // compid 1 (autopilot) as primary control
	cmd.param3 = -1.0f; // -1 = leave secondary control unchanged
	cmd.param4 = -1.0f; // -1 = leave secondary control unchanged
	cmd.param5 = NAN;
	cmd.param6 = NAN;
	cmd.param7 = static_cast<float>(_param_gimbal_device_id); // gimbal device id
	cmd.source_system = 1;
	cmd.source_component = 1;
	cmd.confirmation = 0;
	cmd.from_external = false;

	_vehicle_command_pub->publish(cmd);
	_has_control = true;

	RCLCPP_INFO(get_logger(), "Requested gimbal control (setting primary to 1/1)");
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GimbalControllerNode>());
	rclcpp::shutdown();
	return 0;
}
