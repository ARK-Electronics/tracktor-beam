#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/marker.pb.h>
#include <gz/msgs/empty.pb.h>
#include <chrono>

class TagPoseVisualizer : public rclcpp::Node
{
public:
	TagPoseVisualizer()
		: Node("tag_pose_visualizer")
	{
		// Subscribe to world-frame target pose from precision_land
		_pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
			"/target_pose_world",
			rclcpp::QoS(10),
			std::bind(&TagPoseVisualizer::poseCallback, this, std::placeholders::_1));

		RCLCPP_INFO(get_logger(), "Tag pose visualizer started - calling Gazebo marker service");
	}

private:
	void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
	{
		// Create marker message
		gz::msgs::Marker marker_msg;
		marker_msg.set_ns("aruco_target");
		marker_msg.set_id(0);
		marker_msg.set_action(gz::msgs::Marker::ADD_MODIFY);
		marker_msg.set_type(gz::msgs::Marker::BOX);
		marker_msg.set_visibility(gz::msgs::Marker::GUI);

		// Set position (pose is already in world/map frame from precision_land)
		marker_msg.mutable_pose()->mutable_position()->set_x(msg->pose.position.x);
		marker_msg.mutable_pose()->mutable_position()->set_y(msg->pose.position.y);
		marker_msg.mutable_pose()->mutable_position()->set_z(msg->pose.position.z);

		// Set orientation
		marker_msg.mutable_pose()->mutable_orientation()->set_w(msg->pose.orientation.w);
		marker_msg.mutable_pose()->mutable_orientation()->set_x(msg->pose.orientation.x);
		marker_msg.mutable_pose()->mutable_orientation()->set_y(msg->pose.orientation.y);
		marker_msg.mutable_pose()->mutable_orientation()->set_z(msg->pose.orientation.z);

		// Set scale (0.5m x 0.5m x 0.02m to match ArUco marker size)
		marker_msg.mutable_scale()->set_x(0.5);
		marker_msg.mutable_scale()->set_y(0.5);
		marker_msg.mutable_scale()->set_z(0.02);

		// Set color (green with transparency)
		marker_msg.mutable_material()->mutable_ambient()->set_r(0.0f);
		marker_msg.mutable_material()->mutable_ambient()->set_g(1.0f);
		marker_msg.mutable_material()->mutable_ambient()->set_b(0.0f);
		marker_msg.mutable_material()->mutable_ambient()->set_a(0.8f);

		marker_msg.mutable_material()->mutable_diffuse()->set_r(0.0f);
		marker_msg.mutable_material()->mutable_diffuse()->set_g(1.0f);
		marker_msg.mutable_material()->mutable_diffuse()->set_b(0.0f);
		marker_msg.mutable_material()->mutable_diffuse()->set_a(0.8f);

		// Set lifetime (1 second to account for slower update rate)
		marker_msg.mutable_lifetime()->set_sec(0);
		marker_msg.mutable_lifetime()->set_nsec(500000000);

		// Throttle service calls to reduce timeouts
		auto now = std::chrono::steady_clock::now();
		if (std::chrono::duration_cast<std::chrono::milliseconds>(now - _last_service_call).count() < 100) {
			return; // Skip if called within last 100ms
		}
		_last_service_call = now;

		// Call the marker service asynchronously
		gz::msgs::Empty response;
		bool result;
		unsigned int timeout = 500; // 500ms timeout

		bool executed = _gz_node.Request("/marker", marker_msg, timeout, response, result);

		if (!executed || !result) {
			RCLCPP_DEBUG(get_logger(), "Marker service call failed or timed out");
		}
	}

	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _pose_sub;
	gz::transport::Node _gz_node;
	std::chrono::steady_clock::time_point _last_service_call = std::chrono::steady_clock::now();
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TagPoseVisualizer>());
	rclcpp::shutdown();
	return 0;
}
