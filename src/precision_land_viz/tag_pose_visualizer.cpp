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
		_pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
			"/target_pose_world",
			rclcpp::QoS(1).best_effort(),
			std::bind(&TagPoseVisualizer::poseCallback, this, std::placeholders::_1));

		RCLCPP_INFO(get_logger(), "Tag pose visualizer started - calling Gazebo marker service");
	}

private:
	void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
	{
		gz::msgs::Marker marker_msg;
		marker_msg.set_ns("aruco_target");
		marker_msg.set_id(0);
		marker_msg.set_action(gz::msgs::Marker::ADD_MODIFY);
		marker_msg.set_type(gz::msgs::Marker::BOX);
		marker_msg.set_visibility(gz::msgs::Marker::GUI);

		marker_msg.mutable_pose()->mutable_position()->set_x(msg->pose.position.x);
		marker_msg.mutable_pose()->mutable_position()->set_y(msg->pose.position.y);
		marker_msg.mutable_pose()->mutable_position()->set_z(msg->pose.position.z);

		marker_msg.mutable_pose()->mutable_orientation()->set_w(msg->pose.orientation.w);
		marker_msg.mutable_pose()->mutable_orientation()->set_x(msg->pose.orientation.x);
		marker_msg.mutable_pose()->mutable_orientation()->set_y(msg->pose.orientation.y);
		marker_msg.mutable_pose()->mutable_orientation()->set_z(msg->pose.orientation.z);

		// TODO: get from sim or use params
		marker_msg.mutable_scale()->set_x(0.5);
		marker_msg.mutable_scale()->set_y(0.5);
		marker_msg.mutable_scale()->set_z(0.02);

		// Set color green with transparency
		marker_msg.mutable_material()->mutable_ambient()->set_r(0.0f);
		marker_msg.mutable_material()->mutable_ambient()->set_g(1.0f);
		marker_msg.mutable_material()->mutable_ambient()->set_b(0.0f);
		marker_msg.mutable_material()->mutable_ambient()->set_a(0.8f);
		marker_msg.mutable_material()->mutable_diffuse()->set_r(0.0f);
		marker_msg.mutable_material()->mutable_diffuse()->set_g(1.0f);
		marker_msg.mutable_material()->mutable_diffuse()->set_b(0.0f);
		marker_msg.mutable_material()->mutable_diffuse()->set_a(0.8f);

		marker_msg.mutable_lifetime()->set_sec(0);
		marker_msg.mutable_lifetime()->set_nsec(500000000);

		gz::msgs::Empty response;
		bool result;
		unsigned int timeout = 500;

		bool executed = _gz_node.Request("/marker", marker_msg, timeout, response, result);

		if (!executed || !result) {
			RCLCPP_DEBUG(get_logger(), "Marker service call failed or timed out");
		}
	}

private:
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _pose_sub;
	gz::transport::Node _gz_node;
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TagPoseVisualizer>());
	rclcpp::shutdown();
	return 0;
}
