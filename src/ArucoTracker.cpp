#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class ArucoTrackerNode : public rclcpp::Node
{
public:
	ArucoTrackerNode();

private:
	void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
	cv::Mat processImage(const cv::Mat& img);

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_sub;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_pub;
	cv::Ptr<cv::aruco::Dictionary> _dictionary;
};

ArucoTrackerNode::ArucoTrackerNode() : Node("aruco_tracker_node")
{
	_dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5));
	qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

	// USB camera: image_raw
	// CSI camera: TODO:
	_image_sub = this->create_subscription<sensor_msgs::msg::Image>(
		"/image_raw", qos, std::bind(&ArucoTrackerNode::image_callback, this, std::placeholders::_1));

	_image_pub = this->create_publisher<sensor_msgs::msg::Image>(
		"/image_proc", qos);
}

void ArucoTrackerNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
	try {
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

		// Detects and boxes aruco markers
		cv::Mat img_proc = detect_and_box(cv_ptr->image);

		cv_bridge::CvImage out_msg;
		out_msg.header = msg->header; // Same timestamp and tf frame as input image
		out_msg.encoding = sensor_msgs::image_encodings::BGR8;
		out_msg.image = img_proc;

		_image_pub->publish(*out_msg.toImageMsg().get());

	} catch (const cv_bridge::Exception& e) {
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
	}
}

cv::Mat ArucoTrackerNode::detect_and_box(const cv::Mat& img)
{
	std::vector<int> ids;
	std::vector<std::vector<cv::Point2f>> corners;
	cv::aruco::detectMarkers(img, _dictionary, corners, ids);

	// If at least one marker detected
	if (ids.size() > 0) {
		cv::aruco::drawDetectedMarkers(img, corners, ids);
	}

	return img; // Return the image with detected markers
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ArucoTrackerNode>());
	rclcpp::shutdown();
	return 0;
}
