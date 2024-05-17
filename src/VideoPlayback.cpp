#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class VideoPublisher : public rclcpp::Node {
public:
    VideoPublisher() : Node("video_publisher"), cap_("moving_car_beach.mp4") {
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Error opening video file");
            rclcpp::shutdown();
        }

        // Get the frame rate of the video
        double fps = cap_.get(cv::CAP_PROP_FPS);
        if (fps <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid FPS value");
            rclcpp::shutdown();
        }

        // Calculate the interval between frames in milliseconds
        int frame_interval = static_cast<int>(1000 / fps);

        // Publisher for the video frames
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);

        // Timer to publish video frames at the frame rate of the video
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(frame_interval),
            std::bind(&VideoPublisher::publishFrame, this));
    }

private:
    void publishFrame() {
        cv::Mat frame;
        if (!cap_.read(frame)) {
            RCLCPP_INFO(this->get_logger(), "End of video file");
            rclcpp::shutdown();
            return;
        }

        // Resize the frame to fit within a 640x480 window
        cv::Mat resized_frame;
        // cv::resize(frame, resized_frame, cv::Size(1080, 720));
        cv::resize(frame, resized_frame, cv::Size(640, 480));

        // Convert the resized frame to a ROS Image message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", resized_frame).toImageMsg();
        image_pub_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoPublisher>());
    rclcpp::shutdown();
    return 0;
}
