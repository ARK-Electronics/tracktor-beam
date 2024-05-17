#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class VehicleTracker : public rclcpp::Node {
public:
    VehicleTracker() : Node("vehicle_tracker") {
        // Subscription to the camera image topic
        _image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10, std::bind(&VehicleTracker::image_callback, this, std::placeholders::_1));
        
        // Publisher for the processed image topic
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_proc", 10);

        // Initialize background subtractor
        _bg_subtractor = cv::createBackgroundSubtractorMOG2();
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert ROS Image message to OpenCV Mat
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Apply background subtraction
        cv::Mat fg_mask;
        _bg_subtractor->apply(frame, fg_mask);

        // Find contours in the foreground mask
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(fg_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Draw bounding boxes around detected vehicles
        for (const auto& contour : contours) {
            cv::Rect bounding_box = cv::boundingRect(contour);
            if (bounding_box.area() > 500) { // Filter small contours based on area
                cv::rectangle(frame, bounding_box, cv::Scalar(0, 255, 0), 2);
            }
        }

        // Display the frame with bounding boxes
        cv::imshow("Vehicle Tracker", frame);
        cv::waitKey(1);

        // Convert the processed frame back to a ROS Image message
        auto processed_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();

        // Publish the processed image
        image_pub_->publish(*processed_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_sub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    cv::Ptr<cv::BackgroundSubtractor> _bg_subtractor;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleTracker>());
    rclcpp::shutdown();
    return 0;
}
