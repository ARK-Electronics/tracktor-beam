#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class MovingObjectTracker : public rclcpp::Node {
public:
    MovingObjectTracker() : Node("moving_object_tracker") {
        // Subscription to the camera image topic
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10, std::bind(&MovingObjectTracker::imageCallback, this, std::placeholders::_1));

        // Publisher for the processed image topic
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_proc", 10);

        // Initialize background subtractor with custom parameters
        background_subtractor_ = cv::createBackgroundSubtractorMOG2(500, 16, true);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert ROS Image message to OpenCV Mat
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Stabilize the frame using feature matching
        if (!prev_frame_.empty()) {
            std::vector<cv::Point2f> prev_pts, curr_pts;
            cv::goodFeaturesToTrack(prev_frame_gray_, prev_pts, 100, 0.01, 30);
            std::vector<uchar> status;
            std::vector<float> err;
            if (!prev_pts.empty()) {
                cv::calcOpticalFlowPyrLK(prev_frame_gray_, frame_gray_, prev_pts, curr_pts, status, err);
                std::vector<cv::Point2f> good_prev_pts, good_curr_pts;
                for (size_t i = 0; i < status.size(); ++i) {
                    if (status[i]) {
                        good_prev_pts.push_back(prev_pts[i]);
                        good_curr_pts.push_back(curr_pts[i]);
                    }
                }
                if (!good_prev_pts.empty()) {
                    cv::Mat H = cv::findHomography(good_prev_pts, good_curr_pts, cv::RANSAC);
                    cv::warpPerspective(frame, stabilized_frame_, H, frame.size());
                }
            }
        } else {
            stabilized_frame_ = frame.clone();
        }

        // Convert the stabilized frame to grayscale
        cv::cvtColor(stabilized_frame_, frame_gray_, cv::COLOR_BGR2GRAY);

        // Apply background subtraction
        cv::Mat fg_mask;
        background_subtractor_->apply(stabilized_frame_, fg_mask);

        // Apply morphological operations to reduce noise
        cv::morphologyEx(fg_mask, fg_mask, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), 2);
        cv::morphologyEx(fg_mask, fg_mask, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 2);

        // Find contours of the moving objects
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(fg_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Draw bounding boxes around the moving objects
        for (const auto& contour : contours) {
            if (cv::contourArea(contour) > 1000) { // Increased minimum contour area threshold
                cv::Rect bbox = cv::boundingRect(contour);
                cv::rectangle(stabilized_frame_, bbox, cv::Scalar(0, 255, 0), 2);
            }
        }

        // Display the frame with the bounding boxes
        cv::imshow("Moving Object Tracker", stabilized_frame_);
        cv::waitKey(1);

        // Convert the processed frame back to a ROS Image message
        auto processed_msg = cv_bridge::CvImage(msg->header, "bgr8", stabilized_frame_).toImageMsg();

        // Publish the processed image
        image_pub_->publish(*processed_msg);

        // Update the previous frame
        frame_gray_.copyTo(prev_frame_gray_);
        prev_frame_ = stabilized_frame_.clone();
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    cv::Mat prev_frame_, frame_gray_, prev_frame_gray_, stabilized_frame_;
    cv::Ptr<cv::BackgroundSubtractor> background_subtractor_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MovingObjectTracker>());
    rclcpp::shutdown();
    return 0;
}
