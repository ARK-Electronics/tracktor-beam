#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <future>

class VehicleTracker : public rclcpp::Node {
public:
    VehicleTracker() : Node("vehicle_tracker") {
        // Subscription to the camera image topic
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10, std::bind(&VehicleTracker::imageCallback, this, std::placeholders::_1));
        
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_proc", 10);

        // Load YOLO model
        loadYOLOModel("yolov3.cfg", "yolov3.weights", "coco.names");
    }

private:
    void loadYOLOModel(const std::string& cfg, const std::string& weights, const std::string& names) {
        net_ = cv::dnn::readNetFromDarknet(cfg, weights);
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

        // net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        // net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

        // net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        // net_.setPreferableTarget(cv::dnn::DNN_TARGET_OPENCL);

        std::ifstream ifs(names.c_str());
        std::string line;
        while (std::getline(ifs, line)) {
            class_names_.push_back(line);
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (last_future_.valid() && last_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
            // RCLCPP_INFO(this->get_logger(), "Skipping frame to avoid overload");
            return;
        }

        last_future_ = std::async(std::launch::async, &VehicleTracker::processImage, this, msg);
    }

    void processImage(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert ROS Image message to OpenCV Mat
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Prepare the frame for YOLO
        cv::Mat blob;
        cv::dnn::blobFromImage(frame, blob, 1 / 255.0, cv::Size(416, 416), cv::Scalar(), true, false);

        // Set the input to the network
        net_.setInput(blob);

        // Run forward pass to get the network output
        std::vector<cv::Mat> outputs;
        net_.forward(outputs, getOutputNames(net_));

        // Process the outputs
        processDetections(frame, outputs);

        // Display the frame with bounding boxes
        cv::imshow("Vehicle Tracker", frame);
        cv::waitKey(1);

        auto processed_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();

        image_pub_->publish(*processed_msg);
    }

    std::vector<std::string> getOutputNames(const cv::dnn::Net& net) {
        static std::vector<std::string> names;
        if (names.empty()) {
            std::vector<int> outLayers = net.getUnconnectedOutLayers();
            std::vector<std::string> layersNames = net.getLayerNames();
            names.resize(outLayers.size());
            for (size_t i = 0; i < outLayers.size(); ++i)
                names[i] = layersNames[outLayers[i] - 1];
        }
        return names;
    }

    void processDetections(cv::Mat& frame, const std::vector<cv::Mat>& outputs) {
        std::vector<int> class_ids;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;

        float confidence_threshold = 0.25;

        for (const auto& output : outputs) {
            for (int i = 0; i < output.rows; ++i) {
                const auto* data = output.ptr<float>(i);
                float confidence = data[4];
                if (confidence >= confidence_threshold) {
                    cv::Mat scores = output.row(i).colRange(5, output.cols);
                    cv::Point class_id_point;
                    double max_class_score;
                    minMaxLoc(scores, 0, &max_class_score, 0, &class_id_point);
                    if (max_class_score > confidence_threshold) {
                        int center_x = static_cast<int>(data[0] * frame.cols);
                        int center_y = static_cast<int>(data[1] * frame.rows);
                        int width = static_cast<int>(data[2] * frame.cols);
                        int height = static_cast<int>(data[3] * frame.rows);
                        int left = center_x - width / 2;
                        int top = center_y - height / 2;

                        class_ids.push_back(class_id_point.x);
                        confidences.push_back(static_cast<float>(max_class_score));
                        boxes.emplace_back(left, top, width, height);
                    }
                }
            }
        }

        // Perform non-maximum suppression to eliminate redundant overlapping boxes with lower confidences
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, 0.5, 0.4, indices);

        for (int idx : indices) {
            cv::Rect box = boxes[idx];
            drawPred(class_ids[idx], confidences[idx], box.x, box.y, box.x + box.width, box.y + box.height, frame);
        }
    }

    void drawPred(int class_id, float confidence, int left, int top, int right, int bottom, cv::Mat& frame) {
        // Draw a bounding box.
        cv::rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 255, 0), 3);

        // Get the label for the class name and its confidence.
        std::string label = cv::format("%.2f", confidence);
        if (!class_names_.empty()) {
            CV_Assert(class_id < (int)class_names_.size());
            label = class_names_[class_id] + ":" + label;
        }

        // Display the label at the top of the bounding box.
        int baseLine;
        cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        top = std::max(top, label_size.height);
        cv::rectangle(frame, cv::Point(left, top - round(1.5 * label_size.height)),
                      cv::Point(left + round(1.5 * label_size.width), top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
        cv::putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0), 1);
    }

    std::future<void> last_future_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    cv::dnn::Net net_;
    std::vector<std::string> class_names_;
};

int main(int argc, char** argv) {

    // std::cout << "Hello :)" << std::endl;
    // std::cout << cv::getBuildInformation() << std::endl;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleTracker>());
    rclcpp::shutdown();
    return 0;
}
