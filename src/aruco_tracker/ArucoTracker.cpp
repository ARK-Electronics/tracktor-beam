#include "ArucoTracker.hpp"
#include <sstream>

ArucoTrackerNode::ArucoTrackerNode()
	: Node("aruco_tracker_node")
{
	loadParameters();

	// TODO: params to adjust detector params
	// See: https://docs.opencv.org/4.x/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html
	auto detectorParams = cv::aruco::DetectorParameters();

	// See: https://docs.opencv.org/4.x/d1/d21/aruco__dictionary_8hpp.html
	auto dictionary = cv::aruco::getPredefinedDictionary(_param_dictionary);

	_detector = std::make_unique<cv::aruco::ArucoDetector>(dictionary, detectorParams);

	auto qos = rclcpp::QoS(1).best_effort();

	_image_sub = create_subscription<sensor_msgs::msg::Image>(
			     "/camera", qos, std::bind(&ArucoTrackerNode::image_callback, this, std::placeholders::_1));

	_camera_info_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
				   "/camera_info", qos, std::bind(&ArucoTrackerNode::camera_info_callback, this, std::placeholders::_1));

	// Publishers
	_image_pub = create_publisher<sensor_msgs::msg::Image>("/image_proc", qos);
	_target_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", qos);
}

void ArucoTrackerNode::loadParameters()
{
	declare_parameter<int>("aruco_id", 0);
	declare_parameter<int>("dictionary", 2); // DICT_4X4_250
	declare_parameter<double>("marker_size", 0.5);

	get_parameter("aruco_id", _param_aruco_id);
	get_parameter("dictionary", _param_dictionary);
	get_parameter("marker_size", _param_marker_size);
}

void ArucoTrackerNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
	try {
		// Convert ROS image message to OpenCV image
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

		// Detect markers
		std::vector<int> ids;
		std::vector<std::vector<cv::Point2f>> corners;
		_detector->detectMarkers(cv_ptr->image, corners, ids);
		cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);

		if (!_camera_matrix.empty() && !_dist_coeffs.empty()) {

			std::vector<std::vector<cv::Point2f>> undistortedCorners;

			for (const auto& corner : corners) {
				std::vector<cv::Point2f> undistortedCorner;
				cv::undistortPoints(corner, undistortedCorner, _camera_matrix, _dist_coeffs, cv::noArray(), _camera_matrix);
				undistortedCorners.push_back(undistortedCorner);
			}

			for (size_t i = 0; i < ids.size(); i++) {
				if (ids[i] != _param_aruco_id) {
					continue;
				}

				// Calculate marker size from camera intrinsics
				float half_size = _param_marker_size / 2.0f;
				std::vector<cv::Point3f> objectPoints = {
					cv::Point3f(-half_size,  half_size, 0),  // top left
					cv::Point3f(half_size,  half_size, 0),   // top right
					cv::Point3f(half_size, -half_size, 0),   // bottom right
					cv::Point3f(-half_size, -half_size, 0)   // bottom left
				};

				// Use PnP solver to estimate pose
				cv::Vec3d rvec, tvec;
				cv::solvePnP(objectPoints, undistortedCorners[i], _camera_matrix, cv::noArray(), rvec, tvec);
				// Annotate the image
				cv::drawFrameAxes(cv_ptr->image, _camera_matrix, cv::noArray(), rvec, tvec, _param_marker_size);

				// Quaternion from rotation matrix
				cv::Mat rot_mat;
				cv::Rodrigues(rvec, rot_mat);
				cv::Quatd quat = cv::Quatd::createFromRotMat(rot_mat).normalize();

				// Publish target pose
				geometry_msgs::msg::PoseStamped pose_msg;
				pose_msg.header.stamp = msg->header.stamp;
				pose_msg.header.frame_id = "camera_frame";
				pose_msg.pose.position.x = tvec[0];
				pose_msg.pose.position.y = tvec[1];
				pose_msg.pose.position.z = tvec[2];
				pose_msg.pose.orientation.x = quat.x;
				pose_msg.pose.orientation.y = quat.y;
				pose_msg.pose.orientation.z = quat.z;
				pose_msg.pose.orientation.w = quat.w;

				_target_pose_pub->publish(pose_msg);

				// Annotate the image
				annotate_image(cv_ptr, tvec);

				// NOTE: we break here, meaning we only publish the pose of the first target we see
				break;
			}

		} else {
			RCLCPP_ERROR(get_logger(), "Missing camera calibration");
		}

		// Always publish image
		cv_bridge::CvImage out_msg;
		out_msg.header = msg->header;
		out_msg.encoding = sensor_msgs::image_encodings::BGR8;
		out_msg.image = cv_ptr->image;
		_image_pub->publish(*out_msg.toImageMsg().get());

	} catch (const cv_bridge::Exception& e) {
		RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
	}
}

void ArucoTrackerNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
	// Always update the camera matrix and distortion coefficients from the new message
	_camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();   // Use clone to ensure a deep copy
	_dist_coeffs = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();   // Use clone to ensure a deep copy

	// Log the first row of the camera matrix to verify correct values
	RCLCPP_INFO(get_logger(), "Camera matrix updated:\n[%f, %f, %f]\n[%f, %f, %f]\n[%f, %f, %f]",
		    _camera_matrix.at<double>(0, 0), _camera_matrix.at<double>(0, 1), _camera_matrix.at<double>(0, 2),
		    _camera_matrix.at<double>(1, 0), _camera_matrix.at<double>(1, 1), _camera_matrix.at<double>(1, 2),
		    _camera_matrix.at<double>(2, 0), _camera_matrix.at<double>(2, 1), _camera_matrix.at<double>(2, 2));
	RCLCPP_INFO(get_logger(), "Camera Matrix: fx=%f, fy=%f, cx=%f, cy=%f",
		    _camera_matrix.at<double>(0, 0), // fx
		    _camera_matrix.at<double>(1, 1), // fy
		    _camera_matrix.at<double>(0, 2), // cx
		    _camera_matrix.at<double>(1, 2)  // cy
		   );

	// Check if focal length is zero after update
	if (_camera_matrix.at<double>(0, 0) == 0) {
		RCLCPP_ERROR(get_logger(), "Focal length is zero after update!");

	} else {
		RCLCPP_INFO(get_logger(), "Updated camera intrinsics from camera_info topic.");

		RCLCPP_INFO(get_logger(), "Unsubscribing from camera info topic");
		_camera_info_sub.reset();
	}
}

void ArucoTrackerNode::annotate_image(cv_bridge::CvImagePtr image, const cv::Vec3d& target)
{
	// Annotate the image with the target position and marker size
	std::ostringstream stream;
	stream << std::fixed << std::setprecision(2);
	stream << "X: "  << target[0] << " Y: " << target[1]  << " Z: " << target[2];
	std::string text_xyz = stream.str();

	int fontFace = cv::FONT_HERSHEY_SIMPLEX;
	double fontScale = 1;
	int thickness = 2;
	int baseline = 0;
	cv::Size textSize = cv::getTextSize(text_xyz, fontFace, fontScale, thickness, &baseline);
	baseline += thickness;
	cv::Point textOrg((image->image.cols - textSize.width - 10), (image->image.rows - 10));
	cv::putText(image->image, text_xyz, textOrg, fontFace, fontScale, cv::Scalar(0, 255, 255), thickness, 8);
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ArucoTrackerNode>());
	rclcpp::shutdown();
	return 0;
}