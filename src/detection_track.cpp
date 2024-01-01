#include <Eigen/Dense>
#include <fmt/format.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "farmbot_interfaces/msg/float32_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include "vision_interfaces/msg/detection_array.hpp"
#include "image_geometry/pinhole_camera_model.h"

class DetectTracker : public rclcpp::Node {
    private:
        sensor_msgs::msg::NavSatFix gps;
        bool front_initialized = false;
        sensor_msgs::msg::CameraInfo cam_front_info;
        bool back_initialized = false;
        sensor_msgs::msg::CameraInfo cam_back_info;
        geometry_msgs::msg::PoseStamped odom_pose;
        geometry_msgs::msg::PoseStamped ref_pose;

        std::array<float, 6> front_translation = {1.0, 0.0, 1.2};
        geometry_msgs::msg::PoseStamped front_camera_pose;
        std::array<float, 6> back_translation = {-1.0, 0.0, 1.2};
        geometry_msgs::msg::PoseStamped back_camera_pose;

        message_filters::Subscriber<sensor_msgs::msg::Image> color_front_sub_;
        message_filters::Subscriber<vision_interfaces::msg::DetectionArray> detection_front_sub_;
        message_filters::Subscriber<sensor_msgs::msg::Image> color_back_sub_;
        message_filters::Subscriber<vision_interfaces::msg::DetectionArray> detection_back_sub_;

        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_front_info_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_back_info_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ref_sub_;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr anottated_front_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr anottated_back_pub_;

        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, vision_interfaces::msg::DetectionArray>>> sync_left_;
        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, vision_interfaces::msg::DetectionArray>>> sync_right_;
    public:
        DetectTracker() : Node("detect_tracker") {
            color_front_sub_.subscribe(this, "/Robotti/color_front/image_color");
            detection_front_sub_.subscribe(this, "/front/detections");
            color_back_sub_.subscribe(this, "/Robotti/color_rear/image_color");
            detection_back_sub_.subscribe(this, "/rear/detections");

            gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/fb/loc/fix", 10, std::bind(&DetectTracker::gps_callback, this, std::placeholders::_1));
            cam_front_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("/Robotti/color_front/camera_info", 10, std::bind(&DetectTracker::cam_front_callback, this, std::placeholders::_1));
            cam_back_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("/Robotti/color_rear/camera_info", 10, std::bind(&DetectTracker::cam_rear_callback, this, std::placeholders::_1));
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/fb/loc/odom", 10, std::bind(&DetectTracker::odom_callback, this, std::placeholders::_1));
            ref_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/fb/loc/ref", 10, std::bind(&DetectTracker::odom_callback, this, std::placeholders::_1));

            anottated_front_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/front/color_annotated_2", 10);
            anottated_back_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/rear/color_annotated_2", 10);

            sync_left_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, vision_interfaces::msg::DetectionArray>>>(10);
            sync_left_->connectInput(color_front_sub_, detection_front_sub_);
            sync_left_->registerCallback(std::bind(&DetectTracker::camera_front, this, std::placeholders::_1, std::placeholders::_2));

            sync_right_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, vision_interfaces::msg::DetectionArray>>>(10);
            sync_right_->connectInput(color_back_sub_, detection_back_sub_);
            sync_right_->registerCallback(std::bind(&DetectTracker::camera_back, this, std::placeholders::_1, std::placeholders::_2));
        }
    private:
        void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) { gps = *msg; }
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) { odom_pose.pose = msg->pose.pose; odom_pose.header = msg->header; }
        void ref_callback(const nav_msgs::msg::Odometry::SharedPtr msg) { ref_pose.pose = msg->pose.pose; ref_pose.header = msg->header; }
        void cam_front_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            front_initialized = true;
            cam_front_info = *msg;
            cam_front_info_.reset();
            RCUTILS_LOG_INFO("FRONT CAMERA INFO RECEIVED, CLOSING SUBSCRIPTION");
            front_camera_pose.pose.position.x = 1.0;
            front_camera_pose.pose.position.y = 0.0;
            front_camera_pose.pose.position.z = 1.2;
            // front_camera_pose.pose.orientation.w = 0.965926;
            // front_camera_pose.pose.orientation.x = 0.0;
            // front_camera_pose.pose.orientation.y = 0.258819;
            // front_camera_pose.pose.orientation.z = 0.0;
        }
        void cam_rear_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            back_initialized = true;
            cam_back_info = *msg;
            cam_back_info_.reset();
            RCUTILS_LOG_INFO("REAR CAMERA INFO RECEIVED, CLOSING SUBSCRIPTION");
            front_camera_pose.pose.position.x = -1.0;
            front_camera_pose.pose.position.y = 0.0;
            front_camera_pose.pose.position.z = 1.2;
            // front_camera_pose.pose.orientation.w = 1.32679e-06;
            // front_camera_pose.pose.orientation.x = -0.258819;
            // front_camera_pose.pose.orientation.y = 3.434e-07;
            // front_camera_pose.pose.orientation.z = 0.965926;
        }
        void camera_front(const sensor_msgs::msg::Image::ConstSharedPtr& color_msg, const vision_interfaces::msg::DetectionArray::ConstSharedPtr& detection_msg) {
            if (!front_initialized) { return; }
            // RCUTILS_LOG_INFO("Front camera callback");
            cv::Mat color_img;
            try {
                color_img = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8)->image;
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }
            float x, y, z;
            geometry_msgs::msg::PoseStamped detection_pose;
            detection_pose.header.frame_id = "map";
            detection_pose.header.stamp = this->now();
            for (auto detection : detection_msg->detections) {
                // detection_pose = coordinates_in3d(detection.bbox.center.position.x, detection.bbox.center.position.y, detection.bbox.distance);
                detection_pose = coordinates_in3d(detection.bbox.center.position.x, detection.bbox.center.position.y, detection.bbox.distance, cam_front_info);
                // detection_pose = transform_point(detection_pose, front_camera_pose);
                // detection_pose = transform_point(detection_pose, odom_pose);

                x = detection_pose.pose.position.x;
                y = detection_pose.pose.position.y;
                z = detection_pose.pose.position.z;

                std::string text = fmt::format("{:.2f}", x) + ", " + fmt::format("{:.2f}", y) + ", " + fmt::format("{:.2f}", z);
                cv::circle(color_img, cv::Point(detection.bbox.center.position.x, detection.bbox.center.position.y), 5, cv::Scalar(0, 0, 255), -1);
                cv::putText(color_img, text, cv::Point(detection.bbox.center.position.x - detection.bbox.width/2, detection.bbox.center.position.y - detection.bbox.height/2 - 5), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 2);
                cv::rectangle(color_img, cv::Point(detection.bbox.center.position.x - detection.bbox.width/2, detection.bbox.center.position.y - detection.bbox.height/2), cv::Point(detection.bbox.center.position.x + detection.bbox.width/2, detection.bbox.center.position.y + detection.bbox.height/2), cv::Scalar(0, 255, 0), 2); 
                // cv::putText(color_img, std::to_string(detection.id), cv::Point(detection.bbox.center.position.x - detection.bbox.width/2, detection.bbox.center.position.y - detection.bbox.height/2), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
                // RCLCPP_INFO(this->get_logger(), "%i --> %f, %f, %f", detection.id, x, y, z);
            }
            RCUTILS_LOG_INFO("----------------------------");
            auto newcv_prt = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", color_img).toImageMsg();
            anottated_front_pub_->publish(*newcv_prt);
        }
        void camera_back(const sensor_msgs::msg::Image::ConstSharedPtr& color_msg, const vision_interfaces::msg::DetectionArray::ConstSharedPtr& detection_msg) {            
        }
        
        geometry_msgs::msg::PoseStamped coordinates_in3d(int pixel_x, int pixel_y, float distance) {
            const std::array<double, 9>& intrinsic_matrix = cam_front_info.k;
            float fx = intrinsic_matrix[0];
            float fy = intrinsic_matrix[4];
            float cx = intrinsic_matrix[2];
            float cy = intrinsic_matrix[5];
            float x = (pixel_x - cx) * distance / fx;
            float y = (pixel_y - cy) * distance / fy;
            float z = distance;
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = z;
            return pose;
        }

        geometry_msgs::msg::PoseStamped coordinates_in3d(int pixel_x, int pixel_y, float distance, sensor_msgs::msg::CameraInfo cam_info) {
            image_geometry::PinholeCameraModel cam_model;
            cam_model.fromCameraInfo(cam_front_info);
            cv::Point3d ray = cam_model.projectPixelTo3dRay(cv::Point2d(pixel_x, pixel_y));
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = ray.x * distance;
            pose.pose.position.y = ray.y * distance;
            pose.pose.position.z = ray.z * distance;
            return pose;
        }
        
        geometry_msgs::msg::PoseStamped transform_point(geometry_msgs::msg::PoseStamped source, geometry_msgs::msg::PoseStamped target) {
            Eigen::Vector4f source_translation = {source.pose.position.x, source.pose.position.y, source.pose.position.z, 1};
            Eigen::Matrix4f target_mat = Eigen::Matrix4f::Identity();
            target_mat(1,3) = target.pose.position.y;
            target_mat(2,3) = target.pose.position.z;
            target_mat(0,3) = target.pose.position.x; 
            Eigen::Quaternionf target_quat(target.pose.orientation.w, target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z);
            Eigen::Matrix3f target_rot = target_quat.toRotationMatrix();
            target_mat.block<3,3>(0,0) = target_rot;
            Eigen::Vector4f transformed_point = target_mat * source_translation;
            geometry_msgs::msg::PoseStamped transformed_pose;
            transformed_pose.pose.position.x = transformed_point[0];
            transformed_pose.pose.position.y = transformed_point[1];
            transformed_pose.pose.position.z = transformed_point[2];
            return transformed_pose;
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectTracker>());
    rclcpp::shutdown();
    return 0;
}
 