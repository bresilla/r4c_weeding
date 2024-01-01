#include <utility>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "farmbot_interfaces/msg/float32_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include "vision_interfaces/msg/detection_array.hpp"
#include "vision_interfaces/msg/detection.hpp"
#include "vision_interfaces/msg/bounding_box2_d.hpp"


class DetectTracker : public rclcpp::Node {
    private:   
        std::unordered_map<int, std::pair<int, int>> object_dict_back;
        int object_count_back = 0;
        std::unordered_map<int, std::pair<int, int>> object_dict_front;
        int object_count_front = 0;

        message_filters::Subscriber<sensor_msgs::msg::Image> color_front_sub_;
        message_filters::Subscriber<sensor_msgs::msg::Image> depth_front_sub_;
        message_filters::Subscriber<sensor_msgs::msg::Image> color_back_sub_;
        message_filters::Subscriber<sensor_msgs::msg::Image> depth_back_sub_;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr anottated_front_pub_;
        rclcpp::Publisher<vision_interfaces::msg::DetectionArray>::SharedPtr detection_front_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr anottated_back_pub_;
        rclcpp::Publisher<vision_interfaces::msg::DetectionArray>::SharedPtr detection_back_pub_;

        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> sync_left_;
        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> sync_right_;

    public:
        DetectTracker() : Node("image_subscriber") {
            color_front_sub_.subscribe(this, "/Robotti/color_front/image_color");
            depth_front_sub_.subscribe(this, "/Robotti/depth_front/image");
            color_back_sub_.subscribe(this, "/Robotti/color_rear/image_color");
            depth_back_sub_.subscribe(this, "/Robotti/depth_rear/image");

            anottated_front_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/front/color_annotated", 10);
            detection_front_pub_ = this->create_publisher<vision_interfaces::msg::DetectionArray>("/front/detections", 10);
            anottated_back_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/rear/color_annotated", 10);
            detection_back_pub_ = this->create_publisher<vision_interfaces::msg::DetectionArray>("/rear/detections", 10);

            sync_left_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>>(10);
            sync_left_->connectInput(color_front_sub_, depth_front_sub_);
            sync_left_->registerCallback(std::bind(&DetectTracker::camera_front, this, std::placeholders::_1, std::placeholders::_2));

            sync_right_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>>(10);
            sync_right_->connectInput(color_back_sub_, depth_back_sub_);
            sync_right_->registerCallback(std::bind(&DetectTracker::camera_back, this, std::placeholders::_1, std::placeholders::_2));
        }
    
    private:
        void camera_front(const sensor_msgs::msg::Image::ConstSharedPtr& color_msg, const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
            cv::Mat color_img;
            try {
                color_img = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8)->image;
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }
            auto ret = blober(color_img, depth_msg, object_dict_front, object_count_front);
            auto newcv_prt = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", ret.first).toImageMsg();
            anottated_front_pub_->publish(*newcv_prt);
            auto detection = ret.second;
            detection.header.frame_id = "front_camera";
            detection_front_pub_->publish(detection);
        }
        void camera_back(const sensor_msgs::msg::Image::ConstSharedPtr& color_msg, const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
            cv::Mat color_img;
            try {
                color_img = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8)->image;
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }
            auto ret = blober(color_img, depth_msg, object_dict_back, object_count_back);
            auto newcv_prt = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", ret.first).toImageMsg();
            anottated_back_pub_->publish(*newcv_prt);
            auto detection = ret.second;
            detection.header.frame_id = "rear_camera";
            detection_back_pub_->publish(detection);
        }
        int get_object_id(int centroid_x, int centroid_y, const std::unordered_map<int, std::pair<int, int>>& object_dict) {
            for (const auto& object : object_dict) {
                int object_id = object.first;
                int centroid_x_dict = object.second.first;
                int centroid_y_dict = object.second.second;
                double distance = std::sqrt(std::pow(centroid_x - centroid_x_dict, 2) + std::pow(centroid_y - centroid_y_dict, 2));
                if (distance < 50) { return object_id; }
            }
            return -1;
        }

        std::pair<cv::Mat, vision_interfaces::msg::DetectionArray> blober(cv::Mat image, const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg, std::unordered_map<int, std::pair<int, int>>& dict, int& count) {
            int height = image.rows;
            // int width = image.cols;
            cv::Mat img = image;
            cv::GaussianBlur(img, img, cv::Size(9, 9), 0);
            cv::Mat hsv_img;
            cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);

            cv::Scalar bound_lower = cv::Scalar(30, 30, 0);
            cv::Scalar bound_upper = cv::Scalar(90, 255, 255);

            cv::Mat mask_green;
            cv::inRange(hsv_img, bound_lower, bound_upper, mask_green);

            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
            cv::morphologyEx(mask_green, mask_green, cv::MORPH_CLOSE, kernel);
            cv::morphologyEx(mask_green, mask_green, cv::MORPH_OPEN, kernel);
            cv::dilate(mask_green, mask_green, kernel, cv::Point(-1, -1), 1);
            cv::erode(mask_green, mask_green, kernel, cv::Point(-1, -1), 1);

            cv::Mat seg_img;
            cv::bitwise_and(img, img, seg_img, mask_green);

            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hier;
            cv::findContours(mask_green.clone(), contours, hier, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            int min_area = 1000;
            std::vector<std::vector<cv::Point>> large_contours;
            for (const auto& cnt : contours) {
                if (cv::contourArea(cnt) > min_area) {
                    large_contours.push_back(cnt);
                }
            }
            vision_interfaces::msg::DetectionArray detections;
            detections.header.stamp = depth_msg->header.stamp;
            float* depths = (float*)(&depth_msg->data[0]);
            cv::drawContours(seg_img, large_contours, -1, cv::Scalar(0, 255, 0), 5); 
            for (const auto& e : large_contours) {
                cv::Moments moments = cv::moments(e);
                if (moments.m00 != 0) {
                    int x = static_cast<int>(moments.m10 / moments.m00);
                    int y = static_cast<int>(moments.m01 / moments.m00);
                    // if (y < 0.5 * height) { continue;}

                    vision_interfaces::msg::Detection detection;
                    cv::Rect2d rect = cv::boundingRect(e);
                    int object_id = get_object_id(x, y, dict);
                    float distance = depths[x + depth_msg->width * y];
                    cv::circle(seg_img, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
                    if (object_id == -1) {
                        dict[count] = std::make_pair(x, y);
                        count++;
                    } else {
                        dict[object_id] = std::make_pair(x, y);
                    }
                    detection.id = object_id;
                    detection.bbox.center.position.x = x;
                    detection.bbox.center.position.y = y;
                    detection.bbox.distance = distance;
                    detection.bbox.width = rect.width;
                    detection.bbox.height = rect.height;
                    detections.detections.push_back(detection);
                    cv::putText(seg_img, std::to_string(object_id), cv::Point(x+5, y), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 3 );
                }
            }
            return std::make_pair(seg_img, detections);
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectTracker>());
    rclcpp::shutdown();
    return 0;
}
