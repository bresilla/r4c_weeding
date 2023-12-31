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





class ImageSubscriber : public rclcpp::Node {
private:
    farmbot_interfaces::msg::Float32Stamped dist;
    sensor_msgs::msg::NavSatFix gps;
    bool initialized = false;
    sensor_msgs::msg::CameraInfo colot_intrinsics;
    sensor_msgs::msg::CameraInfo depth_intrinsics;
    

    std::unordered_map<int, std::pair<int, int>> object_dict_back;
    int object_count_back = 0;
    std::unordered_map<int, std::pair<int, int>> object_dict_front;
    int object_count_front = 0;


    message_filters::Subscriber<sensor_msgs::msg::Image> color_front_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_front_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> color_back_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_back_sub_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_front_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr dapth_info_front_sub_;

    rclcpp::Subscription<farmbot_interfaces::msg::Float32Stamped>::SharedPtr dist_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr anottated_front_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr anottated_back_pub_;

    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> sync_left_;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> sync_right_;

public:
    ImageSubscriber() : Node("image_subscriber") {
        color_front_sub_.subscribe(this, "/Robotti/color_front/image_color");
        depth_front_sub_.subscribe(this, "/Robotti/depth_front/image");
        color_back_sub_.subscribe(this, "/Robotti/color_rear/image_color");
        depth_back_sub_.subscribe(this, "/Robotti/depth_rear/image");

        dist_sub_ = this->create_subscription<farmbot_interfaces::msg::Float32Stamped>("/fb/loc/dist", 10, std::bind(&ImageSubscriber::distance_callback, this, std::placeholders::_1));
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/fb/loc/fix", 10, std::bind(&ImageSubscriber::gps_callback, this, std::placeholders::_1));
        anottated_front_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/front/color_annotated", 10);
        anottated_back_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/rear/color_annotated", 10);

        sync_left_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>>(10);
        sync_left_->connectInput(color_front_sub_, depth_front_sub_);
        sync_left_->registerCallback(std::bind(&ImageSubscriber::camera_front, this, std::placeholders::_1, std::placeholders::_2));

        sync_right_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>>(10);
        sync_right_->connectInput(color_back_sub_, depth_back_sub_);
        sync_right_->registerCallback(std::bind(&ImageSubscriber::camera_back, this, std::placeholders::_1, std::placeholders::_2));

        camera_info_front_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("/Robotti/color_front/camera_info", 10, std::bind(&ImageSubscriber::camera_info_front_callback, this, std::placeholders::_1));
        dapth_info_front_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("/Robotti/depth_front/camera_info", 10, std::bind(&ImageSubscriber::depth_info_front_callback, this, std::placeholders::_1));
    }
private:
    void distance_callback(const farmbot_interfaces::msg::Float32Stamped::SharedPtr msg) {
        dist = *msg;
    }
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        gps = *msg;
    }
    void camera_info_front_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        colot_intrinsics = *msg;
    }
    void depth_info_front_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        depth_intrinsics = *msg;
        initialized = true;
    }

    void camera_front(const sensor_msgs::msg::Image::ConstSharedPtr& color_msg, const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
        cv::Mat color_img, depth_img;
        try {
            color_img = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8)->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat img = blober(color_img, depth_msg, object_dict_front, object_count_front, -1);
        auto newcv_prt = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
        anottated_front_pub_->publish(*newcv_prt);
    }
    void camera_back(const sensor_msgs::msg::Image::ConstSharedPtr& color_msg, const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
        cv::Mat color_img, depth_img;
        try {
            color_img = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8)->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat img = blober(color_img, depth_msg, object_dict_back, object_count_back, -1);
        auto newcv_prt = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
        anottated_back_pub_->publish(*newcv_prt);
    }

    std::array<double, 4> getFOV() {
        auto camera_info = depth_intrinsics;
        double focal_length_x = camera_info.k[0];
        double image_width = camera_info.width;
        double hfov_rad = 2 * std::atan(image_width / (2 * focal_length_x));
        double hfov_deg = hfov_rad * (180.0 / M_PI);
        std::array<double, 2> hfov = {hfov_deg, hfov_deg};
        double focal_length_y = camera_info.k[4];
        double image_height = camera_info.height;
        double vfov_rad = 2 * std::atan(image_height / (2 * focal_length_y));
        double vfov_deg = vfov_rad * (180.0 / M_PI);
        std::array<double, 2> vfov = {vfov_deg, vfov_deg};
        //return vfov and hfov
        return {hfov[0], hfov[1], vfov[0], vfov[1]};
    }

    std::array<float, 3> calculate_3d_coordinates(float distance, int pixel_x, int pixel_y) {
        const std::array<double, 9>& intrinsic_matrix = depth_intrinsics.k;
        // Assuming pixel_x and pixel_y are coordinates in the depth image
        // Convert pixel coordinates to 3D coordinates using depth information and intrinsic matrix
        float fx = intrinsic_matrix[0];
        float fy = intrinsic_matrix[4];
        float cx = intrinsic_matrix[2];
        float cy = intrinsic_matrix[5];

        int adjusted_y = depth_intrinsics.height - pixel_y;

        float x = (pixel_x - cx) * distance / fx;
        float y = (adjusted_y - cy) * distance / fy;
        float z = distance;

        return {x, y, z};
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

    cv::Mat blober(cv::Mat image, const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg, std::unordered_map<int, std::pair<int, int>>& dict, int& count, int transfrom_x) {
        int height = image.rows;
        int width = image.cols;
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
        
        float* depths = (float*)(&depth_msg->data[0]);
        cv::drawContours(seg_img, large_contours, -1, cv::Scalar(0, 255, 0), 5); 
        for (const auto& e : large_contours) {
            cv::Moments moments = cv::moments(e);
            if (moments.m00 != 0) {
                int x = static_cast<int>(moments.m10 / moments.m00);
                int y = static_cast<int>(moments.m01 / moments.m00);
                if (y < 0.5 * height) { continue;}
                int object_id = get_object_id(x, y, dict);
                float distance = depths[x + depth_msg->width * y];
                auto dist_3d = calculate_3d_coordinates(distance, x, y);
                cv::circle(seg_img, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
                if (object_id == -1) {
                    dict[count] = std::make_pair(x, y);
                    count++;
                } else {
                    dict[object_id] = std::make_pair(x, y);
                }

                std::stringstream ss;
                ss << std::fixed << std::setprecision(2);  // Set precision to two decimal points
                ss << "(" << dist_3d[0] << "," << dist_3d[1] << ")";
                std::string text = ss.str();
                cv::putText(seg_img, text, cv::Point(x+5, y+20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);

                // RCLCPP_INFO(this->get_logger(), "Object ID: %d, x: %d, y: %d, distance: %g", object_id, x, y, distance);
                // RCLCPP_INFO(this->get_logger(), "ID:%d  X:%g, Y:%g, Z:%g", object_id, dist_3d[0], dist_3d[1], dist_3d[2]);
                cv::putText(seg_img, std::to_string(object_id), cv::Point(x+5, y), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 3 );
            }
        }
        return seg_img;
    }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}
