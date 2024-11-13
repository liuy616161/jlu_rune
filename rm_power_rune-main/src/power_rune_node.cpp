#include <thread>

#include "PowerRune.h"
#include "power_rune_node.hpp"

namespace power_rune{


    PowerRuneNode::PowerRuneNode(const rclcpp::NodeOptions & options)
    : Node("power_rune",options)
    {
        RCLCPP_INFO(this->get_logger(),"Starting PowerRuneNode !");

        // Power_Rune
        power_rune_ = std::make_unique<PowerRune>();

        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
        //cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
        //cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
        //pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
        
        Param::IMAGE_WIDTH = camera_info -> width;

        RCLCPP_INFO(this->get_logger(), "Param:   width:%d   height:%d",camera_info -> width,camera_info -> height );

        Param::IMAGE_HEIGHT = camera_info -> height;


        //在power_rune/config.yaml中更正tvec_c2g
        for(int i=0;i<9;i++) Param::INTRINSIC_MATRIX.at<double>(i/3,i%3)=camera_info->k[i];
        for(int i=0;i<5;i++) Param::DIST_COEFFS.at<double>(i)=camera_info->d[i];
        //Param::INTRINSIC_MATRIX = camera_info -> k;
        //哈工大为opencv格式，君为ros格式，此处需要改正
        //Param::DIST_COEFFS = camera_info -> d;
        
        cam_info_sub_.reset();
        });

        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&PowerRuneNode::imageCallback, this, std::placeholders::_1));

        #if SHOW_IMAGE>=1
        image_show_pub_=image_transport::create_publisher(this,"img_show");
        image_armor_pub_=image_transport::create_publisher(this,"img_armor");
        image_arrow_pub_=image_transport::create_publisher(this,"img_arrow");
        
        debug_img_timer_=this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PowerRuneNode::publish_debug_img,this)
        );
        #endif

        
    }

    #if SHOW_IMAGE>=1
    void PowerRuneNode::publish_debug_img(){
        auto img_show_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", power_rune_->get_img_show()).toImageMsg();
        image_show_pub_.publish(*img_show_msg);
        auto img_arrow_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", power_rune_->get_img_arrow()).toImageMsg();
        image_arrow_pub_.publish(*img_arrow_msg);
        auto img_armor_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", power_rune_->get_img_armor()).toImageMsg();
        image_armor_pub_.publish(*img_armor_msg);
    }
    #endif

    void PowerRuneNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
    {   

        auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

        auto start{std::chrono::steady_clock::now()};

        power_rune_->runOnce(img, 0.0, 0.0);

        auto future_time = start + std::chrono::milliseconds(1000 / power_rune::Param::FPS);
        if (std::chrono::steady_clock::now() < future_time) {
            std::this_thread::sleep_until(future_time);
        }    

    }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(power_rune::PowerRuneNode)