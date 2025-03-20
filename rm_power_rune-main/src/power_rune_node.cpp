#include <thread>

#include "PowerRune.h"
#include "power_rune_node.hpp"

namespace power_rune{


    PowerRuneNode::PowerRuneNode(const rclcpp::NodeOptions & options)
    : Node("power_rune_node", options),
      last_save_time_(this->now())
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
        image_show_pub_=this->create_publisher<sensor_msgs::msg::Image>("img_show",10);
        image_armor_pub_=this->create_publisher<sensor_msgs::msg::Image>("img_armor",10);
        image_arrow_pub_=this->create_publisher<sensor_msgs::msg::Image>("img_arrow",10);
        image_src_pub_=this->create_publisher<sensor_msgs::msg::Image>("img_src",10);
        
        debug_img_timer_=this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PowerRuneNode::publish_debug_img,this)
        );
        #endif

        last_save_time_ = this->now();
    }

    #if SHOW_IMAGE>=1
    void PowerRuneNode::publish_debug_img(){
        auto img_show_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", power_rune_->get_img_show()).toImageMsg();
        image_show_pub_->publish(*img_show_msg);
        auto img_arrow_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", power_rune_->get_img_arrow()).toImageMsg();
        image_arrow_pub_->publish(*img_arrow_msg);
        auto img_armor_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", power_rune_->get_img_armor()).toImageMsg();
        image_armor_pub_->publish(*img_armor_msg);
        auto img_src_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", power_rune_->get_img_src()).toImageMsg();
        image_src_pub_->publish(*img_src_msg);
    }
    #endif

    void PowerRuneNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
    {   

        auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;
        cv::Mat imgbgr=img.clone();
        auto start{std::chrono::steady_clock::now()};

        cv::cvtColor(imgbgr,imgbgr,cv::COLOR_RGB2BGR);

        power_rune_->runOnce(imgbgr, 0.0, 0.0);

        auto future_time = start + std::chrono::milliseconds(1000 / power_rune::Param::FPS);
        if (std::chrono::steady_clock::now() < future_time) {
            std::this_thread::sleep_until(future_time);
        }    
        //
        
        // 检查是否达到保存间隔
        /*rclcpp::Time current_time = this->now();
        if ((current_time - last_save_time_).seconds() >= SAVE_INTERVAL) {
            static int frame_count = 0;
            cv::Mat install = power_rune_->get_img_src().clone();
            if (!install.empty()) {
                //cv::cvtColor(install, install, cv::COLOR_BGR2RGB);
                std::string filename = "/home/tars-go/Documents/src/frame_" + std::to_string(frame_count++) + ".png";
                if (!cv::imwrite(filename, install)) {
                    RCLCPP_WARN(this->get_logger(), "Failed to save image: %s", filename.c_str());
                }
                last_save_time_ = current_time;  // 更新上次保存时间
            }
        }*/
    }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(power_rune::PowerRuneNode)