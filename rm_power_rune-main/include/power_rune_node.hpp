
// ROS
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "PowerRune.h"

namespace power_rune{

class PowerRuneNode : public rclcpp::Node
{
public:
    PowerRuneNode(const rclcpp::NodeOptions & options);
private:

      // 添加上次保存时间的成员变量
    rclcpp::Time last_save_time_;
    // 添加保存间隔的常量（1秒）
    const double SAVE_INTERVAL = 1.0;  // 单位：秒
    
    

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

    std::unique_ptr<PowerRune> power_rune_;

    //debug show
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr  image_show_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr  image_arrow_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr  image_armor_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr  image_src_pub_;
    rclcpp::TimerBase::SharedPtr debug_img_timer_;
    
    void publish_debug_img();

    // Camera info part
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    cv::Point2f cam_center_;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;

    // Image subscrpition
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

};

}
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(power_rune::PowerRuneNode)