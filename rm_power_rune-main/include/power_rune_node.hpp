
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

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

    std::unique_ptr<PowerRune> power_rune_;

    //debug show
    image_transport::Publisher  image_show_pub_;
    image_transport::Publisher  image_arrow_pub_;
    image_transport::Publisher  image_armor_pub_;
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