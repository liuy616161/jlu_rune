#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/int64.hpp>
#include "power_rune_debugger.h"
#include "param_manager.h"

namespace power_rune {

class DebugNode : public rclcpp::Node {
public:
    DebugNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("power_rune_debug_node", options)
    {
        RCLCPP_INFO(get_logger(), "Starting Power Rune Debug Node");
        
        // Initialize parameter manager
        ParamManager::getInstance().initialize(shared_from_this());
        
        // Load default config
        std::string config_file = declare_parameter("config_file", "config/armor_debug.yaml");
        if (!ParamManager::getInstance().loadConfig(config_file)) {
            RCLCPP_WARN(get_logger(), "Failed to load config file: %s, using default parameters", 
                      config_file.c_str());
        }
        
        // Create publishers
        result_pub_ = create_publisher<sensor_msgs::msg::Image>("/power_rune/result_image", 10);
        arrow_binary_pub_ = create_publisher<sensor_msgs::msg::Image>("/power_rune/arrow_binary", 10);
        armor_binary_pub_ = create_publisher<sensor_msgs::msg::Image>("/power_rune/armor_binary", 10);
        armor_roi_pub_ = create_publisher<sensor_msgs::msg::Image>("/power_rune/armor_roi", 10);
        
        // Create image subscription
        auto qos = rclcpp::SensorDataQoS();
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", qos,
            std::bind(&DebugNode::onImageReceived, this, std::placeholders::_1));
        
        RCLCPP_INFO(get_logger(), "Power Rune Debug Node initialized");
    }

private:
    void onImageReceived(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        try {
            // Convert ROS Image to OpenCV format
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            cv::Mat frame = cv_ptr->image;
            
            // Process frame
            debugger_.processFrame(frame);
            
            // Publish results
            publishResults(msg->header);
            
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "CV bridge exception: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Exception during image processing: %s", e.what());
        }
    }
    
    void publishResults(const std_msgs::msg::Header& header) {
        // Publish result image
        cv::Mat result_image = debugger_.getResultImage();
        if (!result_image.empty()) {
            auto result_msg = cv_bridge::CvImage(header, "bgr8", result_image).toImageMsg();
            result_pub_->publish(*result_msg);
        }
        
        // Publish arrow binary
        cv::Mat arrow_binary = debugger_.getArrowBinary();
        if (!arrow_binary.empty()) {
            auto arrow_msg = cv_bridge::CvImage(header, "mono8", arrow_binary).toImageMsg();
            arrow_binary_pub_->publish(*arrow_msg);
        }
        
        // Publish armor binary
        cv::Mat armor_binary = debugger_.getArmorBinary();
        if (!armor_binary.empty()) {
            auto armor_msg = cv_bridge::CvImage(header, "mono8", armor_binary).toImageMsg();
            armor_binary_pub_->publish(*armor_msg);
        }
        
        // Publish armor ROI
        cv::Mat armor_roi = debugger_.getArmorRoi();
        if (!armor_roi.empty()) {
            auto roi_msg = cv_bridge::CvImage(header, "bgr8", armor_roi).toImageMsg();
            armor_roi_pub_->publish(*roi_msg);
        }
    }
    
    PowerRuneDebugger debugger_;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr arrow_binary_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr armor_binary_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr armor_roi_pub_;
};

}  // namespace power_rune

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(power_rune::DebugNode)