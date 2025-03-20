#include <rclcpp/rclcpp.hpp>
#include <QApplication>
#include <thread>
#include "debug_gui.h"
#include "param_manager.h"

using namespace power_rune;

int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("power_rune_debug_gui");
    
    // Initialize parameter manager
    ParamManager::getInstance().initialize(node);
    
    // Load default config
    if (!ParamManager::getInstance().loadConfig("config/armor_debug.yaml")) {
        RCLCPP_WARN(node->get_logger(), "Failed to load default config file, using built-in defaults");
    }
    
    // Initialize Qt application
    QApplication app(argc, argv);
    
    // Create debug GUI
    DebugGUI gui(node);
    gui.show();
    
    // Create ROS2 executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    // Run ROS2 spinner in a separate thread
    std::thread ros_thread([&executor]() {
        executor.spin();
    });
    
    // Start Qt event loop
    int result = app.exec();
    
    // Clean up
    rclcpp::shutdown();
    if (ros_thread.joinable()) {
        ros_thread.join();
    }
    
    return result;
}