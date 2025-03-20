#ifndef DEBUG_GUI_H
#define DEBUG_GUI_H

#include <QMainWindow>
#include <QTabWidget>
#include <QLabel>
#include <QSlider>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QTimer>
#include <QComboBox>
#include <map>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "param_manager.h"

namespace power_rune {

class DebugGUI : public QMainWindow {
    Q_OBJECT

public:
    explicit DebugGUI(std::shared_ptr<rclcpp::Node> node, QWidget* parent = nullptr);
    ~DebugGUI();

private slots:
    // 刷新显示
    void updateDisplays();
    
    // 参数调整响应
    void onIntValueChanged(int value);
    void onDoubleValueChanged(double value);
    void onSliderValueChanged(int value);
    
    // 配置操作
    void onSaveConfig();
    void onLoadConfig();
    void onResetConfig();
    
    // 显示切换
    void onTabChanged(int index);
    
        // 曝光设置
    void onExposureChanged(int value);

private:
    // 初始化界面
    void setupUI();
    
    // 创建参数控制界面
    void createParameterControls();
    
    // 接收图像回调
    void onImageReceived(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void onArrowBinaryReceived(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void onArmorBinaryReceived(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void onResultImageReceived(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void onArmorRoiReceived(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    
    // 图像处理与显示
    void displayImage(const cv::Mat& image, QLabel* label);
    
    // ROS节点和订阅
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr arrow_binary_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr armor_binary_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr result_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr armor_roi_sub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr exposure_pub_;
    
    // 图像显示组件
    QTabWidget* image_tabs_;
    QLabel* original_image_label_;
    QLabel* arrow_binary_label_;
    QLabel* armor_binary_label_;
    QLabel* result_image_label_;
    QLabel* armor_roi_label_;
    
    // 状态显示
    QLabel* status_label_;
    QLabel* fps_label_;
    
    // 参数控制组件
    QTabWidget* param_tabs_;
    std::map<std::string, QWidget*> param_controls_;
    
    // 图像缓存
    cv::Mat original_image_;
    cv::Mat arrow_binary_;
    cv::Mat armor_binary_;
    cv::Mat result_image_;
    cv::Mat armor_roi_;
    
    // 刷新定时器
    QTimer* update_timer_;
    
    // 性能统计
    int frame_count_;
    std::chrono::time_point<std::chrono::steady_clock> last_time_;
};

}  // namespace power_rune

#endif // DEBUG_GUI_H