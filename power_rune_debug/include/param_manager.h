#ifndef PARAM_MANAGER_H
#define PARAM_MANAGER_H

#include <string>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <functional>
#include "Param.h"

namespace power_rune {

class ParamManager {
public:
    using ParamChangedCallback = std::function<void(const std::string&, const YAML::Node&)>;
    
    static ParamManager& getInstance() {
        static ParamManager instance;
        return instance;
    }
    
    // 初始化参数管理器，关联到ROS2节点
    void initialize(std::shared_ptr<rclcpp::Node> node);
    
    // 加载参数文件
    bool loadConfig(const std::string& filename);
    
    // 保存参数到文件
    bool saveConfig(const std::string& filename);
    
    // 更新单个参数
    template<typename T>
    void updateParam(const std::string& param_name, const T& value);
    
    // 获取参数值
    template<typename T>
    T getParam(const std::string& param_name, const T& default_value = T());
    
    // 同步参数值到Param静态成员
    void syncToParam();
    
    // 从Param静态成员同步参数
    void syncFromParam();
    
    // 设置参数变更回调
    void setParamChangedCallback(ParamChangedCallback callback);
    
    // 获取当前配置
    const YAML::Node& getConfig() const { return config_; }

private:
    ParamManager();
    ~ParamManager() = default;
    ParamManager(const ParamManager&) = delete;
    ParamManager& operator=(const ParamManager&) = delete;
    
    // 声明ROS参数
    void declareRosParameters();
    
    // 参数变更回调
    void onParameterEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);
    
    std::shared_ptr<rclcpp::Node> node_;
    YAML::Node config_;
    std::mutex config_mutex_;
    std::string config_file_;
    ParamChangedCallback param_callback_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_event_sub_;
};

}  // namespace power_rune

#endif // PARAM_MANAGER_H