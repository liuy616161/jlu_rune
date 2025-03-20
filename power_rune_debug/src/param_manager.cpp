#include "param_manager.h"
#include <fstream>
#include <iostream>

namespace power_rune {

ParamManager::ParamManager() 
    : config_file_("") 
{
}

void ParamManager::initialize(std::shared_ptr<rclcpp::Node> node) {
    node_ = node;
    
    // 订阅参数事件
    param_event_sub_ = node_->create_subscription<rcl_interfaces::msg::ParameterEvent>(
        "/parameter_events", 
        10, 
        std::bind(&ParamManager::onParameterEvent, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(node_->get_logger(), "ParamManager initialized");
}

bool ParamManager::loadConfig(const std::string& filename) {
    try {
        std::lock_guard<std::mutex> lock(config_mutex_);
        config_ = YAML::LoadFile(filename);
        config_file_ = filename;
        
        // 同步到Param静态成员
        syncToParam();
        
        // 如果已关联节点，声明ROS参数
        if (node_) {
            declareRosParameters();
        }
        
        return true;
    } catch (const YAML::Exception& e) {
        if (node_) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load config file: %s", e.what());
        }
        return false;
    }
}

bool ParamManager::saveConfig(const std::string& filename) {
    try {
        // 先从Param同步最新值
        syncFromParam();
        
        std::lock_guard<std::mutex> lock(config_mutex_);
        std::string save_file = filename.empty() ? config_file_ : filename;
        
        if (save_file.empty()) {
            if (node_) {
                RCLCPP_ERROR(node_->get_logger(), "No filename specified for saving config");
            }
            return false;
        }
        
        std::ofstream fout(save_file);
        if (!fout.is_open()) {
            if (node_) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to open file for writing: %s", save_file.c_str());
            }
            return false;
        }
        
        fout << config_;
        
        if (node_) {
            RCLCPP_INFO(node_->get_logger(), "Config saved to: %s", save_file.c_str());
        }
        
        return true;
    } catch (const std::exception& e) {
        if (node_) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to save config: %s", e.what());
        }
        return false;
    }
}

template<typename T>
void ParamManager::updateParam(const std::string& param_name, const T& value) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    
    // 解析参数路径
    std::istringstream ss(param_name);
    std::string segment;
    std::vector<std::string> segments;
    
    while (std::getline(ss, segment, '.')) {
        segments.push_back(segment);
    }
    
    // 更新配置
    YAML::Node* current = &config_;
    for (size_t i = 0; i < segments.size() - 1; ++i) {
        if (!(*current)[segments[i]]) {
            (*current)[segments[i]] = YAML::Node(YAML::NodeType::Map);
        }
        current = &(*current)[segments[i]];
    }
    
    (*current)[segments.back()] = value;
    
    // 更新ROS参数
    if (node_) {
        try {
            node_->set_parameter(rclcpp::Parameter(param_name, value));
        } catch (const rclcpp::exceptions::ParameterNotDeclaredException&) {
            // 如果参数未声明，先声明再设置
            rcl_interfaces::msg::ParameterDescriptor desc;
            desc.description = "Dynamic parameter";
            node_->declare_parameter(param_name, value, desc);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to set parameter %s: %s", 
                        param_name.c_str(), e.what());
        }
    }
    
    // 同步到Param静态成员
    syncToParam();
    
    // 触发回调
    if (param_callback_) {
        param_callback_(param_name, (*current)[segments.back()]);
    }
}

template<typename T>
T ParamManager::getParam(const std::string& param_name, const T& default_value) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    
    // 解析参数路径
    std::istringstream ss(param_name);
    std::string segment;
    std::vector<std::string> segments;
    
    while (std::getline(ss, segment, '.')) {
        segments.push_back(segment);
    }
    
    // 查询配置
    YAML::Node current = config_;
    for (const auto& seg : segments) {
        if (!current[seg]) {
            return default_value;
        }
        current = current[seg];
    }
    
    try {
        return current.as<T>();
    } catch (const YAML::Exception&) {
        return default_value;
    }
}

void ParamManager::syncToParam() {
    // 根据头文件同步参数到Param静态成员
    if (!config_.IsNull()) {
        if (config_["detect"]["brightness_threshold"]["blue"]["arrow"]) {
            Param::ARROW_BRIGHTNESS_THRESHOLD = config_["detect"]["brightness_threshold"]["blue"]["arrow"].as<int>();
        }
        
        if (config_["detect"]["brightness_threshold"]["blue"]["armor"]) {
            Param::ARMOR_BRIGHTNESS_THRESHOLD = config_["detect"]["brightness_threshold"]["blue"]["armor"].as<int>();
        }
        
        if (config_["detect"]["local_roi"]["distance_ratio"]) {
            Param::LOCAL_ROI_DISTANCE_RATIO = config_["detect"]["local_roi"]["distance_ratio"].as<double>();
        }
        
        if (config_["detect"]["local_roi"]["width"]) {
            Param::LOCAL_ROI_WIDTH = config_["detect"]["local_roi"]["width"].as<float>();
        }
        
        if (config_["detect"]["armor_center_vertical_distance_threshold"]) {
            Param::ARMOR_CENTER_VERTICAL_DISTANCE_THRESHOLD = 
                config_["detect"]["armor_center_vertical_distance_threshold"].as<double>();
        }
        
        if (config_["detect"]["global_roi_length_ratio"]) {
            Param::GLOBAL_ROI_LENGTH_RATIO = config_["detect"]["global_roi_length_ratio"].as<double>();
        }
        
        // 箭头参数
        if (config_["detect"]["arrow"]["lightline"]["area"]["min"]) {
            Param::MIN_ARROW_LIGHTLINE_AREA = config_["detect"]["arrow"]["lightline"]["area"]["min"].as<double>();
        }
        
        if (config_["detect"]["arrow"]["lightline"]["area"]["max"]) {
            Param::MAX_ARROW_LIGHTLINE_AREA = config_["detect"]["arrow"]["lightline"]["area"]["max"].as<double>();
        }
        
        if (config_["detect"]["arrow"]["lightline"]["aspect_ratio_max"]) {
            Param::MAX_ARROW_LIGHTLINE_ASPECT_RATIO = 
                config_["detect"]["arrow"]["lightline"]["aspect_ratio_max"].as<double>();
        }
        
        if (config_["detect"]["arrow"]["lightline"]["num"]["min"]) {
            Param::MIN_ARROW_LIGHTLINE_NUM = config_["detect"]["arrow"]["lightline"]["num"]["min"].as<int>();
        }
        
        if (config_["detect"]["arrow"]["lightline"]["num"]["max"]) {
            Param::MAX_ARROW_LIGHTLINE_NUM = config_["detect"]["arrow"]["lightline"]["num"]["max"].as<int>();
        }
        
        if (config_["detect"]["arrow"]["same_area_ratio_max"]) {
            Param::MAX_SAME_ARROW_AREA_RATIO = config_["detect"]["arrow"]["same_area_ratio_max"].as<double>();
        }
        
        if (config_["detect"]["arrow"]["aspect_ratio"]["min"]) {
            Param::MIN_ARROW_ASPECT_RATIO = config_["detect"]["arrow"]["aspect_ratio"]["min"].as<double>();
        }
        
        if (config_["detect"]["arrow"]["aspect_ratio"]["max"]) {
            Param::MAX_ARROW_ASPECT_RATIO = config_["detect"]["arrow"]["aspect_ratio"]["max"].as<double>();
        }
        
        if (config_["detect"]["arrow"]["area_max"]) {
            Param::MAX_ARROW_AREA = config_["detect"]["arrow"]["area_max"].as<double>();
        }
        
        // 装甲板参数
        if (config_["detect"]["armor"]["lightline"]["area"]["min"]) {
            Param::MIN_ARMOR_LIGHTLINE_AREA = config_["detect"]["armor"]["lightline"]["area"]["min"].as<double>();
        }
        
        if (config_["detect"]["armor"]["lightline"]["area"]["max"]) {
            Param::MAX_ARMOR_LIGHTLINE_AREA = config_["detect"]["armor"]["lightline"]["area"]["max"].as<double>();
        }
        
        if (config_["detect"]["armor"]["lightline"]["contour_area"]["min"]) {
            Param::MIN_ARMOR_LIGHTLINE_CONTOUR_AREA = 
                config_["detect"]["armor"]["lightline"]["contour_area"]["min"].as<double>();
        }
        
        if (config_["detect"]["armor"]["lightline"]["contour_area"]["max"]) {
            Param::MAX_ARMOR_LIGHTLINE_CONTOUR_AREA = 
                config_["detect"]["armor"]["lightline"]["contour_area"]["max"].as<double>();
        }
        
        if (config_["detect"]["armor"]["lightline"]["aspect_ratio"]["min"]) {
            Param::MIN_ARMOR_LIGHTLINE_ASPECT_RATIO = 
                config_["detect"]["armor"]["lightline"]["aspect_ratio"]["min"].as<double>();
        }
        
        if (config_["detect"]["armor"]["lightline"]["aspect_ratio"]["max"]) {
            Param::MAX_ARMOR_LIGHTLINE_ASPECT_RATIO = 
                config_["detect"]["armor"]["lightline"]["aspect_ratio"]["max"].as<double>();
        }
        
        if (config_["detect"]["armor"]["same"]["area_ratio_max"]) {
            Param::MAX_SAME_ARMOR_AREA_RATIO = config_["detect"]["armor"]["same"]["area_ratio_max"].as<double>();
        }
        
        if (config_["detect"]["armor"]["same"]["distance"]["min"]) {
            Param::MIN_SAME_ARMOR_DISTANCE = config_["detect"]["armor"]["same"]["distance"]["min"].as<double>();
        }
        
        if (config_["detect"]["armor"]["same"]["distance"]["max"]) {
            Param::MAX_SAME_ARMOR_DISTANCE = config_["detect"]["armor"]["same"]["distance"]["max"].as<double>();
        }
        
        // 中心R参数
        if (config_["detect"]["centerR"]["area"]["min"]) {
            Param::MIN_CENTER_AREA = config_["detect"]["centerR"]["area"]["min"].as<double>();
        }
        
        if (config_["detect"]["centerR"]["area"]["max"]) {
            Param::MAX_CENTER_AREA = config_["detect"]["centerR"]["area"]["max"].as<double>();
        }
        
        if (config_["detect"]["centerR"]["aspect_ratio_max"]) {
            Param::MAX_CENTER_ASPECT_RATIO = config_["detect"]["centerR"]["aspect_ratio_max"].as<double>();
        }
    }
}

void ParamManager::syncFromParam() {
    // 从Param静态成员同步到配置
    std::lock_guard<std::mutex> lock(config_mutex_);
    
    config_["detect"]["brightness_threshold"]["blue"]["arrow"] = Param::ARROW_BRIGHTNESS_THRESHOLD;
    config_["detect"]["brightness_threshold"]["blue"]["armor"] = Param::ARMOR_BRIGHTNESS_THRESHOLD;
    config_["detect"]["local_roi"]["distance_ratio"] = Param::LOCAL_ROI_DISTANCE_RATIO;
    config_["detect"]["local_roi"]["width"] = Param::LOCAL_ROI_WIDTH;
    config_["detect"]["armor_center_vertical_distance_threshold"] = Param::ARMOR_CENTER_VERTICAL_DISTANCE_THRESHOLD;
    config_["detect"]["global_roi_length_ratio"] = Param::GLOBAL_ROI_LENGTH_RATIO;
    
    // 箭头参数
    config_["detect"]["arrow"]["lightline"]["area"]["min"] = Param::MIN_ARROW_LIGHTLINE_AREA;
    config_["detect"]["arrow"]["lightline"]["area"]["max"] = Param::MAX_ARROW_LIGHTLINE_AREA;
    config_["detect"]["arrow"]["lightline"]["aspect_ratio_max"] = Param::MAX_ARROW_LIGHTLINE_ASPECT_RATIO;
    config_["detect"]["arrow"]["lightline"]["num"]["min"] = Param::MIN_ARROW_LIGHTLINE_NUM;
    config_["detect"]["arrow"]["lightline"]["num"]["max"] = Param::MAX_ARROW_LIGHTLINE_NUM;
    config_["detect"]["arrow"]["same_area_ratio_max"] = Param::MAX_SAME_ARROW_AREA_RATIO;
    config_["detect"]["arrow"]["aspect_ratio"]["min"] = Param::MIN_ARROW_ASPECT_RATIO;
    config_["detect"]["arrow"]["aspect_ratio"]["max"] = Param::MAX_ARROW_ASPECT_RATIO;
    config_["detect"]["arrow"]["area_max"] = Param::MAX_ARROW_AREA;
    
    // 装甲板参数
    config_["detect"]["armor"]["lightline"]["area"]["min"] = Param::MIN_ARMOR_LIGHTLINE_AREA;
    config_["detect"]["armor"]["lightline"]["area"]["max"] = Param::MAX_ARMOR_LIGHTLINE_AREA;
    config_["detect"]["armor"]["lightline"]["contour_area"]["min"] = Param::MIN_ARMOR_LIGHTLINE_CONTOUR_AREA;
    config_["detect"]["armor"]["lightline"]["contour_area"]["max"] = Param::MAX_ARMOR_LIGHTLINE_CONTOUR_AREA;
    config_["detect"]["armor"]["lightline"]["aspect_ratio"]["min"] = Param::MIN_ARMOR_LIGHTLINE_ASPECT_RATIO;
    config_["detect"]["armor"]["lightline"]["aspect_ratio"]["max"] = Param::MAX_ARMOR_LIGHTLINE_ASPECT_RATIO;
    config_["detect"]["armor"]["same"]["area_ratio_max"] = Param::MAX_SAME_ARMOR_AREA_RATIO;
    config_["detect"]["armor"]["same"]["distance"]["min"] = Param::MIN_SAME_ARMOR_DISTANCE;
    config_["detect"]["armor"]["same"]["distance"]["max"] = Param::MAX_SAME_ARMOR_DISTANCE;
    
    // 中心R参数
    config_["detect"]["centerR"]["area"]["min"] = Param::MIN_CENTER_AREA;
    config_["detect"]["centerR"]["area"]["max"] = Param::MAX_CENTER_AREA;
    config_["detect"]["centerR"]["aspect_ratio_max"] = Param::MAX_CENTER_ASPECT_RATIO;
}

void ParamManager::setParamChangedCallback(ParamChangedCallback callback) {
    param_callback_ = callback;
}

void ParamManager::declareRosParameters() {
    if (!node_) return;
    
    // 声明参数的辅助函数
    auto declareParam = [this](const std::string& name, const auto& value) {
        try {
            rclcpp::ParameterType type;
            if (std::is_same<decltype(value), int>::value || 
                std::is_same<decltype(value), int64_t>::value) {
                type = rclcpp::ParameterType::PARAMETER_INTEGER;
            } else if (std::is_same<decltype(value), double>::value ||
                       std::is_same<decltype(value), float>::value) {
                type = rclcpp::ParameterType::PARAMETER_DOUBLE;
            } else if (std::is_same<decltype(value), bool>::value) {
                type = rclcpp::ParameterType::PARAMETER_BOOL;
            } else if (std::is_same<decltype(value), std::string>::value) {
                type = rclcpp::ParameterType::PARAMETER_STRING;
            } else {
                RCLCPP_WARN(node_->get_logger(), "Unsupported parameter type for %s", name.c_str());
                return;
            }
            
            rcl_interfaces::msg::ParameterDescriptor desc;
            desc.description = "Configured from YAML file";
            desc.type = type;
            
            if (!node_->has_parameter(name)) {
                node_->declare_parameter(name, value, desc);
                RCLCPP_DEBUG(node_->get_logger(), "Declared parameter %s", name.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to declare parameter %s: %s", name.c_str(), e.what());
        }
    };
    
    // 声明参数
    declareParam("detect.brightness_threshold.blue.arrow", Param::ARROW_BRIGHTNESS_THRESHOLD);
    declareParam("detect.brightness_threshold.blue.armor", Param::ARMOR_BRIGHTNESS_THRESHOLD);
    declareParam("detect.local_roi.distance_ratio", Param::LOCAL_ROI_DISTANCE_RATIO);
    declareParam("detect.local_roi.width", Param::LOCAL_ROI_WIDTH);
    declareParam("detect.armor_center_vertical_distance_threshold", Param::ARMOR_CENTER_VERTICAL_DISTANCE_THRESHOLD);
    declareParam("detect.global_roi_length_ratio", Param::GLOBAL_ROI_LENGTH_RATIO);
    
    // 箭头参数
    declareParam("detect.arrow.lightline.area.min", Param::MIN_ARROW_LIGHTLINE_AREA);
    declareParam("detect.arrow.lightline.area.max", Param::MAX_ARROW_LIGHTLINE_AREA);
    declareParam("detect.arrow.lightline.aspect_ratio_max", Param::MAX_ARROW_LIGHTLINE_ASPECT_RATIO);
    declareParam("detect.arrow.lightline.num.min", Param::MIN_ARROW_LIGHTLINE_NUM);
    declareParam("detect.arrow.lightline.num.max", Param::MAX_ARROW_LIGHTLINE_NUM);
    declareParam("detect.arrow.same_area_ratio_max", Param::MAX_SAME_ARROW_AREA_RATIO);
    declareParam("detect.arrow.aspect_ratio.min", Param::MIN_ARROW_ASPECT_RATIO);
    declareParam("detect.arrow.aspect_ratio.max", Param::MAX_ARROW_ASPECT_RATIO);
    declareParam("detect.arrow.area_max", Param::MAX_ARROW_AREA);
    
    // 装甲板参数
    declareParam("detect.armor.lightline.area.min", Param::MIN_ARMOR_LIGHTLINE_AREA);
    declareParam("detect.armor.lightline.area.max", Param::MAX_ARMOR_LIGHTLINE_AREA);
    declareParam("detect.armor.lightline.contour_area.min", Param::MIN_ARMOR_LIGHTLINE_CONTOUR_AREA);
    declareParam("detect.armor.lightline.contour_area.max", Param::MAX_ARMOR_LIGHTLINE_CONTOUR_AREA);
    declareParam("detect.armor.lightline.aspect_ratio.min", Param::MIN_ARMOR_LIGHTLINE_ASPECT_RATIO);
    declareParam("detect.armor.lightline.aspect_ratio.max", Param::MAX_ARMOR_LIGHTLINE_ASPECT_RATIO);
    declareParam("detect.armor.same.area_ratio_max", Param::MAX_SAME_ARMOR_AREA_RATIO);
    declareParam("detect.armor.same.distance.min", Param::MIN_SAME_ARMOR_DISTANCE);
    declareParam("detect.armor.same.distance.max", Param::MAX_SAME_ARMOR_DISTANCE);
    
    // 中心R参数
    declareParam("detect.centerR.area.min", Param::MIN_CENTER_AREA);
    declareParam("detect.centerR.area.max", Param::MAX_CENTER_AREA);
    declareParam("detect.centerR.aspect_ratio_max", Param::MAX_CENTER_ASPECT_RATIO);
}

void ParamManager::onParameterEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
    // 只处理当前节点的参数变更
    if (event->node != node_->get_fully_qualified_name()) {
        return;
    }
    
    // 处理变更的参数
    for (const auto& changed_param : event->changed_parameters) {
        const std::string& name = changed_param.name;
        const auto& value = changed_param.value;
        
        // 查找配置中的参数
        std::istringstream ss(name);
        std::string segment;
        std::vector<std::string> segments;
        
        while (std::getline(ss, segment, '.')) {
            segments.push_back(segment);
        }
        
        if (segments.empty()) continue;
        
        // 更新配置
        std::lock_guard<std::mutex> lock(config_mutex_);
        YAML::Node* node = &config_;
        for (size_t i = 0; i < segments.size() - 1; ++i) {
            if (!(*node)[segments[i]]) {
                (*node)[segments[i]] = YAML::Node(YAML::NodeType::Map);
            }
            node = &(*node)[segments[i]];
        }
        
        // 根据类型设置值
        switch (value.type) {
            case rclcpp::ParameterType::PARAMETER_BOOL:
                (*node)[segments.back()] = value.bool_value;
                break;
            case rclcpp::ParameterType::PARAMETER_INTEGER:
                (*node)[segments.back()] = value.integer_value;
                break;
            case rclcpp::ParameterType::PARAMETER_DOUBLE:
                (*node)[segments.back()] = value.double_value;
                break;
            case rclcpp::ParameterType::PARAMETER_STRING:
                (*node)[segments.back()] = value.string_value;
                break;
            default:
                RCLCPP_WARN(node_->get_logger(), "Unsupported parameter type for %s", name.c_str());
                continue;
        }
        
        // 同步到Param静态成员
        syncToParam();
        
        // 触发回调
        if (param_callback_) {
            param_callback_(name, (*node)[segments.back()]);
        }
    }
}

// 模板实例化
template void ParamManager::updateParam<int>(const std::string&, const int&);
template void ParamManager::updateParam<double>(const std::string&, const double&);
template void ParamManager::updateParam<float>(const std::string&, const float&);
template void ParamManager::updateParam<bool>(const std::string&, const bool&);
template void ParamManager::updateParam<std::string>(const std::string&, const std::string&);

template int ParamManager::getParam<int>(const std::string&, const int&);
template double ParamManager::getParam<double>(const std::string&, const double&);
template float ParamManager::getParam<float>(const std::string&, const float&);
template bool ParamManager::getParam<bool>(const std::string&, const bool&);
template std::string ParamManager::getParam<std::string>(const std::string&, const std::string&);

}  // namespace power_rune