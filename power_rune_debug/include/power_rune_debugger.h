#ifndef POWER_RUNE_DEBUGGER_H
#define POWER_RUNE_DEBUGGER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include "Detector.h"
#include "param_manager.h"

namespace power_rune {

class PowerRuneDebugger {
public:
    PowerRuneDebugger();
    ~PowerRuneDebugger() = default;
    
    // Process a single frame
    bool processFrame(const cv::Mat& frame);
    
    // Get the output images
    cv::Mat getOriginalImage() const { return original_image_; }
    cv::Mat getArrowBinary() const { return detector_.get_imgArrow(); }
    cv::Mat getArmorBinary() const { return detector_.get_imgArmor(); }
    cv::Mat getResultImage() const;
    cv::Mat getArmorRoi() const { return detector_.get_imgsrc(); }
    
    // Get detector status
    Status getStatus() const { return detector_.m_status; }
    
    // Get additional info for visualization
    std::string getStatusInfo() const;

private:
    Detector detector_;
    cv::Mat original_image_;
    
    // Create a visualization image with detection results
    cv::Mat createResultImage() const;
};

}  // namespace power_rune

#endif // POWER_RUNE_DEBUGGER_H