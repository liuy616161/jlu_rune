#include "power_rune_debugger.h"
#include <sstream>

namespace power_rune {

PowerRuneDebugger::PowerRuneDebugger() {
    // Initialize any necessary components
}

bool PowerRuneDebugger::processFrame(const cv::Mat& frame) {
    // Store original image
    original_image_ = frame.clone();
    
    // Create a Frame object with the current time
    Frame current_frame{frame, std::chrono::steady_clock::now(), 0.0, 0.0, 0.0};
    
    // Run detection
    return detector_.detect(current_frame);
}

cv::Mat PowerRuneDebugger::getResultImage() const {
    return createResultImage();
}

cv::Mat PowerRuneDebugger::createResultImage() const {
    // Create a visualization image with detection results
    cv::Mat result;
    if (!original_image_.empty()) {
        result = original_image_.clone();
    } else {
        return result; // Return empty mat if original image is empty
    }
    
    // Draw detection results based on status
    if (detector_.m_status == Status::SUCCESS) {
        // Draw arrows, armor, center R, etc.
        // This is a simplified version
        cv::putText(result, "Arrow detected", cv::Point(10, 30), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        cv::putText(result, "Armor detected", cv::Point(10, 60), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        cv::putText(result, "Center R detected", cv::Point(10, 90), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
    } else if (detector_.m_status == Status::ARROW_FAILURE) {
        cv::putText(result, "Arrow detection failed", cv::Point(10, 30), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
    } else if (detector_.m_status == Status::ARMOR_FAILURE) {
        cv::putText(result, "Armor detection failed", cv::Point(10, 30), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
    } else if (detector_.m_status == Status::CENTER_FAILURE) {
        cv::putText(result, "Center R detection failed", cv::Point(10, 30), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
    }
    
    return result;
}

std::string PowerRuneDebugger::getStatusInfo() const {
    std::stringstream ss;
    
    switch (detector_.m_status) {
        case Status::SUCCESS:
            ss << "Status: Success";
            break;
        case Status::ARROW_FAILURE:
            ss << "Status: Arrow detection failed";
            break;
        case Status::ARMOR_FAILURE:
            ss << "Status: Armor detection failed";
            break;
        case Status::CENTER_FAILURE:
            ss << "Status: Center R detection failed";
            break;
        default:
            ss << "Status: Unknown";
            break;
    }
    
    return ss.str();
}

}  // namespace power_rune