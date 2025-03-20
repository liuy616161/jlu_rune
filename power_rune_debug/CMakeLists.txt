cmake_minimum_required(VERSION 3.8)
project(power_rune_debug)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Qt5 COMPONENTS Widgets Core REQUIRED)

# Qt auto processing
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
)

# Link to existing libpower_rune_detector library if available
# You may need to modify this path to your actual detector library
find_library(POWER_RUNE_DETECTOR power_rune_detector
  PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../power_rune/build
)

# Core library sources
set(CORE_SOURCES
  src/param_manager.cpp
  src/power_rune_debugger.cpp
)

# Debug node sources
set(NODE_SOURCES
  src/debug_node.cpp
)

# GUI sources
set(GUI_SOURCES
  src/debug_gui.cpp
  src/main.cpp
)

# Build core library
add_library(power_rune_debug_core SHARED ${CORE_SOURCES})
ament_target_dependencies(power_rune_debug_core
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(power_rune_debug_core
  ${OpenCV_LIBS}
  yaml-cpp
  # Add your detector library here if needed
  # If POWER_RUNE_DETECTOR was found, link against it
  $<$<BOOL:${POWER_RUNE_DETECTOR}>:${POWER_RUNE_DETECTOR}>
)

# Build debug node
add_library(power_rune_debug_node SHARED ${NODE_SOURCES})
target_link_libraries(power_rune_debug_node
  power_rune_debug_core
  ${OpenCV_LIBS}
)
ament_target_dependencies(power_rune_debug_node
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
)

# Register node
rclcpp_components_register_node(power_rune_debug_node
  PLUGIN "power_rune::DebugNode"
  EXECUTABLE power_rune_debug_node
)

# Build GUI application
add_executable(power_rune_debug_gui ${GUI_SOURCES})
target_link_libraries(power_rune_debug_gui
  power_rune_debug_core
  ${OpenCV_LIBS}
  Qt5::Widgets
  Qt5::Core
)
ament_target_dependencies(power_rune_debug_gui
  rclcpp
  sensor_msgs
  cv_bridge
)

# Install targets
install(TARGETS
  power_rune_debug_core
  power_rune_debug_node
  power_rune_debug_gui
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and config
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY
  include/
  DESTINATION include
)

ament_package()