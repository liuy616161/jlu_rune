#include "debug_gui.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QFileDialog>
#include <QMessageBox>
#include <QScrollArea>
#include <QSplitter>
#include <QMenuBar>
#include <QStatusBar>
#include <QLabel>
#include <std_msgs/msg/int64.hpp>
#include <cv_bridge/cv_bridge.h>

namespace power_rune {

DebugGUI::DebugGUI(std::shared_ptr<rclcpp::Node> node, QWidget* parent)
    : QMainWindow(parent),
      node_(node),
      frame_count_(0)
{
    // 设置窗口属性
    setWindowTitle("能量机关识别调试工具");
    resize(1280, 800);
    
    // 设置定时器
    update_timer_ = new QTimer(this);
    connect(update_timer_, &QTimer::timeout, this, &DebugGUI::updateDisplays);
    update_timer_->start(33); // ~30 FPS
    
    // 初始化界面
    setupUI();
    
    // 创建参数控制
    createParameterControls();
    
    // 创建ROS订阅
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    
    image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", qos,
        std::bind(&DebugGUI::onImageReceived, this, std::placeholders::_1));
    
    arrow_binary_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "/power_rune/arrow_binary", qos,
        std::bind(&DebugGUI::onArrowBinaryReceived, this, std::placeholders::_1));
    
    armor_binary_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "/power_rune/armor_binary", qos,
        std::bind(&DebugGUI::onArmorBinaryReceived, this, std::placeholders::_1));
    
    result_image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "/power_rune/result_image", qos,
        std::bind(&DebugGUI::onResultImageReceived, this, std::placeholders::_1));
    
    armor_roi_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "/power_rune/armor_roi", qos,
        std::bind(&DebugGUI::onArmorRoiReceived, this, std::placeholders::_1));
    
    // 创建曝光发布器
    exposure_pub_ = node_->create_publisher<std_msgs::msg::Int64>("/exposure_time", 10);
    
    // 初始化时间
    last_time_ = std::chrono::steady_clock::now();
    
    statusBar()->showMessage("调试界面已启动");
}

DebugGUI::~DebugGUI() {
    if (update_timer_) {
        update_timer_->stop();
        delete update_timer_;
    }
}

void DebugGUI::setupUI() {
    // 创建中央部件
    QWidget* central_widget = new QWidget(this);
    setCentralWidget(central_widget);
    
    // 创建主布局
    QHBoxLayout* main_layout = new QHBoxLayout(central_widget);
    
    // 创建分割器
    QSplitter* main_splitter = new QSplitter(Qt::Horizontal);
    main_layout->addWidget(main_splitter);
    
    // ---- 左侧：图像显示区域 ----
    QWidget* image_widget = new QWidget();
    QVBoxLayout* image_layout = new QVBoxLayout(image_widget);
    
    // 创建图像标签页
    image_tabs_ = new QTabWidget();
    connect(image_tabs_, &QTabWidget::currentChanged, this, &DebugGUI::onTabChanged);
    
    // 创建各种图像显示标签
    original_image_label_ = new QLabel("等待图像...");
    original_image_label_->setAlignment(Qt::AlignCenter);
    original_image_label_->setMinimumSize(640, 480);
    original_image_label_->setStyleSheet("border: 1px solid #CCCCCC; background-color: #F0F0F0;");
    
    arrow_binary_label_ = new QLabel("等待箭头二值图...");
    arrow_binary_label_->setAlignment(Qt::AlignCenter);
    arrow_binary_label_->setMinimumSize(640, 480);
    arrow_binary_label_->setStyleSheet("border: 1px solid #CCCCCC; background-color: #F0F0F0;");
    
    armor_binary_label_ = new QLabel("等待装甲板二值图...");
    armor_binary_label_->setAlignment(Qt::AlignCenter);
    armor_binary_label_->setMinimumSize(640, 480);
    armor_binary_label_->setStyleSheet("border: 1px solid #CCCCCC; background-color: #F0F0F0;");
    
    result_image_label_ = new QLabel("等待结果图像...");
    result_image_label_->setAlignment(Qt::AlignCenter);
    result_image_label_->setMinimumSize(640, 480);
    result_image_label_->setStyleSheet("border: 1px solid #CCCCCC; background-color: #F0F0F0;");
    
    armor_roi_label_ = new QLabel("等待装甲板ROI...");
    armor_roi_label_->setAlignment(Qt::AlignCenter);
    armor_roi_label_->setMinimumSize(640, 480);
    armor_roi_label_->setStyleSheet("border: 1px solid #CCCCCC; background-color: #F0F0F0;");
    
    // 添加到标签页
    image_tabs_->addTab(result_image_label_, "结果图像");
    image_tabs_->addTab(original_image_label_, "原始图像");
    image_tabs_->addTab(arrow_binary_label_, "箭头二值图");
    image_tabs_->addTab(armor_binary_label_, "装甲板二值图");
    image_tabs_->addTab(armor_roi_label_, "装甲板ROI");
    
    image_layout->addWidget(image_tabs_);
    
    // 状态栏
    QHBoxLayout* status_layout = new QHBoxLayout();
    fps_label_ = new QLabel("FPS: 0.0");
    status_label_ = new QLabel("就绪");
    
    status_layout->addWidget(fps_label_);
    status_layout->addStretch();
    status_layout->addWidget(status_label_);
    
    image_layout->addLayout(status_layout);
    
    // ---- 右侧：参数控制区域 ----
    QWidget* param_widget = new QWidget();
    QVBoxLayout* param_layout = new QVBoxLayout(param_widget);
    
    // 参数控制标签页
    param_tabs_ = new QTabWidget();
    
    // 曝光控制
    QGroupBox* camera_group = new QGroupBox("相机控制");
    QGridLayout* camera_layout = new QGridLayout(camera_group);
    
    QLabel* exposure_label = new QLabel("曝光时间(μs):");
    QSpinBox* exposure_spin = new QSpinBox();
    exposure_spin->setRange(1000, 20000);
    exposure_spin->setValue(5000);
    exposure_spin->setSingleStep(100);
    connect(exposure_spin, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &DebugGUI::onExposureChanged);
    
    QSlider* exposure_slider = new QSlider(Qt::Horizontal);
    exposure_slider->setRange(1000, 20000);
    exposure_slider->setValue(5000);
    connect(exposure_slider, &QSlider::valueChanged, exposure_spin, &QSpinBox::setValue);
    connect(exposure_spin, QOverload<int>::of(&QSpinBox::valueChanged), 
            exposure_slider, &QSlider::setValue);
    
    camera_layout->addWidget(exposure_label, 0, 0);
    camera_layout->addWidget(exposure_spin, 0, 1);
    camera_layout->addWidget(exposure_slider, 0, 2);
    
    param_layout->addWidget(camera_group);
    
    // 配置操作
    QHBoxLayout* config_layout = new QHBoxLayout();
    QPushButton* save_btn = new QPushButton("保存配置");
    QPushButton* load_btn = new QPushButton("加载配置");
    QPushButton* reset_btn = new QPushButton("重置配置");
    
    connect(save_btn, &QPushButton::clicked, this, &DebugGUI::onSaveConfig);
    connect(load_btn, &QPushButton::clicked, this, &DebugGUI::onLoadConfig);
    connect(reset_btn, &QPushButton::clicked, this, &DebugGUI::onResetConfig);
    
    config_layout->addWidget(save_btn);
    config_layout->addWidget(load_btn);
    config_layout->addWidget(reset_btn);
    
    param_layout->addLayout(config_layout);
    param_layout->addWidget(param_tabs_);
    
    // 添加到分割器
    main_splitter->addWidget(image_widget);
    main_splitter->addWidget(param_widget);
    main_splitter->setSizes(QList<int>() << 700 << 300);
    
    // 创建菜单
    QMenu* file_menu = menuBar()->addMenu("文件");
    file_menu->addAction("保存配置", this, &DebugGUI::onSaveConfig);
    file_menu->addAction("加载配置", this, &DebugGUI::onLoadConfig);
    file_menu->addSeparator();
    file_menu->addAction("退出", this, &QWidget::close);
    
    QMenu* view_menu = menuBar()->addMenu("视图");
    view_menu->addAction("重置布局", [main_splitter]() {
        main_splitter->setSizes(QList<int>() << 700 << 300);
    });
}

void DebugGUI::createParameterControls() {
    // 创建参数标签页
    
    // 阈值参数组
    QWidget* threshold_tab = new QWidget();
    QVBoxLayout* threshold_layout = new QVBoxLayout(threshold_tab);
    
    // 亮度阈值组
    QGroupBox* brightness_group = new QGroupBox("亮度阈值");
    QGridLayout* brightness_layout = new QGridLayout(brightness_group);
    
    int row = 0;
    
    // 蓝色箭头阈值
    QLabel* blue_arrow_label = new QLabel("蓝色箭头:");
    QSpinBox* blue_arrow_spin = new QSpinBox();
    blue_arrow_spin->setRange(0, 255);
    blue_arrow_spin->setValue(Param::ARROW_BRIGHTNESS_THRESHOLD);
    blue_arrow_spin->setProperty("param_name", "detect.brightness_threshold.blue.arrow");
    connect(blue_arrow_spin, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &DebugGUI::onIntValueChanged);
    
    QSlider* blue_arrow_slider = new QSlider(Qt::Horizontal);
    blue_arrow_slider->setRange(0, 255);
    blue_arrow_slider->setValue(Param::ARROW_BRIGHTNESS_THRESHOLD);
    blue_arrow_slider->setProperty("param_name", "detect.brightness_threshold.blue.arrow");
    connect(blue_arrow_slider, &QSlider::valueChanged, this, &DebugGUI::onSliderValueChanged);
    connect(blue_arrow_spin, QOverload<int>::of(&QSpinBox::valueChanged), 
            blue_arrow_slider, &QSlider::setValue);
    connect(blue_arrow_slider, &QSlider::valueChanged, 
            blue_arrow_spin, &QSpinBox::setValue);
    
    brightness_layout->addWidget(blue_arrow_label, row, 0);
    brightness_layout->addWidget(blue_arrow_spin, row, 1);
    brightness_layout->addWidget(blue_arrow_slider, row, 2);
    
    param_controls_["detect.brightness_threshold.blue.arrow"] = blue_arrow_spin;
    
    row++;
    
    // 蓝色装甲板阈值
    QLabel* blue_armor_label = new QLabel("蓝色装甲板:");
    QSpinBox* blue_armor_spin = new QSpinBox();
    blue_armor_spin->setRange(0, 255);
    blue_armor_spin->setValue(Param::ARMOR_BRIGHTNESS_THRESHOLD);
    blue_armor_spin->setProperty("param_name", "detect.brightness_threshold.blue.armor");
    connect(blue_armor_spin, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &DebugGUI::onIntValueChanged);
    
    QSlider* blue_armor_slider = new QSlider(Qt::Horizontal);
    blue_armor_slider->setRange(0, 255);
    blue_armor_slider->setValue(Param::ARMOR_BRIGHTNESS_THRESHOLD);
    blue_armor_slider->setProperty("param_name", "detect.brightness_threshold.blue.armor");
    connect(blue_armor_slider, &QSlider::valueChanged, this, &DebugGUI::onSliderValueChanged);
    connect(blue_armor_spin, QOverload<int>::of(&QSpinBox::valueChanged), 
            blue_armor_slider, &QSlider::setValue);
    connect(blue_armor_slider, &QSlider::valueChanged, 
            blue_armor_spin, &QSpinBox::setValue);
    
    brightness_layout->addWidget(blue_armor_label, row, 0);
    brightness_layout->addWidget(blue_armor_spin, row, 1);
    brightness_layout->addWidget(blue_armor_slider, row, 2);
    
    param_controls_["detect.brightness_threshold.blue.armor"] = blue_armor_spin;
    
    threshold_layout->addWidget(brightness_group);
    
    // ROI参数组
    QGroupBox* roi_group = new QGroupBox("ROI设置");
    QGridLayout* roi_layout = new QGridLayout(roi_group);
    
    row = 0;
    
    // 局部ROI距离比例
    QLabel* roi_dist_ratio_label = new QLabel("局部ROI距离比例:");
    QDoubleSpinBox* roi_dist_ratio_spin = new QDoubleSpinBox();
    roi_dist_ratio_spin->setRange(0.5, 2.0);
    roi_dist_ratio_spin->setSingleStep(0.1);
    roi_dist_ratio_spin->setValue(Param::LOCAL_ROI_DISTANCE_RATIO);
    roi_dist_ratio_spin->setDecimals(1);
    roi_dist_ratio_spin->setProperty("param_name", "detect.local_roi.distance_ratio");
    connect(roi_dist_ratio_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &DebugGUI::onDoubleValueChanged);
    
    roi_layout->addWidget(roi_dist_ratio_label, row, 0);
    roi_layout->addWidget(roi_dist_ratio_spin, row, 1);
    
    param_controls_["detect.local_roi.distance_ratio"] = roi_dist_ratio_spin;
    
    row++;
    
    // 局部ROI宽度
    QLabel* roi_width_label = new QLabel("局部ROI宽度:");
    QSpinBox* roi_width_spin = new QSpinBox();
    roi_width_spin->setRange(30, 200);
    roi_width_spin->setValue(static_cast<int>(Param::LOCAL_ROI_WIDTH));
    roi_width_spin->setProperty("param_name", "detect.local_roi.width");
    connect(roi_width_spin, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &DebugGUI::onIntValueChanged);
    
    QSlider* roi_width_slider = new QSlider(Qt::Horizontal);
    roi_width_slider->setRange(30, 200);
    roi_width_slider->setValue(static_cast<int>(Param::LOCAL_ROI_WIDTH));
    roi_width_slider->setProperty("param_name", "detect.local_roi.width");
    connect(roi_width_slider, &QSlider::valueChanged, this, &DebugGUI::onSliderValueChanged);
    connect(roi_width_spin, QOverload<int>::of(&QSpinBox::valueChanged), 
            roi_width_slider, &QSlider::setValue);
    connect(roi_width_slider, &QSlider::valueChanged, 
            roi_width_spin, &QSpinBox::setValue);
    
    roi_layout->addWidget(roi_width_label, row, 0);
    roi_layout->addWidget(roi_width_spin, row, 1);
    roi_layout->addWidget(roi_width_slider, row, 2);
    
    param_controls_["detect.local_roi.width"] = roi_width_spin;
    
    row++;
    
    // 垂直距离阈值
    QLabel* vert_dist_label = new QLabel("垂直距离阈值:");
    QSpinBox* vert_dist_spin = new QSpinBox();
    vert_dist_spin->setRange(30, 200);
    vert_dist_spin->setValue(static_cast<int>(Param::ARMOR_CENTER_VERTICAL_DISTANCE_THRESHOLD));
    vert_dist_spin->setProperty("param_name", "detect.armor_center_vertical_distance_threshold");
    connect(vert_dist_spin, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &DebugGUI::onIntValueChanged);
    
    QSlider* vert_dist_slider = new QSlider(Qt::Horizontal);
    vert_dist_slider->setRange(30, 200);
    vert_dist_slider->setValue(static_cast<int>(Param::ARMOR_CENTER_VERTICAL_DISTANCE_THRESHOLD));
    vert_dist_slider->setProperty("param_name", "detect.armor_center_vertical_distance_threshold");
    connect(vert_dist_slider, &QSlider::valueChanged, this, &DebugGUI::onSliderValueChanged);
    connect(vert_dist_spin, QOverload<int>::of(&QSpinBox::valueChanged), 
            vert_dist_slider, &QSlider::setValue);
    connect(vert_dist_slider, &QSlider::valueChanged, 
            vert_dist_spin, &QSpinBox::setValue);
    
    roi_layout->addWidget(vert_dist_label, row, 0);
    roi_layout->addWidget(vert_dist_spin, row, 1);
    roi_layout->addWidget(vert_dist_slider, row, 2);
    
    param_controls_["detect.armor_center_vertical_distance_threshold"] = vert_dist_spin;
    
    row++;
    
    // 全局ROI比例
    QLabel* global_ratio_label = new QLabel("全局ROI比例:");
    QDoubleSpinBox* global_ratio_spin = new QDoubleSpinBox();
    global_ratio_spin->setRange(0.5, 3.0);
    global_ratio_spin->setSingleStep(0.1);
    global_ratio_spin->setValue(Param::GLOBAL_ROI_LENGTH_RATIO);
    global_ratio_spin->setDecimals(1);
    global_ratio_spin->setProperty("param_name", "detect.global_roi_length_ratio");
    connect(global_ratio_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &DebugGUI::onDoubleValueChanged);
    
    roi_layout->addWidget(global_ratio_label, row, 0);
    roi_layout->addWidget(global_ratio_spin, row, 1);
    
    param_controls_["detect.global_roi_length_ratio"] = global_ratio_spin;
    
    threshold_layout->addWidget(roi_group);
    threshold_layout->addStretch();
    
    // 箭头参数标签页
    QWidget* arrow_tab = new QWidget();
    QVBoxLayout* arrow_layout = new QVBoxLayout(arrow_tab);
    
    // 箭头灯条参数组
    QGroupBox* arrow_lightline_group = new QGroupBox("箭头灯条参数");
    QGridLayout* arrow_lightline_layout = new QGridLayout(arrow_lightline_group);
    
    row = 0;
    
    // 最小面积
    QLabel* arrow_min_area_label = new QLabel("最小面积:");
    QSpinBox* arrow_min_area_spin = new QSpinBox();
    arrow_min_area_spin->setRange(10, 1000);
    arrow_min_area_spin->setValue(static_cast<int>(Param::MIN_ARROW_LIGHTLINE_AREA));
    arrow_min_area_spin->setProperty("param_name", "detect.arrow.lightline.area.min");
    connect(arrow_min_area_spin, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &DebugGUI::onIntValueChanged);
    
    arrow_lightline_layout->addWidget(arrow_min_area_label, row, 0);
    arrow_lightline_layout->addWidget(arrow_min_area_spin, row, 1);
    
    param_controls_["detect.arrow.lightline.area.min"] = arrow_min_area_spin;
    
    row++;
    
    // 最大面积
    QLabel* arrow_max_area_label = new QLabel("最大面积:");
    QSpinBox* arrow_max_area_spin = new QSpinBox();
    arrow_max_area_spin->setRange(100, 2000);
    arrow_max_area_spin->setValue(static_cast<int>(Param::MAX_ARROW_LIGHTLINE_AREA));
    arrow_max_area_spin->setProperty("param_name", "detect.arrow.lightline.area.max");
    connect(arrow_max_area_spin, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &DebugGUI::onIntValueChanged);
    
    arrow_lightline_layout->addWidget(arrow_max_area_label, row, 0);
    arrow_lightline_layout->addWidget(arrow_max_area_spin, row, 1);
    
    param_controls_["detect.arrow.lightline.area.max"] = arrow_max_area_spin;
    
    row++;
    
    // 最大长宽比
    QLabel* arrow_aspect_ratio_label = new QLabel("最大长宽比:");
    QDoubleSpinBox* arrow_aspect_ratio_spin = new QDoubleSpinBox();
    arrow_aspect_ratio_spin->setRange(1.0, 10.0);
    arrow_aspect_ratio_spin->setSingleStep(0.1);
    arrow_aspect_ratio_spin->setValue(Param::MAX_ARROW_LIGHTLINE_ASPECT_RATIO);
    arrow_aspect_ratio_spin->setDecimals(1);
    arrow_aspect_ratio_spin->setProperty("param_name", "detect.arrow.lightline.aspect_ratio_max");
    connect(arrow_aspect_ratio_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &DebugGUI::onDoubleValueChanged);
    
    arrow_lightline_layout->addWidget(arrow_aspect_ratio_label, row, 0);
    arrow_lightline_layout->addWidget(arrow_aspect_ratio_spin, row, 1);
    
    param_controls_["detect.arrow.lightline.aspect_ratio_max"] = arrow_aspect_ratio_spin;
    
    row++;
    
    // 最小数量
    QLabel* arrow_min_num_label = new QLabel("最小数量:");
    QSpinBox* arrow_min_num_spin = new QSpinBox();
    arrow_min_num_spin->setRange(1, 20);
    arrow_min_num_spin->setValue(Param::MIN_ARROW_LIGHTLINE_NUM);
    arrow_min_num_spin->setProperty("param_name", "detect.arrow.lightline.num.min");
    connect(arrow_min_num_spin, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &DebugGUI::onIntValueChanged);
    
    arrow_lightline_layout->addWidget(arrow_min_num_label, row, 0);
    arrow_lightline_layout->addWidget(arrow_min_num_spin, row, 1);
    
    param_controls_["detect.arrow.lightline.num.min"] = arrow_min_num_spin;
    
    row++;
    
    // 最大数量
    QLabel* arrow_max_num_label = new QLabel("最大数量:");
    QSpinBox* arrow_max_num_spin = new QSpinBox();
    arrow_max_num_spin->setRange(5, 30);
    arrow_max_num_spin->setValue(Param::MAX_ARROW_LIGHTLINE_NUM);
    arrow_max_num_spin->setProperty("param_name", "detect.arrow.lightline.num.max");
    connect(arrow_max_num_spin, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &DebugGUI::onIntValueChanged);
    
    arrow_lightline_layout->addWidget(arrow_max_num_label, row, 0);
    arrow_lightline_layout->addWidget(arrow_max_num_spin, row, 1);
    
    param_controls_["detect.arrow.lightline.num.max"] = arrow_max_num_spin;
    
    arrow_layout->addWidget(arrow_lightline_group);
    
    // 箭头本身参数组
    QGroupBox* arrow_param_group = new QGroupBox("箭头参数");
    QGridLayout* arrow_param_layout = new QGridLayout(arrow_param_group);
    
    row = 0;
    
    // 面积比例最大值
    QLabel* arrow_area_ratio_label = new QLabel("面积比例最大值:");
    QDoubleSpinBox* arrow_area_ratio_spin = new QDoubleSpinBox();
    arrow_area_ratio_spin->setRange(1.0, 10.0);
    arrow_area_ratio_spin->setSingleStep(0.1);
    arrow_area_ratio_spin->setValue(Param::MAX_SAME_ARROW_AREA_RATIO);
    arrow_area_ratio_spin->setDecimals(1);
    arrow_area_ratio_spin->setProperty("param_name", "detect.arrow.same_area_ratio_max");
    connect(arrow_area_ratio_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &DebugGUI::onDoubleValueChanged);
    
    arrow_param_layout->addWidget(arrow_area_ratio_label, row, 0);
    arrow_param_layout->addWidget(arrow_area_ratio_spin, row, 1);
    
    param_controls_["detect.arrow.same_area_ratio_max"] = arrow_area_ratio_spin;
    
    row++;
    
    // 最小长宽比
    QLabel* arrow_min_aspect_label = new QLabel("最小长宽比:");
        QDoubleSpinBox* arrow_min_aspect_spin = new QDoubleSpinBox();
    arrow_min_aspect_spin->setRange(1.0, 10.0);
    arrow_min_aspect_spin->setSingleStep(0.1);
    arrow_min_aspect_spin->setValue(Param::MIN_ARROW_ASPECT_RATIO);
    arrow_min_aspect_spin->setDecimals(1);
    arrow_min_aspect_spin->setProperty("param_name", "detect.arrow.aspect_ratio.min");
    connect(arrow_min_aspect_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &DebugGUI::onDoubleValueChanged);
    
    arrow_param_layout->addWidget(arrow_min_aspect_label, row, 0);
    arrow_param_layout->addWidget(arrow_min_aspect_spin, row, 1);
    
    param_controls_["detect.arrow.aspect_ratio.min"] = arrow_min_aspect_spin;
    
    row++;
    
    // 最大长宽比
    QLabel* arrow_max_aspect_label = new QLabel("最大长宽比:");
    QDoubleSpinBox* arrow_max_aspect_spin = new QDoubleSpinBox();
    arrow_max_aspect_spin->setRange(2.0, 20.0);
    arrow_max_aspect_spin->setSingleStep(0.1);
    arrow_max_aspect_spin->setValue(Param::MAX_ARROW_ASPECT_RATIO);
    arrow_max_aspect_spin->setDecimals(1);
    arrow_max_aspect_spin->setProperty("param_name", "detect.arrow.aspect_ratio.max");
    connect(arrow_max_aspect_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &DebugGUI::onDoubleValueChanged);
    
    arrow_param_layout->addWidget(arrow_max_aspect_label, row, 0);
    arrow_param_layout->addWidget(arrow_max_aspect_spin, row, 1);
    
    param_controls_["detect.arrow.aspect_ratio.max"] = arrow_max_aspect_spin;
    
    row++;
    
    // 最大面积
    QLabel* arrow_max_area_param_label = new QLabel("最大面积:");
    QSpinBox* arrow_max_area_param_spin = new QSpinBox();
    arrow_max_area_param_spin->setRange(1000, 10000);
    arrow_max_area_param_spin->setValue(static_cast<int>(Param::MAX_ARROW_AREA));
    arrow_max_area_param_spin->setProperty("param_name", "detect.arrow.area_max");
    connect(arrow_max_area_param_spin, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &DebugGUI::onIntValueChanged);
    
    arrow_param_layout->addWidget(arrow_max_area_param_label, row, 0);
    arrow_param_layout->addWidget(arrow_max_area_param_spin, row, 1);
    
    param_controls_["detect.arrow.area_max"] = arrow_max_area_param_spin;
    
    arrow_layout->addWidget(arrow_param_group);
    arrow_layout->addStretch();
    
    // 装甲板参数标签页
    QWidget* armor_tab = new QWidget();
    QVBoxLayout* armor_layout = new QVBoxLayout(armor_tab);
    
    // 装甲板灯条参数组
    QGroupBox* armor_lightline_group = new QGroupBox("装甲板灯条参数");
    QGridLayout* armor_lightline_layout = new QGridLayout(armor_lightline_group);
    
    row = 0;
    
    // 最小面积
    QLabel* armor_min_area_label = new QLabel("最小面积:");
    QSpinBox* armor_min_area_spin = new QSpinBox();
    armor_min_area_spin->setRange(500, 10000);
    armor_min_area_spin->setValue(static_cast<int>(Param::MIN_ARMOR_LIGHTLINE_AREA));
    armor_min_area_spin->setProperty("param_name", "detect.armor.lightline.area.min");
    connect(armor_min_area_spin, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &DebugGUI::onIntValueChanged);
    
    armor_lightline_layout->addWidget(armor_min_area_label, row, 0);
    armor_lightline_layout->addWidget(armor_min_area_spin, row, 1);
    
    param_controls_["detect.armor.lightline.area.min"] = armor_min_area_spin;
    
    row++;
    
    // 最大面积
    QLabel* armor_max_area_label = new QLabel("最大面积:");
    QSpinBox* armor_max_area_spin = new QSpinBox();
    armor_max_area_spin->setRange(5000, 50000);
    armor_max_area_spin->setValue(static_cast<int>(Param::MAX_ARMOR_LIGHTLINE_AREA));
    armor_max_area_spin->setProperty("param_name", "detect.armor.lightline.area.max");
    connect(armor_max_area_spin, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &DebugGUI::onIntValueChanged);
    
    armor_lightline_layout->addWidget(armor_max_area_label, row, 0);
    armor_lightline_layout->addWidget(armor_max_area_spin, row, 1);
    
    param_controls_["detect.armor.lightline.area.max"] = armor_max_area_spin;
    
    row++;
    
    // 最小轮廓面积
    QLabel* armor_min_contour_label = new QLabel("最小轮廓面积:");
    QSpinBox* armor_min_contour_spin = new QSpinBox();
    armor_min_contour_spin->setRange(100, 5000);
    armor_min_contour_spin->setValue(static_cast<int>(Param::MIN_ARMOR_LIGHTLINE_CONTOUR_AREA));
    armor_min_contour_spin->setProperty("param_name", "detect.armor.lightline.contour_area.min");
    connect(armor_min_contour_spin, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &DebugGUI::onIntValueChanged);
    
    armor_lightline_layout->addWidget(armor_min_contour_label, row, 0);
    armor_lightline_layout->addWidget(armor_min_contour_spin, row, 1);
    
    param_controls_["detect.armor.lightline.contour_area.min"] = armor_min_contour_spin;
    
    row++;
    
    // 最大轮廓面积
    QLabel* armor_max_contour_label = new QLabel("最大轮廓面积:");
    QSpinBox* armor_max_contour_spin = new QSpinBox();
    armor_max_contour_spin->setRange(1000, 20000);
    armor_max_contour_spin->setValue(static_cast<int>(Param::MAX_ARMOR_LIGHTLINE_CONTOUR_AREA));
    armor_max_contour_spin->setProperty("param_name", "detect.armor.lightline.contour_area.max");
    connect(armor_max_contour_spin, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &DebugGUI::onIntValueChanged);
    
    armor_lightline_layout->addWidget(armor_max_contour_label, row, 0);
    armor_lightline_layout->addWidget(armor_max_contour_spin, row, 1);
    
    param_controls_["detect.armor.lightline.contour_area.max"] = armor_max_contour_spin;
    
    row++;
    
    // 最小长宽比
    QLabel* armor_min_aspect_label = new QLabel("最小长宽比:");
    QDoubleSpinBox* armor_min_aspect_spin = new QDoubleSpinBox();
    armor_min_aspect_spin->setRange(1.0, 5.0);
    armor_min_aspect_spin->setSingleStep(0.1);
    armor_min_aspect_spin->setValue(Param::MIN_ARMOR_LIGHTLINE_ASPECT_RATIO);
    armor_min_aspect_spin->setDecimals(1);
    armor_min_aspect_spin->setProperty("param_name", "detect.armor.lightline.aspect_ratio.min");
    connect(armor_min_aspect_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &DebugGUI::onDoubleValueChanged);
    
    armor_lightline_layout->addWidget(armor_min_aspect_label, row, 0);
    armor_lightline_layout->addWidget(armor_min_aspect_spin, row, 1);
    
    param_controls_["detect.armor.lightline.aspect_ratio.min"] = armor_min_aspect_spin;
    
    row++;
    
    // 最大长宽比
    QLabel* armor_max_aspect_label = new QLabel("最大长宽比:");
    QDoubleSpinBox* armor_max_aspect_spin = new QDoubleSpinBox();
    armor_max_aspect_spin->setRange(2.0, 10.0);
    armor_max_aspect_spin->setSingleStep(0.1);
    armor_max_aspect_spin->setValue(Param::MAX_ARMOR_LIGHTLINE_ASPECT_RATIO);
    armor_max_aspect_spin->setDecimals(1);
    armor_max_aspect_spin->setProperty("param_name", "detect.armor.lightline.aspect_ratio.max");
    connect(armor_max_aspect_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &DebugGUI::onDoubleValueChanged);
    
    armor_lightline_layout->addWidget(armor_max_aspect_label, row, 0);
    armor_lightline_layout->addWidget(armor_max_aspect_spin, row, 1);
    
    param_controls_["detect.armor.lightline.aspect_ratio.max"] = armor_max_aspect_spin;
    
    armor_layout->addWidget(armor_lightline_group);
    
    // 装甲板匹配参数组
    QGroupBox* armor_match_group = new QGroupBox("装甲板匹配参数");
    QGridLayout* armor_match_layout = new QGridLayout(armor_match_group);
    
    row = 0;
    
    // 最大面积比
    QLabel* armor_area_ratio_label = new QLabel("最大面积比:");
    QDoubleSpinBox* armor_area_ratio_spin = new QDoubleSpinBox();
    armor_area_ratio_spin->setRange(1.0, 10.0);
    armor_area_ratio_spin->setSingleStep(0.1);
    armor_area_ratio_spin->setValue(Param::MAX_SAME_ARMOR_AREA_RATIO);
    armor_area_ratio_spin->setDecimals(1);
    armor_area_ratio_spin->setProperty("param_name", "detect.armor.same.area_ratio_max");
    connect(armor_area_ratio_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &DebugGUI::onDoubleValueChanged);
    
    armor_match_layout->addWidget(armor_area_ratio_label, row, 0);
    armor_match_layout->addWidget(armor_area_ratio_spin, row, 1);
    
    param_controls_["detect.armor.same.area_ratio_max"] = armor_area_ratio_spin;
    
    row++;
    
    // 最小距离
    QLabel* armor_min_dist_label = new QLabel("最小距离:");
    QSpinBox* armor_min_dist_spin = new QSpinBox();
    armor_min_dist_spin->setRange(10, 200);
    armor_min_dist_spin->setValue(static_cast<int>(Param::MIN_SAME_ARMOR_DISTANCE));
    armor_min_dist_spin->setProperty("param_name", "detect.armor.same.distance.min");
    connect(armor_min_dist_spin, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &DebugGUI::onIntValueChanged);
    
    armor_match_layout->addWidget(armor_min_dist_label, row, 0);
    armor_match_layout->addWidget(armor_min_dist_spin, row, 1);
    
    param_controls_["detect.armor.same.distance.min"] = armor_min_dist_spin;
    
    row++;
    
    // 最大距离
    QLabel* armor_max_dist_label = new QLabel("最大距离:");
    QSpinBox* armor_max_dist_spin = new QSpinBox();
    armor_max_dist_spin->setRange(50, 300);
    armor_max_dist_spin->setValue(static_cast<int>(Param::MAX_SAME_ARMOR_DISTANCE));
    armor_max_dist_spin->setProperty("param_name", "detect.armor.same.distance.max");
    connect(armor_max_dist_spin, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &DebugGUI::onIntValueChanged);
    
    armor_match_layout->addWidget(armor_max_dist_label, row, 0);
    armor_match_layout->addWidget(armor_max_dist_spin, row, 1);
    
    param_controls_["detect.armor.same.distance.max"] = armor_max_dist_spin;
    
    armor_layout->addWidget(armor_match_group);
    armor_layout->addStretch();
    
    // 中心R参数标签页
    QWidget* center_tab = new QWidget();
    QVBoxLayout* center_layout = new QVBoxLayout(center_tab);
    
    // 中心R参数组
    QGroupBox* center_group = new QGroupBox("中心R参数");
    QGridLayout* center_param_layout = new QGridLayout(center_group);
    
    row = 0;
    
    // 最小面积
    QLabel* center_min_area_label = new QLabel("最小面积:");
    QSpinBox* center_min_area_spin = new QSpinBox();
    center_min_area_spin->setRange(50, 5000);
    center_min_area_spin->setValue(static_cast<int>(Param::MIN_CENTER_AREA));
    center_min_area_spin->setProperty("param_name", "detect.centerR.area.min");
    connect(center_min_area_spin, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &DebugGUI::onIntValueChanged);
    
    center_param_layout->addWidget(center_min_area_label, row, 0);
    center_param_layout->addWidget(center_min_area_spin, row, 1);
    
    param_controls_["detect.centerR.area.min"] = center_min_area_spin;
    
    row++;
    
    // 最大面积
    QLabel* center_max_area_label = new QLabel("最大面积:");
    QSpinBox* center_max_area_spin = new QSpinBox();
    center_max_area_spin->setRange(500, 10000);
    center_max_area_spin->setValue(static_cast<int>(Param::MAX_CENTER_AREA));
    center_max_area_spin->setProperty("param_name", "detect.centerR.area.max");
    connect(center_max_area_spin, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &DebugGUI::onIntValueChanged);
    
    center_param_layout->addWidget(center_max_area_label, row, 0);
    center_param_layout->addWidget(center_max_area_spin, row, 1);
    
    param_controls_["detect.centerR.area.max"] = center_max_area_spin;
    
    row++;
    
    // 最大长宽比
    QLabel* center_aspect_label = new QLabel("最大长宽比:");
    QDoubleSpinBox* center_aspect_spin = new QDoubleSpinBox();
    center_aspect_spin->setRange(1.0, 5.0);
    center_aspect_spin->setSingleStep(0.1);
    center_aspect_spin->setValue(Param::MAX_CENTER_ASPECT_RATIO);
    center_aspect_spin->setDecimals(1);
    center_aspect_spin->setProperty("param_name", "detect.centerR.aspect_ratio_max");
    connect(center_aspect_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &DebugGUI::onDoubleValueChanged);
    
    center_param_layout->addWidget(center_aspect_label, row, 0);
    center_param_layout->addWidget(center_aspect_spin, row, 1);
    
    param_controls_["detect.centerR.aspect_ratio_max"] = center_aspect_spin;
    
    center_layout->addWidget(center_group);
    center_layout->addStretch();
    
    // 添加所有标签页
    param_tabs_->addTab(threshold_tab, "阈值设置");
    param_tabs_->addTab(arrow_tab, "箭头参数");
    param_tabs_->addTab(armor_tab, "装甲板参数");
    param_tabs_->addTab(center_tab, "中心R参数");
}

void DebugGUI::onIntValueChanged(int value) {
    QSpinBox* spin_box = qobject_cast<QSpinBox*>(sender());
    if (!spin_box) return;
    
    QString param_name = spin_box->property("param_name").toString();
    if (param_name.isEmpty()) return;
    
    // 更新参数
    ParamManager::getInstance().updateParam<int>(param_name.toStdString(), value);
}

void DebugGUI::onDoubleValueChanged(double value) {
    QDoubleSpinBox* spin_box = qobject_cast<QDoubleSpinBox*>(sender());
    if (!spin_box) return;
    
    QString param_name = spin_box->property("param_name").toString();
    if (param_name.isEmpty()) return;
    
    // 更新参数
    ParamManager::getInstance().updateParam<double>(param_name.toStdString(), value);
}

void DebugGUI::onSliderValueChanged(int value) {
    QSlider* slider = qobject_cast<QSlider*>(sender());
    if (!slider) return;
    
    QString param_name = slider->property("param_name").toString();
    if (param_name.isEmpty()) return;
    
    // 更新对应的SpinBox，由SpinBox触发参数更新
    for (auto it = param_controls_.begin(); it != param_controls_.end(); ++it) {
        if (it->first == param_name.toStdString()) {
            QSpinBox* spin_box = qobject_cast<QSpinBox*>(it->second);
            if (spin_box) {
                spin_box->setValue(value);
                return;
            }
            break;
        }
    }
    
    // 如果没有找到对应的SpinBox，直接更新参数
    ParamManager::getInstance().updateParam<int>(param_name.toStdString(), value);
}

void DebugGUI::onExposureChanged(int value) {
    if (!exposure_pub_) return;
    
    auto msg = std::make_unique<std_msgs::msg::Int64>();
    msg->data = value;
    exposure_pub_->publish(std::move(msg));
    
    status_label_->setText(QString("曝光时间已更新: %1 μs").arg(value));
}

void DebugGUI::onSaveConfig() {
    QString filename = QFileDialog::getSaveFileName(this, 
                                                  "保存配置", 
                                                  QString::fromStdString(ParamManager::getInstance().getConfig()["config_file"].as<std::string>("config/armor_debug.yaml")), 
                                                  "YAML文件 (*.yaml)");
    if (filename.isEmpty()) return;
    
    if (ParamManager::getInstance().saveConfig(filename.toStdString())) {
        statusBar()->showMessage(QString("配置已保存到 %1").arg(filename), 3000);
    } else {
        QMessageBox::critical(this, "保存失败", "无法保存配置文件，请检查文件权限");
    }
}

void DebugGUI::onLoadConfig() {
    QString filename = QFileDialog::getOpenFileName(this, 
                                                  "加载配置", 
                                                  QString::fromStdString(ParamManager::getInstance().getConfig()["config_file"].as<std::string>("config/armor_debug.yaml")), 
                                                  "YAML文件 (*.yaml)");
    if (filename.isEmpty()) return;
    
    if (ParamManager::getInstance().loadConfig(filename.toStdString())) {
        statusBar()->showMessage(QString("配置已从 %1 加载").arg(filename), 3000);
        
        // 更新控件显示
        for (auto it = param_controls_.begin(); it != param_controls_.end(); ++it) {
            const std::string& param_name = it->first;
            QWidget* control = it->second;
            
            if (QSpinBox* spin_box = qobject_cast<QSpinBox*>(control)) {
                spin_box->setValue(ParamManager::getInstance().getParam<int>(param_name, spin_box->value()));
            } 
            else if (QDoubleSpinBox* double_spin = qobject_cast<QDoubleSpinBox*>(control)) {
                double_spin->setValue(ParamManager::getInstance().getParam<double>(param_name, double_spin->value()));
            }
        }
    } else {
        QMessageBox::critical(this, "加载失败", "无法加载配置文件，请检查文件格式");
    }
}

void DebugGUI::onResetConfig() {
    if (QMessageBox::question(this, "确认重置", 
                            "确定要重置所有参数到默认值吗？", 
                            QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes) {
        return;
    }
    
    // 重新加载默认配置
    if (ParamManager::getInstance().loadConfig("config/armor_debug.yaml")) {
        statusBar()->showMessage("参数已重置为默认值", 3000);
        
        // 更新控件显示
        for (auto it = param_controls_.begin(); it != param_controls_.end(); ++it) {
            const std::string& param_name = it->first;
            QWidget* control = it->second;
            
            if (QSpinBox* spin_box = qobject_cast<QSpinBox*>(control)) {
                spin_box->setValue(ParamManager::getInstance().getParam<int>(param_name, spin_box->value()));
            } 
            else if (QDoubleSpinBox* double_spin = qobject_cast<QDoubleSpinBox*>(control)) {
                double_spin->setValue(ParamManager::getInstance().getParam<double>(param_name, double_spin->value()));
            }
        }
    } else {
        QMessageBox::critical(this, "重置失败", "无法加载默认配置文件");
    }
}

void DebugGUI::onTabChanged(int index) {
    // 标签页切换时可能需要执行的操作
}

void DebugGUI::updateDisplays() {
    // 计算FPS
    frame_count_++;
    auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - last_time_).count();
    
    if (elapsed >= 1000) {  // Update FPS once per second
        double fps = static_cast<double>(frame_count_) * 1000.0 / static_cast<double>(elapsed);
        fps_label_->setText(QString("FPS: %1").arg(fps, 0, 'f', 1));
        frame_count_ = 0;
        last_time_ = current_time;
    }
}

void DebugGUI::onImageReceived(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        original_image_ = cv_ptr->image.clone();
        displayImage(original_image_, original_image_label_);
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "CV bridge exception: %s", e.what());
    }
}

void DebugGUI::onArrowBinaryReceived(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "mono8");
        arrow_binary_ = cv_ptr->image.clone();
        displayImage(arrow_binary_, arrow_binary_label_);
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "CV bridge exception: %s", e.what());
    }
}

void DebugGUI::onArmorBinaryReceived(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "mono8");
        armor_binary_ = cv_ptr->image.clone();
        displayImage(armor_binary_, armor_binary_label_);
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "CV bridge exception: %s", e.what());
    }
}

void DebugGUI::onResultImageReceived(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        result_image_ = cv_ptr->image.clone();
        displayImage(result_image_, result_image_label_);
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "CV bridge exception: %s", e.what());
    }
}

void DebugGUI::onArmorRoiReceived(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        armor_roi_ = cv_ptr->image.clone();
        displayImage(armor_roi_, armor_roi_label_);
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "CV bridge exception: %s", e.what());
    }
}

void DebugGUI::displayImage(const cv::Mat& image, QLabel* label) {
    if (image.empty()) return;
    
    QImage qimg;
    if (image.channels() == 3) {
        // Convert BGR to RGB
        cv::Mat rgb_image;
        cv::cvtColor(image, rgb_image, cv::COLOR_BGR2RGB);
        qimg = QImage(rgb_image.data, rgb_image.cols, rgb_image.rows, 
                    rgb_image.step, QImage::Format_RGB888);
    } else if (image.channels() == 1) {
        // Grayscale image
        qimg = QImage(image.data, image.cols, image.rows, 
                    image.step, QImage::Format_Grayscale8);
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Unsupported image format: %d channels", image.channels());
        return;
    }
    
    // Scale and display the image
    label->setPixmap(QPixmap::fromImage(qimg).scaled(
        label->width(), label->height(),
        Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

}  // namespace power_rune