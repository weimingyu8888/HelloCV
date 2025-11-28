#include <cv_bridge/cv_bridge.h>

#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpsp>
#include <referee_pkg/msg/multi_object.hpp>
#include <referee_pkg/msg/object.hpp>
#include <referee_pkg/msg/race_stage.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>

#include "sensor_msgs/msg/image.hpp"

using namespace std;
using namespace rclcpp;
using namespace cv;

class VisionNode : public rclcpp::Node {
public:
    VisionNode(string name) : Node(name), current_stage_(0) {
        RCLCPP_INFO(this->get_logger(), "Initializing VisionNode");
        
        // 声明所有参数
        declare_parameters();
        
        // 订阅摄像头图像
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            bind(&VisionNode::image_callback, this, std::placeholders::_1));

        // 订阅比赛阶段信息
        stage_sub_ = this->create_subscription<referee_pkg::msg::RaceStage>(
            "/referee/race_stage", 10,
            bind(&VisionNode::stage_callback, this, std::placeholders::_1));

        // 发布识别结果
        target_pub_ = this->create_publisher<referee_pkg::msg::MultiObject>(
            "/vision/target", 10);

        // 参数回调处理
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&VisionNode::parameters_callback, this, std::placeholders::_1));

        // 创建调试窗口（如果启用）
        if (show_windows_) {
            cv::namedWindow("Vision Detection Result", cv::WINDOW_AUTOSIZE);
            cv::namedWindow("Green Mask", cv::WINDOW_AUTOSIZE);
            cv::namedWindow("Black Mask", cv::WINDOW_AUTOSIZE);
            cv::namedWindow("Red Mask", cv::WINDOW_AUTOSIZE);
        }

        RCLCPP_INFO(this->get_logger(), "VisionNode initialized successfully");
    }

    ~VisionNode() {
        if (show_windows_) {
            cv::destroyWindow("Vision Detection Result");
            cv::destroyWindow("Green Mask");
            cv::destroyWindow("Black Mask");
            cv::destroyWindow("Red Mask");
        }
    }

private:
    void declare_parameters();
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters);
    void image_callback(sensor_msgs::msg::Image::SharedPtr msg);
    void stage_callback(referee_pkg::msg::RaceStage::SharedPtr msg);
    
    // 目标点计算方法
    vector<Point2f> calculate_sphere_points(const Point2f &center, float radius);
    vector<Point2f> calculate_rect_points(const RotatedRect &rect);
    vector<Point2f> calculate_rect_points_from_contour(const vector<Point>& contour);
    
    // 目标类型识别
    string recognize_armor_number(const Mat& roi);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<referee_pkg::msg::RaceStage>::SharedPtr stage_sub_;
    rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr target_pub_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    vector<Point2f> points_vector_;
    vector<string> types_vector_;
    int current_stage_;
    
    // 参数变量
    bool debug_mode_;
    bool show_windows_;
    
    // 颜色参数
    Scalar red_lower1_, red_upper1_, red_lower2_, red_upper2_;
    Scalar green_lower_, green_upper_;
    
    // 黑色装甲板参数
    int black_threshold_;
    int number_threshold_;
    
    // 形态学参数
    int kernel_small_size_;
    int kernel_medium_size_;
    int kernel_large_size_;
    
    // 轮廓参数
    int min_area_;
    int max_area_;
    
    // 球体参数
    int sphere_min_area_;
    int sphere_min_radius_;
    int sphere_max_radius_;
    double sphere_min_circularity_;
    
    // 装甲板参数
    int armor_min_area_;
    int armor_max_area_;
    double armor_min_rect_ratio_;
    double armor_min_aspect_ratio_;
    double armor_max_aspect_ratio_;
    
    // 矩形参数
    int rect_min_area_;
    double rect_approx_epsilon_;
    
    // 数字识别参数
    int target_width_;
    int target_height_;
    int binary_threshold1_;
    int binary_threshold2_;
    int binary_threshold3_;
    int morph_kernel_size_;
    string template_path_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisionNode>("vision_node");
    RCLCPP_INFO(node->get_logger(), "Starting VisionNode");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

void VisionNode::declare_parameters() {
    // 调试和显示设置
    this->declare_parameter<bool>("debug_mode", true);
    this->declare_parameter<bool>("show_windows", true);
    
    // 红色检测参数
    this->declare_parameter<std::vector<int64_t>>("colors.red.lower1", std::vector<int64_t>{0, 120, 70});
    this->declare_parameter<std::vector<int64_t>>("colors.red.upper1", std::vector<int64_t>{10, 255, 255});
    this->declare_parameter<std::vector<int64_t>>("colors.red.lower2", std::vector<int64_t>{170, 120, 70});
    this->declare_parameter<std::vector<int64_t>>("colors.red.upper2", std::vector<int64_t>{180, 255, 255});
    
    // 绿色检测参数
    this->declare_parameter<std::vector<int64_t>>("colors.green.lower", std::vector<int64_t>{75, 40, 40});
    this->declare_parameter<std::vector<int64_t>>("colors.green.upper", std::vector<int64_t>{105, 255, 255});
    
    // 黑色装甲板参数
    this->declare_parameter<int>("black_armor.threshold", 80);
    this->declare_parameter<int>("black_armor.number_threshold", 200);
    
    // 形态学参数
    this->declare_parameter<int>("morphology.kernel_small", 3);
    this->declare_parameter<int>("morphology.kernel_medium", 5);
    this->declare_parameter<int>("morphology.kernel_large", 3);
    
    // 轮廓参数
    this->declare_parameter<int>("contours.min_area", 100);
    this->declare_parameter<int>("contours.max_area", 50000);
    
    // 球体参数
    this->declare_parameter<int>("sphere.min_area", 300);
    this->declare_parameter<int>("sphere.min_radius", 15);
    this->declare_parameter<int>("sphere.max_radius", 200);
    this->declare_parameter<double>("sphere.min_circularity", 0.7);
    
    // 装甲板参数
    this->declare_parameter<int>("armor.min_area", 100);
    this->declare_parameter<int>("armor.max_area", 50000);
    this->declare_parameter<double>("armor.min_rect_ratio", 0.6);
    this->declare_parameter<double>("armor.min_aspect_ratio", 0.7);
    this->declare_parameter<double>("armor.max_aspect_ratio", 2.5);
    
    // 矩形参数
    this->declare_parameter<int>("rectangle.min_area", 50);
    this->declare_parameter<double>("rectangle.approx_epsilon", 0.03);
    
    // 数字识别参数
    this->declare_parameter<int>("number_recognition.target_width", 60);
    this->declare_parameter<int>("number_recognition.target_height", 80);
    this->declare_parameter<int>("number_recognition.binary_threshold1", 200);
    this->declare_parameter<int>("number_recognition.binary_threshold2", 50);
    this->declare_parameter<int>("number_recognition.binary_threshold3", 127);
    this->declare_parameter<int>("number_recognition.morph_kernel_size", 2);
    this->declare_parameter<string>("number_recognition.template_path", 
        "/home/wangsong/Documents/222/Vision_Arena_2025-main/src/target_model_pkg/urdf/armor/textures/");
    
    // 获取参数初始值
    update_parameters();
}

void VisionNode::update_parameters() {
    debug_mode_ = this->get_parameter("debug_mode").as_bool();
    show_windows_ = this->get_parameter("show_windows").as_bool();
    
    // 颜色参数
    auto red_lower1 = this->get_parameter("colors.red.lower1").as_integer_array();
    auto red_upper1 = this->get_parameter("colors.red.upper1").as_integer_array();
    auto red_lower2 = this->get_parameter("colors.red.lower2").as_integer_array();
    auto red_upper2 = this->get_parameter("colors.red.upper2").as_integer_array();
    auto green_lower = this->get_parameter("colors.green.lower").as_integer_array();
    auto green_upper = this->get_parameter("colors.green.upper").as_integer_array();
    
    red_lower1_ = Scalar(red_lower1[0], red_lower1[1], red_lower1[2]);
    red_upper1_ = Scalar(red_upper1[0], red_upper1[1], red_upper1[2]);
    red_lower2_ = Scalar(red_lower2[0], red_lower2[1], red_lower2[2]);
    red_upper2_ = Scalar(red_upper2[0], red_upper2[1], red_upper2[2]);
    green_lower_ = Scalar(green_lower[0], green_lower[1], green_lower[2]);
    green_upper_ = Scalar(green_upper[0], green_upper[1], green_upper[2]);
    
    // 黑色装甲板参数
    black_threshold_ = this->get_parameter("black_armor.threshold").as_int();
    number_threshold_ = this->get_parameter("black_armor.number_threshold").as_int();
    
    // 形态学参数
    kernel_small_size_ = this->get_parameter("morphology.kernel_small").as_int();
    kernel_medium_size_ = this->get_parameter("morphology.kernel_medium").as_int();
    kernel_large_size_ = this->get_parameter("morphology.kernel_large").as_int();
    
    // 轮廓参数
    min_area_ = this->get_parameter("contours.min_area").as_int();
    max_area_ = this->get_parameter("contours.max_area").as_int();
    
    // 球体参数
    sphere_min_area_ = this->get_parameter("sphere.min_area").as_int();
    sphere_min_radius_ = this->get_parameter("sphere.min_radius").as_int();
    sphere_max_radius_ = this->get_parameter("sphere.max_radius").as_int();
    sphere_min_circularity_ = this->get_parameter("sphere.min_circularity").as_double();
    
    // 装甲板参数
    armor_min_area_ = this->get_parameter("armor.min_area").as_int();
    armor_max_area_ = this->get_parameter("armor.max_area").as_int();
    armor_min_rect_ratio_ = this->get_parameter("armor.min_rect_ratio").as_double();
    armor_min_aspect_ratio_ = this->get_parameter("armor.min_aspect_ratio").as_double();
    armor_max_aspect_ratio_ = this->get_parameter("armor.max_aspect_ratio").as_double();
    
    // 矩形参数
    rect_min_area_ = this->get_parameter("rectangle.min_area").as_int();
    rect_approx_epsilon_ = this->get_parameter("rectangle.approx_epsilon").as_double();
    
    // 数字识别参数
    target_width_ = this->get_parameter("number_recognition.target_width").as_int();
    target_height_ = this->get_parameter("number_recognition.target_height").as_int();
    binary_threshold1_ = this->get_parameter("number_recognition.binary_threshold1").as_int();
    binary_threshold2_ = this->get_parameter("number_recognition.binary_threshold2").as_int();
    binary_threshold3_ = this->get_parameter("number_recognition.binary_threshold3").as_int();
    morph_kernel_size_ = this->get_parameter("number_recognition.morph_kernel_size").as_int();
    template_path_ = this->get_parameter("number_recognition.template_path").as_string();
}

rcl_interfaces::msg::SetParametersResult VisionNode::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for (const auto &param : parameters) {
        if (param.get_name() == "debug_mode") {
            debug_mode_ = param.as_bool();
            RCLCPP_INFO(this->get_logger(), "Parameter updated: debug_mode = %s", debug_mode_ ? "true" : "false");
        }
        else if (param.get_name() == "show_windows") {
            bool old_value = show_windows_;
            show_windows_ = param.as_bool();
            RCLCPP_INFO(this->get_logger(), "Parameter updated: show_windows = %s", show_windows_ ? "true" : "false");
            
            // 动态创建或销毁窗口
            if (show_windows_ && !old_value) {
                cv::namedWindow("Vision Detection Result", cv::WINDOW_AUTOSIZE);
                cv::namedWindow("Green Mask", cv::WINDOW_AUTOSIZE);
                cv::namedWindow("Black Mask", cv::WINDOW_AUTOSIZE);
                cv::namedWindow("Red Mask", cv::WINDOW_AUTOSIZE);
            } else if (!show_windows_ && old_value) {
                cv::destroyWindow("Vision Detection Result");
                cv::destroyWindow("Green Mask");
                cv::destroyWindow("Black Mask");
                cv::destroyWindow("Red Mask");
            }
        }
        // 可以继续添加其他参数的处理...
        else {
            // 对于其他参数，直接更新所有参数
            update_parameters();
            RCLCPP_INFO(this->get_logger(), "Parameters updated");
            break;
        }
    }
    
    return result;
}

void VisionNode::stage_callback(referee_pkg::msg::RaceStage::SharedPtr msg) {
    current_stage_ = msg->stage;
    RCLCPP_INFO(this->get_logger(), "Stage changed to: %d", current_stage_);
}

vector<Point2f> VisionNode::calculate_sphere_points(const Point2f &center, float radius) {
    vector<Point2f> points;
    points.push_back(Point2f(center.x - radius, center.y)); // 左点
    points.push_back(Point2f(center.x, center.y + radius)); // 下点
    points.push_back(Point2f(center.x + radius, center.y)); // 右点
    points.push_back(Point2f(center.x, center.y - radius)); // 上点
    return points;
}

vector<Point2f> VisionNode::calculate_rect_points(const RotatedRect &rect) {
    vector<Point2f> points;
    Point2f vertices[4];
    rect.points(vertices);
    vector<Point2f> temp_points(vertices, vertices + 4);

    // 步骤1：找左下点（y最大，y相同则x最小）
    Point2f bottom_left = temp_points[0];
    for (const auto& p : temp_points) {
        if (p.y > bottom_left.y || (p.y == bottom_left.y && p.x < bottom_left.x)) {
            bottom_left = p;
        }
    }

    // 步骤2：排除起点，收集其他点（修复精度判定）
    vector<Point2f> others;
    for (const auto& p : temp_points) {
        if ( (p.x - bottom_left.x)*(p.x - bottom_left.x) + (p.y - bottom_left.y)*(p.y - bottom_left.y) > 1e-6 ) {
            others.push_back(p);
        }
    }

    // 步骤3：图像坐标系下的逆时针排序（修复叉乘符号）
    sort(others.begin(), others.end(), [&bottom_left](const Point2f& a, const Point2f& b) {
        float vax = a.x - bottom_left.x;
        float vay = a.y - bottom_left.y;
        float vbx = b.x - bottom_left.x;
        float vby = b.y - bottom_left.y;
        return (vax * vby - vay * vbx) < 0; // 图像坐标系下，<0是逆时针
    });

    // 步骤4：组装顺序（左下→右下→右上→左上）
    points.push_back(bottom_left); // 1号
    points.push_back(others[0]); // 2号（右下）
    points.push_back(others[1]); // 3号（右上）
    points.push_back(others[2]); // 4号（左上）

    return points;
}

vector<Point2f> VisionNode::calculate_rect_points_from_contour(const vector<Point>& contour) {
    vector<Point2f> points;
    vector<Point> approx;
    double epsilon = rect_approx_epsilon_ * arcLength(contour, true);
    approxPolyDP(contour, approx, epsilon, true);
    if (approx.size() == 4) {
        // 找到左下点（x最小，y最大）
        Point2f bottom_left = approx[0];
        int start_idx = 0;
        for (int i = 1; i < 4; i++) {
            if (approx[i].y > bottom_left.y ||
                (approx[i].y == bottom_left.y && approx[i].x < bottom_left.x)) {
                bottom_left = approx[i];
                start_idx = i;
            }
        }
        // 按逆时针顺序添加点
        for (int i = 0; i < 4; i++) {
            points.push_back(Point2f(approx[(start_idx + i) % 4].x,
                                    approx[(start_idx + i) % 4].y));
        }
    }
    return points;
}

string VisionNode::recognize_armor_number(const Mat& roi) {
    if (roi.empty()) return "armor_red_1";

    // 预处理：突出白色数字（黑色背景）
    Mat gray, binary;
    cvtColor(roi, gray, COLOR_BGR2GRAY);
    
    // 使用参数化的阈值
    threshold(gray, binary, binary_threshold1_, 255, THRESH_BINARY);
    Mat mask;
    threshold(gray, mask, binary_threshold2_, 255, THRESH_BINARY);
    binary &= mask;
    threshold(binary, binary, binary_threshold3_, 255, THRESH_BINARY);

    Mat kernel = getStructuringElement(MORPH_RECT, Size(morph_kernel_size_, morph_kernel_size_));
    morphologyEx(binary, binary, MORPH_OPEN, kernel);

    // 提取数字的最小包围矩形
    vector<vector<Point>> contours;
    findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return "armor_red_1";

    int max_area_idx = 0;
    double max_area = contourArea(contours[0]);
    for (size_t i = 1; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            max_area_idx = i;
        }
    }
    RotatedRect num_rect = minAreaRect(contours[max_area_idx]);
    Rect num_bound = num_rect.boundingRect();

    Rect safe_num_bound = num_bound & Rect(0, 0, roi.cols, roi.rows);
    if (safe_num_bound.width <= 0 || safe_num_bound.height <= 0) {
        return "armor_red_1";
    }

    // 使用参数化的尺寸
    Mat num_cropped = binary(safe_num_bound);
    Mat num_normalized;
    resize(num_cropped, num_normalized, Size(target_width_, target_height_));

    // 特征增强mask（保持原有逻辑）
    int width = num_normalized.cols;
    int height = num_normalized.rows;

    // 数字1的增强mask
    Mat num_1_enhanced = num_normalized.clone();
    Mat mask_1 = Mat::zeros(num_normalized.size(), CV_8UC1);
    Rect center_roi(width*1/5, 0, width*2/5, height);
    mask_1(center_roi) = 255;
    num_1_enhanced &= mask_1;

    // 数字3的增强mask
    Mat num_3_enhanced = num_normalized.clone();
    Mat mask_3 = Mat::zeros(num_normalized.size(), CV_8UC1);
    Rect top_roi(0, 0, width, height/2);
    Rect bottom_roi(0, height*2/3, width, height/3);
    Rect middle_roi(width*1/4, height*1/4, width*2/4, height*1/2);
    mask_3(top_roi) = 255;
    mask_3(bottom_roi) = 255;
    mask_3(middle_roi) = 255;
    num_3_enhanced &= mask_3;

    // 数字4的增强mask
    Mat num_4_enhanced = num_normalized.clone();
    Mat mask_4 = Mat::zeros(num_normalized.size(), CV_8UC1);
    Rect left_vertical_roi(0, 0, width/4, height);
    Rect top_horizontal_roi(0, 0, width, height/3);
    Rect right_diagonal_roi(width/2, height/3, width/2, height/3);
    mask_4(left_vertical_roi) = 255;
    mask_4(top_horizontal_roi) = 255;
    mask_4(right_diagonal_roi) = 255;
    num_4_enhanced &= mask_4;

    // 加载模板
    vector<pair<string, Mat>> num_templates;
    num_templates.emplace_back("armor_red_1", imread(template_path_ + "small_num1.png", IMREAD_GRAYSCALE));
    num_templates.emplace_back("armor_red_2", imread(template_path_ + "small_num2.png", IMREAD_GRAYSCALE));
    num_templates.emplace_back("armor_red_3", imread(template_path_ + "small_num3.png", IMREAD_GRAYSCALE));
    num_templates.emplace_back("armor_red_4", imread(template_path_ + "small_num4.png", IMREAD_GRAYSCALE));
    num_templates.emplace_back("armor_red_5", imread(template_path_ + "small_num5.png", IMREAD_GRAYSCALE));

    // 模板预处理
    for (auto& [type, temp] : num_templates) {
        if (temp.empty()) continue;
        threshold(temp, temp, 127, 255, THRESH_BINARY);
        resize(temp, temp, Size(target_width_, target_height_));
    }

    // 模板匹配
    double max_score = -1.0;
    string best_match = "armor_red_1";
    for (const auto& [type, temp] : num_templates) {
        if (temp.empty()) continue;
        
        Mat target = num_normalized;
        if (type == "armor_red_1") {
            target = num_1_enhanced;
        } else if (type == "armor_red_3") {
            target = num_3_enhanced;
        } else if (type == "armor_red_4") {
            target = num_4_enhanced;
        }

        Mat result;
        matchTemplate(target, temp, result, TM_CCOEFF_NORMED);
        double min_val, max_val;
        minMaxLoc(result, &min_val, &max_val);
        
        if (debug_mode_) {
            RCLCPP_DEBUG(this->get_logger(), "模板%s 相似度：%.4f", type.c_str(), max_val);
        }
        
        if (max_val > max_score) {
            max_score = max_val;
            best_match = type;
        }
    }
    
    if (debug_mode_) {
        RCLCPP_DEBUG(this->get_logger(), "最佳匹配：%s，最终相似度：%.4f", best_match.c_str(), max_score);
    }

    return best_match;
}

void VisionNode::image_callback(sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        // 图像转换
        RCLCPP_DEBUG(this->get_logger(), "Image encoding: %s", msg->encoding.c_str());
        cv_bridge::CvImagePtr cv_ptr;
        Mat image;
        
        if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            image = cv_ptr->image;
        } else if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
            cvtColor(cv_ptr->image, image, COLOR_RGB2BGR);
        } else {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            image = cv_ptr->image;
            RCLCPP_WARN(this->get_logger(), "Unsupported image encoding: %s, forced to BGR8", msg->encoding.c_str());
        }

        if (image.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty image");
            return;
        }

        // 创建结果图像
        cv::Mat result_image = image.clone();

        // 转换到 HSV 空间
        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

        points_vector_.clear();
        types_vector_.clear();

        int sphere_count = 0;
        int armor_count = 0;
        int rect_count = 0;

        // 颜色检测 - 使用参数化的颜色范围
        cv::Mat red_mask1, red_mask2, red_mask;
        cv::inRange(hsv, red_lower1_, red_upper1_, red_mask1);
        cv::inRange(hsv, red_lower2_, red_upper2_, red_mask2);
        red_mask = red_mask1 | red_mask2;

        cv::Mat green_mask;
        cv::inRange(hsv, green_lower_, green_upper_, green_mask);

        // 黑色检测 - 使用参数化的阈值
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        cv::Mat black_armor_mask;
        cv::threshold(gray, black_armor_mask, black_threshold_, 255, cv::THRESH_BINARY_INV);
        cv::Mat number_mask;
        cv::threshold(gray, number_mask, number_threshold_, 255, cv::THRESH_BINARY);
        black_armor_mask = black_armor_mask - number_mask;

        if (debug_mode_) {
            int black_non_zero = cv::countNonZero(black_armor_mask);
            RCLCPP_DEBUG(this->get_logger(), "Black armor mask non-zero pixels: %d", black_non_zero);
        }

        // 形态学操作 - 使用参数化的核大小
        cv::Mat kernel_small = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_small_size_, kernel_small_size_));
        cv::Mat kernel_medium = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_medium_size_, kernel_medium_size_));
        cv::Mat kernel_large = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_large_size_, kernel_large_size_));
        
        cv::Mat red_morph, green_morph, black_morph;
        cv::morphologyEx(red_mask, red_morph, cv::MORPH_CLOSE, kernel_medium);
        cv::morphologyEx(red_morph, red_morph, cv::MORPH_OPEN, kernel_medium);
        
        cv::morphologyEx(green_mask, green_morph, cv::MORPH_OPEN, kernel_small);
        cv::morphologyEx(green_morph, green_morph, cv::MORPH_CLOSE, kernel_small);
        
        cv::morphologyEx(black_armor_mask, black_morph, cv::MORPH_CLOSE, kernel_large);
        cv::morphologyEx(black_morph, black_morph, cv::MORPH_OPEN, kernel_large);

        if (debug_mode_) {
            int green_non_zero = cv::countNonZero(green_morph);
            RCLCPP_DEBUG(this->get_logger(), "Green mask non-zero pixels: %d", green_non_zero);
        }

        // 检测红色目标（球体）- 使用参数化的阈值
        std::vector<std::vector<cv::Point>> red_contours;
        cv::findContours(red_morph, red_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (size_t i = 0; i < red_contours.size(); i++) {
            double area = cv::contourArea(red_contours[i]);
            if (area < sphere_min_area_) continue;

            Point2f center;
            float radius = 0;
            minEnclosingCircle(red_contours[i], center, radius);
            double perimeter = arcLength(red_contours[i], true);
            double circularity = 4 * CV_PI * area / (perimeter * perimeter);

            if (circularity > sphere_min_circularity_ && radius > sphere_min_radius_ && radius < sphere_max_radius_) {
                vector<Point2f> sphere_points = calculate_sphere_points(center, radius);
                for (int j = 0; j < 4; j++) {
                    points_vector_.push_back(sphere_points[j]);
                }
                types_vector_.push_back("sphere");

                // 绘制球体
                if (show_windows_) {
                    cv::circle(result_image, center, static_cast<int>(radius), cv::Scalar(0, 255, 0), 2);
                    cv::circle(result_image, center, 3, cv::Scalar(0, 0, 255), -1);

                    vector<cv::Scalar> point_colors = {
                        cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), 
                        cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 255)
                    };

                    for (int j = 0; j < 4; j++) {
                        cv::circle(result_image, sphere_points[j], 6, point_colors[j], -1);
                        cv::circle(result_image, sphere_points[j], 6, cv::Scalar(0, 0, 0), 2);

                        string point_text = to_string(j + 1);
                        cv::putText(result_image, point_text,
                            cv::Point(sphere_points[j].x + 10, sphere_points[j].y - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 3);
                        cv::putText(result_image, point_text,
                            cv::Point(sphere_points[j].x + 10, sphere_points[j].y - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, point_colors[j], 2);
                    }

                    string info_text = "R:" + to_string((int)radius);
                    cv::putText(result_image, info_text, cv::Point(center.x - 15, center.y + 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
                }

                sphere_count++;
                if (debug_mode_) {
                    RCLCPP_INFO(this->get_logger(), "Sphere %d, Point 1 (左): (%.1f, %.1f)",
                                sphere_count, sphere_points[0].x, sphere_points[0].y);
                    RCLCPP_INFO(this->get_logger(), "Found sphere: (%.1f, %.1f) R=%.1f C=%.3f",
                                center.x, center.y, radius, circularity);
                }
            }
        }

        // 检测黑色目标（装甲板）- 使用参数化的阈值
        std::vector<std::vector<cv::Point>> black_contours;
        cv::findContours(black_morph, black_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (debug_mode_) {
            RCLCPP_DEBUG(this->get_logger(), "Found black contours count: %zu", black_contours.size());
            for (size_t i = 0; i < black_contours.size(); i++) {
                double area = cv::contourArea(black_contours[i]);
                RCLCPP_DEBUG(this->get_logger(), "Black contour %zu area: %.1f", i, area);
            }
        }

        for (size_t i = 0; i < black_contours.size(); i++) {
            double area = cv::contourArea(black_contours[i]);
            if (area < armor_min_area_ || area > armor_max_area_) continue;

            RotatedRect rect = minAreaRect(black_contours[i]);
            if (rect.size.width <= 0 || rect.size.height <= 0) continue;

            double rect_area = rect.size.width * rect.size.height;
            double rect_ratio = area / rect_area;
            float width = rect.size.width;
            float height = rect.size.height;
            if (width < height) swap(width, height);
            float aspect_ratio = width / height;

            if (rect_ratio > armor_min_rect_ratio_ && aspect_ratio > armor_min_aspect_ratio_ && aspect_ratio < armor_max_aspect_ratio_) {
                vector<Point2f> rect_points = calculate_rect_points(rect);
                string armor_type = "armor_red_1";
                
                Rect bound_rect = boundingRect(black_contours[i]);
                Rect safe_roi = bound_rect & Rect(0, 0, image.cols, image.rows);
                if (safe_roi.width > 0 && safe_roi.height > 0) {
                    Mat roi = image(safe_roi);
                    armor_type = recognize_armor_number(roi);
                }

                for (int j = 0; j < 4; j++) {
                    points_vector_.push_back(rect_points[j]);
                }
                types_vector_.push_back(armor_type);

                // 绘制装甲板
                if (show_windows_) {
                    for (int j = 0; j < 4; j++) {
                        cv::circle(result_image, rect_points[j], 6, cv::Scalar(255, 165, 0), -1);
                        cv::circle(result_image, rect_points[j], 6, cv::Scalar(0, 0, 0), 2);

                        string point_text = to_string(j + 1);
                        cv::putText(result_image, point_text,
                            cv::Point(rect_points[j].x + 10, rect_points[j].y - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 3);
                        cv::putText(result_image, point_text,
                            cv::Point(rect_points[j].x + 10, rect_points[j].y - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 165, 0), 2);
                    }
                }

                armor_count++;
                if (debug_mode_) {
                    RCLCPP_INFO(this->get_logger(), "Armor %d (%s), Point 1 (左下): (%.1f, %.1f)",
                                armor_count, armor_type.c_str(), rect_points[0].x, rect_points[0].y);
                    RCLCPP_INFO(this->get_logger(), "Found %s: center(%.1f, %.1f) size(%.1f×%.1f)",
                                armor_type.c_str(), rect.center.x, rect.center.y,
                                rect.size.width, rect.size.height);
                }
            }
        }

        // 改进的绿色目标检测
        std::vector<std::vector<cv::Point>> green_contours;
        cv::findContours(green_morph, green_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (size_t i = 0; i < green_contours.size(); i++) {
            double area = cv::contourArea(green_contours[i]);
            if (debug_mode_) {
                RCLCPP_DEBUG(this->get_logger(), "Green contour %zu: area=%.1f", i, area);
            }
            if (area < rect_min_area_) continue;

            vector<Point2f> rect_points = calculate_rect_points_from_contour(green_contours[i]);
            if (rect_points.size() == 4) {
                string rect_type = "rect";

                for (int j = 0; j < 4; j++) {
                    points_vector_.push_back(rect_points[j]);
                }
                types_vector_.push_back(rect_type);

                // 绘制绿色矩形
                if (show_windows_) {
                    for (int j = 0; j < 4; j++) {
                        line(result_image, rect_points[j], rect_points[(j+1)%4], Scalar(0, 255, 255), 2);
                        cv::circle(result_image, rect_points[j], 6, cv::Scalar(0, 255, 0), -1);
                        cv::circle(result_image, rect_points[j], 6, cv::Scalar(0, 0, 0), 2);

                        string point_text = to_string(j + 1);
                        cv::putText(result_image, point_text,
                            cv::Point(rect_points[j].x + 10, rect_points[j].y - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 3);
                        cv::putText(result_image, point_text,
                            cv::Point(rect_points[j].x + 10, rect_points[j].y - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                    }
                }

                rect_count++;
                if (debug_mode_) {
                    RCLCPP_INFO(this->get_logger(), "Found green rect: area=%.1f, vertex count=4", area);
                }
            }
        }

        // 发布识别结果
        referee_pkg::msg::MultiObject msg_object;
        msg_object.header = msg->header;
        msg_object.num_objects = types_vector_.size();
        for (int k = 0; k < msg_object.num_objects; k++) {
            referee_pkg::msg::Object obj;
            obj.target_type = types_vector_[k];

            for (int j = 0; j < 4; j++) {
                int index = 4 * k + j;
                if (index < points_vector_.size()) {
                    geometry_msgs::msg::Point corner;
                    corner.x = points_vector_[index].x;
                    corner.y = points_vector_[index].y;
                    corner.z = 0.0;
                    obj.corners.push_back(corner);
                }
            }

            msg_object.objects.push_back(obj);
        }

        target_pub_->publish(msg_object);

        // 显示结果图像和掩码（如果启用）
        if (show_windows_) {
            cv::imshow("Vision Detection Result", result_image);
            cv::imshow("Green Mask", green_morph);
            cv::imshow("Black Mask", black_morph);
            cv::imshow("Red Mask", red_morph);
            cv::waitKey(1);
        }

        // 汇总日志输出
        if (debug_mode_) {
            RCLCPP_INFO(this->get_logger(), "Published %d targets: %d spheres, %d armors, %d green rectangles",
                        msg_object.num_objects, sphere_count, armor_count, rect_count);
        }

    } catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
    }
}