#include <rclcpp/rclcpp.hpp>
#include <referee_pkg/srv/hit_armor.hpp>
#include <referee_pkg/msg/multi_object.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cmath>
#include <vector>
#include <algorithm>

using namespace std;

class HitArmorNode : public rclcpp::Node {
public:
    HitArmorNode() : Node("shooter_node") {
        RCLCPP_INFO(this->get_logger(), "HitArmorNode started.");

        // 相机内参（从Gazebo输出获取）
        this->declare_parameter("fx", 600.0);
        this->declare_parameter("fy", 600.0);
        this->declare_parameter("cx", 320.0);
        this->declare_parameter("cy", 240.0);
        this->declare_parameter("default_distance", 3.0);

        fx_ = this->get_parameter("fx").as_double();
        fy_ = this->get_parameter("fy").as_double();
        cx_ = this->get_parameter("cx").as_double();
        cy_ = this->get_parameter("cy").as_double();
        default_distance_ = this->get_parameter("default_distance").as_double();

        // 订阅视觉检测结果
        vision_sub_ = this->create_subscription<referee_pkg::msg::MultiObject>(
            "/vision/target", 10,
            std::bind(&HitArmorNode::vision_callback, this, std::placeholders::_1)
        );

        
        service_ = this->create_service<referee_pkg::srv::HitArmor>(
            "/referee/hit_arror", 
            std::bind(&HitArmorNode::solve_hit_request, this,
                      std::placeholders::_1,
                      std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "HitArmor service ready at /referee/hit_arror");
    }

private:
    vector<geometry_msgs::msg::Point> last_corners_;
    rclcpp::Subscription<referee_pkg::msg::MultiObject>::SharedPtr vision_sub_;
    rclcpp::Service<referee_pkg::srv::HitArmor>::SharedPtr service_;

    double fx_, fy_, cx_, cy_, default_distance_;

    void vision_callback(const referee_pkg::msg::MultiObject::SharedPtr msg) {
        for (auto &obj : msg->objects) {
            if (obj.target_type.find("armor") != string::npos) {
                if (obj.corners.size() == 4) {
                    last_corners_ = obj.corners;
                    RCLCPP_DEBUG(this->get_logger(), "Updated armor corners");
                }
            }
        }
    }

    // 计算装甲板3D位置和姿态
    void calculate_armor_pose(const vector<geometry_msgs::msg::Point>& corners,
                             double& center_x, double& center_y, double& width, double& height) {
        // 四个点是在装甲板局部坐标系下的物理坐标
        // 计算中心点（在装甲板局部坐标系中）
        center_x = 0.0;
        center_y = 0.0;
        for (const auto& point : corners) {
            center_x += point.x;
            center_y += point.y;
        }
        center_x /= 4.0;
        center_y /= 4.0;

        // 计算装甲板尺寸
        width = sqrt(pow(corners[1].x - corners[0].x, 2) + pow(corners[1].y - corners[0].y, 2));
        height = sqrt(pow(corners[3].x - corners[0].x, 2) + pow(corners[3].y - corners[0].y, 2));

        RCLCPP_INFO(this->get_logger(), "Armor size: %.3f x %.3f, center: (%.3f, %.3f)", 
                   width, height, center_x, center_y);
    }

    // 改进的弹道计算
    void calculate_trajectory(double target_x, double target_y, double target_z,
                             double v0, double g, 
                             double& yaw, double& pitch, double& roll) {
        
        // 1. 计算水平距离
        double horizontal_distance = sqrt(target_x * target_x + target_z * target_z);
        
        // 2. 使用迭代法寻找最佳发射角
        double best_pitch = 0.0;
        double min_error = 1e9;
        
        // 在合理范围内搜索最佳角度
        for (double test_pitch = 0.1; test_pitch < M_PI/2 - 0.1; test_pitch += 0.01) {
            double error = simulate_trajectory(v0, test_pitch, g, target_x, target_y, target_z);
            
            if (error < min_error) {
                min_error = error;
                best_pitch = test_pitch;
            }
        }
        
        // 3. 计算偏航角
        yaw = atan2(target_x, target_z);
        
        // 4. 使用找到的最佳俯仰角
        pitch = best_pitch;
        
        // 5. 横滚角通常为0
        roll = 0.0;
        
        RCLCPP_INFO(this->get_logger(), 
                   "Target: (%.3f, %.3f, %.3f), Best pitch: %.3f rad (%.1f°)", 
                   target_x, target_y, target_z, pitch, pitch * 180/M_PI);
    }

    // 弹道模拟
    double simulate_trajectory(double v0, double pitch, double g, 
                              double target_x, double target_y, double target_z) {
        double dt = 0.01;
        double t = 0.0;
        double x = 0.0, y = 0.0, z = 0.0;
        
        double vx = v0 * cos(pitch);
        double vy = v0 * sin(pitch);
        double vz = 0.0;
        
        double min_distance = 1e9;
        
        // 模拟弹道直到落地或超时
        while (t < 10.0 && y >= 0) {
            x += vx * dt;
            z += vz * dt;
            y += vy * dt;
            
            // 重力影响
            vy -= g * dt;
            
            // 计算与目标的距离
            double dx = x - target_x;
            double dy = y - target_y;
            double dz = z - target_z;
            double distance = sqrt(dx*dx + dy*dy + dz*dz);
            
            min_distance = min(min_distance, distance);
            
            // 如果已经飞过目标点，停止模拟
            if (x > target_x * 1.5) break;
            
            t += dt;
        }
        
        return min_distance;
    }

    void solve_hit_request(
        const std::shared_ptr<referee_pkg::srv::HitArmor::Request> req,
        std::shared_ptr<referee_pkg::srv::HitArmor::Response> res) {
        
        RCLCPP_INFO(this->get_logger(), "=== Hit Armor Service Called ===");
        RCLCPP_INFO(this->get_logger(), "Gravity: %.3f m/s²", req->g);

        // 检查输入数据
        if (req->modelpoint.size() != 4) {
            RCLCPP_ERROR(this->get_logger(), "Invalid model points: expected 4, got %zu", 
                        req->modelpoint.size());
            res->yaw = 0.0;
            res->pitch = 0.0;
            res->roll = 0.0;
            return;
        }

        // 打印接收到的四个点（调试用）
        for (size_t i = 0; i < req->modelpoint.size(); i++) {
            RCLCPP_DEBUG(this->get_logger(), "Model point %zu: (%.3f, %.3f, %.3f)", 
                        i, req->modelpoint[i].x, req->modelpoint[i].y, req->modelpoint[i].z);
        }

        // 计算装甲板3D信息
        double center_x, center_y, width, height;
        calculate_armor_pose(req->modelpoint, center_x, center_y, width, height);

       
        
        double target_x, target_y, target_z;
        
        //使用固定距离（简单但不够准确）
        target_z = default_distance_;
        
 
        
        // 将装甲板局部坐标转换到相机坐标系
        target_x = center_x;  // 这里需要根据实际情况调整
        target_y = center_y;
        
        RCLCPP_INFO(this->get_logger(), "Estimated target position: (%.3f, %.3f, %.3f)", 
                   target_x, target_y, target_z);

        // 计算弹道
        double v0 = 1.5;  // 弹丸初速度1.5m/s
        calculate_trajectory(target_x, target_y, target_z, v0, req->g, 
                           res->yaw, res->pitch, res->roll);

        RCLCPP_INFO(this->get_logger(), 
                   "Final angles: yaw=%.3f° pitch=%.3f° roll=%.3f°",
                   res->yaw * 180/M_PI, res->pitch * 180/M_PI, res->roll * 180/M_PI);
        
        RCLCPP_INFO(this->get_logger(), "=== Hit Armor Service Completed ===");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = make_shared<HitArmorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
