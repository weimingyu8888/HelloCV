#include <rclcpp/rclcpp.hpp>
#include <referee_pkg/srv/hit_armor.hpp>
#include <referee_pkg/msg/multi_object.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cmath>
#include <vector>

using namespace std;

class HitArmorNode : public rclcpp::Node {
public:
    HitArmorNode() : Node("shooter_node") {

        RCLCPP_INFO(this->get_logger(), "HitArmorNode started.");

        // 参数：相机内参（必须与你 Vision 的一致）
        this->declare_parameter("fx", 600.0);
        this->declare_parameter("fy", 600.0);
        this->declare_parameter("cx", 320.0);
        this->declare_parameter("cy", 240.0);
        this->declare_parameter("armor_distance", 3.0);  // 无深度时使用

        fx_ = this->get_parameter("fx").as_double();
        fy_ = this->get_parameter("fy").as_double();
        cx_ = this->get_parameter("cx").as_double();
        cy_ = this->get_parameter("cy").as_double();
        assume_Z_ = this->get_parameter("armor_distance").as_double();

        // 订阅 VisionNode 提供的装甲板信息
        vision_sub_ = this->create_subscription<referee_pkg::msg::MultiObject>(
            "/vision/target", 10,
            std::bind(&HitArmorNode::vision_callback, this, std::placeholders::_1)
        );

        // 裁判系统调用的 service server
        service_ = this->create_service<referee_pkg::srv::HitArmor>(
            "/referee/hit_armor",
            std::bind(&HitArmorNode::solve_hit_request, this,
                      std::placeholders::_1,
                      std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "HitArmor service ready.");
    }

private:

    // 最新的装甲板角点
    vector<geometry_msgs::msg::Point> last_corners_;
    rclcpp::Subscription<referee_pkg::msg::MultiObject>::SharedPtr vision_sub_;
    rclcpp::Service<referee_pkg::srv::HitArmor>::SharedPtr service_;

    double fx_, fy_, cx_, cy_, assume_Z_;

    // 更新装甲板 4 点
    void vision_callback(const referee_pkg::msg::MultiObject::SharedPtr msg) {
        for (auto &obj : msg->objects) {
            if (obj.target_type.find("armor") != string::npos) {
                if (obj.corners.size() == 4) {
                    last_corners_ = obj.corners;
                }
            }
        }
    }

    // 服务回调：裁判请求 → 返回角度
    void solve_hit_request(
        const std::shared_ptr<referee_pkg::srv::HitArmor::Request> req,
        std::shared_ptr<referee_pkg::srv::HitArmor::Response> res)
    {
        if (last_corners_.size() != 4) {
            RCLCPP_WARN(this->get_logger(), "No armor detected yet!");
            res->yaw = 0;
            res->pitch = 0;
            res->roll = 0;
            return;
        }

        // 计算装甲板中心点（像素）
        double px=0, py=0;
        for (auto &p : last_corners_) {
            px += p.x;
            py += p.y;
        }
        px /= 4.0;
        py /= 4.0;

        // 投影到相机坐标系
        double X = (px - cx_) * assume_Z_ / fx_;
        double Y = (py - cy_) * assume_Z_ / fy_;
        double Z = assume_Z_;

        // 弹丸速度（比赛固定）
        double v = 1.5;
        double g = req->g;

        // 解 pitch：包含重力补偿
        double d = sqrt(X*X + Z*Z);  
        double pitch = atan2(Y, d);

        // 解 yaw
        double yaw = atan2(X, Z);

        // roll 统一为 0
        double roll = 0;

        res->yaw = yaw;
        res->pitch = pitch;
        res->roll = roll;

        RCLCPP_INFO(this->get_logger(),
                    "HitArmor result → yaw=%.3f°, pitch=%.3f°, roll=0",
                    yaw * 180/M_PI, pitch * 180/M_PI);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = make_shared<HitArmorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
