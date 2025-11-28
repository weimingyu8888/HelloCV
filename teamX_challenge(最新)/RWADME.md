算法原理
1.VisionNode
    1. 把图像转成 HSV
    2. 根据颜色提取红球、黑板、绿色方块
    3. 轮廓筛选（面积、矩形度、圆度等等）
    4. 找装甲数字 → 匹配模板
    5. 输出对应目标类型 + 四个角点
2.ShooterNode
      取装甲板的 4 个角点
    • 算出中心点 (px, py)
    • 用相机内参投影成 (X, Y, Z)
    • 计算 yaw, pitch（加上重力补偿公式）
    • 返回裁判系统
依赖
     ROS2 Humble
    • rclcpp
    • OpenCV
    • cv_bridge
    • geometry_msgs
    • sensor_msgs
    • referee_pkg
    • pose_msg

编译代码

colcon build
source install/setup.bash

节点启动
ros2 launch player_pkg vision.launch.py
ros2 launch player_pkg shooter.launch.py

模型
ros2 launch target_model_pkg target_action.launch.py model:=src/target_model_pkg/urdf/armor/armor_1.sdf model_name:=armor_1  
裁判
ros2 launch referee_pkg referee_pkg_launch.xml     TeamName:="YOUR_TEAM_NAME"     StageSelect:=0     ModeSelect:=0
相机
ros2 launch camera_sim_pkg camera.launch.py




语雀内容
https://www.yuque.com/u59622601/dtp5zk/swi0yk6rgilcgc69?singleDoc# 《技术报告》



