#ifndef _ROBOT_CONTROLLER_HPP_
#define _ROBOT_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose2_d.hpp"
#include "info_interfaces/msg/robot.hpp"

class RobotController {
public:
    RobotController();
    
    geometry_msgs::msg::Pose2D calculate_movement(
        const info_interfaces::msg::Robot::SharedPtr robot_info,
        const std::vector<std::pair<int, int>>& path
    );

    bool should_shoot(
        const info_interfaces::msg::Robot::SharedPtr robot_info,
        int bullet_count
    );

private:
    const int m_dir[4][2]{{-1,-1},{1,-1},{1,1},{-1,1}};
    int m_move_count;
};

#endif // _ROBOT_CONTROLLER_HPP_