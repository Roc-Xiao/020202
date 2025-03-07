#include "robot_controller.hpp"
#include "game_config.hpp"
#include <cmath>

RobotController::RobotController() : m_move_count(0) {}

geometry_msgs::msg::Pose2D RobotController::calculate_movement(
    const info_interfaces::msg::Robot::SharedPtr robot_info,
    const std::vector<std::pair<int, int>>& path)
{
    geometry_msgs::msg::Pose2D pose;

    if (path.size() >= 2) {
        pose.x = (path[1].first - robot_info->our_robot_grid_pos.x) * game_params::speed_scale;
        pose.y = (path[1].second - robot_info->our_robot_grid_pos.y) * game_params::speed_scale;

        if (!robot_info->enemy_real_pos_vec.empty()) {
            int32_t dy = static_cast<int32_t>(robot_info->enemy_real_pos_vec[0].y) -
                        static_cast<int32_t>(robot_info->our_robot_real_pos.y);
            int32_t dx = static_cast<int32_t>(robot_info->enemy_real_pos_vec[0].x) -
                        static_cast<int32_t>(robot_info->our_robot_real_pos.x);
            pose.theta = std::atan2(dy, dx);
        }
        else {
            pose.theta = 0;
        }
    }
    else {
        pose.x = m_dir[m_move_count][0];
        pose.y = m_dir[m_move_count][1];
        m_move_count = (m_move_count + 1) % 4;

        if (!robot_info->enemy_real_pos_vec.empty()) {
            int32_t dy = static_cast<int32_t>(robot_info->enemy_real_pos_vec[0].y) -
                        static_cast<int32_t>(robot_info->our_robot_real_pos.y);
            int32_t dx = static_cast<int32_t>(robot_info->enemy_real_pos_vec[0].x) -
                        static_cast<int32_t>(robot_info->our_robot_real_pos.x);
            pose.theta = std::atan2(dy, dx);
        }
        else {
            pose.theta = 0;
        }
    }

    return pose;
}

bool RobotController::should_shoot(
    const info_interfaces::msg::Robot::SharedPtr robot_info,
    int bullet_count)
{
    if (robot_info->enemy_real_pos_vec.empty() || bullet_count <= 0) {
        return false;
    }

    int dx = static_cast<int>(robot_info->enemy_real_pos_vec[0].x - robot_info->our_robot_real_pos.x);
    int dy = static_cast<int>(robot_info->enemy_real_pos_vec[0].y - robot_info->our_robot_real_pos.y);
    int distance = std::abs(dx) + std::abs(dy);
                  
    return distance < game_params::attack_distance;
} 