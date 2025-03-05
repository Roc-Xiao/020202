#include "navigation_node.hpp"

navigation::Node::Node(const std::string& name) : rclcpp::Node(name) {
    m_bullet_num = 10;
    m_last_hp = 1.0;
    m_last_real_x = 0;
    m_last_real_y = 0;
    m_should_stop = false;
    m_need_recover = false;
    
    m_pose_publisher = this->create_publisher<geometry_msgs::msg::Pose2D>(
        topic_name::pose, 1);
    m_shoot_publisher = this->create_publisher<example_interfaces::msg::Bool>(
        topic_name::shoot, 1);
        
    m_map_subscription = this->create_subscription<info_interfaces::msg::Map>(
        topic_name::map, 1,
        std::bind(&navigation::Node::handle_map_update, this, std::placeholders::_1));
    m_area_subscription = this->create_subscription<info_interfaces::msg::Area>(
        topic_name::area, 1,
        std::bind(&navigation::Node::handle_area_update, this, std::placeholders::_1));
    m_robot_subscription = this->create_subscription<info_interfaces::msg::Robot>(
        topic_name::robot, 1,
        std::bind(&navigation::Node::handle_robot_update, this, std::placeholders::_1));
}

void navigation::Node::handle_map_update(const info_interfaces::msg::Map::SharedPtr map) {
    RCLCPP_INFO(get_logger(), "Map received");
    m_map = map;
}

void navigation::Node::handle_area_update(const info_interfaces::msg::Area::SharedPtr area) {
    RCLCPP_INFO(get_logger(), "Area info received");
    m_area = *area;
}

void navigation::Node::handle_robot_update(const info_interfaces::msg::Robot::SharedPtr robot) {
    if (!m_map) return;
    
    std::vector<std::pair<int, int>> path;
    geometry_msgs::msg::Pose2D pose;
    
    if (robot->our_robot_hp < game_params::danger_hp || 
        m_need_recover || 
        m_bullet_num < game_params::danger_bullet_num) {
        
        m_need_recover = true;
        if (robot->our_robot_hp >= 1.0) {
            m_need_recover = false;
            return;
        }
        
        path = PathPlanner::find_path(
            m_map,
            robot->our_robot_grid_pos.x,
            robot->our_robot_grid_pos.y,
            m_area.recover_grid_pos.x,
            m_area.recover_grid_pos.y
        );
    }
    else if (!robot->enemy_grid_pos_vec.empty()) {
        path = PathPlanner::find_path(
            m_map,
            robot->our_robot_grid_pos.x,
            robot->our_robot_grid_pos.y,
            robot->enemy_grid_pos_vec[0].x,
            robot->enemy_grid_pos_vec[0].y
        );
    }
    
    pose = m_controller.calculate_movement(robot, path);
    m_pose_publisher->publish(pose);
    
    if (m_controller.should_shoot(robot, m_bullet_num)) {
        example_interfaces::msg::Bool shoot;
        shoot.data = true;
        m_shoot_publisher->publish(shoot);
    }
    
    m_last_real_x = robot->our_robot_real_pos.x;
    m_last_real_y = robot->our_robot_real_pos.y;
    m_last_hp = robot->our_robot_hp;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<navigation::Node>("navigation_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}