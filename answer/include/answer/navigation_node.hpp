#ifndef _NAVIGATION_NODE_HPP_
#define _NAVIGATION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose2_d.hpp"
#include "example_interfaces/msg/bool.hpp"
#include "info_interfaces/msg/map.hpp"
#include "info_interfaces/msg/area.hpp"
#include "info_interfaces/msg/robot.hpp"
#include "path_planner.hpp"
#include "robot_controller.hpp"
#include "topic_config.hpp"
#include "game_config.hpp"

namespace navigation {
    class Node : public rclcpp::Node {
    public:
        explicit Node(const std::string& name);
        
    private:
        void handle_map_update(const info_interfaces::msg::Map::SharedPtr map);
        void handle_area_update(const info_interfaces::msg::Area::SharedPtr area);
        void handle_robot_update(const info_interfaces::msg::Robot::SharedPtr robot);
        
    private:
        info_interfaces::msg::Map::SharedPtr m_map;
        info_interfaces::msg::Area m_area;
        RobotController m_controller;
        bool m_need_recover;
        bool m_should_stop;
        int m_bullet_num;
        double m_last_hp;
        uint32_t m_last_real_x;
        uint32_t m_last_real_y;
        
        rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr m_pose_publisher;
        rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr m_shoot_publisher;
        rclcpp::Subscription<info_interfaces::msg::Map>::SharedPtr m_map_subscription;
        rclcpp::Subscription<info_interfaces::msg::Area>::SharedPtr m_area_subscription;
        rclcpp::Subscription<info_interfaces::msg::Robot>::SharedPtr m_robot_subscription;
    };
}

#endif // !_NAVIGATION_NODE_HPP_