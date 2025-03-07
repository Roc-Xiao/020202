#ifndef _VISION_NODE_HPP_
#define _VISION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/msg/image.hpp"
#include "info_interfaces/msg/map.hpp"
#include "info_interfaces/msg/area.hpp"
#include "info_interfaces/msg/robot.hpp"
#include "vision_processor.hpp"
#include "map_generator.hpp"
#include "topic_config.hpp"
#include "game_config.hpp"

namespace vision {
    class Node : public rclcpp::Node {
    public:
        explicit Node(const std::string& name);
        
    private:
        void process_image(const sensor_msgs::msg::Image::SharedPtr ros_img);
        
    private:
        bool m_initialized;
        int m_hp_block_width;
        rclcpp::Publisher<info_interfaces::msg::Map>::SharedPtr m_map_publisher;
        rclcpp::Publisher<info_interfaces::msg::Area>::SharedPtr m_area_publisher;
        rclcpp::Publisher<info_interfaces::msg::Robot>::SharedPtr m_robot_publisher;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_img_subscription;
    };
}

#endif // _VISION_NODE_HPP_