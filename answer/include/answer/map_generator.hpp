#ifndef _MAP_GENERATOR_HPP_
#define _MAP_GENERATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "info_interfaces/msg/map.hpp"
#include "info_interfaces/msg/area.hpp"
#include "info_interfaces/msg/robot.hpp"

class MapGenerator {
public:
    static info_interfaces::msg::Map generate_map(const cv::Mat& binary_image, int grid_width, int grid_height);
    static info_interfaces::msg::Area detect_game_areas(const cv::Mat& image);
    static info_interfaces::msg::Robot detect_robots(const cv::Mat& image, int hp_block_width);
};

#endif // !_MAP_GENERATOR_HPP_