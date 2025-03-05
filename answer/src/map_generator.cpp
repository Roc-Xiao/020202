#include "map_generator.hpp"
#include "vision_processor.hpp"
#include "game_config.hpp"

info_interfaces::msg::Map MapGenerator::generate_map(
    const cv::Mat& binary_image, 
    int grid_width, 
    int grid_height)
{
    info_interfaces::msg::Map map;
    map.row = game_params::grid_num_h;
    map.col = game_params::grid_num_v;
    map.grid_width = grid_width;
    map.grid_height = grid_height;
    map.mat.resize(game_params::grid_num_h * game_params::grid_num_v);

    for (int j = 0; j < game_params::grid_num_h; j++) {
        for (int i = 0; i < game_params::grid_num_v; i++) {
            map.mat[i + j * game_params::grid_num_v] = 
                binary_image.at<uchar>((j + 0.5) * grid_height, (i + 0.5) * grid_width);
        }
    }

    return map;
}

info_interfaces::msg::Area MapGenerator::detect_game_areas(const cv::Mat& image)
{
    info_interfaces::msg::Area area;
    std::vector<cv::Point> centers;
    std::vector<cv::Rect> rects;

    // 检测密码发射区
    std::tie(centers, rects) = VisionProcessor::detect_colored_regions(
        "password", image, 
        cv::Scalar(90, 95, 250), cv::Scalar(100, 105, 255),
        20, 20, 100, 100);
    if (!centers.empty()) {
        area.password_grid_pos.x = centers[0].x / (image.cols / game_params::grid_num_v);
        area.password_grid_pos.y = centers[0].y / (image.rows / game_params::grid_num_h);
    }

    // 检测补给区（恢复区）
    std::tie(centers, rects) = VisionProcessor::detect_colored_regions(
        "recover", image,
        cv::Scalar(55, 80, 105), cv::Scalar(65, 90, 115),
        20, 20, 100, 100);
    if (!centers.empty()) {
        area.recover_grid_pos.x = centers[0].x / (image.cols / game_params::grid_num_v);
        area.recover_grid_pos.y = centers[0].y / (image.rows / game_params::grid_num_h);
    }

    // 检测基地区域
    std::tie(centers, rects) = VisionProcessor::detect_colored_regions(
        "base", image,
        cv::Scalar(165, 115, 145), cv::Scalar(175, 125, 155),
        10, 10, 100, 100);
    if (!centers.empty()) {
        area.base_grid_pos.x = centers[0].x / (image.cols / game_params::grid_num_v);
        area.base_grid_pos.y = centers[0].y / (image.rows / game_params::grid_num_h);
    }

    // 检测绿色入口
    std::tie(centers, rects) = VisionProcessor::detect_colored_regions(
        "green_entrance", image,
        cv::Scalar(25, 195, 110), cv::Scalar(35, 205, 120),
        0, 0, 100, 100);
    if (!centers.empty()) {
        area.green_in_grid_pos.x = centers[0].x / (image.cols / game_params::grid_num_v);
        area.green_in_grid_pos.y = centers[0].y / (image.rows / game_params::grid_num_h);
    }

    // 检测绿色出口
    std::tie(centers, rects) = VisionProcessor::detect_colored_regions(
        "green_exit", image,
        cv::Scalar(25, 195, 110), cv::Scalar(35, 205, 120),
        0, 0, 100, 100);
    if (!centers.empty()) {
        area.green_out_grid_pos.x = centers[0].x / (image.cols / game_params::grid_num_v);
        area.green_out_grid_pos.y = centers[0].y / (image.rows / game_params::grid_num_h);
    }

    // 检测紫色入口
    std::tie(centers, rects) = VisionProcessor::detect_colored_regions(
        "purple_entrance", image,
        cv::Scalar(190, 95, 210), cv::Scalar(200, 105, 220),
        0, 0, 100, 100);
    if (!centers.empty()) {
        area.purple_in_grid_pos.x = centers[0].x / (image.cols / game_params::grid_num_v);
        area.purple_in_grid_pos.y = centers[0].y / (image.rows / game_params::grid_num_h);
    }

    // 检测紫色出口
    std::tie(centers, rects) = VisionProcessor::detect_colored_regions(
        "purple_exit", image,
        cv::Scalar(190, 95, 210), cv::Scalar(200, 105, 220),
        0, 0, 100, 100);
    if (!centers.empty()) {
        area.purple_out_grid_pos.x = centers[0].x / (image.cols / game_params::grid_num_v);
        area.purple_out_grid_pos.y = centers[0].y / (image.rows / game_params::grid_num_h);
    }
    
    return area;
}

info_interfaces::msg::Robot MapGenerator::detect_robots(
    const cv::Mat& image, 
    int hp_block_width)
{
    info_interfaces::msg::Robot robot;
    std::vector<cv::Point> centers;
    std::vector<cv::Rect> rects;

    // 检测己方机器人
    std::tie(centers, rects) = VisionProcessor::detect_colored_regions(
        "our_robot", image,
        cv::Scalar(85, 165, 235), cv::Scalar(93, 175, 245),
        0, 0, 100, 100);
    if (!centers.empty()) {
        robot.our_robot_real_pos.x = centers[0].x;
        robot.our_robot_real_pos.y = centers[0].y;
        robot.our_robot_grid_pos.x = centers[0].x / (image.cols / game_params::grid_num_v);
        robot.our_robot_grid_pos.y = centers[0].y / (image.rows / game_params::grid_num_h);
    }

    // 检测敌方机器人
    std::tie(centers, rects) = VisionProcessor::detect_colored_regions(
        "enemy_robot", image,
        cv::Scalar(250, 100, 100), cv::Scalar(255, 108, 108),
        0, 0, 50, 50);
    for (const auto& center : centers) {
        if (center.x > 1900) continue;
        info_interfaces::msg::Point enemy_pos;
        enemy_pos.x = center.x;
        enemy_pos.y = center.y;
        robot.enemy_real_pos_vec.push_back(enemy_pos);
        
        enemy_pos.x = center.x / (image.cols / game_params::grid_num_v);
        enemy_pos.y = center.y / (image.rows / game_params::grid_num_h);
        robot.enemy_grid_pos_vec.push_back(enemy_pos);
    }

    return robot;
} 