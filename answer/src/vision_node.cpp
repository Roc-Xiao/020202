#include "vision_node.hpp"
#include "game_config.hpp"

vision::Node::Node(const std::string& name) : rclcpp::Node(name) {
    m_initialized = false;
    m_hp_block_width = 0;
    
    m_map_publisher = this->create_publisher<info_interfaces::msg::Map>(
        topic_name::map, 1);
    m_area_publisher = this->create_publisher<info_interfaces::msg::Area>(
        topic_name::area, 1);
    m_robot_publisher = this->create_publisher<info_interfaces::msg::Robot>(
        topic_name::robot, 1);
        
    m_img_subscription = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", 10, 
        std::bind(&vision::Node::process_image, this, std::placeholders::_1));
}

void vision::Node::process_image(const sensor_msgs::msg::Image::SharedPtr ros_img) {
    cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(ros_img, ros_img->encoding);
    
    if (!m_initialized) {
        RCLCPP_INFO(get_logger(), "Initializing map info");
        
        // 生成地图
        cv::Mat binary;
        cv::inRange(cv_img->image, 
            cv::Scalar(55, 55, 55), cv::Scalar(60, 60, 60), 
            binary);
        
        auto map = MapGenerator::generate_map(
            binary, 
            cv_img->image.cols / game_params::grid_num_v,
            cv_img->image.rows / game_params::grid_num_h
        );
        m_map_publisher->publish(map);
        
        // 检测游戏区域
        auto area = MapGenerator::detect_game_areas(cv_img->image);
        m_area_publisher->publish(area);
        
        // 获取血条宽度
        std::vector<cv::Point> centers;
        std::vector<cv::Rect> rects;
        std::tie(centers, rects) = VisionProcessor::detect_colored_regions(
            "hp_bar", cv_img->image,
            cv::Scalar(125, 125, 125), cv::Scalar(137, 137, 137),
            0, 0, 1000, 1000);
            
        if (!rects.empty()) {
            m_hp_block_width = rects[0].width;
            m_initialized = true;
        }
    }
    else {
        // 检测机器人
        auto robot = MapGenerator::detect_robots(cv_img->image, m_hp_block_width);
        m_robot_publisher->publish(robot);
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<vision::Node>("vision_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 