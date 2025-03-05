#include "vision_processor.hpp"

std::pair<std::vector<cv::Point>, std::vector<cv::Rect>> VisionProcessor::detect_colored_regions(
    const std::string& area_name,
    cv::InputArray image,
    cv::Scalar lower,
    cv::Scalar upper,
    int min_width,
    int min_height,
    int max_width,
    int max_height)
{
    cv::Mat binary;
    cv::inRange(image, lower, upper, binary);
    
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::erode(binary, binary, kernel);
    cv::dilate(binary, binary, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    
    std::vector<cv::Point> point_vec;
    std::vector<cv::Rect> rect_vec;

    for (const auto& contour : contours) {
        cv::Moments m = cv::moments(contour, false);
        cv::Rect rect = cv::boundingRect(contour);

        if (rect.width < min_width || rect.height < min_height || 
            rect.width > max_width || rect.height > max_height) {
            continue;
        }

        rect_vec.push_back(rect);
        
        if (m.m00 != 0) {
            int x = static_cast<int>(m.m10 / m.m00);
            int y = static_cast<int>(m.m01 / m.m00);
            point_vec.push_back(cv::Point(x, y));
        }
    }

    return {point_vec, rect_vec};
} 