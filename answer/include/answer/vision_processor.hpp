#ifndef _VISION_PROCESSOR_HPP_
#define _VISION_PROCESSOR_HPP_

#include <opencv2/opencv.hpp>
#include <vector>
#include <utility>
#include <string>

class VisionProcessor {
public:
    static std::pair<std::vector<cv::Point>, std::vector<cv::Rect>> detect_colored_regions(
        const std::string& area_name,
        cv::InputArray image,
        cv::Scalar lower,
        cv::Scalar upper,
        int min_width,
        int min_height,
        int max_width,
        int max_height
    );
};

#endif // !_VISION_PROCESSOR_HPP_