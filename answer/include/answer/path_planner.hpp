#pragma once

#include <vector>
#include <utility>
#include "info_interfaces/msg/map.hpp"
#include "info_interfaces/msg/point.hpp"

class PathPlanner {
public:
    struct Node {
        int x, y;
        double g_cost;  // 从起点到当前点的代价
        double h_cost;  // 从当前点到终点的预估代价
        Node* parent;
        
        Node(int x, int y) : x(x), y(y), g_cost(0), h_cost(0), parent(nullptr) {}
        double f_cost() const { return g_cost + h_cost; }
    };

    static std::vector<std::pair<int, int>> find_path(
        const info_interfaces::msg::Map::SharedPtr map,
        int src_x, int src_y,
        int dst_x, int dst_y
    );

    std::vector<info_interfaces::msg::Point> plan_path(
        const info_interfaces::msg::Map& map,
        const info_interfaces::msg::Point& start,
        const info_interfaces::msg::Point& goal);

private:
    static int manhattan_distance(int x1, int y1, int x2, int y2);
    double calculate_heuristic(int x1, int y1, int x2, int y2);
    std::vector<info_interfaces::msg::Point> reconstruct_path(Node* goal_node);
    void smooth_path(std::vector<info_interfaces::msg::Point>& path);
};