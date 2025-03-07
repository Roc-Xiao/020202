#include "path_planner.hpp"
#include <queue>
#include <cmath>

struct CompareNode {
    bool operator()(const PathPlanner::Node* a, const PathPlanner::Node* b) {
        return a->f_cost() > b->f_cost();
    }
};

std::vector<info_interfaces::msg::Point> PathPlanner::plan_path(
    const info_interfaces::msg::Map& map,
    const info_interfaces::msg::Point& start,
    const info_interfaces::msg::Point& goal)
{
    // 初始化开放列表和关闭列表
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> open_list;
    std::vector<std::vector<bool>> closed_list(map.row, std::vector<bool>(map.col, false));
    std::vector<std::vector<Node*>> nodes(map.row, std::vector<Node*>(map.col, nullptr));

    // 创建起点节点，将浮点数转换为整数
    Node* start_node = new Node(static_cast<int>(start.x), static_cast<int>(start.y));
    start_node->h_cost = calculate_heuristic(
        static_cast<int>(start.x), 
        static_cast<int>(start.y), 
        static_cast<int>(goal.x), 
        static_cast<int>(goal.y));
    open_list.push(start_node);
    nodes[static_cast<int>(start.y)][static_cast<int>(start.x)] = start_node;

    // 方向数组（8个方向）
    const int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};

    // A*搜索
    while (!open_list.empty()) {
        Node* current = open_list.top();
        open_list.pop();

        // 到达目标
        if (current->x == static_cast<int>(goal.x) && current->y == static_cast<int>(goal.y)) {
            return reconstruct_path(current);
        }

        closed_list[current->y][current->x] = true;

        // 检查8个方向的相邻节点
        for (int i = 0; i < 8; i++) {
            int new_x = current->x + dx[i];
            int new_y = current->y + dy[i];

            // 检查边界
            if (new_x < 0 || new_x >= map.col || new_y < 0 || new_y >= map.row) {
                continue;
            }

            // 检查障碍物和已访问节点
            if (map.mat[new_y * map.col + new_x] == 0 || closed_list[new_y][new_x]) {
                continue;
            }

            // 计算新的g_cost
            double new_g_cost = current->g_cost + 
                ((dx[i] != 0 && dy[i] != 0) ? 1.414 : 1.0);

            Node* neighbor = nodes[new_y][new_x];
            if (neighbor == nullptr) {
                neighbor = new Node(new_x, new_y);
                nodes[new_y][new_x] = neighbor;
            } else if (new_g_cost >= neighbor->g_cost) {
                continue;
            }

            neighbor->parent = current;
            neighbor->g_cost = new_g_cost;
            neighbor->h_cost = calculate_heuristic(new_x, new_y, static_cast<int>(goal.x), static_cast<int>(goal.y));
            open_list.push(neighbor);
        }
    }

    // 没有找到路径
    return std::vector<info_interfaces::msg::Point>();
}

double PathPlanner::calculate_heuristic(int x1, int y1, int x2, int y2)
{
    // 使用欧几里得距离作为启发函数
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

std::vector<info_interfaces::msg::Point> PathPlanner::reconstruct_path(Node* goal_node)
{
    std::vector<info_interfaces::msg::Point> path;
    Node* current = goal_node;

    while (current != nullptr) {
        info_interfaces::msg::Point point;
        point.x = current->x;
        point.y = current->y;
        path.push_back(point);
        current = current->parent;
    }

    // 反转路径（从起点到终点）
    std::reverse(path.begin(), path.end());
    
    // 路径平滑处理
    smooth_path(path);
    
    return path;
}

void PathPlanner::smooth_path(std::vector<info_interfaces::msg::Point>& path)
{
    if (path.size() <= 2) return;

    std::vector<info_interfaces::msg::Point> smoothed_path;
    smoothed_path.push_back(path[0]);

    for (size_t i = 1; i < path.size() - 1; i++) {
        // 使用三点平滑
        info_interfaces::msg::Point prev = path[i - 1];
        info_interfaces::msg::Point curr = path[i];
        info_interfaces::msg::Point next = path[i + 1];

        // 计算平滑后的点
        info_interfaces::msg::Point smoothed;
        smoothed.x = (prev.x + 2 * curr.x + next.x) / 4;
        smoothed.y = (prev.y + 2 * curr.y + next.y) / 4;

        smoothed_path.push_back(smoothed);
    }

    smoothed_path.push_back(path.back());
    path = smoothed_path;
}

std::vector<std::pair<int, int>> PathPlanner::find_path(
    const info_interfaces::msg::Map::SharedPtr map,
    int src_x, int src_y,
    int dst_x, int dst_y)
{
    // 创建临时的 Point 对象
    info_interfaces::msg::Point start, goal;
    start.x = src_x;
    start.y = src_y;
    goal.x = dst_x;
    goal.y = dst_y;

    // 创建 PathPlanner 实例并调用 plan_path
    PathPlanner planner;
    std::vector<info_interfaces::msg::Point> point_path = 
        planner.plan_path(*map, start, goal);

    // 将 Point 路径转换为 pair<int, int> 路径
    std::vector<std::pair<int, int>> path;
    for (const auto& point : point_path) {
        path.emplace_back(
            static_cast<int>(point.x),
            static_cast<int>(point.y)
        );
    }

    return path;
} 