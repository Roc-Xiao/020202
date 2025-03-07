#ifndef _GAME_CONFIG_HPP_
#define _GAME_CONFIG_HPP_

namespace game_params {
    const int grid_num_v = 256; // 水平方向网格数量
    const int grid_num_h = 128; // 垂直方向网格数量
    const int attack_distance = 80; // 判断攻击距离
    const double speed_scale = 1.0 / 32.0; // 速度
    const double danger_hp = 0.8; // 回城生命值
    const int danger_bullet_num = 5; // 回城子弹数
}

#endif // _GAME_CONFIG_HPP_