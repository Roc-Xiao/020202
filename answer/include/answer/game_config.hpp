 #ifndef _GAME_CONFIG_HPP_
#define _GAME_CONFIG_HPP_

namespace game_params {
    const int grid_num_v = 256; // 水平方向网格数量
    const int grid_num_h = 128; // 垂直方向网格数量
    const int attack_distance = 80; // 攻击距离,敌人在攻击距离内再攻击
    const double danger_hp = 0.9; // 生命值低于这个值就要补充
    const int danger_bullet_num = 5; // 子弹低于这个值就要补充
    const double speed_scale = 1.0 / 32.0; // 速度缩放比例
}

#endif // ^^ !_GAME_CONFIG_HPP_