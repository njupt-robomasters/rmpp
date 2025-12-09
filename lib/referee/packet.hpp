#pragma once

#include <cstdint>

// 0x0001，比赛状态数据（1Hz）
// 服务器→全体机器人
struct __attribute__((packed)) game_status_t {
    // 比赛类型
    // 1：RoboMaster 机甲大师超级对抗赛
    // 2：RoboMaster 机甲大师高校单项赛
    // 3：ICRA RoboMaster 高校人工智能挑战赛
    // 4：RoboMaster 机甲大师高校联盟赛3V3对抗
    // 5：RoboMaster 机甲大师高校联盟赛步兵对抗
    uint8_t game_type : 4;

    // 当前比赛阶段
    // 0：未开始比赛
    // 1：准备阶段
    // 2：十五秒裁判系统自检阶段
    // 3：五秒倒计时
    // 4：比赛中
    // 5：比赛结算中
    uint8_t game_progress : 4;

    uint16_t stage_remain_time; // 当前阶段剩余时间，单位：秒
    uint64_t SyncTimeStamp;     // UNIX时间，当机器人正确连接到裁判系统的NTP服务器后生效
};

// 0x0002，比赛结果数据（比赛结束触发发送）
// 服务器→全体机器人
struct __attribute__((packed)) game_result_t {
    // 0：平局
    // 1：红方胜利
    // 2：蓝方胜利
    uint8_t winner;
};

// 0x0003，机器人血量数据（3Hz）
// 服务器→全体机器人
// 注：若机器人未上场或者被罚下，则血量为0
struct __attribute__((packed)) game_robot_HP_t {
    uint16_t ally_1_robot_HP; // 己方1号英雄机器人血量
    uint16_t ally_2_robot_HP; // 己方2号工程机器人血量
    uint16_t ally_3_robot_HP; // 己方3步兵机器人血量
    uint16_t ally_4_robot_HP; // 己方4号步兵机器人血量
    uint16_t reserved;        // 保留位
    uint16_t ally_7_robot_HP; // 己方7号哨兵机器人血量
    uint16_t ally_outpost_HP; // 己方前哨站血量
    uint16_t ally_base_HP;    // 己方基地血量
};

// 0x0101，场地事件数据（1Hz）
// 服务器→己方全体机器人
// 注：
// 0：未占领/未激活
// 1：已占领/已激活
struct __attribute__((packed)) event_data_t {
    struct __attribute__((packed)) {
        // bit 0
        // 己方与资源区不重叠的补给区占领状态，1为已占领
        uint8_t supply_non_overlap : 1;
        // bit 1
        // 己方与资源区重叠的补给区占领状态，1为已占领
        uint8_t supply_overlap : 1;
        // bit 2
        // 己方补给区的占领状态，1为已占领（仅RMUL适用）
        uint8_t supply_status : 1;

        // bit 3-4
        // 己方小能量机关的激活状态
        // 0为未激活
        // 1为已激活
        // 2为正在激活
        uint8_t small_energy_status : 2;

        // bit 5-6
        // 己方大能量机关的激活状态
        // 0为未激活
        // 1为已激活
        // 2为正在激活
        uint8_t large_energy_status : 2;

        // bit 7-8
        // 方中央高地的占领状态，1为被己方占领，2为被对方占领
        uint8_t central_highland : 2;

        // bit 9-10
        // 己方梯形高地的占领状态，1为已占领
        uint8_t trapezoid_highland : 2;

        // bit 11-19
        // 对方飞镖最后一次击中己方前哨站或基地的时间（0-420，开局默认为0）
        uint16_t last_hit_time : 9;

        // bit 20-22
        // 对方飞镖最后一次击中己方前哨站或基地的具体目标
        // 开局默认为0
        // 1为击中前哨站
        // 2为击中基地固定目标
        // 3为击中基地随机固定目标
        // 4为击中基地随机移动目标
        // 5为击中基地末端移动目标
        uint8_t last_hit_target : 3;

        // bit 23-24
        // 中心增益点的占领状态（仅RMUL适用）
        // 0为未被占领
        // 1为被己方占领
        // 2为被对方占领
        // 3为被双方占领
        uint8_t center_buff : 2;

        // bit 25-26
        // 己方堡垒增益点的占领状态
        // 0为未被占领
        // 1为被己方占领
        // 2为被对方占领
        // 3为被双方占领
        uint8_t fortress_buff : 2;

        // bit 27-28
        // 己方前哨站增益点的占领状态
        // 0为未被占领
        // 1为被己方占领
        // 2为被对方占领
        uint8_t outpost_buff : 2;

        // bit 29
        // 己方基地增益点的占领状态，1为已占领
        uint8_t base_buff : 1;

        // bit 30-31，保留位
        uint8_t reserved : 2;
    } event_data;
};

// 0x0104，裁判警告数据（1Hz，己方判罚/判负时触发发送）
// 服务器→被判罚方全体机器人
struct __attribute__((packed)) referee_warning_t {
    // 己方最后一次受到判罚的等级
    // 1：双方黄牌
    // 2：黄牌
    // 3：红牌
    // 4：判负
    uint8_t level;

    uint8_t offending_robot_id; // 己方最后一次受到判罚的违规机器人ID（如红1机器人ID为1，蓝1机器人ID为101），判负和双方黄牌时，该值为0
    uint8_t count;              // 己方最后一次受到判罚的违规机器人对应判罚等级的违规次数（开局默认为0）
};

// 0x0105，飞镖发射相关数据（1Hz）
// 服务器→己方全体机器人
struct __attribute__((packed)) dart_info_t {
    uint8_t dart_remaining_time; // 己方飞镖发射剩余时间，单位：秒

    struct __attribute__((packed)) {
        // bit 0-2
        // 最近一次己方飞镖击中的目标
        // 开局默认为0
        // 1为击中前哨站
        // 2为击中基地固定目标
        // 3为击中基地随机固定目标
        // 4为击中基地随机移动目标
        // 5为击中基地末端移动目标
        uint8_t last_hit_target : 3;

        // bit 3-5
        // 对方最近被击中的目标累计被击中计次数
        // 开局默认为0，至多为4
        uint8_t hit_count : 3;

        // bit 6-7
        // 飞镖此时选定的击打目标
        // 开局默认或未选定/选定前哨站时为0
        // 选中基地固定目标为1
        // 选中基地随机固定目标为2
        // 选中基地随机移动目标为3
        // 选中基地末端移动目标为4
        uint8_t selected_target : 2;

        // bit 8-15
        // 保留位
        uint8_t reserved : 8;
    } dart_info;
};

// 0x0201，机器人性能体系数据（10Hz）
// 主控模块→对应机器人
struct __attribute__((packed)) robot_status_t {
    uint8_t robot_id;                            // 本机器人ID
    uint8_t robot_level;                         // 机器人等级
    uint16_t current_HP;                         // 机器人当前血量
    uint16_t maximum_HP;                         // 机器人血量上限
    uint16_t shooter_barrel_cooling_value;       // 机器人射击热量每秒冷却值
    uint16_t shooter_barrel_heat_limit;          // 机器人射击热量上限
    uint16_t chassis_power_limit;                // 机器人底盘功率上限
    uint8_t power_management_gimbal_output : 1;  // gimbal口输出，0为无输出，1为24V输出
    uint8_t power_management_chassis_output : 1; // chassis口输出，0为无输出，1为24V输出
    uint8_t power_management_shooter_output : 1; // shooter口输出，0为无输出，1为24V 输出
};

// 0x0202，实时底盘缓冲能量和射击热量数据（10Hz）
// 主控模块→对应机器人
struct __attribute__((packed)) power_heat_data_t {
    uint16_t reserved_1;                 // 保留位
    uint16_t reserved_2;                 // 保留位
    float reserved_3;                    // 保留位
    uint16_t buffer_energy;              // 缓冲能量（单位：J）
    uint16_t shooter_17mm_1_barrel_heat; // 第1个17mm发射机构的射击热量
    uint16_t shooter_42mm_barrel_heat;   // 42mm发射机构的射击热量
};

// 0x0203，机器人位置数据
// 主控模块→对应机器人（1Hz）
struct __attribute__((packed)) robot_pos_t {
    float x;     // 本机器人位置x坐标，单位：m
    float y;     // 本机器人位置y坐标，单位：m
    float angle; // 本机器人测速模块的朝向，单位：度。正北为0度
};

// 0x0204，机器人增益和底盘能量数据（3Hz）
// 服务器→对应机器人
struct __attribute__((packed)) buff_t {
    uint8_t recovery_buff;      // 机器人回血增益（百分比，值为10表示每秒恢复血量上限的10%）
    uint16_t cooling_buff;      // 机器人射击热量冷却增益具体值（直接值，值为x表示热量冷却增加x/s）
    uint8_t defence_buff;       // 机器人防御增益（百分比，值为50 表示50%防御增益）
    uint8_t vulnerability_buff; // 机器人负防御增益（百分比，值为30 表示-30%防御增益）
    uint16_t attack_buff;       // 机器人攻击增益（百分比，值为50 表示50%攻击增益）

    struct {
        uint8_t remaining_energy_125 : 1; // 在剩余能量≥125%时为1，其余情况为0
        uint8_t remaining_energy_100 : 1; // 在剩余能量≥100%时为1，其余情况为0
        uint8_t remaining_energy_50 : 1;  // 在剩余能量≥50%时为1，其余情况为0
        uint8_t remaining_energy_30 : 1;  // 在剩余能量≥30%时为1，其余情况为0
        uint8_t remaining_energy_15 : 1;  // 在剩余能量≥15%时为1，其余情况为0
        uint8_t remaining_energy_5 : 1;   // 在剩余能量≥5%时为1，其余情况为0
        uint8_t remaining_energy_1 : 1;   // 在剩余能量≥1%时为1，其余情况为0
    } remaining_energy;                   // 机器人剩余能量值反馈，以16进制标识机器人剩余能量值比例，仅在机器人剩余能量小于50%时反馈，其余默认反馈0x80。机器人初始能量视为100%
};

// 0x0206，伤害状态数据（伤害发生后发送）
// 主控模块→对应机器人
// 0x0206的受伤害情况为机器人裁判系统本地判定，即时发送，但实际是否受到对应伤害受规则条例影响，请以服务器最终判定为准
struct __attribute__((packed)) hurt_data_t {
    // 当扣血原因为装甲模块被弹丸攻击、受撞击或离线时，该4bit组成的数值为装甲模块或测速模块的ID编号
    // 当其他原因导致扣血时，该数值为0
    uint8_t armor_id : 4;

    // 血量变化类型
    // 0：装甲模块被弹丸攻击导致扣血
    // 1：装甲模块或超级电容管理模块离线导致扣血
    // 5：装甲模块受到撞击导致扣血
    uint8_t HP_deduction_reason : 4;
};

// 0x0207，实时射击数据（弹丸发射后发送）
// 主控模块→对应机器人
struct __attribute__((packed)) shoot_data_t {
    // 弹丸类型
    // bit 1：17mm弹丸
    // bit 2：42mm弹丸
    uint8_t bullet_type;

    // 发射机构ID
    // 1：17mm发射机构
    // 2：保留位
    // 3：42mm发射机构
    uint8_t shooter_number;

    uint8_t launching_frequency; // 弹丸射速（单位：Hz）

    float initial_speed; // 弹丸初速度（单位：m/s）
};

// 0x0208，允许发弹量（10Hz）
// 服务器→己方英雄、步兵、哨兵、空中机器人
struct __attribute__((packed)) projectile_allowance_t {
    uint16_t projectile_allowance_17mm;     // 机器人自身拥有的17mm弹丸允许发弹量
    uint16_t projectile_allowance_42mm;     // 42mm弹丸允许发弹量
    uint16_t remaining_gold_coin;           // 剩余金币数量
    uint16_t projectile_allowance_fortress; // 堡垒增益点提供的储备17mm 弹丸允许发弹量；该值与机器人是否实际占领堡垒无关
};

// 0x0209，机器人RFID模块状态（3Hz）
// 服务器→己方装有RFID模块的机器人
// 注：
// 1. bit位值为1/0的含义：是否已检测到该增益点RFID卡
// 2. 所有RFID卡仅在赛内生效。在赛外，即使检测到对应的RFID卡，对应值也为0
struct __attribute__((packed)) rfid_status_t {
    struct {
        uint8_t base_buff : 1;               // bit 0: 己方基地增益点
        uint8_t central_highland_self : 1;   // bit 1: 己方中央高地增益点
        uint8_t central_highland_oppo : 1;   // bit 2: 对方中央高地增益点
        uint8_t trapezoid_highland_self : 1; // bit 3: 己方梯形高地增益点
        uint8_t trapezoid_highland_oppo : 1; // bit 4: 对方梯形高地增益点

        // 地形跨越增益点（飞坡）
        uint8_t ramp_self_front : 1; // bit 5: 己方地形跨越增益点（飞坡）（靠近己方一侧飞坡前）
        uint8_t ramp_self_back : 1;  // bit 6: 己方地形跨越增益点（飞坡）（靠近己方一侧飞坡后）
        uint8_t ramp_oppo_front : 1; // bit 7: 对方地形跨越增益点（飞坡）（靠近对方一侧飞坡前）
        uint8_t ramp_oppo_back : 1;  // bit 8: 对方地形跨越增益点（飞坡）（靠近对方一侧飞坡后）

        // 地形跨越增益点（中央高地）
        uint8_t central_under_self : 1; // bit 9: 己方地形跨越增益点（中央高地下方）
        uint8_t central_over_self : 1;  // bit 10: 己方地形跨越增益点（中央高地上方）
        uint8_t central_under_oppo : 1; // bit 11: 对方地形跨越增益点（中央高地下方）
        uint8_t central_over_oppo : 1;  // bit 12: 对方地形跨越增益点（中央高地上方）

        // 地形跨越增益点（公路）
        uint8_t road_under_self : 1; // bit 13: 己方地形跨越增益点（公路下方）
        uint8_t road_over_self : 1;  // bit 14: 己方地形跨越增益点（公路上方）
        uint8_t road_under_oppo : 1; // bit 15: 对方地形跨越增益点（公路下方）
        uint8_t road_over_oppo : 1;  // bit 16: 对方地形跨越增益点（公路上方）

        uint8_t fortress_self : 1;           // bit 17: 己方堡垒增益点
        uint8_t outpost_self : 1;            // bit 18: 己方前哨站增益点
        uint8_t supply_non_overlap_self : 1; // bit 19: 己方与资源区不重叠的补给区/RMUL补给区
        uint8_t supply_overlap_self : 1;     // bit 20: 己方与资源区重叠的补给区

        uint8_t assembly_self : 1; // bit 21: 己方装配增益点
        uint8_t assembly_oppo : 1; // bit 22: 对方装配增益点
        uint8_t center_buff : 1;   // bit 23: 中心增益点（仅RMUL适用）
        uint8_t fortress_oppo : 1; // bit 24: 对方堡垒增益点
        uint8_t outpost_oppo : 1;  // bit 25: 对方前哨站增益点

        // 地形跨越增益点（隧道）- 第一部分
        uint8_t tunnel_road_under_self : 1;     // bit 26: 己方地形跨越增益点（隧道）（靠近己方一侧公路区下方）
        uint8_t tunnel_road_over_self : 1;      // bit 27: 己方地形跨越增益点（隧道）（靠近己方一侧公路区上方）
        uint8_t tunnel_trapezoid_low_self : 1;  // bit 28: 己方地形跨越增益点（隧道）（靠近己方梯形高地较低处）
        uint8_t tunnel_trapezoid_high_self : 1; // bit 29: 己方地形跨越增益点（隧道）（靠近己方梯形高地较高处）
        uint8_t tunnel_road_under_oppo : 1;     // bit 30: 对方地形跨越增益点（隧道）（靠近对方一侧公路区下方）
        uint8_t tunnel_road_over_oppo : 1;      // bit 31: 对方地形跨越增益点（隧道）（靠近对方一侧公路区上方）
    } rfid_status;

    struct {
        // 地形跨越增益点（隧道）- 第二部分
        uint8_t tunnel_trapezoid_low_oppo : 1;  // 对方地形跨越增益点（隧道）（靠近对方梯形高地较低处）
        uint8_t tunnel_trapezoid_high_oppo : 1; // 对方地形跨越增益点（隧道）（靠近对方梯形高地较高处）
    } rfid_status_2;
};

// 0x020A，飞镖选手端指令数据（3Hz）
// 服务器→己方飞镖机器人
struct __attribute__((packed)) dart_client_cmd_t {
    // 当前飞镖发射站的状态
    // 1：关闭
    // 2：正在开启或者关闭中
    // 0：已经开启
    uint8_t dart_launch_opening_status;

    uint8_t reserved;                // 保留位
    uint16_t target_change_time;     // 切换击打目标时的比赛剩余时间，单位：秒，无/未切换动作，默认为0
    uint16_t latest_launch_cmd_time; // 最后一次操作手确定发射指令时的比赛剩余时间，单位：秒，初始值为0
};

// 0x020B，地面机器人位置数据（1Hz）
// 服务器→己方哨兵机器人
// 场地围挡在红方补给站附近的交点为坐标原点，沿场地长边向蓝方为X轴正方向，沿场地短边向红方停机坪为Y轴正方向
struct __attribute__((packed)) ground_robot_position_t {
    float hero_x;       // 己方英雄机器人位置x轴坐标，单位：m
    float hero_y;       // 己方英雄机器人位置y轴坐标，单位：m
    float engineer_x;   // 己方工程机器人位置x轴坐标，单位：m
    float engineer_y;   // 己方工程机器人位置y轴坐标，单位：m
    float standard_3_x; // 己方3号步兵机器人位置x轴坐标，单位：m
    float standard_3_y; // 己方3号步兵机器人位置y轴坐标，单位：m
    float standard_4_x; // 己方4号步兵机器人位置x轴坐标，单位：m
    float standard_4_y; // 己方4号步兵机器人位置y 轴坐标，单位：m
    float reserved_1;   // 保留位
    float reserved_2;   // 保留位
};

// 0x020C，雷达标记进度数据（1Hz）
// 服务器→己方雷达机器人
struct __attribute__((packed)) radar_mark_data_t {
    struct {
        uint8_t mark_1 : 1;         // bit 0：对方1号英雄机器人易伤情况
        uint8_t mark_2 : 1;         // bit 1：对方2号工程机器人易伤情况
        uint8_t mark_3 : 1;         // bit 2：对方3号步兵机器人易伤情况
        uint8_t mark_4 : 1;         // bit 3：对方4号步兵机器人易伤情况
        uint8_t mark_7 : 1;         // bit 4：对方哨兵机器人易伤情况
        uint8_t special_mark_1 : 1; // bit 5：己方1号英雄机器人特殊标识情况
        uint8_t special_mark_2 : 1; // bit 6：己方2号工程机器人特殊标识情况
        uint8_t special_mark_3 : 1; // bit 7：己方3号步兵机器人特殊标识情况
        uint8_t special_mark_4 : 1; // bit 8：己方4号步兵机器人特殊标识情况
        uint8_t special_mark_7 : 1; // bit 9：己方哨兵机器人特殊标识情况
        uint8_t reserved : 6;       // bit 10-15: 保留位
    } mark_progress;
};

// 0x020D，哨兵自主决策信息同步（1Hz）
// 服务器→己方哨兵机器人
struct __attribute__((packed)) sentry_info_t {
    struct {
        // bit 0-10
        // 除远程兑换外，哨兵机器人成功兑换的允许发弹量
        // 开局为0
        // 在哨兵机器人成功兑换一定允许发弹量后，该值将变为哨兵机器人成功兑换的允许发弹量值
        uint16_t ammo_exchanged : 11;

        // bit 11-14
        // 哨兵机器人成功远程兑换允许发弹量的次数
        // 开局为0
        // 在哨兵机器人成功远程兑换允许发弹量后，该值将变为哨兵机器人成功远程兑换允许发弹量的次数
        uint8_t remote_ammo_exchange_count : 4;

        // bit 15-18
        // 哨兵机器人成功远程兑换血量的次数
        // 开局为0
        // 在哨兵机器人成功远程兑换血量后，该值将变为哨兵机器人成功远程兑换血量的次数
        uint8_t remote_health_exchange_count : 4;

        // bit 19
        // 哨兵机器人当前是否可以确认免费复活
        // 可以确认免费复活时值为1，否则为0
        uint8_t can_free_revive : 1;

        // bit 20
        // 哨兵机器人当前是否可以兑换立即复活
        // 可以兑换立即复活时值为1，否则为0
        uint8_t can_instant_revive : 1;

        // bit 21-30
        // 哨兵机器人当前若兑换立即复活需要花费的金币数
        uint16_t instant_revive_cost : 10;

        // bit 31
        // 保留位
        uint8_t reserved : 1;
    } sentry_info;

    struct {
        // bit 0-11
        // 保留位
        uint16_t reserved_bits_0_11 : 12;

        // bit 12-13
        // 哨兵当前姿态
        // 1为进攻姿态
        // 2为防御姿态
        // 3为移动姿态
        uint8_t sentinel_posture : 2;

        // bit 14
        // 己方能量机关是否能够进入正在激活状态，1 为当前可激活
        uint8_t energy_activation_ready : 1;

        // bit 15
        // 保留位
        uint8_t reserved_bit_15 : 1;
    } sentry_info_2;
};

// 0x020E，雷达自主决策信息同步（1Hz）
// 服务器→己方雷达机器人
struct __attribute__((packed)) radar_info_t {
    struct {
        // bit 0-1
        // 雷达是否拥有触发双倍易伤的机会
        // 开局为0，数值为雷达拥有触发双倍易伤的机会，至多为2
        uint8_t radar_double_vulnerability_chance : 2;

        // bit 2
        // 对方是否正在被触发双倍易伤
        // 0：对方未被触发双倍易伤
        // 1：对方正在被触发双倍易伤
        uint8_t enemy_double_vulnerability_active : 1;

        // bit 3-4
        // 己方加密等级（即对方干扰波难度等级）
        // 开局为1，最高为3
        uint8_t encryption_level : 2;

        // bit 5
        // 当前是否可以修改密钥，1为可修改
        uint8_t can_change_key : 1;

        // bit 6-7，保留位
        uint8_t reserved : 2;
    } radar_info;
};

// 0x0301，机器人交互数据（频率上限为30Hz）
// 注：
// 机器人交互数据通过常规链路发送，其数据段包含一个统一的数据段头结构。数据段头结构包括内容ID、发送者和接收者的ID、内容数据段
// 机器人交互数据包的总长不超过127个字节，减去frame_header、cmd_id和frame_tail的9个字节以及数据段头结构的6个字节，故机器人交互数据的内容数据段最大为112个字节
// 每1000毫秒，英雄、工程、步兵、空中机器人、飞镖能够接收数据的上限为3720字节，雷达和哨兵机器人能够接收数据的上限为5120字节
// 由于存在多个内容ID，但整个cmd_id上行频率最大为30Hz，请合理安排带宽
struct __attribute__((packed)) robot_interaction_data_t {
    // 子内容ID，需为开放的子内容ID
    // 0x0200~0x02FF，机器人之间通信
    // 0x0100，选手端删除图层
    // 0x0101，选手端绘制一个图形
    // 0x0102，选手端绘制两个图形
    // 0x0103，选手端绘制五个图形
    // 0x0104，选手端绘制七个图形
    // 0x0110，选手端绘制字符图形
    // 0x0120，哨兵自主决策指令
    // 0x0121，雷达自主决策指令
    uint16_t data_cmd_id;

    uint16_t sender_id; // 发送者ID，需与自身ID 匹配，ID 编号详见附录

    // 接收者ID
    // 仅限己方通信
    // 需为规则允许的多机通讯接收者
    // 若接收者为选手端，则仅可发送至发送者对应的选手端
    // ID 编号详见附录
    uint16_t receiver_id;

    uint8_t user_data[112]; // 内容数据段，最大112字节
};

// 0x0301的字内容：0x0100，选手端删除图层
struct __attribute__((packed)) interaction_layer_delete_t {
    // 删除操作
    // 0：空操作
    // 1：删除图层
    // 2：删除所有
    uint8_t delete_type;

    uint8_t layer; // 图层数：0~9
};

// 0x0301的字内容：0x0101，选手端绘制一个图形
struct __attribute__((packed)) interaction_figure_t {
    // 图形名
    // 在图形删除、修改等操作中，作为索引
    uint8_t figure_name[3];

    // todo
    uint32_t operate_tpye : 3;
    uint32_t figure_tpye : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t details_a : 9;
    uint32_t details_b : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t details_c : 10;
    uint32_t details_d : 11;
    uint32_t details_e : 11;
};

// 0x0301的字内容：0x0102，选手端绘制两个图形
struct __attribute__((packed)) interaction_figure_2_t {
    interaction_figure_t interaction_figure[2];
};

// 0x0301的字内容：0x0103，选手端绘制五个图形
struct __attribute__((packed)) interaction_figure_3_t {
    interaction_figure_t interaction_figure[5];
};

// 0x0301的字内容：0x0104，选手端绘制七个图形
struct __attribute__((packed)) interaction_figure_4_t {
    interaction_figure_t interaction_figure[7];
};

// 0x0301的字内容：0x0110，选手端绘制字符图形
struct __attribute__((packed)) ext_client_custom_character_t {
    uint8_t data[30]; // graphic_data_struct暂时用data代替
};

// 0x0301的字内容：0x0120，哨兵自主决策指令
struct __attribute__((packed)) sentry_cmd_t {
    struct {
        // bit 0
        // 哨兵机器人是否确认复活
        // 0 表示哨兵机器人确认不复活，即使此时哨兵的复活读条已经完成
        // 1 表示哨兵机器人确认复活，若复活读条完成将立即复活
        uint32_t confirm_revive : 1;

        // bit 1
        // 哨兵机器人是否确认兑换立即复活
        // 0表示哨兵机器人确认不兑换立即复活
        // 1表示哨兵机器人确认兑换立即复活，若此时哨兵机器人符合兑换立即复活的规则要求，则会立即消耗金币兑换立即复活
        uint32_t confirm_instant_revive : 1;

        // bit 2-12
        // 哨兵将要兑换的发弹量值
        // 开局为0，修改此值后，哨兵在补血点即可兑换允许发弹量
        // 此值的变化需要单调递增，否则视为不合法
        // 示例：此值开局仅能为0，此后哨兵可将其从0修改至X，则消耗X金币成功兑换X允许发弹量。此后哨兵可将其从X修改至X+Y，以此类推
        uint32_t ammo_to_exchange : 11;

        // bit 13-16
        // 哨兵远程兑换发弹量的请求次数，开局为0，修改此值即可请求远程兑换发弹量
        // 此值的变化需要单调递增且每次仅能增加1，否则视为不合法
        uint32_t remote_ammo_request_count : 4;
        // 示例：此值开局仅能为0，此后哨兵可将其从0修改至1，则消耗金币远程兑换允许发弹量。此后哨兵可将其从1 修改至2，以此类推

        // bit 17-20
        // 哨兵远程兑换血量的请求次数
        // 开局为0，修改此值即可请求远程兑换血量
        // 此值的变化需要单调递增且每次仅能增加1，否则视为不合法
        // 示例：此值开局仅能为0，此后哨兵可将其从0修改至1，则消耗金币远程兑换血量。此后哨兵可将其从1 修改至2，以此类推
        uint32_t remote_health_request_count : 4;

        // 在哨兵发送该子命令时，服务器将按照从相对低位到相对高位的原则依次处理这些指令，直至全部成功或不能处理为止
        // 示例：
        // 若队伍金币数为0，此时哨兵战亡，
        // “是否确认复活”的值为1，
        // “是否确认兑换立即复活”的值为1，
        // “确认兑换的允许发弹量值”为100。（假定之前哨兵未兑换过允许发弹量）
        // 由于此时队伍金币数不足以使哨兵兑换立即复活，则服务器将会忽视后续指令，等待哨兵发送的下一组指令

        // bit 21-22
        // 哨兵修改当前姿态指令
        // 1为进攻姿态
        // 2为防御姿态
        // 3 为移动姿态
        // 默认为3；修改此值即可改变哨兵姿态
        uint32_t posture_command : 2;

        // bit 23
        // 哨兵机器人是否确认使能量机关进入正在激活状态
        // 1为确认，默认为0
        uint32_t confirm_energy_activation : 1;

        // bit 24-31
        // 保留位
        uint32_t reserved : 8;
    } sentry_cmd;
};

// 0x0301的字内容：0x0121，雷达自主决策指令
struct __attribute__((packed)) radar_cmd_t {
    // 雷达是否确认触发双倍易伤
    // 开局为0，修改此值即可请求触发双倍易伤，若此时雷达拥有触发双倍易伤的机会，则可触发
    // 此值的变化需要单调递增且每次仅能增加1，否则视为不合法
    // 示例：此值开局仅能为0，此后雷达可将其从0 修改至1，若雷达拥有触发双倍易伤的机会，则触发双倍易伤。此后雷达可将其从1修改至2，以此类推
    // 若雷达请求双倍易伤时，双倍易伤正在生效，则第二次双倍易伤将在第一次双倍易伤结束后生效
    uint8_t radar_cmd;

    // 密钥更新或验证指令
    // 每个字节均为ASCII码编码的字母或数字。开局为随机值。byte1为指令类型，byte2-7为密钥值
    // 当byte1值为1时，修改此值即可更新己方加密密钥
    // 当byte1值为2时，修改此值即可将雷达破解的对方密钥传输给服务器以验证是否正确破解
    // 注意：
    // 仅开局和每次对方破解成功使得加密等级（己方干扰波难度）提高时可以修改密钥，其余时间修改无效
    // 当byte1值为2时，每次更新验证密钥后的10秒内，再次更新无效
    uint8_t password_cmd;
    uint8_t password_1;
    uint8_t password_2;
    uint8_t password_3;
    uint8_t password_4;
    uint8_t password_5;
    uint8_t password_6;
};

// 0x0302（图传链路），自定义控制器与机器人交互数据（频率上限为30Hz）
// 自定义控制器→选手端图传连接的机器人
// 或者
// 0x0309（图传链路），自定义控制器接收机器人数据（频率上限为10Hz）
// 己方机器人→对应操作手选手端连接的自定义控制器
struct __attribute__((packed)) robot_custom_data_t {
    uint8_t data[30];
};

// 0x0303，选手端小地图交互数据（选手端触发发送）
// 选手端点击→服务器→发送方选择的己方机器人
struct __attribute__((packed)) map_command_t {
    // 目标位置x 轴坐标，单位m
    // 当发送目标机器人ID时，该值为0
    float target_position_x;

    // 目标位置y 轴坐标，单位m
    // 当发送目标机器人ID时，该值为0
    float target_position_y;

    // 云台手按下的键盘按键通用键值
    // 无按键按下，则为0
    uint8_t cmd_keyboard;

    // 对方机器人ID
    // 当发送坐标数据时，该值为0
    uint8_t target_robot_id;

    // 信息来源ID
    // 信息来源的ID，ID 对应关系详见附录
    uint16_t cmd_source;
};

// 0x0304（图传链路），键鼠遥控数据（30Hz）
// 选手端→选手端图传连接的机器人
// 注：通过遥控器发送的键鼠遥控数据将同步通过图传链路发送给对应机器人
struct __attribute__((packed)) remote_control_t {
    int16_t mouse_x;          // 鼠标x轴移动速度，负值标识向左移动
    int16_t mouse_y;          // 鼠标y轴移动速度，负值标识向下移动
    int16_t mouse_z;          // 鼠标滚轮移动速度，负值标识向后滚动
    int8_t left_button_down;  // 鼠标左键是否按下：0为未按下；1为按下
    int8_t right_button_down; // 鼠标右键是否按下：0为未按下，1为按下

    struct {
        // 键盘按键信息，每个bit 对应一个按键，0 为未按下，1 为按下
        uint16_t w : 1;     // bit 0: W 键
        uint16_t s : 1;     // bit 1: S 键
        uint16_t a : 1;     // bit 2: A 键
        uint16_t d : 1;     // bit 3: D 键
        uint16_t shift : 1; // bit 4: Shift 键
        uint16_t ctrl : 1;  // bit 5: Ctrl 键
        uint16_t q : 1;     // bit 6: Q 键
        uint16_t e : 1;     // bit 7: E 键
        uint16_t r : 1;     // bit 8: R 键
        uint16_t f : 1;     // bit 9: F 键
        uint16_t g : 1;     // bit 10: G 键
        uint16_t z : 1;     // bit 11: Z 键
        uint16_t x : 1;     // bit 12: X 键
        uint16_t c : 1;     // bit 13: C 键
        uint16_t v : 1;     // bit 14: V 键
        uint16_t b : 1;     // bit 15: B 键
    } keyboard_value;

    uint16_t reserved; // 保留位
};

// 0x0305，选手端小地图接收雷达数据（频率上限为5Hz）
// 雷达→服务器→己方所有选手端
// 注：当x、y超出边界时显示在对应边缘处，当x、y均为0时，视为未发送此机器人坐标
struct __attribute__((packed)) map_robot_data_t {
    uint16_t hero_position_x;       // 英雄机器人x位置坐标，单位：cm
    uint16_t hero_position_y;       // 英雄机器人y位置坐标，单位：cm
    uint16_t engineer_position_x;   // 工程机器人x位置坐标，单位：cm
    uint16_t engineer_position_y;   // 工程机器人y位置坐标，单位：cm
    uint16_t infantry_3_position_x; // 3号步兵机器人x位置坐标，单位：cm
    uint16_t infantry_3_position_y; // 3号步兵机器人y位置坐标，单位：cm
    uint16_t infantry_4_position_x; // 4号步兵机器人x位置坐标，单位：cm
    uint16_t infantry_4_position_y; // 4号步兵机器人y位置坐标，单位：cm
    uint16_t infantry_5_position_x; // 5号步兵机器人x位置坐标，单位：cm
    uint16_t infantry_5_position_y; // 5号步兵机器人y位置坐标，单位：cm
    uint16_t sentry_position_x;     // 哨兵机器人x位置坐标，单位：cm
    uint16_t sentry_position_y;     // 哨兵机器人y位置坐标，单位：cm
};

// 0x0306，自定义控制器与选手端交互数据（频率上限为30Hz）
// 自定义控制器→选手端
// 注：操作手可使用自定义控制器模拟键鼠操作选手端
struct __attribute__((packed)) custom_client_data_t {
    // 键盘键值
    // bit 0-7：按键1键值
    // bit 8-15：按键2键值
    // 仅响应选手端开放的按键
    // 使用通用键值，支持2键无冲，键值顺序变更不会改变按下状态，若无新的按键信息，将保持上一帧数据的按下状态
    uint16_t key_value;

    // 位置信息使用绝对像素点值（赛事客户端使用的分辨率为1920×1080，屏幕左上角为（0，0））

    uint16_t x_position : 12; // 鼠标X轴像素位置
    uint8_t mouse_left : 4;   // 鼠标左键状态
    uint16_t y_position : 12; // 鼠标Y轴像素位置
    uint8_t mouse_right : 4;  // 鼠标右键状态
    uint16_t reserved;        // 保留位
};

// 0x0307，选手端小地图接收路径数据（频率上限为1Hz）
// 哨兵/半自动控制机器人→对应操作手选手端
// 注：哨兵机器人或半自动控制方式的机器人可通过常规链路向对应的操作手选手端发送路径坐标数据，该路径会在小地图上显示
struct __attribute__((packed)) map_data_t {
    // 1：到目标点攻击
    // 2：到目标点防守
    // 3：移动到目标点
    uint8_t intention;

    // 小地图左下角为坐标原点，
    // 水平向右为X轴正方向，
    // 竖直向上为Y轴正方向。
    // 显示位置将按照场地尺寸与小地图尺寸等比缩放，
    // 超出边界的位置将在边界处显示

    // 增量相较于上一个点位进行计算，共49个新点位，X与Y轴增量对应组成点位
    uint16_t start_position_x; // 路径起点x轴坐标，单位：dm
    uint16_t start_position_y; // 路径起点y轴坐标，单位：dm
    int8_t delta_x[49];        // 路径点x轴增量数组，单位：dm
    int8_t delta_y[49];        // 路径点y轴增量数组，单位：dm

    // 发送者ID
    // 需与自身ID匹配，ID编号详见附录
    uint16_t sender_id;
};

// 0x0308，选手端小地图接收机器人数据（频率上限为3Hz）
// 己方机器人→己方选手端
// 己方机器人可通过常规链路向己方任意选手端发送自定义的消息，该消息会在己方选手端特定位置显示
struct __attribute__((packed)) custom_info_t {
    // 发送者的ID
    // 需要校验发送者的ID正确性
    uint16_t sender_id;

    // 接收者的ID
    // 需要校验接收者的ID正确性
    // 仅支持发送己方选手端
    uint16_t receiver_id;

    // 字符
    // 以utf-16 格式编码发送，支持显示中文。
    // 编码发送时请注意数据的大小端问题
    uint8_t user_data[30];
};

// 0x0310（图传链路），机器人发送给自定义客户端的数据（频率上限为 50Hz）
// 己方机器人→图传链路→对应操作手选手端连接的自定义客户端
struct __attribute__((packed)) robot_custom_data_2_t {
    uint8_t data[150];
};