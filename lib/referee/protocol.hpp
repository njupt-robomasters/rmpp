#pragma once

#include <cstdint>

// 原始协议单位与位段保持不变，离线默认值仅在首次接收前有效
// 0x0001，比赛状态数据（1Hz）
// 服务器→全体机器人
struct __attribute__((packed)) GameStatus {
    // 比赛类型
    // 1：RoboMaster机甲大师超级对抗赛
    // 2：RoboMaster机甲大师高校单项赛
    // 3：ICRA RoboMaster高校人工智能挑战赛
    // 4：RoboMaster机甲大师高校联盟赛3V3对抗
    // 5：RoboMaster机甲大师高校联盟赛步兵对抗
    uint8_t game_type : 4;

    // 当前比赛阶段
    // 0：未开始比赛
    // 1：准备阶段
    // 2：十五秒裁判系统自检阶段
    // 3：五秒倒计时
    // 4：比赛中
    // 5：比赛结算中
    uint8_t game_progress : 4;

    // 当前阶段剩余时间，单位：秒
    uint16_t stage_remain_time;

    // UNIX时间，当机器人正确连接到裁判系统的NTP服务器后生效
    uint64_t sync_timestamp;
};

// 0x0002，比赛结果数据（比赛结束触发发送）
// 服务器→全体机器人
struct __attribute__((packed)) GameResult {
    // 0：平局
    // 1：红方胜利
    // 2：蓝方胜利
    uint8_t winner;
};

// 0x0003，机器人血量数据（3Hz）
// 服务器→全体机器人
// 若机器人未上场或者被罚下，则血量为0
struct __attribute__((packed)) GameRobotHp {
    uint16_t ally_1_robot_HP;  // 己方1号英雄机器人血量
    uint16_t ally_2_robot_HP;  // 己方2号工程机器人血量
    uint16_t ally_3_robot_HP;  // 己方3步兵机器人血量
    uint16_t ally_4_robot_HP;  // 己方4号步兵机器人血量
    int16_t damage_difference; // 己方总伤害减对方总伤害
    uint16_t ally_7_robot_HP;  // 己方7号哨兵机器人血量
    uint16_t ally_outpost_HP;  // 己方前哨站血量
    uint16_t ally_base_HP;     // 己方基地血量
    uint16_t enemy_outpost_HP; // 对方前哨站血量
    uint16_t enemy_base_HP;    // 对方基地血量
};

// 0x0101，场地事件数据（1Hz）
// 服务器→己方全体机器人
struct __attribute__((packed)) EventData {
    uint32_t event_data;
};

// 0x0104，裁判警告数据（1Hz，己方判罚/判负时触发发送）
// 服务器→被判罚方全体机器人
struct __attribute__((packed)) RefereeWarning {
    // 己方最后一次受到判罚的等级
    // 1：双方黄牌
    // 2：黄牌
    // 3：红牌
    // 4：判负
    uint8_t level;

    // 己方最后一次受到判罚的违规机器人ID（如红1机器人ID为1，蓝1机器人ID为101），判负和双方黄牌时，该值为0
    uint8_t offending_robot_id;

    // 己方最后一次受到判罚的违规机器人对应判罚等级的违规次数（开局默认为0）
    uint8_t count;
};

// 0x0105，飞镖发射相关数据（1Hz）
// 服务器→己方全体机器人
struct __attribute__((packed)) DartInfo {
    uint8_t dart_remaining_time; // 己方飞镖发射剩余时间，单位：秒
    uint16_t dart_info;
};

// 0x0201，机器人性能体系数据（10Hz，离线模式下可用）
// 主控模块→对应机器人
struct __attribute__((packed)) RobotStatus {
    uint8_t robot_id;                            // 本机器人ID
    uint8_t robot_level;                         // 机器人等级
    uint16_t current_HP;                         // 机器人当前血量
    uint16_t maximum_HP;                         // 机器人血量上限
    uint16_t shooter_barrel_cooling_value;       // 机器人射击热量每秒冷却值
    uint16_t shooter_barrel_heat_limit = 65535;  // 机器人射击热量上限
    uint16_t chassis_power_limit = 120;          // 机器人底盘功率上限
    float bullet_speed_limit;                    // 弹丸初速度上限，m/s
    uint8_t power_management_gimbal_output : 1;  // gimbal口输出，0为无输出，1为24V输出
    uint8_t power_management_chassis_output : 1; // chassis口输出，0为无输出，1为24V输出
    uint8_t power_management_shooter_output : 1; // shooter口输出，0为无输出，1为24V输出
};

// 0x0202，实时底盘缓冲能量和射击热量数据（10Hz，离线模式下可用）
// 主控模块→对应机器人
struct __attribute__((packed)) PowerHeatData {
    uint16_t reserved_1;                   // 保留位
    uint16_t reserved_2;                   // 保留位
    float reserved_3;                      // 保留位
    uint16_t buffer_energy = 60;           // 缓冲能量（单位：J）
    uint16_t shooter_17mm_barrel_heat = 0; // 第1个17mm发射机构的射击热量
    uint16_t shooter_42mm_barrel_heat = 0; // 42mm发射机构的射击热量
};

// 0x0203，机器人位置数据（1Hz，离线模式下可用）；总览表误列16，详细字段为12
// 主控模块→对应机器人
struct __attribute__((packed)) RobotPos {
    float x;     // 本机器人位置x坐标，单位：m
    float y;     // 本机器人位置y坐标，单位：m
    float angle; // 本机器人测速模块的朝向，单位：度。正北为0度
};

// 0x0204，机器人增益和底盘能量数据（3Hz）
// 服务器→对应机器人
struct __attribute__((packed)) Buff {
    uint8_t recovery_buff;      // 机器人回血增益（百分比，值为10表示每秒恢复血量上限的10%）
    uint16_t cooling_buff;      // 机器人射击热量冷却增益具体值（直接值，值为x表示热量冷却增加x/s）
    uint8_t defence_buff;       // 机器人防御增益（百分比，值为50 表示50%防御增益）
    uint8_t vulnerability_buff; // 机器人负防御增益（百分比，值为30 表示-30%防御增益）
    uint16_t attack_buff;       // 机器人攻击增益（百分比，值为50 表示50%攻击增益）
    uint8_t remaining_energy;   // bit 0~6 依次表示能量 >=125%、100%、50%、30%、15%、5%、1%
};

// 0x0206，伤害状态数据（伤害发生后发送，离线模式下可用）
// 主控模块→对应机器人
// 0x0206的受伤害情况为机器人裁判系统本地判定，即时发送，但实际是否受到对应伤害受规则条例影响，请以服务器最终判定为准
struct __attribute__((packed)) HurtData {
    // 当扣血原因为装甲模块被弹丸攻击、受撞击或离线时，该4bit组成的数值为装甲模块或测速模块的ID编号
    // 当其他原因导致扣血时，该数值为0
    uint8_t armor_id : 4;

    // 血量变化类型
    // 0：装甲模块被弹丸攻击导致扣血
    // 1：装甲模块或超级电容管理模块离线导致扣血
    // 5：装甲模块受到撞击导致扣血
    uint8_t HP_deduction_reason : 4;
};

// 0x0207，实时射击数据（弹丸发射后发送，离线模式下可用）
// 主控模块→对应机器人
struct __attribute__((packed)) ShootData {
    // 弹丸类型
    // 1：17mm弹丸
    // 2：42mm弹丸
    uint8_t bullet_type;

    // 发射机构ID
    // 1：17mm发射机构
    // 2：保留位
    // 3：42mm发射机构
    uint8_t shooter_number;

    // 弹丸射速（单位：Hz）
    uint8_t launching_frequency;

    // 弹丸初速度（单位：m/s）
    float initial_speed;
};

// 0x0208，允许发弹量（10Hz）
// 服务器→己方英雄、步兵、哨兵、空中机器人
struct __attribute__((packed)) ProjectileAllowance {
    uint16_t projectile_allowance_17mm;     // 机器人自身拥有的17mm弹丸允许发弹量
    uint16_t projectile_allowance_42mm;     // 42mm弹丸允许发弹量
    uint16_t remaining_gold_coin;           // 剩余金币数量
    uint16_t projectile_allowance_fortress; // 堡垒增益点提供的储备17mm弹丸允许发弹量；该值与机器人是否实际占领堡垒无关
};

// 0x0209，机器人RFID模块状态（3Hz）
// 服务器→己方装有RFID模块的机器人
// 所有RFID卡仅在赛内生效。在赛外，即使检测到对应的RFID卡，对应值也为0
struct __attribute__((packed)) RfidStatus {
    uint32_t rfid_status;
    uint8_t rfid_status_2;
};

// 0x020A，飞镖选手端指令数据（3Hz）
// 服务器→己方飞镖机器人
struct __attribute__((packed)) DartClientCmd {
    // 当前飞镖发射站的状态
    // 1：关闭
    // 2：正在开启或者关闭中
    // 0：已经开启
    uint8_t dart_launch_opening_status;

    // 保留位
    uint8_t reserved;

    // 切换击打目标时的比赛剩余时间，单位：秒，无/未切换动作，默认为0
    uint16_t target_change_time;

    // 最后一次操作手确定发射指令时的比赛剩余时间，单位：秒，初始值为0
    uint16_t latest_launch_cmd_time;
};

// 0x020B，地面机器人位置数据（1Hz）
// 服务器→己方哨兵机器人
// 场地围挡在红方补给站附近的交点为坐标原点，沿场地长边向蓝方为X轴正方向，沿场地短边向红方停机坪为Y轴正方向
struct __attribute__((packed)) GroundRobotPosition {
    float hero_x;       // 己方英雄机器人位置x轴坐标，单位：m
    float hero_y;       // 己方英雄机器人位置y轴坐标，单位：m
    float engineer_x;   // 己方工程机器人位置x轴坐标，单位：m
    float engineer_y;   // 己方工程机器人位置y轴坐标，单位：m
    float standard_3_x; // 己方3号步兵机器人位置x轴坐标，单位：m
    float standard_3_y; // 己方3号步兵机器人位置y轴坐标，单位：m
    float standard_4_x; // 己方4号步兵机器人位置x轴坐标，单位：m
    float standard_4_y; // 己方4号步兵机器人位置y轴坐标，单位：m
    float reserved_1;   // 保留位
    float reserved_2;   // 保留位
};

// 0x020C，雷达标记进度数据（1Hz）
// 服务器→己方雷达机器人
struct __attribute__((packed)) RadarMarkData {
    uint16_t mark_progress;
};

// 0x020D，哨兵自主决策信息同步（1Hz）
// 服务器→己方哨兵机器人
struct __attribute__((packed)) SentryInfo {
    uint32_t sentry_info;
    uint16_t sentry_info_2;
    uint64_t sentry_info_3; // 各姿态剩余可持续时间，秒
};

// 0x020E，雷达自主决策信息同步（1Hz）
// 服务器→己方雷达机器人
struct __attribute__((packed)) RadarInfo {
    uint8_t radar_info;
};

// 0x0303，选手端小地图指令；坐标 m，详细字段表长度为 12
struct __attribute__((packed)) MapCommand {
    float target_position_x, target_position_y;
    uint8_t cmd_keyboard, target_robot_id;
    uint16_t cmd_source;
};

// 0x0305，双方位置，坐标 cm
struct __attribute__((packed)) MapRobotData {
    uint16_t opponent_hero_position_x, opponent_hero_position_y;
    uint16_t opponent_engineer_position_x, opponent_engineer_position_y;
    uint16_t opponent_infantry_3_position_x, opponent_infantry_3_position_y;
    uint16_t opponent_infantry_4_position_x, opponent_infantry_4_position_y;
    uint16_t opponent_aerial_position_x, opponent_aerial_position_y;
    uint16_t opponent_sentry_position_x, opponent_sentry_position_y;
    uint16_t ally_hero_position_x, ally_hero_position_y;
    uint16_t ally_engineer_position_x, ally_engineer_position_y;
    uint16_t ally_infantry_3_position_x, ally_infantry_3_position_y;
    uint16_t ally_infantry_4_position_x, ally_infantry_4_position_y;
    uint16_t ally_aerial_position_x, ally_aerial_position_y;
    uint16_t ally_sentry_position_x, ally_sentry_position_y;
};

// 0x0306，自定义控制器与选手端交互，坐标为像素
struct __attribute__((packed)) CustomClientData {
    uint16_t key_value;
    uint16_t x_position : 12;
    uint16_t mouse_left : 4;
    uint16_t y_position : 12;
    uint16_t mouse_right : 4;
    uint16_t reserved;
};

// 0x0307，路径起点和增量均为 dm，含 sender_id 共 105 字节；总览表误列103
struct __attribute__((packed)) MapData {
    uint8_t intention;
    uint16_t start_position_x, start_position_y;
    int8_t delta_x[49], delta_y[49];
    uint16_t sender_id;
};

// 0x0308，user_data 为小端 UTF-16 编码的 30 字节
struct __attribute__((packed)) CustomInfo {
    uint16_t sender_id, receiver_id;
    uint8_t user_data[30];
};

// 0x0A01，对方位置，坐标 cm
struct __attribute__((packed)) RadarPosition {
    uint16_t hero_x, hero_y, engineer_x, engineer_y;
    uint16_t infantry_3_x, infantry_3_y, infantry_4_x, infantry_4_y;
    uint16_t aerial_x, aerial_y, sentry_x, sentry_y;
};

// 0x0A02，对方血量
struct __attribute__((packed)) RadarHp {
    uint16_t hero, engineer, infantry_3, infantry_4, reserved, sentry;
};

// 0x0A03，对方允许发弹量
struct __attribute__((packed)) RadarAllowance {
    uint16_t hero, infantry_3, infantry_4, aerial, sentry;
};

// 0x0A04，对方宏观状态；场地状态保留协议位掩码
struct __attribute__((packed)) RadarTeam {
    uint16_t remaining_gold_coin, total_gold_coin;
    uint32_t event_data;
};

struct __attribute__((packed)) RadarRobotBuff {
    uint8_t recovery;
    uint16_t cooling;
    uint8_t defence, vulnerability;
    uint16_t attack;
};

// 0x0A05，对方增益、姿态及存活状态
struct __attribute__((packed)) RadarBuff {
    RadarRobotBuff hero, engineer, infantry_3, infantry_4, sentry;
    uint8_t sentry_posture;
    uint8_t hero_status, engineer_status, infantry_3_status, infantry_4_status, sentry_status;
};

// 0x0A06，6 字节 ASCII 密钥，不包含结束符
struct __attribute__((packed)) RadarKey {
    uint8_t data[6];
};

// 0x0301 子内容，全部字段保持线上编码
struct __attribute__((packed)) LayerDelete {
    uint8_t delete_type, layer;
};

struct __attribute__((packed)) Figure {
    uint8_t name[3];
    uint32_t operation : 3;
    uint32_t type : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t details_a : 9;
    uint32_t details_b : 9;
    uint32_t width : 10;
    uint32_t x : 11;
    uint32_t y : 11;
    uint32_t details_c : 10;
    uint32_t details_d : 11;
    uint32_t details_e : 11;
};

struct __attribute__((packed)) Character {
    Figure figure;
    uint8_t text[30];
};

struct __attribute__((packed)) SentryCommand {
    uint32_t command;
};

struct __attribute__((packed)) RadarCommand {
    uint8_t command, password_command, password[6];
};

// 变长字节数据只保存本帧内容及实际长度
struct CustomData {
    uint8_t data[30]{};
    uint16_t size = 0;
};

struct ClientData {
    uint8_t data[300]{};
    uint16_t size = 0;
};

struct Interaction {
    uint16_t data_cmd_id = 0, sender_id = 0, receiver_id = 0;
    uint16_t size = 0;

    // data_cmd_id 确定有效成员；图形数量由 0x0101~0x0104 确定
    union {
        uint8_t user_data[112]{};
        LayerDelete layer_delete;
        Figure figures[7];
        Character character;
        SentryCommand sentry_command;
        RadarCommand radar_command;
    };
};

