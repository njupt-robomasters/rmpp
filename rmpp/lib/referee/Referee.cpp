#include "Referee.hpp"

Referee::Referee(const config_t& config) : config(config), parser(config) {}

void Referee::SetYaw(const UnitFloat<>& yaw_ecd, const UnitFloat<>& yaw_imu) {
    this->yaw_ecd = yaw_ecd;
    this->yaw_imu = yaw_imu;
}

void Referee::OnLoop() {
    parser.OnLoop();

    is_connect = parser.is_connect;

    // 机器人数据
    robot.id = parser.robot_status.robot_id; // 机器人ID
    // 机器人类型
    if (robot.id == 1 || robot.id == 101) robot.type = HERO;
    if (robot.id == 2 || robot.id == 102) robot.type = ENGINEER;
    if (robot.id == 3 || robot.id == 103) robot.type = INFANTRY;
    if (robot.id == 7 || robot.id == 107) robot.type = SENTRY;
    robot.hp = parser.robot_status.current_HP; // 当前血量

    // 比赛数据
    game.is_red = robot.id < 100;                                           // 红蓝方
    game.type = game_type_e(parser.game_status.game_type);                  // 比赛类型
    game.game_progress = game_progress_e(parser.game_status.game_progress); // 当前比赛阶段
    game.stage_remain_time = parser.game_status.stage_remain_time * s;      // 当前阶段剩余时间

    // 底盘数据
    chassis.power_limit = parser.robot_status.chassis_power_limit * W; // 底盘功率上限
    chassis.buffer_energy = parser.power_heat_data.buffer_energy * J;  // 缓冲能量

    // 发射机构数据
    shooter.heat_limit = parser.robot_status.shooter_barrel_heat_limit;                   // 射击热量上限
    if (robot.type == HERO) {                                                             // 英雄，使用的42mm弹丸数据
        shooter.heat_current = parser.power_heat_data.shooter_42mm_barrel_heat;           // 当前射击热量
        shooter.bullet_allowance = parser.projectile_allowance.projectile_allowance_42mm; // 允许发弹量
    } else {                                                                              // 其他默认使用17mm弹丸数据
        shooter.heat_current = parser.power_heat_data.shooter_17mm_1_barrel_heat;         // 当前射击热量
        shooter.bullet_allowance = parser.projectile_allowance.projectile_allowance_17mm; // 允许发弹量
    }
    shooter.heat_remain = shooter.heat_limit - shooter.heat_current;  // 剩余射击热量
    shooter.bullet_freq = parser.shoot_data.launching_frequency * Hz; // 射击频率
    shooter.bullet_speed = parser.shoot_data.initial_speed * m_s;     // 弹丸初速度

    // RFID数据
    rfid.in_home = parser.rfid_status.rfid_status & (1 << 19);                     // 是否在启动区
    rfid.in_center = parser.rfid_status.rfid_status & (1 << 23);                   // 是否在中心增益点
    rfid.center_buff = center_buff_e((parser.event_data.event_data >> 23) & 0b11); // 中心增益点的占领状态

    // 伤害方向
    if (dwt_hurt != parser.dwt_hurt_data) {              // 有新的伤害
        if (parser.hurt_data.HP_deduction_reason == 0) { // 0：装甲模块被弹丸攻击导致扣血
            dwt_hurt = parser.dwt_hurt_data;

            // 计算伤害方向
            switch (parser.hurt_data.armor_id) {
                case 0:
                    hurt_dir_by_imu = 0 * deg + (yaw_imu - yaw_ecd);
                    break;
                case 1:
                    hurt_dir_by_imu = 90 * deg + (yaw_imu - yaw_ecd);
                    break;
                case 2:
                    hurt_dir_by_imu = 180 * deg + (yaw_imu - yaw_ecd);
                    break;
                case 3:
                    hurt_dir_by_imu = 270 * deg + (yaw_imu - yaw_ecd);
                    break;
                default:
                    break;
            }
        }
    }

    // 更新伤害方向
    hurt.is_hurt = dwt_hurt.GetDT() < HURT_TIMEOUT;
    hurt.dir = hurt_dir_by_imu - yaw_imu;
}