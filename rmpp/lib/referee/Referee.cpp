#include "Referee.hpp"

Referee::Referee(const config_t& config) : config(config), parser(config) {}

void Referee::OnLoop(const UnitFloat<>& imu_yaw, const UnitFloat<>& gimbal_yaw) {
    parser.OnLoop();

    // 本机器人ID
    robot_id = parser.robot_status.robot_id;

    // 机器人当前血量
    hp = parser.robot_status.current_HP;

    // 比赛数据
    game.is_red = robot_id < 100;
    game.progress = game_progress_e(parser.game_status.game_progress);
    game.stage_remain_time = parser.game_status.stage_remain_time * s;

    // 底盘数据
    chassis.power_limit = parser.robot_status.chassis_power_limit * W;
    chassis.buffer_energy = parser.power_heat_data.buffer_energy * J;

    // 发射机构数据
    shooter.heat_limit = parser.robot_status.shooter_barrel_heat_limit;
    if (robot_id == 1 || robot_id == 101) { // 英雄
        shooter.heat_current = parser.power_heat_data.shooter_42mm_barrel_heat;
        shooter.bullet_allowance = parser.projectile_allowance.projectile_allowance_42mm;
    } else {
        shooter.heat_current = parser.power_heat_data.shooter_17mm_1_barrel_heat;
        shooter.bullet_allowance = parser.projectile_allowance.projectile_allowance_17mm;
    }
    shooter.heat_remain = (int16_t)(shooter.heat_limit - shooter.heat_current);
    shooter.bullet_freq = parser.shoot_data.launching_frequency * Hz;
    shooter.bullet_speed = parser.shoot_data.initial_speed * m_s;

    switch ((parser.event_data.event_data >> 23) & 0b11) {
        case 0: // 未被占领
            center_buff = EMPTY;
            break;
        case 1: // 己方占领
            if ((parser.rfid_status.rfid_status >> 23) & 0b1) {
                center_buff = SELF;
            } else {
                center_buff = TEAM;
            }
            break;
        case 2: // 对方占领
            center_buff = ENEMY;
            break;
        case 3: // 双方占领
            center_buff = BOTH;
            break;
        default:
            break;
    }

    // 伤害数据
    if (parser.hurt_data.HP_deduction_reason == 0 && hurt.dwt != parser.dwt_hurt_data) {
        hurt.dwt = parser.dwt_hurt_data;
        hurt.cnt++;

        // 计算伤害方向
        switch (parser.hurt_data.armor_id) {
            case 0:
                hurt.dir.by_world = imu_yaw - gimbal_yaw + 0 * deg;
                break;
            case 1:
                hurt.dir.by_world = imu_yaw - gimbal_yaw + 90 * deg;
                break;
            case 2:
                hurt.dir.by_world = imu_yaw - gimbal_yaw + 180 * deg;
                break;
            case 3:
                hurt.dir.by_world = imu_yaw - gimbal_yaw + 270 * deg;
                break;
            default:
                break;
        }
    } else {
        // 更新伤害方向
        hurt.dir.by_gimbal = hurt.dir.by_world - imu_yaw;
    }
}