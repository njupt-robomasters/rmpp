#include "Referee.hpp"

Referee::Referee(const config_t& config) : config(config), parser(config) {}

void Referee::SetYaw(const UnitFloat<>& imu_yaw, const UnitFloat<>& gimbal_yaw) {
    this->imu_yaw = imu_yaw;
    this->gimbal_yaw = gimbal_yaw;
}

void Referee::OnLoop() {
    parser.OnLoop();

    // 机器人ID
    robot_id = parser.robot_status.robot_id;

    // 当前血量
    hp = parser.robot_status.current_HP;

    // 比赛数据
    game.is_red = robot_id < 100;                                      // 红蓝方
    game.progress = game_progress_e(parser.game_status.game_progress); // 当前比赛阶段
    game.stage_remain_time = parser.game_status.stage_remain_time * s; // 当前阶段剩余时间

    // 底盘数据
    chassis.power_limit = parser.robot_status.chassis_power_limit * W; // 底盘功率上限
    chassis.buffer_energy = parser.power_heat_data.buffer_energy * J;  // 缓冲能量

    // 发射机构数据
    shooter.heat_limit = parser.robot_status.shooter_barrel_heat_limit;                   // 射击热量上限
    if (robot_id == 1 || robot_id == 101) {                                               // 英雄，使用的42mm弹丸数据
        shooter.heat_current = parser.power_heat_data.shooter_42mm_barrel_heat;           // 当前射击热量
        shooter.bullet_allowance = parser.projectile_allowance.projectile_allowance_42mm; // 允许发弹量
    } else {                                                                              // 其他默认使用17mm弹丸数据
        shooter.heat_current = parser.power_heat_data.shooter_17mm_1_barrel_heat;         // 当前射击热量
        shooter.bullet_allowance = parser.projectile_allowance.projectile_allowance_17mm; // 允许发弹量
    }
    shooter.heat_remain = shooter.heat_limit - shooter.heat_current; // 剩余射击热量
    shooter.bullet_freq = parser.shoot_data.launching_frequency * Hz;           // 射击频率
    shooter.bullet_speed = parser.shoot_data.initial_speed * m_s;               // 弹丸初速度

    // 中心增益点的占领状态
    switch ((parser.event_data.event_data >> 23) & 0b11) {
        case 0: // 未被占领
            center_buff = EMPTY;
            break;
        case 1: // 己方占领
            if ((parser.rfid_status.rfid_status >> 23) & 0b1) {
                center_buff = SELF; // 自己占领
            } else {
                center_buff = TEAM; // 队友占领
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

    // 伤害方向
    if (hurt_dwt != parser.dwt_hurt_data) {              // 有新的伤害
        if (parser.hurt_data.HP_deduction_reason == 0) { // 0：装甲模块被弹丸攻击导致扣血
            hurt_dwt = parser.dwt_hurt_data;

            // 计算伤害方向
            switch (parser.hurt_data.armor_id) {
                case 0:
                    hurt_dir_by_imu = 0 * deg - gimbal_yaw + imu_yaw;
                    break;
                case 1:
                    hurt_dir_by_imu = 90 * deg - gimbal_yaw + imu_yaw;
                    break;
                case 2:
                    hurt_dir_by_imu = 180 * deg - gimbal_yaw + imu_yaw;
                    break;
                case 3:
                    hurt_dir_by_imu = 270 * deg - gimbal_yaw + imu_yaw;
                    break;
                default:
                    break;
            }
        }
    }

    // 更新伤害方向
    is_hurt = hurt_dwt.GetDT() < HURT_TIMEOUT;
    hurt_dir = hurt_dir_by_imu - imu_yaw;
}