#include "referee.hpp"
#include "math/unit.hpp"

Referee::Referee(const Config& config) : config(config), parser_(config) {}

void Referee::set_yaw(float yaw_ecd, float yaw_imu) {
    this->yaw_ecd_ = yaw_ecd;
    this->yaw_imu_ = yaw_imu;
}

void Referee::on_loop() {
    parser_.on_loop();

    is_connect = parser_.is_connect;

    // 机器人数据
    robot.id = parser_.robot_status.robot_id; // 机器人ID
    // 机器人类型
    if (robot.id == 1 || robot.id == 101)
        robot.type = HERO;
    if (robot.id == 2 || robot.id == 102)
        robot.type = ENGINEER;
    if (robot.id == 3 || robot.id == 103)
        robot.type = INFANTRY;
    if (robot.id == 7 || robot.id == 107)
        robot.type = SENTRY;
    robot.hp = parser_.robot_status.current_HP; // 当前血量

    // 比赛数据
    game.is_red = robot.id < 100;                                         // 红蓝方
    game.type = GameType(parser_.game_status.game_type);                  // 比赛类型
    game.game_progress = GameProgress(parser_.game_status.game_progress); // 当前比赛阶段
    game.stage_remain_time = parser_.game_status.stage_remain_time;       // 当前阶段剩余时间

    // 底盘数据
    chassis.power_limit = parser_.robot_status.chassis_power_limit; // 底盘功率上限
    chassis.buffer_energy = parser_.power_heat_data.buffer_energy;  // 缓冲能量

    // 发射机构数据
    shooter.heat_limit = parser_.robot_status.shooter_barrel_heat_limit;                   // 射击热量上限
    if (robot.type == HERO) {                                                              // 英雄，使用的42mm弹丸数据
        shooter.heat_current = parser_.power_heat_data.shooter_42mm_barrel_heat;           // 当前射击热量
        shooter.bullet_allowance = parser_.projectile_allowance.projectile_allowance_42mm; // 允许发弹量
    } else {                                                                               // 其他默认使用17mm弹丸数据
        shooter.heat_current = parser_.power_heat_data.shooter_17mm_1_barrel_heat;         // 当前射击热量
        shooter.bullet_allowance = parser_.projectile_allowance.projectile_allowance_17mm; // 允许发弹量
    }
    shooter.heat_remain = shooter.heat_limit - shooter.heat_current; // 剩余射击热量
    shooter.bullet_freq = parser_.shoot_data.launching_frequency;    // 射击频率
    shooter.bullet_speed = parser_.shoot_data.initial_speed;         // 弹丸初速度

    // RFID数据
    rfid.in_home = parser_.rfid_status.rfid_status & (1 << 19);                  // 是否在启动区
    rfid.in_center = parser_.rfid_status.rfid_status & (1 << 23);                // 是否在中心增益点
    rfid.center_buff = CenterBuff((parser_.event_data.event_data >> 23) & 0b11); // 中心增益点的占领状态

    // 伤害方向
    if (timer_hurt_ != parser_.timer_hurt_data) {         // 有新的伤害
        if (parser_.hurt_data.HP_deduction_reason == 0) { // 0：装甲模块被弹丸攻击导致扣血
            timer_hurt_ = parser_.timer_hurt_data;

            // 计算伤害方向
            switch (parser_.hurt_data.armor_id) {
            case 0:
                hurt_dir_by_imu_ = 0 * deg + (yaw_imu_ - yaw_ecd_);
                break;
            case 1:
                hurt_dir_by_imu_ = 90 * deg + (yaw_imu_ - yaw_ecd_);
                break;
            case 2:
                hurt_dir_by_imu_ = 180 * deg + (yaw_imu_ - yaw_ecd_);
                break;
            case 3:
                hurt_dir_by_imu_ = 270 * deg + (yaw_imu_ - yaw_ecd_);
                break;
            default:
                break;
            }
        }
    }

    // 更新伤害方向
    hurt.is_hurt = timer_hurt_.get_dt() < HURT_TIMEOUT;
    hurt.dir = hurt_dir_by_imu_ - yaw_imu_;
}