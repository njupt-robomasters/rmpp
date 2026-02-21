#pragma once

#include "lib/RefereeParser.hpp"

class Referee {
public:
    using config_t = RefereeParser::config_t;

    config_t config;

    enum game_progress_e {
        NOT_STARTED        = 0, // 未开始比赛
        PREPARING          = 1, // 准备阶段
        REFEREE_SELF_CHECK = 2, // 十五秒裁判系统自检阶段
        COUNTDOWN_5SEC     = 3, // 五秒倒计时
        GAMING             = 4, // 比赛中
        SETTLING           = 5, // 比赛结算中
    };

    enum center_buff_e {
        EMPTY,
        SELF,
        TEAM,
        ENEMY,
        BOTH,
    };

    // 机器人ID
    // 0x0201 robot_status.robot_id
    uint8_t robot_id = 3;

    // 当前血量
    // 0x0201 robot_status.current_HP
    uint16_t hp = 0;

    // 比赛数据
    struct {
        // 红蓝方
        // 0x0201 robot_status.robot_id
        bool is_red = true;

        // 当前比赛阶段
        // 0x0001 game_status.game_progress
        game_progress_e progress = NOT_STARTED;

        // 当前阶段剩余时间
        // 0x0001 game_status.stage_remain_time
        UnitFloat<s> stage_remain_time;
    } game;

    // 底盘数据
    struct {
        // 底盘功率上限
        // 0x0201 robot_status.chassis_power_limit
        UnitFloat<W> power_limit;

        // 缓冲能量
        // 0x0202 power_heat_data.buffer_energy
        UnitFloat<J> buffer_energy;
    } chassis;

    // 发射机构数据
    struct {
        // 射击热量上限
        // 0x0201 robot_status.shooter_barrel_heat_limit
        uint16_t heat_limit = 0;

        // 当前射击热量
        // 0x0202 power_heat_data.shooter_17mm_1_barrel_heat
        // 0x0202 power_heat_data.shooter_42mm_barrel_heat
        uint16_t heat_current = 0;

        // 剩余射击热量
        int16_t heat_remain = 0;

        // 射击频率
        // 0x0207 shoot_data.launching_frequency
        UnitFloat<Hz> bullet_freq;

        // 弹丸初速度
        // 0x0207 shoot_data.initial_speed
        UnitFloat<m_s> bullet_speed;

        // 允许发弹量
        // 0x0208 projectile_allowance.projectile_allowance_17mm
        // 0x0208 projectile_allowance.projectile_allowance_42mm
        uint16_t bullet_allowance = 0;
    } shooter;

    // 中心增益点的占领状态
    // 0x0101 event_data
    // 0x0209 rfid_status
    center_buff_e center_buff = EMPTY;

    // 伤害方向
    bool is_hurt;
    Angle<deg> hurt_dir; // 伤害方向，相对于云台参考系

    Referee(const config_t& config);

    // 设置yaw角度，用于计算伤害方向
    void SetYaw(const UnitFloat<>& imu_yaw, const UnitFloat<>& gimbal_yaw);

    // 需要在循环中调用
    void OnLoop();

private:
    static constexpr float HURT_TIMEOUT = 1;

    RefereeParser parser; // 裁判系统报文解析库

    // 用于伤害方向
    UnitFloat<deg> imu_yaw, gimbal_yaw;
    BSP::Dwt hurt_dwt;
    Angle<deg> hurt_dir_by_imu; // 伤害方向，相对于IMu参考系
};