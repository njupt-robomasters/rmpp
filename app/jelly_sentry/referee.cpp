#include "referee.hpp"
#include "app.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "shooter.hpp"
#include <cmath>

static constexpr float HURT_TIMEOUT = 1; // 受击方向显示时长，s

// 受击方向，收到新的 0x0206 时按装甲 ID 换算到 IMU 参考系并锁存
static uint32_t last_hurt_sequence = 0;
static bool has_hurt = false;
static Angle hurt_dir;
static Timer hurt_timer; // 上次受击后重新开始

// 设备状态：在线绿色，离线粉色
static RobotUi::Color status_color(const bool is_connect) {
    return is_connect ? RobotUi::GREEN : RobotUi::PINK;
}

// 传送带状态：离线粉色，偏离目标弹速超过 1 m/s 橙色
static RobotUi::Color rub_color(const Motor& motor, const float speed) {
    if (!motor.is_connect)
        return RobotUi::PINK;
    if (std::fabs(shooter.bullet_speed.ref - speed) > 1)
        return RobotUi::ORANGE;
    return RobotUi::GREEN;
}

void referee_loop() {
    referee.update();
    referee_sender.set_robot_id(referee.robot_status.robot_id);

    // F 键按下时刷新 UI
    static bool last_f = false;
    if (vt13.key.f && !last_f)
        ui.init(referee_sender);
    last_f = vt13.key.f;

    // 受击方向：仅统计装甲受弹丸攻击；装甲 ID 0~3 依次位于底盘前方逆时针 0°、90°、180°、270°
    if (referee.hurt_sequence != last_hurt_sequence) {
        last_hurt_sequence = referee.hurt_sequence;
        const auto [armor_id, HP_deduction_reason] = referee.hurt_data;
        if (HP_deduction_reason == 0 && armor_id <= 3) {
            hurt_dir = armor_id * 90 * deg + gimbal.yaw.imu_minus_ecd;
            hurt_timer.restart();
            has_hurt = true;
        }
    }

    // 哨兵无舵轮，s1~s4 保持默认绿色；UI 只有一个 pitch 状态位，任一 pitch 电机断联都显示为断联
    ui.update(referee_sender, {
                                  .yaw_ecd = gimbal.yaw.ecd.measure, // 云台相对底盘角度
                                  .yaw = gimbal.yaw.imu.measure,
                                  .pitch = gimbal.pitch.imu.measure,
                                  .is_hurt = has_hurt && hurt_timer.elapsed() < HURT_TIMEOUT,
                                  .hurt_dir = hurt_dir - gimbal.yaw.imu.measure, // 相对云台
                                  .is_detect = aim.vision.is_detect,
                                  .wr = std::isfinite(chassis.wr.measure) ? chassis.wr.measure : 0, // 反馈未知时显示 0
                                  .bullet = vt13.config.bullet_freq,                                // 最高弹频
                                  .shoot = shoot.current,                                           // 拨弹电机电流
                                  .w1 = status_color(w1.is_connect),                                // 底盘
                                  .w2 = status_color(w2.is_connect),
                                  .w3 = status_color(w3.is_connect),
                                  .w4 = status_color(w4.is_connect),
                                  .yaw1 = status_color(yaw1.is_connect), // 云台
                                  .yaw2 = status_color(yaw2.is_connect),
                                  .pitch_motor = status_color(pitch1.is_connect && pitch2.is_connect),
                                  .rub1 = rub_color(rub1, shooter.rub_speed[0]), // 发射机构
                                  .rub2 = rub_color(rub2, shooter.rub_speed[1]),
                                  .shooter = status_color(shoot.is_connect),
                                  .referee = status_color(referee.is_connect), // 裁判系统
                                  .aim = status_color(aim.is_connect_vision),  // 自瞄
                              });
}
