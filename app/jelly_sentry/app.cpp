#include "app.hpp"
#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include "melody/music/c5d5g5.hpp"
#include <cmath>

// 伤害方向显示时长，s
static constexpr float HURT_TIMEOUT = 1;

// 跨周期状态
static uint32_t last_hurt_sequence = 0;
static Angle hurt_dir_imu; // 伤害方向，IMU 参考系
static Timer hurt_timer;   // 自上次装甲受弹丸攻击以来的时间
static bool has_hurt = false;
static uint8_t chassis_divider = 0;
static uint32_t super_cap_send_count = 0;
static uint32_t yaw2_divider = 0;
[[maybe_unused]] static uint32_t pitch1_divider = 0, pitch2_divider = 0, shoot_divider = 0;

static void send_can_cmd() {
    // CAN1：底盘 C620 控制帧 0x200 每 2 ms 发一次，与 C620 500 Hz 回传对齐，降低 CAN1 负载
    // 需在 RoboMaster 调参助手中把 C620 回传频率设为 500 Hz
    if (++chassis_divider >= 2) {
        chassis_divider = 0;
        CanFrame wheel_frame{.id = 0x200, .size = 8};
        w1.write_can_frame(wheel_frame);
        w2.write_can_frame(wheel_frame);
        w3.write_can_frame(wheel_frame);
        w4.write_can_frame(wheel_frame);
        Can(1).send(wheel_frame);
    }

    // CAN1：大 yaw DM4310
    yaw1.send();

    // CAN1：超电 100 Hz 心跳；OFF、底盘失能与超电离线时也发送，保持反馈并支持重连
    if (super_cap_send_count++ % 10 == 0)
        super_cap.send();

    // CAN2：pitch1 / pitch2 发送周期由 gimbal.hpp 的可写变量决定
    if (++pitch1_divider >= pitch1_send_period_ms) {
        pitch1_divider = 0;
        pitch1.send();
    }
    if (++pitch2_divider >= pitch2_send_period_ms) {
        pitch2_divider = 0;
        pitch2.send();
    }

    // // CAN3：小 yaw 7025 转矩与读角度交替，各占总发送频率的一半
    if (++yaw2_divider >= yaw2_send_period_ms) {
        yaw2_divider = 0;
        yaw2.send();
    }

    // // CAN2：两路 P630 ，VESC 速度模式；失能时驱动发送零电流
    rub1.send();
    rub2.send();

    // CAN2：EL05 发送周期由 shooter.hpp 的可写变量决定
    if (++shoot_divider >= shoot_send_period_ms) {
        shoot_divider = 0;
        shoot.send();
    }
}

// 摩擦轮状态：离线粉色，偏离目标弹速超过 1 m/s 橙色
static RobotUi::Color rub_color(const Motor& motor, const float speed) {
    if (!motor.is_connect)
        return RobotUi::PINK;
    if (std::fabs(shooter.bullet_speed.ref - speed) > 1)
        return RobotUi::ORANGE;
    return RobotUi::GREEN;
}

// 伤害方向：装甲受弹丸攻击时按装甲 ID 换算到 IMU 参考系，显示时再换算为相对云台的方向
// 装甲 ID 0~3 依次位于底盘前方逆时针 0°、90°、180°、270°
static void update_hurt() {
    if (referee.hurt_sequence != last_hurt_sequence) {
        last_hurt_sequence = referee.hurt_sequence;
        const auto& hurt = referee.hurt_data;
        if (hurt.HP_deduction_reason == 0 && hurt.armor_id <= 3) {
            hurt_dir_imu = hurt.armor_id * 90 * deg + gimbal.yaw.imu_minus_ecd;
            hurt_timer.restart();
            has_hurt = true;
        }
    }
    ui.is_hurt = has_hurt && hurt_timer.elapsed() < HURT_TIMEOUT;
    ui.hurt_dir = hurt_dir_imu - gimbal.yaw.imu.measure;
}

static void update_ui() {
    // 云台角度
    ui.yaw_ecd = gimbal.yaw.ecd.measure; // 云台相对底盘角度
    ui.yaw = gimbal.yaw.imu.measure;
    ui.pitch = gimbal.pitch.imu.measure;

    // 伤害方向
    update_hurt();

    // 自瞄识别
    ui.is_detect = aim.vision.is_detect;

    // 提示条
    // 弹速或弹频由 shooter_loop 写入
    ui.cap = 0.0f;              // 超级电容剩余能量比例
    ui.wr = chassis.wr.measure; // 底盘旋转速度
    ui.shoot = shoot.current;   // 拨弹电机电流

    // 机器人状态；哨兵无舵轮，s1~s4 保持默认绿色
    ui.robot.w1 = w1.is_connect ? RobotUi::GREEN : RobotUi::PINK; // 底盘
    ui.robot.w2 = w2.is_connect ? RobotUi::GREEN : RobotUi::PINK;
    ui.robot.w3 = w3.is_connect ? RobotUi::GREEN : RobotUi::PINK;
    ui.robot.w4 = w4.is_connect ? RobotUi::GREEN : RobotUi::PINK;
    ui.robot.yaw1 = yaw1.is_connect ? RobotUi::GREEN : RobotUi::PINK; // 云台
    ui.robot.yaw2 = yaw2.is_connect ? RobotUi::GREEN : RobotUi::PINK;
    // UI 只有一个 pitch 状态位，任一 pitch 电机断联都显示为断联
    ui.robot.pitch = pitch1.is_connect && pitch2.is_connect ? RobotUi::GREEN : RobotUi::PINK;
    ui.robot.rub1 = ui.robot.rub3 = rub_color(rub1, shooter.rub_speed[0]); // 发射机构
    ui.robot.rub2 = ui.robot.rub4 = rub_color(rub2, shooter.rub_speed[1]);
    ui.robot.shoot = shoot.is_connect ? RobotUi::GREEN : RobotUi::PINK;
    ui.robot.referee = referee.is_connect ? RobotUi::GREEN : RobotUi::PINK; // 裁判系统
    ui.robot.aim = aim.is_connect_vision ? RobotUi::GREEN : RobotUi::PINK;  // 自瞄

    ui.update();
}

static float cpu_usage = 0; // 主循环占用，SWD 观察

extern "C" void app_main() {
    // 大 yaw 零位从 KvStore 读取，未标定时为 0；标定方法见 gimbal.cpp 组合键
    yaw1.config.offset = kv_store.read("yaw_offset");
    melody.play(C5D5G5);

    Timer loop_timer;
    while (true) {
        if (loop_timer.elapsed() > 1 * ms) {
            loop_timer.restart();
            misc_loop();
            melody.update();
            referee.update();
            vt13.update();
            fsi6x.update();
            attitude.update();

            // 上位机数据：弹速取裁判系统实测初速
            aim.set_referee(referee.robot_status.robot_id < 100, referee.shoot_data.initial_speed);
            aim.update(attitude.yaw, attitude.pitch, attitude.roll);
            nav.update(gimbal.yaw1.measure, yaw1.is_connect);

            chassis_loop();
            gimbal_loop();
            shooter_loop();

            if (vt13.key.f) // F 键刷新 UI
                ui.init();
            update_ui();
            send_can_cmd();
            const float running_time = loop_timer.elapsed();
            cpu_usage = running_time / (1 * ms);
        }
    }
}
