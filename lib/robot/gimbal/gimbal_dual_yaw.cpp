#include "gimbal_dual_yaw.hpp"
#include <algorithm>
#include <cmath>

GimbalDualYaw::GimbalDualYaw(const Motors& motor, const Config& config)
    : Gimbal(config),
      motor(motor) {}

void GimbalDualYaw::set_motors_enable(const bool is_enable) {
    motor.yaw1.set_enable(is_enable);
    motor.yaw2.set_enable(is_enable);
    motor.pitch.set_enable(is_enable);
}

void GimbalDualYaw::update_motors() {
    motor.yaw1.update();
    motor.yaw2.update();
    motor.pitch.update();
}

void GimbalDualYaw::forward() {
    // 从电机读取，离线电机保持最后一次反馈
    yaw1.measure = motor.yaw1.position;
    yaw2.measure = motor.yaw2.position;
    pitch.ecd.measure = motor.pitch.position;

    // 合并大小yaw
    yaw.ecd.measure = yaw1.measure + yaw2.measure;
}

void GimbalDualYaw::backward(const float dt) {
    // 大小 yaw 任一在线即可控制 yaw 轴；pitch 独立
    const bool can_yaw = check_yaw(motor.yaw1.is_connect || motor.yaw2.is_connect);
    const bool can_pitch = check_pitch(motor.pitch.is_connect);

    // 用于参考系转换
    yaw.imu_minus_ecd = yaw.imu.measure - yaw.ecd.measure;
    pitch.imu_minus_ecd = pitch.imu.measure - pitch.ecd.measure;

    if (mode == ECD_MODE) {
        // 换算到imu参考系
        yaw.imu.ref = yaw.ecd.ref + yaw.imu_minus_ecd;
        pitch.imu.ref = pitch.ecd.ref + pitch.imu_minus_ecd;
    } else if (mode == IMU_MODE) {
        // 换算到ecd参考系
        yaw.ecd.ref = yaw.imu.ref - yaw.imu_minus_ecd;
        pitch.ecd.ref = pitch.imu.ref - pitch.imu_minus_ecd;
    }

    // 分配大小yaw
    yaw1.ref = yaw.ecd.ref;                // 大yaw直接设置为目标值
    yaw2.ref = yaw.ecd.ref - yaw1.measure; // 大yaw未转到目标，小yaw来补偿

    // 软件限位
    pitch.ecd.ref = Angle(std::clamp(float(pitch.ecd.ref), config.pitch_min, config.pitch_max));
    yaw1.ref = Angle(yaw1.ref);
    yaw2.ref = Angle(yaw2.ref);
    pitch.ecd.ref = Angle(pitch.ecd.ref);

    // yaw 串级 PID，速度反馈取在线的大 yaw，否则取小 yaw；离线的一侧输出零，在线的一侧单独保持指向
    if (can_yaw) {
        const float speed = motor.yaw1.is_connect ? motor.yaw1.speed : motor.yaw2.speed;
        const float speed_ref = yaw_position_pid.calculate(dt, yaw.ecd.ref - yaw.ecd.measure) - chassis_wr + yaw_speed;
        const float output = yaw_speed_pid.calculate(dt, speed_ref - speed);
        const float output1 = set_output(motor.yaw1, config.yaw_output, motor.yaw1.is_connect ? output : 0.0f);
        const float output2 = set_output(motor.yaw2, config.yaw_output, motor.yaw2.is_connect ? output : 0.0f);

        // PID 跟踪在线电机中被限幅更多的实际输出
        if (!motor.yaw1.is_connect)
            yaw_speed_pid.track(output2);
        else if (!motor.yaw2.is_connect || std::fabs(output1) < std::fabs(output2))
            yaw_speed_pid.track(output1);
        else
            yaw_speed_pid.track(output2);
    } else {
        motor.yaw1.set_current(0);
        motor.yaw2.set_current(0);
    }

    // pitch 串级 PID
    if (can_pitch) {
        const float speed_ref = pitch_position_pid.calculate(dt, pitch.ecd.ref - pitch.ecd.measure) + pitch_speed;
        pitch_speed_pid.track(set_output(motor.pitch, config.pitch_output, pitch_speed_pid.calculate(dt, speed_ref - motor.pitch.speed)));
    } else {
        motor.pitch.set_current(0);
    }

    // 传递软件限位到imu参考系
    yaw.imu.ref = yaw1.ref + yaw.imu_minus_ecd;
    pitch.imu.ref = pitch.ecd.ref + pitch.imu_minus_ecd;
}
