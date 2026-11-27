#include "gimbal_fold.hpp"
#include <algorithm>
#include <cmath>

GimbalFold::GimbalFold(const Motors& motor, const Config& config)
    : config(config), motor(motor),
      yaw1_angle_pid(config.yaw1_angle_pid_config), yaw1_speed_pid(config.yaw1_speed_pid_config),
      yaw2_angle_pid(config.yaw2_angle_pid_config), yaw2_speed_pid(config.yaw2_speed_pid_config),
      pitch_angle_pid(config.pitch_angle_pid_config), pitch_speed_pid(config.pitch_speed_pid_config),
      pitch1_position_pid(config.pitch1_position_pid_config), pitch1_speed_pid(config.pitch1_speed_pid_config) {}

void GimbalFold::set_enable(const bool is_enable) {
    this->is_enable = is_enable;
    motor.yaw1.set_enable(is_enable);
    motor.yaw2.set_enable(is_enable);
    // 折叠关节暂时固定失能，清除旧目标输出
    motor.pitch1.set_enable(false);
    motor.pitch1.set_output(0);
    motor.pitch2.set_enable(is_enable);
}

void GimbalFold::set_speed(const float yaw_speed, const float pitch_speed) {
    this->yaw_speed = yaw_speed;
    this->pitch_speed = pitch_speed;
}

void GimbalFold::set_angle(const Angle yaw, const Angle pitch) {
    this->yaw.ref = yaw;
    this->pitch.ref = pitch;
    yaw_speed = pitch_speed = 0;
}

void GimbalFold::turn_yaw(const float delta) {
    yaw.ref += delta;
}

void GimbalFold::set_pitch1_angle(const Angle angle) {
    pitch1.ref = angle;
}

void GimbalFold::update(const Quaternionf& q, const Vector3f& gyro) {
    const float dt = std::clamp(update_timer.restart(), 0.0001f, 0.01f);

    // 反馈：世界角度与角速度取 IMU，关节角取编码器，离线电机保持最后一次反馈
    motor.yaw1.update();
    motor.yaw2.update();
    motor.pitch1.update();
    motor.pitch2.update();
    const auto euler = quaternion_to_euler(q);
    yaw.measure = euler.yaw;
    pitch.measure = euler.pitch;
    yaw.speed_measure = (q * gyro).z(); // 世界 yaw 角速度；含底盘转动
    pitch.speed_measure = gyro.y();
    if (motor.yaw1.is_connect)
        yaw1.measure = motor.yaw1.angle;
    if (motor.yaw2.is_connect)
        yaw2.measure = motor.yaw2.angle;
    if (motor.pitch1.is_connect)
        pitch1.measure = motor.pitch1.angle;
    if (motor.pitch2.is_connect)
        pitch.ecd_measure = motor.pitch2.angle;

    // 编码器总角与参考系偏差；底盘前进方向、限位和重力补偿使用
    yaw.ecd_measure = yaw1.measure + yaw2.measure;
    yaw.imu_minus_ecd = Angle(yaw.measure - yaw.ecd_measure);
    pitch.imu_minus_ecd = Angle(pitch.measure - pitch.ecd_measure);

    // 按世界系目标转速积分角度
    if (is_enable) {
        yaw.ref += yaw_speed * dt;
        pitch.ref += pitch_speed * dt;
    }

    // 轴可控：yaw 需要大小 yaw 同时在线（大 yaw 世界角要减小 yaw 编码器），pitch 只要求 pitch2
    const bool can_yaw = is_enable && motor.yaw1.is_connect && motor.yaw2.is_connect;
    const bool can_pitch = is_enable && motor.pitch2.is_connect;
    const bool is_yaw_loop = can_yaw && is_yaw_ready;
    const bool is_pitch_loop = can_pitch && is_pitch_ready;
    is_yaw_ready = can_yaw;
    is_pitch_ready = can_pitch;

    // 不可控或恢复首帧：目标跟随实测、清空 PID，恢复后从当前姿态开始
    if (!is_yaw_loop) {
        yaw.ref = yaw.measure;
        yaw.speed_ref = 0;
        yaw1.speed_ref = yaw1.speed_measure = 0;
        yaw1_angle_pid.clear();
        yaw1_speed_pid.clear();
        yaw2_angle_pid.clear();
        yaw2_speed_pid.clear();
    }
    if (!is_pitch_loop) {
        pitch.ref = pitch.measure;
        pitch.speed_ref = 0;
        pitch_angle_pid.clear();
        pitch_speed_pid.clear();
    }

    // 俯仰软件限位：世界目标按 IMU 与编码器偏差换算后钳制，回写编码器系参考供扫描端点使用
    if (is_pitch_loop) {
        const float min = config.pitch_min + pitch.imu_minus_ecd;
        const float max = config.pitch_max + pitch.imu_minus_ecd;
        pitch.ref = std::clamp(float(pitch.ref), min, max);
    }
    pitch.ecd_ref = Angle(pitch.ref - pitch.imu_minus_ecd);

    // 大 yaw 自身世界目标与实测；小 yaw 中位为 0，头部目标即大 yaw 目标
    yaw1.ref = yaw.ref;
    yaw1.world_measure = Angle(yaw.measure - yaw2.measure);

    // 小 yaw：头部角度外环 + 世界角速度内环（IMU），主执行器；越限时截断向外输出，保留回程
    if (is_yaw_loop) {
        const float err = plan_angle_error(Angle(yaw.ref - yaw.measure), config.yaw2_angle_pid_config, config.yaw_max_accel);
        yaw.speed_ref = yaw2_angle_pid.calculate(dt, err) + yaw_speed;
        float output = yaw2_speed_pid.calculate(dt, yaw.speed_ref - yaw.speed_measure);
        if ((yaw2.measure <= config.yaw2_min && output < 0) || (yaw2.measure >= config.yaw2_max && output > 0)) {
            output = 0;
            yaw2_speed_pid.track(output);
        }
        motor.yaw2.set_output(output);
    } else {
        motor.yaw2.set_output(0);
    }

    // 大 yaw：自身世界角外环 + 自身世界角速度内环，把小 yaw 拉回中位
    // 自身世界角速度 = 头部世界角速度 − 小 yaw 编码器速度，底盘转动直接反映在反馈里，无需前馈
    if (is_yaw_loop) {
        const float err = plan_angle_error(Angle(yaw1.ref - yaw1.world_measure), config.yaw1_angle_pid_config, config.yaw_max_accel);
        yaw1.speed_ref = yaw1_angle_pid.calculate(dt, err) + yaw_speed;
        yaw1.speed_measure = yaw.speed_measure - motor.yaw2.speed;
        motor.yaw1.set_output(yaw1_speed_pid.calculate(dt, yaw1.speed_ref - yaw1.speed_measure));
    } else {
        motor.yaw1.set_output(0);
    }

    // 俯仰：世界角度外环 + 世界角速度内环（陀螺），叠加重力补偿
    // 编码器越限时截断向外输出，保留回程；PID 跟踪扣除补偿后的实际输出
    if (is_pitch_loop) {
        const float err = plan_angle_error(Angle(pitch.ref - pitch.measure), config.pitch_angle_pid_config, config.pitch_max_accel);
        pitch.speed_ref = pitch_angle_pid.calculate(dt, err) + pitch_speed;
        float output = pitch_speed_pid.calculate(dt, pitch.speed_ref - pitch.speed_measure);
        const float gravity = gravity_output(config.pitch2_gravity, pitch.measure);
        if ((pitch.ecd_measure <= config.pitch_min && output < 0) || (pitch.ecd_measure >= config.pitch_max && output > 0))
            output = 0;
        motor.pitch2.set_output(output + gravity);
        pitch_speed_pid.track(motor.pitch2.torque_ref - gravity);
    } else {
        motor.pitch2.set_output(0);
    }

    // 折叠关节全程失能，清空闭环状态并跟踪实测角度，输出固定为零
    pitch1_position_pid.clear();
    pitch1_speed_pid.clear();
    pitch1.ref = pitch1.measure;
    motor.pitch1.set_output(0);
}

float GimbalFold::plan_angle_error(const float err, const Pid::Config* const pid_config, const float max_accel) {
    if (!std::isfinite(max_accel) || pid_config == nullptr || pid_config->kp <= 0)
        return err;
    if (max_accel <= 0)
        return 0;

    // 超过线性区阈值 e0 = a_max / kp² 后切换为恒减速度平方根曲线，使 kp·e_eq = ±√(2·a_max·(|e| − e0/2))
    const float e0 = max_accel / (pid_config->kp * pid_config->kp);
    const float abs_err = std::fabs(err);
    if (abs_err <= e0)
        return err;
    return std::copysign(std::sqrt(e0 * (2 * abs_err - e0)), err);
}

float GimbalFold::gravity_output(const Gravity& gravity, const float angle) {
    return gravity.torque * std::cos(angle + gravity.phase);
}
