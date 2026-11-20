#include "gimbal_classic.hpp"
#include <algorithm>
#include <cmath>

GimbalClassic::GimbalClassic(const Motors& motor, const Config& config)
    : motor(motor),
      config(config),
      yaw_speed_pid(config.yaw_speed_pid_config),
      yaw_angle_pid(config.yaw_angle_pid_config),
      pitch_speed_pid(config.pitch_speed_pid_config),
      pitch_angle_pid(config.pitch_angle_pid_config) {}

void GimbalClassic::set_enable(const bool is_enable) {
    this->is_enable = is_enable;
    motor.yaw.set_enable(is_enable);
    motor.pitch.set_enable(is_enable);
}

void GimbalClassic::set_speed(const float yaw_speed, const float pitch_speed) {
    this->yaw_speed = yaw_speed;
    this->pitch_speed = pitch_speed;
}

void GimbalClassic::set_angle(const Angle yaw, const Angle pitch) {
    this->yaw.angle_ref = yaw;
    this->pitch.angle_ref = pitch;
    yaw_speed = pitch_speed = 0;
}

void GimbalClassic::set_chassis_wr(const float chassis_wr) {
    this->chassis_wr = chassis_wr;
}

void GimbalClassic::update(const Quaternionf& q, const Vector3f& gyro) {
    const float dt = std::clamp(update_timer.restart(), DT_MIN, DT_MAX);

    // 更新反馈：IMU 系经 pitch 关节旋转到 yaw 随动系
    motor.yaw.update();
    motor.pitch.update();
    const auto euler = quaternion_to_euler(q);
    yaw.angle_measure = euler.yaw;
    pitch.angle_measure = euler.pitch;
    const Quaternionf pitch_joint(Eigen::AngleAxisf(motor.pitch.angle, Vector3f::UnitY()));
    const Vector3f gyro_axis = pitch_joint * gyro;
    yaw.speed_measure = gyro_axis.z();
    pitch.speed_measure = gyro_axis.y();

    // 可控状态；yaw 的轴系换算依赖 pitch 关节角
    const bool can_pitch = is_enable && motor.pitch.is_connect;
    const bool can_yaw = can_pitch && motor.yaw.is_connect;

    // yaw 目标与反馈同时扣除底盘转速，统一为电机相对转速；角度外环修正随后叠加
    yaw.speed_ref = yaw_speed;
    pitch.speed_ref = pitch_speed;
    update_angle_loop(q, pitch_joint, can_yaw, can_pitch, dt);

    // yaw 速度内环
    if (can_yaw) {
        motor.yaw.set_output(yaw_speed_pid.calculate(dt, yaw.speed_ref - yaw.speed_measure));
    } else {
        yaw.speed_ref = 0;
        yaw_speed_pid.clear();
        motor.yaw.set_output(0);
    }

    // pitch 速度内环，编码器越限时截断向外输出，保留回程；外环目标已钳在限位内，此处兜底超调
    if (can_pitch) {
        float output = pitch_speed_pid.calculate(dt, pitch.speed_ref - pitch.speed_measure);
        if ((motor.pitch.angle <= config.pitch_min && output < 0) || (motor.pitch.angle >= config.pitch_max && output > 0)) {
            output = 0;
            pitch_speed_pid.track(output);
        }
        motor.pitch.set_output(output);
    } else {
        pitch.speed_ref = 0;
        pitch_speed_pid.clear();
        motor.pitch.set_output(0);
    }
}

void GimbalClassic::update_angle_loop(const Quaternionf& q, const Quaternionf& pitch_joint, const bool can_yaw, const bool can_pitch, const float dt) {
    // 不可控或首帧时目标跟随实测，恢复后从当前姿态开始
    const bool can_angle_loop = has_update;
    has_update = true;

    // 世界角度误差闭环，输出世界 yaw/pitch 角速率修正
    float yaw_correction = 0, pitch_correction = 0;
    if (can_yaw && can_angle_loop) {
        yaw.angle_ref += yaw_speed * dt;
        const float err = plan_angle_error(Angle(yaw.angle_ref - yaw.angle_measure), config.yaw_angle_pid_config, config.yaw_max_accel);
        yaw_correction = yaw_angle_pid.calculate(dt, err);
    } else {
        yaw.angle_ref = yaw.angle_measure;
        yaw_angle_pid.clear();
    }
    if (can_pitch && can_angle_loop) {
        // 编码器限位经 IMU 与编码器偏差换算到世界 pitch，钳制累加目标与 set_angle 目标
        const float offset = pitch.angle_measure - motor.pitch.angle;
        const float min = config.pitch_min + offset, max = config.pitch_max + offset;
        const float ref = pitch.angle_ref + pitch_speed * dt;
        pitch.angle_ref = std::clamp(ref, min, max);

        // pitch 编码器角度达到或越过限位时，仅撤销朝限位外侧的速度前馈
        if ((motor.pitch.angle >= config.pitch_max && pitch_speed > 0) || (motor.pitch.angle <= config.pitch_min && pitch_speed < 0))
            pitch.speed_ref -= pitch_speed;

        const float err = plan_angle_error(Angle(pitch.angle_ref - pitch.angle_measure), config.pitch_angle_pid_config, config.pitch_max_accel);
        pitch_correction = pitch_angle_pid.calculate(dt, err);
    } else {
        pitch.angle_ref = pitch.angle_measure;
        pitch_angle_pid.clear();
    }

    // 修正换算到电机轴后叠加：yaw 绕世界 Z，pitch 绕枪口方位的水平横轴
    const Vector3f world_correction(-std::sin(yaw.angle_measure) * pitch_correction,
                                    std::cos(yaw.angle_measure) * pitch_correction,
                                    yaw_correction);
    const Vector3f axis_correction = pitch_joint * (q.conjugate() * world_correction);
    yaw.speed_ref += axis_correction.z();
    pitch.speed_ref += axis_correction.y();
}

float GimbalClassic::plan_angle_error(const float err, const Pid::Config* const pid_config, const float max_accel) {
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
