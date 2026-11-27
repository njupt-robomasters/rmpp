#include "gimbal_fold.hpp"
#include <algorithm>
#include <cmath>

GimbalFold::GimbalFold(const Motors& motor, const Config& config)
    : config(config), motor(motor),
      yaw_speed_pid(config.yaw_speed_pid_config), yaw_position_pid(config.yaw_position_pid_config),
      pitch_speed_pid(config.pitch_speed_pid_config), pitch_position_pid(config.pitch_position_pid_config),
      yaw2_speed_pid(config.yaw2.speed_pid_config), yaw2_position_pid(config.yaw2.position_pid_config),
      pitch1_speed_pid(config.pitch1.speed_pid_config), pitch1_position_pid(config.pitch1.position_pid_config) {}

void GimbalFold::set_enable(const bool is_enable) {
    this->is_enable = is_enable;
    motor.yaw1.set_enable(is_enable);
    motor.yaw2.set_enable(is_enable);
    // 折叠关节暂时固定失能，清除旧目标输出
    motor.pitch1.set_enable(false);
    motor.pitch1.set_output(0);
    motor.pitch2.set_enable(is_enable);
}

void GimbalFold::set_mode(const Mode mode) {
    this->mode = mode;
}

void GimbalFold::set_speed(const float yaw_speed, const float pitch_speed) {
    this->yaw_speed = yaw_speed;
    this->pitch_speed = pitch_speed;
}

void GimbalFold::set_angle(const Angle yaw, const Angle pitch) {
    if (mode == ECD_MODE) {
        this->yaw.ecd.ref = yaw;
        this->pitch.ecd.ref = pitch;
    } else if (mode == IMU_MODE) {
        this->yaw.imu.ref = yaw;
        this->pitch.imu.ref = pitch;
    }
    yaw_speed = pitch_speed = 0;
}

void GimbalFold::turn_yaw(const float delta) {
    yaw.ecd.ref += delta;
    yaw.imu.ref += delta;
}

void GimbalFold::set_chassis_wr(const float chassis_wr) {
    this->chassis_wr = std::isfinite(chassis_wr) ? chassis_wr : 0;
}

void GimbalFold::set_yaw_window(const bool is_yaw_window) {
    this->is_yaw_window = is_yaw_window;
}

void GimbalFold::set_pitch1_angle(const Angle angle) {
    pitch1.ref = angle;
}

void GimbalFold::update(const Quaternionf& q, const Vector3f& gyro) {
    const float dt = std::clamp(update_timer.restart(), 0.0001f, 0.01f);

    const auto euler = quaternion_to_euler(q);
    yaw.imu.measure = euler.yaw;
    pitch.imu.measure = euler.pitch;
    motor.pitch1.set_enable(false);
    motor.yaw1.update();
    motor.yaw2.update();
    motor.pitch1.update();
    motor.pitch2.update();

    // 按当前参考系积分目标转速
    if (is_enable) {
        const Angle yaw_delta = yaw_speed * dt;
        const Angle pitch_delta = pitch_speed * dt;
        if (mode == ECD_MODE) {
            yaw.ecd.ref += yaw_delta;
            pitch.ecd.ref += pitch_delta;
        } else if (mode == IMU_MODE) {
            yaw.imu.ref += yaw_delta;
            pitch.imu.ref += pitch_delta;
        }
    }

    // 从电机读取，离线电机保持最后一次反馈
    if (motor.yaw1.is_connect)
        yaw1.measure = motor.yaw1.angle;
    if (motor.yaw2.is_connect)
        yaw2.measure = motor.yaw2.angle;
    if (motor.pitch1.is_connect)
        pitch1.measure = motor.pitch1.angle;
    if (motor.pitch2.is_connect)
        pitch.ecd.measure = motor.pitch2.angle;

    // IMU 角速度经 pitch2 关节旋转到小 yaw 随动系，Z 分量为小 yaw 轴绝对角速度
    const Quaternionf pitch_joint(Eigen::AngleAxisf(pitch.ecd.measure, Vector3f::UnitY()));
    const Vector3f gyro_axis = pitch_joint * gyro;

    // 合并大小 yaw
    yaw.ecd.measure = yaw1.measure + yaw2.measure;

    // 云台轴：大小 yaw 任一在线即可控制 yaw 总目标，俯仰轴独立；失能或离线时目标跟随实测
    const bool can_yaw = is_enable && (motor.yaw1.is_connect || motor.yaw2.is_connect);
    if (!can_yaw || !is_yaw_ready) {
        yaw.ecd.ref = yaw.ecd.measure;
        yaw.imu.ref = yaw.imu.measure;
        yaw_speed_pid.clear();
        yaw_position_pid.clear();
    }
    is_yaw_ready = can_yaw;

    const bool can_pitch2 = is_enable && motor.pitch2.is_connect;
    if (!can_pitch2 || !is_pitch_ready) {
        pitch.ecd.ref = pitch.ecd.measure;
        pitch.imu.ref = pitch.imu.measure;
        pitch_speed_pid.clear();
        pitch_position_pid.clear();
    }
    is_pitch_ready = can_pitch2;

    // 各电机单独判断，离线电机输出零，不影响其他电机
    const bool can_yaw1 = check_motor(motor.yaw1, is_yaw1_ready, yaw_speed_pid, yaw_position_pid);
    const bool can_yaw2 = check_motor(motor.yaw2, is_yaw2_ready, yaw2_speed_pid, yaw2_position_pid);

    // 折叠关节全程失能，清空闭环状态并跟踪实测角度
    pitch1_speed_pid.clear();
    pitch1_position_pid.clear();
    pitch1.ref = pitch1.measure;

    // 用于参考系转换
    yaw.imu_minus_ecd = yaw.imu.measure - yaw.ecd.measure;
    pitch.imu_minus_ecd = pitch.imu.measure - pitch.ecd.measure;

    if (mode == ECD_MODE) {
        // 换算到 imu 参考系
        yaw.imu.ref = yaw.ecd.ref + yaw.imu_minus_ecd;
        pitch.imu.ref = pitch.ecd.ref + pitch.imu_minus_ecd;
    } else if (mode == IMU_MODE) {
        // 换算到 ecd 参考系
        yaw.ecd.ref = yaw.imu.ref - yaw.imu_minus_ecd;
        pitch.ecd.ref = pitch.imu.ref - pitch.imu_minus_ecd;
    }

    // 大小 yaw 分配
    // 窗口模式：识别到目标时大轴保持世界朝向，小轴优先追踪；超出限位的部分交给大轴
    // 普通模式：大轴持续跟随总目标；小轴离线时由大轴承担剩余误差
    const bool can_yaw_window = is_yaw_window && mode == IMU_MODE && can_yaw1 && can_yaw2;
    if (can_yaw_window) {
        if (!is_yaw_window_ready)
            yaw1_world_ref = yaw1.measure + yaw.imu_minus_ecd;
        const float offset = Angle(yaw.imu.ref - yaw1_world_ref);
        const float limited_offset = std::clamp(offset, config.yaw2.min, config.yaw2.max);
        yaw1_world_ref += offset - limited_offset;
        yaw1.ref = yaw1_world_ref - yaw.imu_minus_ecd;
    } else {
        yaw1.ref = yaw.ecd.ref;
        if (mode == IMU_MODE && !motor.yaw2.is_connect)
            yaw1.ref -= yaw2.measure;
    }
    is_yaw_window_ready = can_yaw_window;
    const float yaw2_ref = Angle(yaw.ecd.ref - yaw1.measure);
    yaw2.ref = std::clamp(yaw2_ref, config.yaw2.min, config.yaw2.max);
    const bool is_yaw2_limit = float(yaw2.ref) != yaw2_ref;

    // 俯仰与折叠关节软件限位
    const float pitch2_ref = std::clamp(float(pitch.ecd.ref), config.pitch_min, config.pitch_max);
    const bool is_pitch2_limit = pitch2_ref != float(pitch.ecd.ref);
    pitch.ecd.ref = pitch2_ref;
    pitch1.ref = std::clamp(float(pitch1.ref), config.pitch1.min, config.pitch1.max);

    // 大 yaw 串级 PID；底盘旋转补偿始终叠加，窗口模式下云台转速由小轴承担
    if (can_yaw1) {
        const float speed_ff = -chassis_wr + (can_yaw_window ? 0 : yaw_speed);
        const float speed_ref = yaw_position_pid.calculate(dt, Angle(yaw1.ref - yaw1.measure)) + speed_ff;
        motor.yaw1.set_output(yaw_speed_pid.calculate(dt, speed_ref - motor.yaw1.speed));
        yaw_speed_pid.track(motor.yaw1.torque_ref);
    } else {
        motor.yaw1.set_output(0);
    }

    // 小 yaw 串级 PID：IMU 轴向绝对角速度闭环；未限位时把世界偏航速率修正换算到电机轴并跟踪目标转速，到达限位时随大 yaw 运动
    if (can_yaw2) {
        float yaw2_correction = yaw2_position_pid.calculate(dt, yaw2.ref - yaw2.measure);
        if (mode == IMU_MODE && !is_yaw2_limit) {
            const Vector3f axis_correction = pitch_joint * (q.conjugate() * Vector3f(0.0f, 0.0f, yaw2_correction));
            yaw2_correction = axis_correction.z();
        }
        float speed_ff = yaw_speed;
        if (is_yaw2_limit)
            speed_ff = chassis_wr + motor.yaw1.speed;
        else if (mode == ECD_MODE)
            speed_ff = chassis_wr + yaw_speed;
        const float speed_ref = yaw2_correction + speed_ff;
        motor.yaw2.set_output(yaw2_speed_pid.calculate(dt, speed_ref - gyro_axis.z()));
        yaw2_speed_pid.track(motor.yaw2.current_ref);
    } else {
        motor.yaw2.set_output(0);
    }

    // 俯仰串级 PID 叠加重力补偿，到达限位时取消速度前馈；PID 跟踪扣除补偿后的实际输出
    if (can_pitch2) {
        float speed_ff = 0;
        if (!is_pitch2_limit)
            speed_ff = pitch_speed;
        const float speed_ref = pitch_position_pid.calculate(dt, pitch.ecd.ref - pitch.ecd.measure) + speed_ff;
        const float gravity = gravity_output(config.pitch2_gravity, pitch.imu.measure);
        const float output = pitch_speed_pid.calculate(dt, speed_ref - motor.pitch2.speed) + gravity;
        motor.pitch2.set_output(output);
        pitch_speed_pid.track(motor.pitch2.torque_ref - gravity);
    } else {
        motor.pitch2.set_output(0);
    }

    // pitch1 保留反馈更新与失能帧发送，输出固定为零
    motor.pitch1.set_output(0);

    // 传递软件限位到 imu 参考系；yaw 总目标不回写，窗口模式下视觉角度持续有效
    pitch.imu.ref = pitch.ecd.ref + pitch.imu_minus_ecd;
}

bool GimbalFold::check_motor(const Motor& motor, bool& is_ready, Pid& speed_pid, Pid& position_pid) const {
    const bool can_control = is_enable && motor.is_connect;
    if (can_control && is_ready)
        return true;

    speed_pid.clear();
    position_pid.clear();
    is_ready = can_control;
    return false;
}

float GimbalFold::gravity_output(const Gravity& gravity, const float angle) {
    return gravity.torque * std::cos(angle + gravity.phase);
}
