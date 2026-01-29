#include "Motor.hpp"

Motor::Motor(const config_t& config) : config(config), speed_pid(config.speed_pid_config), angle_pid(config.angle_pid_config) {}

void Motor::SetEnable(bool is_enable) {
    this->is_enable = is_enable;
}

void Motor::SetCurrent(const UnitFloat<>& current) {
    this->current.ref = current;
    torque.ref = current * config.Kt;
}

void Motor::SetTorque(const UnitFloat<>& torque) {
    this->torque.ref = torque;
    current.ref = torque / config.Kt;
}

Angle<> Motor::SetAngle(const Angle<>& angle, const UnitFloat<>& speed_ff) {
    // 电机掉线或失能，不断把目标角度设置为当前位置（防止电机一下子飞起来）
    if (is_connect == false || is_enable == false) {
        this->angle.ref = this->angle.measure;
    } else {
        this->angle.ref = angle;
    }

    // 速度前馈
    speed.ref = speed_ff;

    // 软件限位
    if (config.is_limit) {
        if (this->angle.ref < config.limit_min) { // 下限位
            this->angle.ref = config.limit_min;
            speed.ref = 0 * default_unit; // 到达限位速度时，取消速度前馈
        }
        if (this->angle.ref > config.limit_max) { // 上限位
            this->angle.ref = config.limit_max;
            speed.ref = 0 * default_unit; // 到达限位速度时，取消速度前馈
        }
    }

    return this->angle.ref; // 返回实际设置的值
}

void Motor::SetSpeed(const UnitFloat<>& speed) {
    this->speed.ref = speed;
}

void Motor::OnLoop() {
    // 电机断联检测
    if (dwt.connect.GetDT() > config.disconnect_timeout) {
        is_connect = false;
    }

    // 电机断联/失能，复位状态
    if (is_connect == false || is_enable == false) {
        // 清空PID
        speed_pid.Clear();
        angle_pid.Clear();
        return;
    }

    // PID计算
    switch (config.control_mode) {
        case OPEN_LOOP_MODE: // 开环电流/力矩控制
            break;

        case SPEED_MODE: {                                         // 速度闭环
            const UnitFloat speed_err = speed.ref - speed.measure; // 计算误差
            speed_pid.Calculate(speed_err);                        // 计算PID

            // 根据PID输出类型设置电流/力矩
            switch (config.pid_out_type) {
                case CURRENT_OUTPUT:
                    SetCurrent(speed_pid.out);
                    break;
                case TORQUE_OUTPUT:
                    SetTorque(speed_pid.out);
                    break;
            }
        }
        break;

        // 角度闭环
        case ANGLE_MODE: {
            if (config.is_limit) { // 限位模式
                const UnitFloat angle_err = angle.ref - angle.measure;
                const UnitFloat speed_err = speed.ref - speed.measure;
                angle_pid.Calculate(angle_err, speed_err);
            } else { // 圆周模式
                const Angle angle_err = angle.ref - angle.measure;
                const UnitFloat speed_err = speed.ref - speed.measure;
                angle_pid.Calculate(angle_err, speed_err);
            }

            // 根据PID输出类型设置电流/力矩
            switch (config.pid_out_type) {
                case CURRENT_OUTPUT:
                    SetCurrent(angle_pid.out);
                    break;
                case TORQUE_OUTPUT:
                    SetTorque(angle_pid.out);
                    break;
            }
        }
        break;

        // 角度-角度闭环（串级PID）
        case ANGLE_SPEED_MODE: {
            // 角度外环
            if (config.is_limit) { // 限位模式
                const UnitFloat angle_err = angle.ref - angle.measure;
                speed.outer = angle_pid.Calculate(angle_err);
            } else { // 圆周模式
                const Angle angle_err = angle.ref - angle.measure;
                speed.outer = angle_pid.Calculate(angle_err);
            }

            // 速度内环
            const UnitFloat speed_err = speed.outer + speed.ref - speed.measure; // 计算误差
            speed_pid.Calculate(speed_err);                                      // 计算PID
            switch (config.pid_out_type) {
                case CURRENT_OUTPUT:
                    SetCurrent(speed_pid.out);
                    break;
                case TORQUE_OUTPUT:
                    SetTorque(speed_pid.out);
                    break;
            }
        }
        break;
    }
}

void Motor::callback(const raw_t& raw) {
    // 保存到成员变量，方便调试
    this->raw = raw;

    // 电机断联检测
    is_connect = true;
    dwt.connect.UpdateDT();;

    // 电机修正
    Angle correct_angle;
    if (!config.is_invert) {
        current.measure = raw.current;
        torque.measure = raw.torque;
        speed.measure = raw.speed;
        correct_angle = raw.angle - config.offset;
    } else {
        current.measure = -raw.current;
        torque.measure = -raw.torque;
        speed.measure = -raw.speed;
        correct_angle = -(raw.angle - config.offset);
    }

    // 减速比!=1时，自动执行软件多圈记数
    if (config.reduction != 1.0f) {
        angle.measure += Angle(correct_angle - last_angle) / config.reduction;
        last_angle = correct_angle;
    } else {
        angle.measure = correct_angle;
    }

    // 功率估计
    power.heat = 3 * config.R * unit::square(current.measure);
    power.mechanical = torque.measure * speed.measure;
    power.total = power.heat + power.mechanical;
}