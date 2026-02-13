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

void Motor::SetSpeed(const UnitFloat<>& speed) {
    this->speed.ref = speed;
}

Angle<> Motor::SetAngle(const Angle<>& angle, const UnitFloat<>& speed_ff) {
    // 电机断联/失能，不断把当前角度设置为目标角度，防止电机一下子飞起来
    if (is_connect == false || is_enable == false) {
        this->angle.ref = this->angle.measure;
        speed.ref = 0 * default_unit;
        is_first_setangle = true;
        return this->angle.ref; // 返回实际设置的角度
    }

    // 断联/失能后第一次设置，先不要应用
    if (is_first_setangle) {
        this->angle.ref = this->angle.measure;
        speed.ref = 0 * default_unit;
        is_first_setangle = false;
        return this->angle.ref; // 返回实际设置的角度
    }

    this->angle.ref = angle; // 应用角度
    speed.ref = speed_ff;    // 应用速度前馈

    // 软件限位
    if (config.is_limit) {
        if (this->angle.ref < config.limit_min) { // 下限位
            this->angle.ref = config.limit_min;
            speed.ref = 0 * default_unit; // 到达限位时，取消速度前馈
        }
        if (this->angle.ref > config.limit_max) { // 上限位
            this->angle.ref = config.limit_max;
            speed.ref = 0 * default_unit; // 到达限位时，取消速度前馈
        }
    }

    return this->angle.ref; // 返回实际设置的角度
}

void Motor::OnLoop() {
    // 电机断联检测
    if (dwt_connect.GetDT() > config.timeout) {
        is_connect = false;
    }

    // 电机断联/失能，清空PID
    if (is_connect == false || is_enable == false) {
        speed_pid.Clear();
        angle_pid.Clear();
        return;
    }

    // PID计算
    switch (config.control_mode) {
        // 开环电流/力矩控制
        case OPEN_LOOP_MODE:
            break;

        // 速度闭环
        case SPEED_MODE: {
            // 计算速度PID
            const UnitFloat speed_err = speed.ref - speed.measure;
            speed_pid.Calculate(speed_err);

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
                // 计算角度PID，使用速度前馈
                const UnitFloat angle_err = angle.ref - angle.measure;
                const UnitFloat speed_err = speed.ref - speed.measure;
                angle_pid.Calculate(angle_err, speed_err);
            } else { // 圆周模式
                // 计算角度PID，使用速度前馈
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

        // 角度-速度闭环（串级PID）
        case ANGLE_SPEED_MODE: {
            // 角度外环
            if (config.is_limit) { // 限位模式
                const UnitFloat angle_err = angle.ref - angle.measure;
                speed.outer = angle_pid.Calculate(angle_err);
            } else { // 圆周模式
                const Angle angle_err = angle.ref - angle.measure;
                speed.outer = angle_pid.Calculate(angle_err);
            }

            // 速度内环，使用速度前馈
            const UnitFloat speed_err = speed.outer + speed.ref - speed.measure;
            speed_pid.Calculate(speed_err);

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
    }
}

void Motor::callback(const raw_t& raw) {
    // 保存到成员变量，方便调试
    this->raw = raw;

    // 更新电机断联检测
    is_connect = true;
    dwt_connect.UpdateDT();;

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
}