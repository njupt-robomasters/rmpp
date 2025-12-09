#include "motor.hpp"

void Motor::SetEnable(const bool is_enable) {
    this->is_enable = is_enable;
}

void Motor::SetReduction(const float reduction) {
    this->reduction = reduction;
}

void Motor::SetKt(const UnitFloat<>& Kt) {
    this->Kt = Kt;
}

void Motor::SetInvert(const bool is_invert) {
    this->is_invert = is_invert;
}

void Motor::SetOffset(const Angle<>& offset) {
    this->offset = offset;
}

void Motor::SetLimit(const bool is_limit, const Angle<>& limit_min, const Angle<>& limit_max) {
    this->is_limit = is_limit;
    this->limit_min = limit_min;
    this->limit_max = limit_max;
}

void Motor::SetCanSendFreq(const UnitFloat<>& can_send_freq) {
    this->can_send_freq = can_send_freq;
}

void Motor::SetPID(const pid_mode_e pid_mode, const pid_type_e pid_type, PID::param_t* pid_param) {
    this->pid_mode = pid_mode;
    this->pid_type = pid_type;
    pid.SetParam(pid_param);
}

void Motor::SetCurrent(const UnitFloat<>& current) {
    this->current.ref = current;
    this->torque.ref = current * Kt;
}

void Motor::SetTorque(const UnitFloat<>& torque) {
    this->torque.ref = torque;
    this->current.ref = torque / Kt;
}

void Motor::SetSpeed(const UnitFloat<>& speed) {
    this->speed.ref = speed;
}

void Motor::SetAngle(Angle<>&& angle, const UnitFloat<>& speed_ff) {
    // 电机上线后第一次设置角度，角度设置为当前位置（防止电机一下子飞起来）
    if (is_online && !is_online_last) {
        angle = this->angle.ref = this->angle.measure;
    }
    is_online_last = is_online;

    // 软件限位
    if (is_limit) {
        if (angle < limit_min) {
            angle = limit_min; // 达到限位会修改传入的值
        }
        if (angle > limit_max) {
            angle = limit_max;
        }
    }
    this->angle.ref = angle;

    // 速度前馈，用于小陀螺
    this->speed.ref = speed_ff;
}

void Motor::OnLoop() {
    if (dwt1.GetDT() > TIMEOUT) {
        is_online = false;
    }

    if (is_enable && is_online) {
        if (pid_mode == SPEED_MODE) { // 速度模式
            const UnitFloat speed_err = speed.ref - speed.measure;
            current.ref = torque.ref = pid.Calculate(speed_err);
        } else if (pid_mode == ANGLE_MODE) { // 角度模式
            if (is_limit) {                      // 限位模式
                const UnitFloat angle_err = angle.ref - angle.measure;
                const UnitFloat speed_err = speed.ref - speed.measure;
                if (pid_type == CURRENT_TYPE) {
                    SetCurrent(pid.Calculate(angle_err, speed_err));
                } else if (pid_type == TORQUE_TYPE) {
                    SetTorque(pid.Calculate(angle_err, speed_err));
                }
            } else { // 圆周模式
                const Angle angle_err = angle.ref - angle.measure;
                const UnitFloat speed_err = speed.ref - speed.measure;
                if (pid_type == CURRENT_TYPE) {
                    SetCurrent(pid.Calculate(angle_err, speed_err));
                } else if (pid_type == TORQUE_TYPE) {
                    SetTorque(pid.Calculate(angle_err, speed_err));
                }
            }
        }
    } else {
        pid.Clear(); // 清空PID
        current.ref = 0 * default_unit;
        torque.ref = 0 * default_unit;
        speed.ref = 0 * default_unit;
        angle.ref = 0 * default_unit;
    }
}

void Motor::callback() {
    // 电机反向修正
    if (!is_invert) {
        current.measure = current.raw;
        torque.measure = torque.raw;
        speed.measure = speed.raw;
        angle.measure = angle.raw;
    } else {
        current.measure = -current.raw;
        torque.measure = -torque.raw;
        speed.measure = -speed.raw;
        angle.measure = -angle.raw;
    }

    // 软件多圈计数
    if (reduction != 1.0f) { // 减速比!=1时，自动执行软件多圈记数
        angle.measure += Angle(angle.raw - angle.last_raw) / reduction;
        angle.last_raw = angle.raw;
    }

    // 电机角度偏移修正
    angle.measure -= offset;

    dwt1.UpdateDT();
    is_online = true;
}