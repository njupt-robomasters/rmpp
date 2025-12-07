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

void Motor::SetPIDParam(pid_output_type_e pid_output_type, PID::param_t* pid_param) {
    this->pid_output_type = pid_output_type;
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
    this->control_mode = SPEED_MODE;
    this->speed.ref = speed;
}

void Motor::SetAngle(Angle<>&& angle, const UnitFloat<>& speed_ff) {
    this->control_mode = ANGLE_MODE;

    // 电机上线后第一次设置角度，角度设置为当前位置（防止电机一下子飞起来）
    if (is_ready && !is_ready_last) {
        angle = this->angle.ref = this->angle.measure;
    }
    is_ready_last = is_ready;

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
    if (is_enable && is_ready) {
        if (control_mode == SPEED_MODE) { // 速度模式
            const UnitFloat speed_err = speed.ref - speed.measure;
            current.ref = torque.ref = pid.Calculate(speed_err);
        } else if (control_mode == ANGLE_MODE) { // 角度模式
            if (is_limit) {                      // 限位模式
                const UnitFloat angle_err = angle.ref - angle.measure;
                const UnitFloat speed_err = speed.ref - speed.measure;
                if (pid_output_type == PID_OUTPUT_CURRENT) {
                    SetCurrent(pid.Calculate(angle_err, speed_err));
                } else if (pid_output_type == PID_OUTPUT_TORQUE) {
                    SetTorque(pid.Calculate(angle_err, speed_err));
                }
            } else { // 圆周模式
                const Angle angle_err = angle.ref - angle.measure;
                const UnitFloat speed_err = speed.ref - speed.measure;
                if (pid_output_type == PID_OUTPUT_CURRENT) {
                    SetCurrent(pid.Calculate(angle_err, speed_err));
                } else if (pid_output_type == PID_OUTPUT_TORQUE) {
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

    // 电机就绪标识
    is_ready = true;

    // 记录CAN回调频率
    dwt.GetDT();
}
