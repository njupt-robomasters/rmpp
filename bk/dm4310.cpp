#include "dm4310.hpp"
#include "bsp_can.hpp"
#include "bsp_dwt.h"

float DM4310::uint_to_float(const int x_int, const float x_min, const float x_max, const int bits) {
    const float span = x_max - x_min;
    const float offset = x_min;
    return static_cast<float>(x_int) * span / static_cast<float>((1 << bits) - 1) + offset;
}

uint16_t DM4310::float_to_uint(const float x, const float x_min, const float x_max, const int bits) {
    const float span = x_max - x_min;
    const float offset = x_min;
    return static_cast<uint16_t>((x - offset) * static_cast<float>((1 << bits) - 1) / span);
}

DM4310::DM4310(const uint8_t id, PID::param_t &pid_param) : motor_id(id), pid(pid_param.SetDefaultMax(10)) {
}

void DM4310::ParseCAN(const uint8_t data[8]) {
    const uint8_t id = data[0] & 0x0F;
    if (id != motor_id)
        return;

    const float dt = BSP_DWT_GetDT(&dwt_cnt);
    can_recv_freq = 1 / dt;

    motor_data.id = data[0] & 0x0F;
    motor_data.err = (data[0] & 0xF0) >> 4;
    motor_data.pos = (data[1] << 8) | data[2];
    motor_data.vel = (data[3] << 4) | (data[4] >> 4);
    motor_data.torque = ((data[4] & 0x0F) << 8) | data[5];
    motor_data.t_mos = data[6];
    motor_data.t_rotor = data[7];

    measure.angle.rad = uint_to_float(motor_data.pos, -P_MAX, P_MAX, 16);
    measure.angle.degree = norm_angle(measure.angle.degree); // 归一化角度
    measure.speed.rps = uint_to_float(motor_data.vel, -V_MAX, V_MAX, 12);
    measure.torque = uint_to_float(motor_data.torque, -T_MAX, T_MAX, 12);

    is_ready = true;
}

void DM4310::ResetReady() {
    this->is_ready = false;
}

[[nodiscard]] bool DM4310::IsReady() const {
    return this->is_ready;
}

void DM4310::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable)
        return;

    this->is_enable = is_enable;

    pid.Clear(); // 清空PID

    if (not is_enable) {
        // 失能力矩置0
        ref.torque = 0;
    }
}

void DM4310::SetAngle(const AngleFloat &angle) {
    ref.angle = angle;
}

void DM4310::SetAngle(const AngleFloat &angle, const Speed &speed) {
    ref.angle = angle;
    ref.speed = speed;
}

void DM4310::AddAngle(const AngleFloat &angle) {
    ref.angle.degree = norm_angle(ref.angle.degree + angle.degree);
}

void DM4310::Update() {
    if (is_enable) {
        const float angle_err = calc_angle_err(ref.angle.degree, measure.angle.degree);
        const float speed_err = ref.speed.aps - measure.speed.aps;
        ref.torque = pid.CalcPosition(angle_err, speed_err);
    }
}

void DM4310::SendCANEnable() {
    // 使能
    uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
    BSP_CAN_Transmit(motor_id, data, 8);
    // // 失能
    // uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
    // BSP_CAN_Transmit(motor_id, data, 8);
}

void DM4310::ClearErr() {
    // 使能
    uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb};
    BSP_CAN_Transmit(motor_id, data, 8);
    // // 失能
    // uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
    // BSP_CAN_Transmit(motor_id, data, 8);
}

void DM4310::SendCANCmd() {
    if (send_enable_cnt++ % 100 == 0) {
        // 每100次调用重新发送使能
        SendCANEnable();
    } else {
        const uint16_t pos_value = 0;
        const uint16_t vel_value = 0;
        const uint16_t kp_value = 0;
        const uint16_t kd_value = 0;
        const uint16_t tff_value = float_to_uint(ref.torque, -T_MAX, T_MAX, 12);

        uint8_t data[8];
        data[0] = pos_value >> 8;
        data[1] = pos_value;
        data[2] = vel_value >> 4;
        data[3] = ((vel_value & 0x0F) << 4) | (kp_value >> 8);
        data[4] = kp_value;
        data[5] = kd_value >> 4;
        data[6] = ((kd_value & 0x0F) << 4) | (tff_value >> 8);
        data[7] = tff_value;

        BSP_CAN_Transmit(motor_id, data, 8);
    }
}
