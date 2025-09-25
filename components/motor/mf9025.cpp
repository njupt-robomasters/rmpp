#include "mf9025.hpp"

#include "app.hpp"
#include "app.hpp"
#include "app.hpp"
#include "app.hpp"
#include "bsp.hpp"

MF9025::MF9025(const uint8_t can_port, const uint8_t motor_id) :
    can_port(can_port), motor_id(motor_id) {
    BSP::CAN::RegisterCallback(std::bind(&MF9025::callback, this,
                                         std::placeholders::_1,
                                         std::placeholders::_2,
                                         std::placeholders::_3,
                                         std::placeholders::_4));
}

void MF9025::callback(const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    if (port != can_port) return;
    if (id != motor_id + 0x140) return;
    if (dlc != 8) return;

    is_ready = true;

    if (data[0] != 0x9C && data[0] != 0xA1) return;

    // 维护dt
    const float dt = dwt.GetDT();

    // 记录CAN反馈报文频率
    can_feedback_freq = 1 / dt;

    // 解析CAN报文
    temperate_motor = data[1];
    current.raw = data[2] | (data[3] << 8);
    speed.raw = data[4] | (data[5] << 8);
    angle.raw = data[6] | (data[7] << 8);

    if (!is_invert) {
        current.measure = (float)current.raw / 2048.0f * 16.5f * A;
        angle.measure = (float)angle.raw / 65535.0f * 360.0f * deg;
        speed.measure = (float)speed.raw * deg_s;
    } else {
        current.measure = -(float)current.raw / 2048.0f * 16.5f * A;
        angle.measure = -(float)angle.raw / 65535.0f * 360.0f * deg;
        speed.measure = -(float)speed.raw * deg_s;
    }
}

void MF9025::sendCANEnable() {
    uint8_t data[8] = {0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    BSP::CAN::TransmitStd(can_port, motor_id + 0x140, data, 8);
}

void MF9025::sendCANDisable() {
    uint8_t data[8] = {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    BSP::CAN::TransmitStd(can_port, motor_id + 0x140, data, 8);
}

void MF9025::sendCANReadState() {
    uint8_t data[8] = {0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    BSP::CAN::TransmitStd(can_port, motor_id + 0x140, data, 8);
}

void MF9025::SetInvert(const bool is_invert) {
    this->is_invert = is_invert;
}

void MF9025::SetPIDParam(PID::param_t* pid_param) {
    pid.SetParam(pid_param);
}

void MF9025::WaitReady() {
    this->is_ready = false;
    while (this->is_ready == false) {
        sendCANDisable(); // 电机是一问一答
        BSP::OS::Delay(1);
    }
}

void MF9025::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable) return;
    this->is_enable = is_enable;

    pid.Clear();
    current.ref = 0;
}

void MF9025::SetAngle(Angle<> angle, UnitFloat<> speed) {
    this->angle.ref = angle;
    this->speed.ref = speed;
}

void MF9025::Update() {
    can_cmd_cnt++;
    if (is_enable) {
        const Angle angle_err = angle.ref - angle.measure;
        const float speed_err = speed.ref - speed.measure;
        current.ref = pid.CalcMIT(angle_err, speed_err);
        sendCANCmd();
    } else {
        if (can_cmd_cnt % 10 == 0) {
            sendCANDisable();
        } else if (can_cmd_cnt % 10 == 1) {
            sendCANReadState();
        }
    }
}

void MF9025::sendCANCmd() {
    if (can_cmd_cnt % 100 == 0) { // 每100次调用重新发送使能
        sendCANEnable();
    } else if (can_cmd_cnt % 10 == 0) {
        int16_t current_cmd;
        if (!is_invert) {
            current_cmd = (int16_t)(current.ref / 16.5f * 2048.0f);
        } else {
            current_cmd = (int16_t)(-current.ref / 16.5f * 2048.0f);
        }

        uint8_t data[8];
        data[0] = 0xA1; // 力矩模式
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
        data[4] = current_cmd;
        data[5] = current_cmd >> 8;
        data[6] = 0;
        data[7] = 0;
        BSP::CAN::TransmitStd(can_port, motor_id + 0x140, data);
    }
}
