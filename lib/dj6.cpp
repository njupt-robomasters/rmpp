#include "DJ6.hpp"
#include <algorithm>
#include <cmath>
#include "bsp/bsp.hpp"

DJ6::DJ6() : is_connected(TIMEOUT, false),
             x(TIMEOUT, 0), y(TIMEOUT, 0), pitch(TIMEOUT, 0), yaw(TIMEOUT, 0),
             left_switch(TIMEOUT, ERR), right_switch(TIMEOUT, ERR) {
    auto callback = std::bind(&DJ6::callback,
                              this,
                              std::placeholders::_1,
                              std::placeholders::_2);
    BSP::UART3::RegisterCallback(callback);
}

void DJ6::callback(const uint8_t data[], const uint16_t size) {
    if (size != 25) return;

    parseSBUS(data);

    is_connected = raw.is_connected;
    if (is_connected.value()) {
        last_receive_time = BSP::Dwt::GetTime();
        y = -getStick(raw.CH1);            // 右手水平
        x = getStick(raw.CH2);             // 右手垂直
        pitch = getStick(raw.CH3);         // 左手垂直
        yaw = -getStick(raw.CH4);          // 左手水平
        left_switch = getSwitch(raw.CH6);  // 左拨杆 CH6
        right_switch = getSwitch(raw.CH7); // 右拨杆 CH7
    } else {
        x = y = pitch = yaw = 0;
        left_switch = right_switch = ERR;
    }
}

void DJ6::parseSBUS(const uint8_t* data) {
    if (data[23] == 0) {
        raw.is_connected = true;
        raw.CH1 = ((uint16_t)data[1] >> 0 | (uint16_t)data[2] << 8) & 0x07FF;
        raw.CH2 = ((uint16_t)data[2] >> 3 | (uint16_t)data[3] << 5) & 0x07FF;
        raw.CH3 = ((uint16_t)data[3] >> 6 | (uint16_t)data[4] << 2 | (uint16_t)data[5] << 10) & 0x07FF;
        raw.CH4 = ((uint16_t)data[5] >> 1 | (uint16_t)data[6] << 7) & 0x07FF;
        raw.CH5 = ((uint16_t)data[6] >> 4 | (uint16_t)data[7] << 4) & 0x07FF;
        raw.CH6 = ((uint16_t)data[7] >> 7 | (uint16_t)data[8] << 1 | (uint16_t)data[9] << 9) & 0x07FF;
        raw.CH7 = ((uint16_t)data[9] >> 2 | (uint16_t)data[10] << 6) & 0x07FF;
        raw.CH8 = ((uint16_t)data[10] >> 5 | (uint16_t)data[11] << 3) & 0x07FF;
        raw.CH9 = ((uint16_t)data[12] << 0 | (uint16_t)data[13] << 8) & 0x07FF;
        raw.CH10 = ((uint16_t)data[13] >> 3 | (uint16_t)data[14] << 5) & 0x07FF;
        raw.CH11 = ((uint16_t)data[14] >> 6 | (uint16_t)data[15] << 2 | (uint16_t)data[16] << 10) & 0x07FF;
        raw.CH12 = ((uint16_t)data[16] >> 1 | (uint16_t)data[17] << 7) & 0x07FF;
        raw.CH13 = ((uint16_t)data[17] >> 4 | (uint16_t)data[18] << 4) & 0x07FF;
        raw.CH14 = ((uint16_t)data[18] >> 7 | (uint16_t)data[19] << 1 | (uint16_t)data[20] << 9) & 0x07FF;
        raw.CH15 = ((uint16_t)data[20] >> 2 | (uint16_t)data[21] << 6) & 0x07FF;
        raw.CH16 = ((uint16_t)data[21] >> 5 | (uint16_t)data[22] << 3) & 0x07FF;
    } else {
        raw.is_connected = false;
    }
}

float DJ6::getStick(const uint16_t value) {
    float result = ((float)value - 1024.0f) / 660.0f;
    result = std::clamp(result, -1.0f, 1.0f);
    if (fabsf(result) < STICK_DEADLINE) result = 0;
    return result;
}

DJ6::switch_e DJ6::getSwitch(const uint16_t value) {
    if (value < 694) {
        return DOWN;
    } else if (value < 1354) {
        return MID;
    } else {
        return UP;
    }
}

void DJ6::resetData() {
    is_connected = false;
    x = y = pitch = yaw = 0;
    left_switch = right_switch = ERR;
}