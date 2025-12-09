#include "dj6.hpp"
#include <algorithm>

DJ6::DJ6() {
    BSP::UART3::RegisterCallback(std::bind(&DJ6::callback,
                                           this,
                                           std::placeholders::_1,
                                           std::placeholders::_2));
}

void DJ6::OnLoop() {
    if (dwt.GetDT() > TIMEOUT) {
        resetData();
    }
}

void DJ6::callback(const uint8_t data[], const uint16_t size) {
    if (size != 25) return;

    parseSBUS(data);

    if (raw.is_connected) {
        dwt.UpdateDT();
        is_connected = true;
        pitch = getStick(raw.CH1);         // 右手垂直
        yaw = -getStick(raw.CH2);          // 右手水平
        y = -getStick(raw.CH3);            // 左手水平
        x = getStick(raw.CH4);             // 左手垂直
        switch_left = getSwitch(raw.CH6);  // 左拨杆 CH6
        switch_right = getSwitch(raw.CH7); // 右拨杆 CH7
    } else {
        resetData();
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
    float ret = ((float)value - 1024.0f) / 660.0f;
    ret = std::clamp(ret, -1.0f, 1.0f);
    if (std::abs(ret) < STICK_DEADLINE) ret = 0;
    return ret;
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
    switch_left = switch_right = ERR;
}