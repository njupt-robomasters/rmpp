#include "DJ6.hpp"

// 遥杆值归一化到 -1~0~1
float DJ6::norm_xy(const uint16_t val) {
    // 原始值范围：min~mid~max => 364~1024~1684
    const int16_t tmp = static_cast<int16_t>(val) - 1024;
    if (tmp > -RC_XY_MIN && tmp < RC_XY_MIN) {
        return 0;
    } else if (tmp > RC_XY_MAX) {
        return 1;
    } else if (tmp < -RC_XY_MAX) {
        return -1;
    } else {
        return (float)tmp / RC_XY_MAX;
    }
}

// 获取拨杆值
DJ6::switch_t DJ6::get_switch(const uint16_t val) {
    if (val < 694) {
        return DOWN;
    } else if (val < 1354) {
        return MID;
    } else {
        return UP;
    }
}

void DJ6::parseDBUS(const uint8_t *data) {
    if (data[23] == 0) {
        channel.is_connected = true;
        channel.CH1 = ((uint16_t) data[1] >> 0 | (uint16_t) data[2] << 8) & 0x07FF;
        channel.CH2 = ((uint16_t) data[2] >> 3 | (uint16_t) data[3] << 5) & 0x07FF;
        channel.CH3 = ((uint16_t) data[3] >> 6 | (uint16_t) data[4] << 2 | (uint16_t) data[5] << 10) & 0x07FF;
        channel.CH4 = ((uint16_t) data[5] >> 1 | (uint16_t) data[6] << 7) & 0x07FF;
        channel.CH5 = ((uint16_t) data[6] >> 4 | (uint16_t) data[7] << 4) & 0x07FF;
        channel.CH6 = ((uint16_t) data[7] >> 7 | (uint16_t) data[8] << 1 | (uint16_t) data[9] << 9) & 0x07FF;
        channel.CH7 = ((uint16_t) data[9] >> 2 | (uint16_t) data[10] << 6) & 0x07FF;
        channel.CH8 = ((uint16_t) data[10] >> 5 | (uint16_t) data[11] << 3) & 0x07FF;
        channel.CH9 = ((uint16_t) data[12] << 0 | (uint16_t) data[13] << 8) & 0x07FF;
        channel.CH10 = ((uint16_t) data[13] >> 3 | (uint16_t) data[14] << 5) & 0x07FF;
        channel.CH11 = ((uint16_t) data[14] >> 6 | (uint16_t) data[15] << 2 | (uint16_t) data[16] << 10) & 0x07FF;
        channel.CH12 = ((uint16_t) data[16] >> 1 | (uint16_t) data[17] << 7) & 0x07FF;
        channel.CH13 = ((uint16_t) data[17] >> 4 | (uint16_t) data[18] << 4) & 0x07FF;
        channel.CH14 = ((uint16_t) data[18] >> 7 | (uint16_t) data[19] << 1 | (uint16_t) data[20] << 9) & 0x07FF;
        channel.CH15 = ((uint16_t) data[20] >> 2 | (uint16_t) data[21] << 6) & 0x07FF;
        channel.CH16 = ((uint16_t) data[21] >> 5 | (uint16_t) data[22] << 3) & 0x07FF;
    } else {
        channel.is_connected = false;
    }
}

void DJ6::ParseData(const uint8_t *data, uint16_t size) {
    if (size != DBUS_FRAME_SIZE)
        return;

    parseDBUS(data);

    is_connected = channel.is_connected;
    if (channel.is_connected) {
        yaw = norm_xy(channel.CH1); // 右手水平
        pitch = norm_xy(channel.CH2); // 右手垂直
        y = norm_xy(channel.CH3); // 左手垂直
        x = norm_xy(channel.CH4); // 左手水平
        left_switch = get_switch(channel.CH6); // 左拨杆 CH6
        right_switch = get_switch(channel.CH7); // 右拨杆 CH7
    }
}
