#include "DJ6.hpp"
#include <cstring>

DJ6::DJ6(const config_t& config) : RC(config) {
    // 注册串口回调
    auto callback = [this](const uint8_t data[], const uint16_t size) {
        this->callback(data, size);
    };
    BSP::UART3::RegisterCallback(callback);
}

void DJ6::OnLoop() {
    // 断联检测
    if (dwt_connect.GetDT() > config.connect_timeout) {
        resetData();
    }
}

void DJ6::callback(const uint8_t data[], const uint16_t size) {
    // 长度检查
    if (size != 25) return;

    // 帧头桢尾检查
    if (data[0] != 0x0F || data[24] != 0x00) return;

    // 解析原始通道值
    std::memcpy(&raw, data, sizeof(raw));

    // 断联标志位检查
    if (raw.is_disconnect) {
        resetData();
        return;
    }

    // 复位断联检测
    is_connect = true;
    dwt_connect.UpdateDT();

    // 解析摇杆和拨杆
    y = -getJoystick(raw.ch0);         // 右手水平
    x = getJoystick(raw.ch1);          // 右手垂直
    pitch = getJoystick(raw.ch2);      // 左手垂直
    yaw = -getJoystick(raw.ch3);       // 左手水平
    wheel = getJoystick(raw.ch4);      // 背面摇杆（-0.86 ~ +0.35）
    switch_left = getSwitch(raw.ch5);  // 左拨杆
    switch_right = getSwitch(raw.ch6); // 右拨杆

    // 保护模式
    // 左拨杆拨下解除保护模式，其他情况为保护模式
    if (switch_left == DOWN) {
        is_protect = false;
    } else {
        is_protect = true;
    }
}

UnitFloat<> DJ6::getJoystick(const uint16_t value) const {
    UnitFloat<ratio> ret = ((float)value - 1024.0f) / 660.0f * ratio;
    ret = unit::clamp(ret, -1 * ratio, 1 * ratio);
    if (unit::abs(ret) < config.joystick_deadline) ret = 0 * ratio;
    return ret;
}

DJ6::switch_e DJ6::getSwitch(const uint16_t value) {
    if (value == 511) return DOWN;
    if (value == 1024) return MID;
    if (value == 1541) return UP;
    return ERR;
}

void DJ6::resetData() {
    is_connect = false;
    is_protect = true;
    x = y = pitch = yaw = 0 * ratio;
    wheel = 0 * ratio;
    switch_left = switch_right = ERR;
}