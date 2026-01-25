#include "FSi6X.hpp"
#include <cstring> // memcpy

FSi6X::FSi6X(const config_t& config) : RC(config) {
    // 注册串口回调
    auto callback = [this](const uint8_t data[], const uint16_t size) {
        this->callback(data, size);
    };
    BSP::UART3::RegisterCallback(callback);
}

void FSi6X::OnLoop() {
    if (dwt_connect.GetDT() > config.connect_timeout) {
        resetData();
    }
}

void FSi6X::callback(const uint8_t data[], const uint16_t size) {
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
    y = -getJoystick(raw.ch0);    // 右手水平
    x = getJoystick(raw.ch1);     // 右手垂直
    pitch = getJoystick(raw.ch2); // 左手垂直
    yaw = -getJoystick(raw.ch3);  // 左手水平
    swa = getSwitch(raw.ch4);
    swb = getSwitch(raw.ch5);
    vra = -getJoystick(raw.ch6);
    swc = getSwitch(raw.ch7);

    // 保护模式
    if (swa == DOWN) {
        is_protect = false;
    } else {
        is_protect = true;
    }
}

UnitFloat<> FSi6X::getJoystick(const uint16_t value) const {
    UnitFloat<> ret = ((float)value - 1024.0f) / 783.0f * ratio; // 240(-784) ~ 1024 ~ 1807(+783)
    ret = unit::clamp(ret, -1 * ratio, 1 * ratio);
    if (unit::abs(ret) < config.joystick_deadline) ret = 0 * ratio;
    return ret;
}

FSi6X::switch_e FSi6X::getSwitch(const uint16_t value) {
    if (value == 240) return UP;
    if (value == 1024) return MID;
    if (value == 1807) return DOWN;
    return ERR;
}

void FSi6X::resetData() {
    is_connect = false;
    is_protect = true;
    x = y = pitch = yaw = 0 * ratio;
    vra = vrb = 0 * ratio;
    swb = swc = ERR;
}