#include "chassis.hpp"

void Chassis_Template::SetEnable(bool is_enable) {
    if (this->is_enable == is_enable) return;
    this->is_enable = is_enable;
}

void Chassis_Template::SetSpeed(const UnitFloat<>& vx, const UnitFloat<>& vy, const UnitFloat<>& vr) {
    this->vx.gimbal.ref = vx;
    this->vy.gimbal.ref = vy;
    this->vr.ref = vr;
}

void Chassis_Template::SetGimbalYaw(const Angle<>& gimbal_yaw) {
    this->gimbal_yaw = gimbal_yaw;
}

void Chassis_Template::SetPowerLimit(const UnitFloat<>& power) {
    this->power_limit = power;
}

void Chassis_Template::OnLoop() {
    backwardCalc();
    forwardCalc();
    powerControl();
}