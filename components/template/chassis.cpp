#include "chassis.hpp"

void Chassis_Template::SetGimbalYaw(const Angle<>& gimbal_yaw) {
    this->gimbal_yaw = gimbal_yaw;
}

void Chassis_Template::SetSpeed(const UnitFloat<>& vx, const UnitFloat<>& vy, const UnitFloat<>& vr) {
    this->vx.gimbal.ref = vx;
    this->vy.gimbal.ref = vy;
    this->vr.ref = vr;
}

void Chassis_Template::SetPowerLimit(const UnitFloat<>& power) {
    this->power_limit = power;
}
