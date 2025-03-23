#include "app_variable.hpp"

Settings settings;

IMU imu;

DJ6 dj6;
REFEREE referee;
UI ui;
RV2 rv2;

Chassis chassis(settings.wheel_pid, settings.power_limit_pid, settings.speed_comp_pid);
Gimbal gimbal(imu, settings.pitch_pid, settings.yaw_pid, settings.shoot_pid);
