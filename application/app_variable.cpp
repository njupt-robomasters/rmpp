#include "app_variable.hpp"

Settings settings;

IMU imu;

DJ6 dj6;
REFEREE referee;
RV2 rv2;
UI ui;
Chassis chassis(settings.wheel_pid, settings.chassis_pid);
Gimbal gimbal(imu, settings.pitch_pid, settings.yaw_pid, settings.shoot_pid);
