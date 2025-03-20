#include "app_variable.hpp"

Settings settings;
DJ6 dj6;
Chassis chassis(settings.chassis_mit);
IMU imu;
Gimbal gimbal(imu, settings.pitch_mit, settings.yaw_mit, settings.shoot_mit);
RV2 rv2;
REFEREE referee;
