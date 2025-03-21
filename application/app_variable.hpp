#pragma once

#include "settings.hpp"
#include "imu.hpp"
#include "dj6.hpp"
#include "referee.hpp"
#include "rv2.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"

extern Settings settings;

extern IMU imu;

extern DJ6 dj6;
extern REFEREE referee;
extern RV2 rv2;

extern Chassis chassis;
extern Gimbal gimbal;
