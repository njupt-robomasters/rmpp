#pragma once

#include "app_settings.hpp"
#include "app_status.hpp"

#include "imu.hpp"

#include "dj6.hpp"
#include "referee.hpp"
#include "ui.hpp"
#include "rv2.hpp"

#include "chassis.hpp"
#include "gimbal.hpp"

extern Settings settings;
extern Status status;

extern IMU imu;

extern DJ6 dj6;
extern REFEREE referee;
extern UI ui;
extern RV2 rv2;

extern Chassis chassis;
extern Gimbal gimbal;
