#pragma once

#include "bsp/pwm.hpp"
#include "bsp/spi.hpp"

#include "imu/imu.hpp"
#include "math/unit.hpp"

inline Imu imu({
    .hardware = {.accel = Spi(1, 1), .gyro = Spi(1, 2), .heater = Pwm(10, 1)},
    .adjust = {.gyro_offset = {0.00316276052f, -0.00411193585f, -0.00242328411f}, .gravity = 9.79961872f},
});
