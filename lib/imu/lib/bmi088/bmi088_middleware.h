#pragma once

#include <stdint.h>

void bmi088_delay_ms(uint32_t ms);

void BMI088_ACCEL_NS_L();

void BMI088_ACCEL_NS_H();

void BMI088_GYRO_NS_L();

void BMI088_GYRO_NS_H();

uint8_t BMI088_read_write_byte(uint8_t reg);