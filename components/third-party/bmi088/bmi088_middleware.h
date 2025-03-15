#pragma once

#include "spi.h"

extern SPI_HandleTypeDef *BMI088_SPI;

void BMI088_ACCEL_NS_L();

void BMI088_ACCEL_NS_H();

void BMI088_GYRO_NS_L();

void BMI088_GYRO_NS_H();

uint8_t BMI088_read_write_byte(uint8_t reg);
