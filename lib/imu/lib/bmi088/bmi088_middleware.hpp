#pragma once

#include <stdint.h>

#include "bsp/spi.hpp"
void bmi088_bind(Spi accel, Spi gyro);

void bmi088_delay_ms(uint32_t ms);
void bmi088_accel_write_reg(uint8_t reg, uint8_t value);
void bmi088_gyro_write_reg(uint8_t reg, uint8_t value);
uint8_t bmi088_accel_read_reg(uint8_t reg);
uint8_t bmi088_gyro_read_reg(uint8_t reg);
void bmi088_accel_read_regs(uint8_t reg, uint8_t* data, uint8_t size);
void bmi088_gyro_read_regs(uint8_t reg, uint8_t* data, uint8_t size);
