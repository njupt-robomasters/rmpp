#pragma once
#include "spi.h"
#include "unit/include_me.hpp"
#include "unit/UnitFloat.hpp"

class BMI088 {
// 应用层
public:
    struct Config {
        SPI_HandleTypeDef* spi_handle;
        GPIO_TypeDef* accel_cs_port;
        uint16_t accel_cs_pin;
        GPIO_TypeDef* gyro_cs_port;
        uint16_t gyro_cs_pin;
    };

    struct Data {
        UnitFloat<m_ss> accel[3];
        UnitFloat<rad_s> gyro[3];
        UnitFloat<C> temperature;
    };

    BMI088(const Config& config);

    bool Init();
    void Calibrate();
    void Read();

    Data data;
    float gyro_offset[3] = {0};
    float accel_scale = 1.0f;
    float g_norm = 1.0f;
    float temp_when_cali = 40.0f;
// 硬件层
private:
    Config hw;
    void Accel_CS(bool state);
    void Gyro_CS(bool state);
    uint8_t ReadWriteByte(uint8_t txdata);

    void AccelWriteReg(uint8_t reg, uint8_t data);
    uint8_t AccelReadReg(uint8_t reg);
    void AccelReadMulti(uint8_t reg, uint8_t* buf, uint8_t len);

    void GyroWriteReg(uint8_t reg, uint8_t data);
    uint8_t GyroReadReg(uint8_t reg);
    void GyroReadMulti(uint8_t reg, uint8_t* buf, uint8_t len);

    bool AccelInit();
    bool GyroInit();

    static constexpr float ACCEL_6G_SEN = 0.00179443359f;
    static constexpr float GYRO_2000_SEN = 0.001065264436f;
    static constexpr float TEMP_FACTOR = 0.125f;
    static constexpr float TEMP_OFFSET = 23.0f;
};
