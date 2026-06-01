#include "bmi088.hpp"
#include "bsp/bsp_dwt.h"

namespace {
    constexpr uint8_t ACC_CHIP_ID = 0x00;
    constexpr uint8_t ACC_CHIP_ID_VALUE = 0x1E;
    constexpr uint8_t ACC_SOFTRESET = 0x7E;
    constexpr uint8_t ACC_XOUT_L = 0x12;
    constexpr uint8_t TEMP_M = 0x22;

    constexpr uint8_t GYRO_CHIP_ID = 0x00;
    constexpr uint8_t GYRO_CHIP_ID_VALUE = 0x0F;
    constexpr uint8_t GYRO_SOFTRESET = 0x14;

    constexpr uint8_t acc_init_cmds[][2] = {
        {0x7D, 0x04}, {0x7C, 0x00}, {0x40, 0x20 | 0x0B | 0x80},
        {0x41, 0x01}, {0x53, 0x08}, {0x58, 0x04}
    };
    constexpr uint8_t gyro_init_cmds[][2] = {
        {0x0F, 0x00}, {0x10, 0x01 | 0x80}, {0x11, 0x00},
        {0x15, 0x80}, {0x16, 0x00}, {0x18, 0x01}
    };
}

BMI088::BMI088(const Config& config) : hw(config) {}

bool BMI088::Init() {
    bool acc_ok = false, gyr_ok = false;
    for (int i = 0; i < 5; i++) { if (AccelInit()) { acc_ok = true; break; } HAL_Delay(10); }
    for (int i = 0; i < 5; i++) { if (GyroInit()) { gyr_ok = true; break; } HAL_Delay(10); }
    return acc_ok && gyr_ok;
}

void BMI088::Read() {
    uint8_t buf[8] = {0};
    int16_t raw_temp;

    AccelReadMulti(ACC_XOUT_L, buf, 6);
    raw_temp = (int16_t)((buf[1] << 8) | buf[0]);
    float ax = (float)raw_temp * ACCEL_6G_SEN * accel_scale;
    raw_temp = (int16_t)((buf[3] << 8) | buf[2]);
    float ay = (float)raw_temp * ACCEL_6G_SEN * accel_scale;
    raw_temp = (int16_t)((buf[5] << 8) | buf[4]);
    float az = (float)raw_temp * ACCEL_6G_SEN * accel_scale;

    data.accel[0] = ax * m_ss;
    data.accel[1] = ay * m_ss;
    data.accel[2] = az * m_ss;

    GyroReadMulti(GYRO_CHIP_ID, buf, 8);
    if (buf[0] == GYRO_CHIP_ID_VALUE) {
        raw_temp = (int16_t)((buf[3] << 8) | buf[2]);
        float gx = (float)raw_temp * GYRO_2000_SEN - gyro_offset[0];
        raw_temp = (int16_t)((buf[5] << 8) | buf[4]);
        float gy = (float)raw_temp * GYRO_2000_SEN - gyro_offset[1];
        raw_temp = (int16_t)((buf[7] << 8) | buf[6]);
        float gz = (float)raw_temp * GYRO_2000_SEN - gyro_offset[2];



        data.gyro[0] = gx * rad_s;
        data.gyro[1] = gy * rad_s;
        data.gyro[2] = gz * rad_s;
    }

    AccelReadMulti(TEMP_M, buf, 2);
    raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
    if (raw_temp > 1023) raw_temp -= 2048;
    data.temperature = ((float)raw_temp * TEMP_FACTOR + TEMP_OFFSET) * C;
}

// 标定
void BMI088::Calibrate() {
    uint16_t CaliTimes = 6000;
    uint8_t buf[8] = {0};
    int16_t raw_temp;
    float gyroMax[3], gyroMin[3];
    float gNormTemp, gNormMax, gNormMin;
    float gNormDiff = 0.0f;
    float gyroDiff[3] = {0.0f};

    uint32_t tick = 0;
    BSP_DWT_UpdateDT(&tick);

    do {
        if (BSP_DWT_GetDT(tick) > 10.0f) {
            // 校准超时
            gyro_offset[0] = 0;
            gyro_offset[1] = 0;
            gyro_offset[2] = 0;
            g_norm = 1.0f;
            temp_when_cali = 40.0f;
            break;
        }

        BSP_DWT_Delay(0.005f);
        g_norm = 0;
        gyro_offset[0] = 0;
        gyro_offset[1] = 0;
        gyro_offset[2] = 0;

        for (uint16_t i = 0; i < CaliTimes; i++) {
            AccelReadMulti(ACC_XOUT_L, buf, 6);
            raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
            float acc_x = (float)raw_temp * ACCEL_6G_SEN;
            raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            float acc_y = (float)raw_temp * ACCEL_6G_SEN;
            raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            float acc_z = (float)raw_temp * ACCEL_6G_SEN;

            gNormTemp = std::sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);
            g_norm += gNormTemp;

            GyroReadMulti(GYRO_CHIP_ID, buf, 8);
            float g_x = 0, g_y = 0, g_z = 0;
            if (buf[0] == GYRO_CHIP_ID_VALUE) {
                raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
                g_x = (float)raw_temp * GYRO_2000_SEN;
                gyro_offset[0] += g_x;
                raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
                g_y = (float)raw_temp * GYRO_2000_SEN;
                gyro_offset[1] += g_y;
                raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
                g_z = (float)raw_temp * GYRO_2000_SEN;
                gyro_offset[2] += g_z;
            }

            // 记录数据极差
            if (i == 0) {
                gNormMax = gNormTemp;
                gNormMin = gNormTemp;
                gyroMax[0] = g_x; gyroMin[0] = g_x;
                gyroMax[1] = g_y; gyroMin[1] = g_y;
                gyroMax[2] = g_z; gyroMin[2] = g_z;
            } else {
                if (gNormTemp > gNormMax) gNormMax = gNormTemp;
                if (gNormTemp < gNormMin) gNormMin = gNormTemp;

                if (g_x > gyroMax[0]) gyroMax[0] = g_x;
                if (g_x < gyroMin[0]) gyroMin[0] = g_x;
                if (g_y > gyroMax[1]) gyroMax[1] = g_y;
                if (g_y < gyroMin[1]) gyroMin[1] = g_y;
                if (g_z > gyroMax[2]) gyroMax[2] = g_z;
                if (g_z < gyroMin[2]) gyroMin[2] = g_z;
            }

            gNormDiff = gNormMax - gNormMin;
            gyroDiff[0] = gyroMax[0] - gyroMin[0];
            gyroDiff[1] = gyroMax[1] - gyroMin[1];
            gyroDiff[2] = gyroMax[2] - gyroMin[2];

            if (gNormDiff > 0.5f || gyroDiff[0] > 0.15f || gyroDiff[1] > 0.15f || gyroDiff[2] > 0.15f)
                break;
            BSP_DWT_Delay(0.0005f);
        }

        g_norm /= (float)CaliTimes;
        gyro_offset[0] /= (float)CaliTimes;
        gyro_offset[1] /= (float)CaliTimes;
        gyro_offset[2] /= (float)CaliTimes;

        AccelReadMulti(TEMP_M, buf, 2);
        raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
        if (raw_temp > 1023) raw_temp -= 2048;
        temp_when_cali = (float)raw_temp * TEMP_FACTOR + TEMP_OFFSET;

    } while (gNormDiff > 0.5f ||
        std::abs(g_norm - 9.8f) > 0.5f ||
        gyroDiff[0] > 0.15f || gyroDiff[1] > 0.15f || gyroDiff[2] > 0.15f ||
        std::abs(gyro_offset[0]) > 0.01f ||
        std::abs(gyro_offset[1]) > 0.01f ||
        std::abs(gyro_offset[2]) > 0.01f);

    accel_scale = 9.81f / g_norm;
}

// 底层 SPI 读写
void BMI088::Accel_CS(bool state) { HAL_GPIO_WritePin(hw.accel_cs_port, hw.accel_cs_pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET); }
void BMI088::Gyro_CS(bool state) { HAL_GPIO_WritePin(hw.gyro_cs_port, hw.gyro_cs_pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET); }
uint8_t BMI088::ReadWriteByte(uint8_t txdata) {
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(hw.spi_handle, &txdata, &rx_data, 1, HAL_MAX_DELAY);
    return rx_data;
}

void BMI088::AccelWriteReg(uint8_t reg, uint8_t data) { Accel_CS(false); ReadWriteByte(reg); ReadWriteByte(data); Accel_CS(true); }
uint8_t BMI088::AccelReadReg(uint8_t reg) {
    uint8_t data;
    Accel_CS(false); ReadWriteByte(reg | 0x80); ReadWriteByte(0x55); data = ReadWriteByte(0x55); Accel_CS(true);
    return data;
}
void BMI088::AccelReadMulti(uint8_t reg, uint8_t* buf, uint8_t len) {
    Accel_CS(false); ReadWriteByte(reg | 0x80); ReadWriteByte(0x55);
    while (len--) { *buf++ = ReadWriteByte(0x55); }
    Accel_CS(true);
}

void BMI088::GyroWriteReg(uint8_t reg, uint8_t data) { Gyro_CS(false); ReadWriteByte(reg); ReadWriteByte(data); Gyro_CS(true); }
uint8_t BMI088::GyroReadReg(uint8_t reg) {
    uint8_t data;
    Gyro_CS(false); ReadWriteByte(reg | 0x80); data = ReadWriteByte(0x55); Gyro_CS(true);
    return data;
}
void BMI088::GyroReadMulti(uint8_t reg, uint8_t* buf, uint8_t len) {
    Gyro_CS(false); ReadWriteByte(reg | 0x80);
    while (len--) { *buf++ = ReadWriteByte(0x55); }
    Gyro_CS(true);
}


bool BMI088::AccelInit() {

    Accel_CS(false); HAL_Delay(1); Accel_CS(true); HAL_Delay(1);
    AccelReadReg(ACC_CHIP_ID); HAL_Delay(1);

    // 软件复位
    AccelWriteReg(ACC_SOFTRESET, 0xB6);
    HAL_Delay(50);

    Accel_CS(false); HAL_Delay(1); Accel_CS(true); HAL_Delay(1);
    AccelReadReg(ACC_CHIP_ID); HAL_Delay(1);

    if (AccelReadReg(ACC_CHIP_ID) != ACC_CHIP_ID_VALUE) return false;

    AccelWriteReg(0x7D, 0x04); HAL_Delay(50); // PWR_CTRL 开启电源
    AccelWriteReg(0x7C, 0x00); HAL_Delay(50); // PWR_CONF 切换为 Active 模式

    for (const auto& cmd : acc_init_cmds) { AccelWriteReg(cmd[0], cmd[1]); HAL_Delay(2); }
    return true;
}

bool BMI088::GyroInit() {
    Gyro_CS(false); HAL_Delay(1); Gyro_CS(true); HAL_Delay(1);
    GyroReadReg(GYRO_CHIP_ID); HAL_Delay(1);

    GyroWriteReg(GYRO_SOFTRESET, 0xB6);
    HAL_Delay(50);

    Gyro_CS(false); HAL_Delay(1); Gyro_CS(true); HAL_Delay(1);
    GyroReadReg(GYRO_CHIP_ID); HAL_Delay(1);

    if (GyroReadReg(GYRO_CHIP_ID) != GYRO_CHIP_ID_VALUE) return false;

    for (const auto& cmd : gyro_init_cmds) { GyroWriteReg(cmd[0], cmd[1]); HAL_Delay(2); }
    return true;
}