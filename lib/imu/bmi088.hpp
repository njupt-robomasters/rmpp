#pragma once

#include "bsp/spi.hpp"
#include "imu/eigen_types.hpp"

class Bmi088 {
  public:
    struct Config {
        Spi accel;
        Spi gyro;
    } const config;

    // 传感器坐标系原始量，尚未校准
    Vector3 gyro = Vector3::Zero();  // rad/s
    Vector3 accel = Vector3::Zero(); // m/s²
    float temperature{};             // °C

    explicit Bmi088(const Config& config);

    // 单次初始化；芯片 ID 或寄存器回读不符时返回 false
    bool init() const;
    bool update();

  private:
    static void write_register(const Spi& spi, uint8_t reg, uint8_t value);

    // 加速度计多一个空读字节；最多读取 8 字节
    static void read_registers(const Spi& spi, uint8_t reg, uint8_t* data, uint8_t size, uint8_t prefix);
};
