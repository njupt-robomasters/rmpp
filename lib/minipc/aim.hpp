#pragma once

#include "bsp/cdc.hpp"
#include "bsp/timer.hpp"
#include "container/fifo.hpp"
#include "math/angle.hpp"

class Aim {
  public:
    struct Config {
        Cdc cdc;
        float timeout = 0.1f; // 断联超时，s
    } const config;

    bool is_connect = false;

    // 自瞄目标，接收回调解析，断联时为 0；角度为最终 IMU 头的世界系目标，pitch 绕 Y 轴右手为正
    bool is_detect = false;
    Angle yaw, pitch;
    bool is_fire = false;
    float distance = 0; // 目标距离，m

  private:
    // 下行原始帧，最近一帧有效数据；断联后保留供 SWD 查看
    struct __attribute__((packed)) RxFrame {
        uint8_t sof[2];    // A5 5B
        uint8_t is_detect; // 识别到目标为 1
        float yaw, pitch;  // 度；pitch 抬头为正
        uint8_t is_fire;   // 允许开火为 1
        float distance;    // m
        uint16_t crc;      // CRC16/MODBUS，覆盖 sof 与 crc 之间的字节，低字节在前
    } raw_rx{};

    // 上行原始帧，set_*() 直接写入，send() 补 CRC 后发出；供 SWD 查看
    struct __attribute__((packed)) TxFrame {
        uint8_t sof[2] = {0xA5, 0x5A};
        float yaw = 0, pitch = 0, roll = 0; // 最终 IMU 头姿态，度；pitch 抬头为正，roll 由后向前看顺时针为正
        uint8_t is_red = 1;                 // 红方 1，蓝方 0
        float bullet_speed = 0;             // 裁判实测初速，m/s
        uint16_t crc = 0;                   // CRC16/MODBUS，覆盖 sof 与 crc 之间的字节，低字节在前
    } raw_tx;

    Fifo<uint8_t, sizeof(RxFrame)> rx_fifo; // 未解析字节，USB 接收中断写入
    Timer callback_timer;                   // 用于断联检测

  public:
    explicit Aim(const Config& config);

    void set_imu(Angle yaw, Angle pitch, Angle roll);
    void set_referee(bool is_red, float bullet_speed);

    void send();

    // 每周期调用：断联检测，断联时目标复位为 0
    void update();

  private:
    void callback(const uint8_t* data, size_t size);
};
