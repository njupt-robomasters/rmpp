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

    // 自瞄目标，接收回调解析，断联时为 0；角度为最终 IMU 头的世界系目标
    // 线上 yaw 俯视逆时针为正，pitch 向下为正，内部同号
    bool is_detect = false;
    Angle yaw, pitch;
    bool is_fire = false;
    float distance = 0; // 目标距离，m

  private:
    static constexpr uint8_t SOF_RX = 0xA1; // 下行帧头
    static constexpr uint8_t SOF_TX = 0xA2; // 上行帧头

    // 下行原始帧，最近一帧有效数据；断联后保留供 SWD 查看
    struct __attribute__((packed)) RxFrame {
        uint8_t sof;       // A1
        uint8_t is_detect; // 识别到目标为 1
        uint8_t is_fire;   // 允许开火为 1
        float yaw, pitch;  // 度；yaw 逆时针为正，pitch 向下为正
        float distance;    // m
        uint16_t crc;      // CRC16/MODBUS，覆盖帧头至距离，低字节在前
    } raw_rx{};

    // 上行原始帧，set_*() 直接写入，send() 补 CRC 后发出；供 SWD 查看
    struct __attribute__((packed)) TxFrame {
        uint8_t sof = SOF_TX;                 // A2
        uint8_t is_red = 1;                   // 红方 1，蓝方 0
        float bullet_speed = 0;               // 裁判实测初速，m/s
        float qx = 0, qy = 0, qz = 0, qw = 1; // 最终 IMU 头姿态四元数，x y z w，机体到世界
        uint16_t crc = 0;                     // CRC16/MODBUS，覆盖帧头至 qw，低字节在前
    } raw_tx;

    Fifo<uint8_t, sizeof(RxFrame)> rx_fifo; // 未解析字节，USB 接收中断写入
    Timer callback_timer;                   // 用于断联检测

  public:
    explicit Aim(const Config& config);

    void set_attitude(const Quaternionf& quaternion);
    void set_referee(bool is_red, float bullet_speed);

    void send();

    // 每周期调用：断联检测，断联时目标复位为 0
    void update();

  private:
    void callback(const uint8_t* data, size_t size);
};
