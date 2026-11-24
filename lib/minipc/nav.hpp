#pragma once

#include "bsp/cdc.hpp"
#include "bsp/timer.hpp"
#include "container/fifo.hpp"

class Nav {
  public:
    struct Config {
        Cdc cdc;
        float timeout = 0.1f; // 断联超时，s
    } const config;

    bool is_connect = false;

    // 导航指令，接收回调解析，断联时为 0
    bool is_through_hole = false; // 过洞为 1，其余为 0
    float vx = 0, vy = 0;         // 雷达所在大 yaw 系速度，m/s
    float yaw = 0;                // 相对大 yaw 当前朝向的期望转角，rad；俯视逆时针为正
    float x = 0, y = 0;           // 上位机自定位的当前坐标，m

  private:
    static constexpr uint8_t SOF_RX = 0xA3; // 下行帧头
    static constexpr uint8_t SOF_TX = 0xA4; // 上行帧头

    // 下行原始帧，最近一帧有效数据；断联后保留供 SWD 查看
    struct __attribute__((packed)) RxFrame {
        uint8_t sof;             // A3
        uint8_t is_through_hole; // 过洞为 1，其余为 0
        float vx, vy;            // 大 yaw 系速度，m/s
        float yaw;               // 相对大 yaw 当前朝向的期望转角，rad；俯视逆时针为正
        float x, y;              // 当前坐标，m
        uint16_t crc;            // CRC16/MODBUS，覆盖帧头至 y，低字节在前
    } raw_rx{};

    // 上行原始帧，set_target() 设置目标点，send() 写入实测角并补 CRC
    struct __attribute__((packed)) TxFrame {
        uint8_t sof = SOF_TX; // A4
        float yaw = 0;        // 大 yaw 相对底盘的编码器角，rad；俯视逆时针为正
        float x = 0, y = 0;   // 目标坐标，m
        uint16_t crc = 0;     // CRC16/MODBUS，覆盖帧头至 y，低字节在前
    } raw_tx;

    Fifo<uint8_t, sizeof(RxFrame)> rx_fifo; // 未解析字节，USB 接收中断写入
    Timer callback_timer;                   // 用于断联检测

  public:
    explicit Nav(const Config& config);

    void set_target(float x, float y);

    void send(float yaw_ecd); // 本次采样的大 yaw 编码器角，rad

    // 每周期调用：断联检测，断联时指令复位为 0
    void update();

  private:
    void callback(const uint8_t* data, size_t size);
};
