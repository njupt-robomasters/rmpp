#pragma once

#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include "container/fifo.hpp"
#include "packet.hpp"
#include <cstring>

// 裁判系统常规链路接收：拼接 RefereeRelay 经 CAN 转发的串口字节流，校验后解析；发送见 RefereeSender
class Referee {
  private:
    static constexpr size_t PAYLOAD_MAX = 118;           // 常规链路最长载荷，B；0x0301
    static constexpr size_t FRAME_MAX = PAYLOAD_MAX + 9; // 帧头 5 B、命令码 2 B、CRC16 2 B
    static constexpr size_t RX_CAPACITY = 256;           // 接收缓冲，B；最长帧加帧头前乱码余量

  public:
    struct Config {
        Can can;              // 经 RefereeRelay 转发的常规链路
        uint32_t rx_id = 0;   // 本板接收 CAN ID；对应 RefereeRelay tx_id
        float timeout = 1.0f; // 断联超时，s
    } const config;

    enum GameProgress : uint8_t {
        NOT_STARTED,
        PREPARING,
        REFEREE_SELF_CHECK,
        COUNTDOWN_5SEC,
        GAMING,
        SETTLING
    };

    bool is_connect = false;

    // 常规链路数据，接收回调解析；断联后保留最后一帧，首次接收前为 0
    GameStatus game_status{};
    RobotStatus robot_status{};
    PowerHeatData power_heat_data{};
    HurtData hurt_data{};
    ShootData shoot_data{};
    RfidStatus rfid_status{};
    uint32_t hurt_sequence = 0; // 收到 0x0206 时递增，供 app 判断新的受击事件

  private:
    // 当前无外部读取，供 SWD 查看
    GameResult game_result{};
    GameRobotHp game_robot_hp{};
    EventData event_data{};
    RefereeWarning referee_warning{};
    DartInfo dart_info{};
    RobotPos robot_pos{};
    Buff buff{};
    ProjectileAllowance projectile_allowance{};
    DartClientCmd dart_client_cmd{};
    GroundRobotPosition ground_robot_position{};
    RadarMarkData radar_mark_data{};
    SentryInfo sentry_info{};
    RadarInfo radar_info{};
    Interaction interaction{};
    MapCommand map_command{};

    Fifo<uint8_t, RX_CAPACITY> rx_fifo; // 未解析字节，CAN 接收中断写入

    Timer callback_timer; // 用于断联检测

  public:
    explicit Referee(const Config& config);

    // 每周期调用：断联检测
    void update();

  private:
    void callback(const uint8_t* data, size_t size);

    void parse(uint16_t cmd_id, const uint8_t* data, size_t size);

    // 载荷长度与结构体一致时整体覆盖，返回是否覆盖
    template <typename T>
    static bool read(T& packet, const uint8_t* data, const size_t size) {
        if (size != sizeof(T))
            return false;
        std::memcpy(&packet, data, size);
        return true;
    }
};
