#pragma once

#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include "container/fifo.hpp"
#include "protocol.hpp"

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
        float timeout = 1.0f; // 连接超时，s
    };

  private:
    const Config config;

  public:
    enum GameProgress : uint8_t {
        NOT_STARTED,
        PREPARING,
        REFEREE_SELF_CHECK,
        COUNTDOWN_5SEC,
        GAMING,
        SETTLING
    };

    bool is_connect = false;
    GameStatus game_status{};
    RobotStatus robot_status{};
    PowerHeatData power_heat_data{};

  private:
    GameResult game_result{};
    GameRobotHp game_robot_hp{};
    EventData event_data{};
    RefereeWarning referee_warning{};
    DartInfo dart_info{};
    RobotPos robot_pos{};
    Buff buff{};
    HurtData hurt_data{};
    ShootData shoot_data{};
    ProjectileAllowance projectile_allowance{};
    RfidStatus rfid_status{};
    DartClientCmd dart_client_cmd{};
    GroundRobotPosition ground_robot_position{};
    RadarMarkData radar_mark_data{};
    SentryInfo sentry_info{};
    RadarInfo radar_info{};
    Interaction interaction{};
    MapCommand map_command{};

    Fifo<uint8_t, RX_CAPACITY> rx; // 未解析字节，CAN 接收中断写入

  public:
    Timer hurt_timer; // 收到 0x0206 时重新开始

  private:
    Timer connect_timer, receive_timer;

  public:
    // 实例及其绑定对象在 BSP 回调有效期间不得销毁或移动
    explicit Referee(const Config& config);

    Referee(const Referee&) = delete;

    void update();

  private:
    void receive(const uint8_t* data, size_t size);

    bool parse(uint16_t cmd_id, const uint8_t* data, size_t size);

    Referee& operator=(const Referee&) = delete;
};
