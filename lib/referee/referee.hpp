#pragma once

#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include "bsp/uart.hpp"
#include "protocol.hpp"
#include <functional>
#include <variant>
#include <vector>

class Referee {
  public:
    struct Config {
        std::variant<Can, Uart> bus;
        uint32_t rx_id = 0;   // CAN 转发接收 ID，UART 不使用
        uint32_t tx_id = 1;   // CAN 转发发送 ID，UART 不使用
        float timeout = 1.0f; // 连接超时，秒
    };

    // 回调在接收上下文执行，数据仅在回调期间有效；启动前注册
    using RxCallback = std::function<void(const uint8_t*, size_t)>;

    enum GameProgress : uint8_t {
        NOT_STARTED,
        PREPARING,
        REFEREE_SELF_CHECK,
        COUNTDOWN_5SEC,
        GAMING,
        SETTLING
    };

    bool is_connected = false;
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
    CustomData custom_robot_data{}; // 0x0302，自定义控制器发给机器人
    MapCommand map_command{};
    MapRobotData map_robot_data{};
    CustomClientData custom_client_data{}; // 0x0306，键鼠控制器发给选手端
    MapData map_data{};
    CustomInfo custom_info{};
    CustomData robot_custom_data{};        // 0x0309，机器人发给自定义控制器
    ClientData robot_custom_client_data{}; // 0x0310，机器人发给自定义客户端
    CustomData custom_client_command{};    // 0x0311，自定义客户端发给机器人
    RadarPosition radar_position{};
    RadarHp radar_hp{};
    RadarAllowance radar_allowance{};
    RadarTeam radar_team{};
    RadarBuff radar_buff{};
    RadarKey radar_key{};

    // 通信绑定
    const std::variant<Can, Uart> bus;
    const uint32_t rx_id; // CAN 转发接收 ID，UART 不使用
    const uint32_t tx_id; // CAN 转发发送 ID，UART 不使用
    const float timeout;  // 连接超时，秒

    // 每次有效伤害报文递增，避免依赖字段变化判断新事件
    uint32_t hurt_sequence = 0;
    std::vector<RxCallback> callbacks_0311;
    uint8_t rx_buf[309]{};
    size_t rx_size = 0;
    uint8_t tx_buf[309]{};
    size_t tx_size = 0, tx_pos = 0;
    uint8_t seq = 0;

  public:
    Timer hurt_timer;

  private:
    Timer frame_timer, receive_timer, interaction_timer;

  public:
    // 实例及其绑定对象在 BSP 回调有效期间不得销毁或移动
    explicit Referee(const Config& config);

    Referee(const Referee&) = delete;

    void add_0311_callback(RxCallback callback);

    void update();

    // 拷贝完整载荷到待发帧；忙或非法长度返回 false，不部分接收
    // 0x0301 的所有调用共用 30 Hz 限额；仅在主循环调用
    bool send(uint16_t cmd_id, const uint8_t* data, size_t size);

    bool is_sending() const {
        return tx_size != 0;
    }

  private:
    Referee& operator=(const Referee&) = delete;

    void receive(const uint8_t* data, size_t size);

    bool deserialize(uint16_t cmd_id, const uint8_t* data, size_t size);
};
