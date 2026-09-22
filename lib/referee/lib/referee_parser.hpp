#pragma once

#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include "math/unit.hpp"
#include "packet.hpp"
#include <cstdint>

class RefereeParser {
  public:
    struct Config {
        // CAN通信参数
        Can can;
        uint32_t master_id = 0;
        uint32_t slave_id = 1;

        float timeout = 1000 * ms; // 断联检测超时时间
    } config;

    // 连接标志
    bool is_connect = false;

    // 接收报文
    game_status_t game_status{};                     // 0x0001，比赛状态数据（1Hz）
    game_result_t game_result{};                     // 0x0002，比赛结果数据（比赛结束触发发送）
    game_robot_HP_t game_robot_hp{};                 // 0x0003，机器人血量数据（3Hz）
    event_data_t event_data{};                       // 0x0101，场地事件数据（1Hz）
    referee_warning_t referee_warning{};             // 0x0104，裁判警告数据（1Hz，己方判罚/判负时触发发送）
    dart_info_t dart_info{};                         // 0x0105，飞镖发射相关数据（1Hz）
    robot_status_t robot_status{};                   // 0x0201，机器人性能体系数据（10Hz，离线模式下可用）
    power_heat_data_t power_heat_data{};             // 0x0202，实时底盘缓冲能量和射击热量数据（10Hz）
    robot_pos_t robot_pos{};                         // 0x0203，机器人位置数据（1Hz）
    buff_t buff{};                                   // 0x0204，机器人增益和底盘能量数据（3Hz）
    hurt_data_t hurt_data{};                         // 0x0206，伤害状态数据（伤害发生后发送，离线模式下可用）
    shoot_data_t shoot_data{};                       // 0x0207，实时射击数据（弹丸发射后发送，离线模式下可用）
    projectile_allowance_t projectile_allowance{};   // 0x0208，允许发弹量（10Hz）
    rfid_status_t rfid_status{};                     // 0x0209，机器人RFID模块状态（3Hz）
    dart_client_cmd_t dart_client_cmd{};             // 0x020A，飞镖选手端指令数据（3Hz）【服务器→己方飞镖机器人】
    ground_robot_position_t ground_robot_position{}; // 0x020B，地面机器人位置数据（1Hz）【服务器→己方哨兵机器人】
    radar_mark_data_t radar_mark_data{};             // 0x020C，雷达标记进度数据（1Hz）【服务器→己方雷达机器人】
    sentry_info_t sentry_info{};                     // 0x020D，哨兵自主决策信息同步（1Hz）【服务器→己方哨兵机器人】
    radar_info_t radar_info{};                       // 0x020E，雷达自主决策信息同步（1Hz）【服务器→己方雷达机器人】

    // 用于统计报文频率
    Timer timer_game_status;
    Timer timer_game_robot_hp;
    Timer timer_event_data;
    Timer timer_referee_warning;
    Timer timer_dart_info;
    Timer timer_robot_status;
    Timer timer_power_heat_data;
    Timer timer_robot_pos;
    Timer timer_buff;
    Timer timer_hurt_data;
    Timer timer_projectile_allowance;
    Timer timer_rfid_status;
    Timer timer_dart_client_cmd;
    Timer timer_ground_robot_position;
    Timer timer_radar_mark_data;
    Timer timer_sentry_info;
    Timer timer_radar_info;

    RefereeParser(const Config& config);

    // 需要在循环中调用
    void on_loop();

  private:
    // 用于断联检测
    Timer timer_connect_;

    enum {
        INIT,
        FRAME_HEADER,
        CMD_ID,
        DATA,
        FRAME_TAIL
    } step_ = FRAME_HEADER;

    struct __attribute__((packed)) {
        struct __attribute__((packed)) {
            uint8_t sof = 0;
            uint16_t data_length = 0;
            uint8_t seq = 0;
            uint8_t crc8 = 0;
        } frame_header;

        uint16_t cmd_id = 0;

        uint8_t data[64]{};

        uint8_t& operator[](const int index) {
            return *((uint8_t*)this + index);
        }
    } packet_;

    uint16_t pos_ = 0;

    void parse_byte(uint8_t byte);

    void callback(uint32_t id, const uint8_t data[8], uint8_t dlc);

    void deserialize(uint16_t cmd_id, const uint8_t data[], uint8_t size);
};