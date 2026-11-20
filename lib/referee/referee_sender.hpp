#pragma once

#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include "container/fifo.hpp"
#include "protocol.hpp"

// 裁判系统常规链路发送：选手端 UI、哨兵自主决策 0x0120、选手端消息 0x0308
// 每次调用排入 1 个请求，send() 按序组帧（帧头、发送者与接收者、CRC）并逐 8 B 经 CAN 交给 RefereeRelay
// 0x0301 全部子内容共用 30 Hz，由 send() 保证；其余频率由调用方决定
class RefereeSender {
  private:
    static constexpr size_t QUEUE_CAPACITY = 20;              // 待发请求数；UI 初始布局 11 个、仪表 2 个加指令余量
    static constexpr size_t CONTENT_MAX = 105;                // 请求内容最长，B；绘制七个图形
    static constexpr size_t FRAME_MAX = 6 + CONTENT_MAX + 9;  // 交互头 6 B，帧头 5 B、命令码 2 B、CRC16 2 B
    static constexpr float INTERACTION_PERIOD = 1.0f / 30.0f; // 0x0301 最小间隔，s
    static constexpr uint16_t SERVER_ID = 0x8080;             // 裁判系统服务器，哨兵与雷达自主决策的接收者

  public:
    enum Color : uint8_t { MAIN, // 己方颜色
                           YELLOW,
                           GREEN,
                           ORANGE,
                           PURPLE,
                           PINK,
                           CYAN,
                           BLACK,
                           WHITE };

    enum Operation : uint8_t { NONE,
                               ADD,
                               MODIFY,
                               DELETE };

    enum Type : uint8_t { LINE,
                          RECTANGLE,
                          CIRCLE,
                          ELLIPSE,
                          ARC,
                          FLOAT,
                          INTEGER,
                          CHARACTER };

    struct Config {
        Can can;            // 经 RefereeRelay 转发的常规链路
        uint32_t tx_id = 1; // 本板发送 CAN ID；对应 RefereeRelay rx_id
    };

  private:
    const Config config;

    uint16_t robot_id = 0; // 本机 ID，0 为未知；未知时暂停发送

    // 待发请求，组帧时补发送者；receiver_id 为 0 表示本机选手端
    struct Request {
        uint16_t cmd_id = 0;     // 0x0301 或 0x0308
        uint16_t content_id = 0; // 0x0301 子内容 ID
        uint16_t receiver_id = 0;
        uint8_t size = 0; // content 有效长度，B
        uint8_t content[CONTENT_MAX]{};
    };

    Fifo<Request, QUEUE_CAPACITY> queue;

    // 当前帧，send() 每次发出至多 8 B
    uint8_t frame[FRAME_MAX]{};
    size_t frame_size = 0, frame_pos = 0;
    uint8_t seq = 0;

    Timer interaction_timer; // 上一帧 0x0301 组帧时重新开始

  public:
    explicit RefereeSender(const Config& config);

    // 本机 ID 来自 Referee 0x0201
    void set_robot_id(uint16_t robot_id);

    // 图形构造：name 为 24 位标识，layer 为 0~9；坐标、线宽、半径单位为像素
    static Figure line(uint32_t name, uint8_t layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t end_x, uint16_t end_y);

    static Figure rectangle(uint32_t name, uint8_t layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t end_x, uint16_t end_y);

    static Figure circle(uint32_t name, uint8_t layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t radius);

    static Figure ellipse(uint32_t name, uint8_t layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t rx, uint16_t ry);

    // 圆弧角度，rad；发送时换算为协议的顺时针度数，正向整圈保留为 360
    static Figure arc(uint32_t name, uint8_t layer, Color color, uint16_t width, uint16_t x, uint16_t y, float start_angle, float end_angle, uint16_t rx, uint16_t ry);

    static Figure integer(uint32_t name, uint8_t layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t font_size, int32_t value);

    // 浮点数按 1/1000 刻度显示
    static Figure floating(uint32_t name, uint8_t layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t font_size, float value);

    static Figure character(uint32_t name, uint8_t layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t font_size);

    // UI：最多 7 个非字符图形；MODIFY 与排队中同子内容、首图形同名的 MODIFY 合并为最新一份
    bool draw(const Figure* figures, size_t count, Operation operation);

    // UI：字符图形，text 为至多 30 B 的 ASCII
    bool draw_text(const Figure& figure, const char* text, size_t size, Operation operation);

    bool delete_layer(uint8_t layer);

    bool delete_all();

    // 丢弃排队中的 UI 请求，保留其他报文；正在发出的帧不受影响
    void clear_ui();

    // 0x0120 哨兵自主决策，发往服务器
    bool send_sentry_command(const SentryCommand& command);

    // 0x0308 选手端消息：receiver_id 为己方选手端 ID，text 为至多 15 个 UTF-16 字符；协议上限 3 Hz
    bool send_custom_info(uint16_t receiver_id, const char16_t* text, size_t size);

    // 仅在主循环调用：CAN 有空位时发出当前帧的下一段 8 B，当前帧发完后组下一帧
    void send();

  private:
    static Figure make_figure(uint32_t name, uint8_t layer, Color color, uint16_t width, uint16_t x, uint16_t y, Type type);

    static void write_arc(Figure& figure, float start_angle, float end_angle);

    static void write_number(Figure& figure, int32_t value);

    static void write_float(Figure& figure, float value);

    bool push(const Request& request);

    static void write_u16(uint8_t* data, uint16_t value);
};
