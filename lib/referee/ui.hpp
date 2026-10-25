#pragma once

#include "math/angle.hpp"
#include "math/unit.hpp"
#include "referee.hpp"

class UiControl {
  public:
    enum Color : uint8_t { MAIN,
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

    using Graphic = Figure;

    explicit UiControl(Referee& referee)
        : referee_(referee) {}

    // name 为 24 位标识，layer 为 0~9；坐标和线宽单位为像素
    static Graphic line(uint32_t name, uint8_t layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t end_x, uint16_t end_y);
    static Graphic rectangle(uint32_t name, uint8_t layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t end_x, uint16_t end_y);
    static Graphic circle(uint32_t name, uint8_t layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t radius);
    static Graphic ellipse(uint32_t name, uint8_t layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t rx, uint16_t ry);
    // 圆弧角度使用 rad，发送时换算为协议的顺时针度数
    static Graphic arc(uint32_t name, uint8_t layer, Color color, uint16_t width, uint16_t x, uint16_t y, float start_angle, float end_angle, uint16_t rx, uint16_t ry);
    static Graphic integer(uint32_t name, uint8_t layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t font_size, int32_t value);
    static Graphic floating(uint32_t name, uint8_t layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t font_size, float value);
    static Graphic character(uint32_t name, uint8_t layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t font_size);
    static void set_number(Graphic& graphic, int32_t value);
    static void set_float(Graphic& graphic, float value);
    static void set_arc(Graphic& graphic, float start_angle, float end_angle);

    // 最多 7 个图形；入队复制数据，满时不部分写入
    bool draw(const Graphic* graphics, size_t count, Operation operation);
    bool draw_text(const Graphic& graphic, const char* text, size_t size, Operation operation);
    bool delete_layer(uint8_t layer);
    bool delete_all();
    // 丢弃尚未交给链路的旧绘制请求，用于显式重新绘制
    void clear_queue();
    void update();

    bool is_pending() const {
        return count_ != 0 || referee_.is_sending();
    }

    uint16_t robot_id() const {
        return referee_.robot_status.robot_id;
    }

  private:
    Referee& referee_;

    struct Request {
        uint16_t content_id = 0;
        uint8_t size = 0;
        uint8_t data[105]{};
    };

    static constexpr size_t QUEUE_CAPACITY = 16;
    Request queue_[QUEUE_CAPACITY]{};
    size_t head_ = 0, count_ = 0;

    static Graphic make_graphic(uint32_t name, uint8_t layer, Color color, uint16_t width, uint16_t x, uint16_t y, Type type);
    static void encode(const Graphic& graphic, Operation operation, uint8_t* data);
    bool enqueue(const Request& request);
};

class RobotUi : public UiControl {
  public:
    struct Config {
        float max_cap = 1.0f;
        float max_wr = 120 * rpm;
        float max_bullet = 25.0f;
        float max_shoot = 10.0f;
    } config_;

    Angle yaw_ecd, yaw, pitch;
    bool is_hurt = false;
    Angle hurt_dir;
    bool is_detect = false;
    float cap = 0.0f, wr = 0.0f, bullet = 0.0f, shoot = 0.0f;
    bool bullet_is_speed = false;

    struct RobotState {
        Color w1 = GREEN, w2 = GREEN, w3 = GREEN, w4 = GREEN;
        Color s1 = GREEN, s2 = GREEN, s3 = GREEN, s4 = GREEN;
        Color yaw1 = GREEN, yaw2 = GREEN, pitch = GREEN;
        Color rub1 = GREEN, rub2 = GREEN, rub3 = GREEN, rub4 = GREEN;
        Color shoot = GREEN, referee = GREEN, aim = GREEN;
    } robot;

    explicit RobotUi(Referee& referee)
        : UiControl(referee) {}

    RobotUi(Referee& referee, const Config& config)
        : UiControl(referee), config_(config) {}

    void init();
    void update();

  private:
    void update_dashboard();
    void enqueue_initial_layout();

    Timer dashboard_timer_;
    bool initialized_ = false;
};
