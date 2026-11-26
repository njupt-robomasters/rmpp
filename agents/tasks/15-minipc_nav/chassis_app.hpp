#pragma once

// 仅替换设备与调度依赖，chassis.cpp 与 gimbal.cpp 的完整生产控制流程原样编译
#include "support.hpp"
#include "math/angle.hpp"

inline constexpr float PITCH_MIN = -0.1f, PITCH_MAX = 0.314f;

struct ChassisMatrix {
    enum Mode { DETACH_MODE, FOLLOW_MODE };
    bool enabled = false;
    Mode mode = DETACH_MODE;
    float vx = 0, vy = 0, wr_input = 0;
    struct { float measure = 0; } wr;
    Angle gimbal_yaw;
    void set_enable(const bool value) { enabled = value; }
    void set_mode(const Mode value) { mode = value; }
    void set_speed(const float x, const float y, const float w) { vx = x; vy = y; wr_input = w; }
    void set_gimbal_yaw(const Angle value) { gimbal_yaw = value; }
    void set_power_ref(float) {}
    void update() {}
};

struct SuperCap {
    enum Mode { OFF, CHARGE, BOOST };
    float power_chassis_ref = 120;
    void set_mode(Mode) {}
    void set_referee_limit(float) {}
    void set_referee_energy(float) {}
    void update() {}
};

struct Referee {
    enum { NOT_STARTED, REFEREE_SELF_CHECK, COUNTDOWN_5SEC, GAMING };
    bool is_connect = false;
    struct { int game_progress = NOT_STARTED; } game_status;
    struct { float chassis_power_limit = 120; int current_HP = 400; } robot_status;
    struct { float buffer_energy = 60; } power_heat_data;
};

struct VT13 {
    enum Mode { C, N, S };
    struct { Mode mode = N; bool fn = false, photo = false; } rc;
    struct { bool r = false; } key;
    struct { float vxy_max = 5, wr_max = 120 * rpm, yaw_speed_max = 360 * deg_s; } config;
    bool is_enable = true, is_aim = false;
    float vx = 0, vy = 0, wr = 0, yaw_speed = 0, pitch_speed = 0;
};

inline ChassisMatrix chassis;
inline SuperCap super_cap;
inline Referee referee;
inline VT13 vt13;
inline struct {
    bool is_connect = true;
    float angle_multi = 0;
    struct { float offset = 0, reduction = 1; bool is_invert = true; } config;
} yaw1;
inline struct { bool is_connect = true; } yaw2;
inline struct {
    bool enabled = false, is_angle_cmd = false, yaw_window = false;
    float yaw_speed = 0, pitch_speed = 0, chassis_wr = 0, turned = 0;
    Angle angle_yaw, angle_pitch, updated_yaw, updated_pitch;
    struct { struct { Angle ref, measure; } ecd; } yaw, pitch;
    struct { Angle measure; } yaw2;
    void set_enable(const bool value) { enabled = value; }
    void set_speed(const float y, const float p) { is_angle_cmd = false; yaw_speed = y; pitch_speed = p; }
    void set_angle(const Angle y, const Angle p) { is_angle_cmd = true; angle_yaw = y; angle_pitch = p; yaw_speed = pitch_speed = 0; }
    void turn_yaw(const float delta) { turned += delta; }
    void set_yaw_window(const bool value) { yaw_window = value; }
    void set_chassis_wr(const float value) { chassis_wr = value; }
    void update(const Angle y, const Angle p) { updated_yaw = y; updated_pitch = p; }
} gimbal;
inline struct { Angle yaw, pitch; } attitude;
inline struct { void write(const char*, float) {} } kv_store;
inline struct { template <typename T> void play(const T&) {} } melody;
inline Nav nav({.cdc = Cdc(2)});
inline bool test_enable = true, test_active = true;
inline bool sentry_enable() { return test_enable; }
inline bool sentry_active() { return test_active; }
inline bool sentry_in_game() { return false; }

void chassis_loop();
void gimbal_loop(bool is_detect, Angle target_yaw, Angle target_pitch, float distance);
