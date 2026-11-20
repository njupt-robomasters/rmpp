#pragma once

// 仅替换设备与调度依赖，chassis.cpp 的完整生产控制流程原样编译
#include "support.hpp"
#include "math/angle.hpp"

struct ChassisMatrix {
    enum Mode { DETACH_MODE, FOLLOW_MODE };
    bool enabled = false;
    Mode mode = DETACH_MODE;
    float vx = 0, vy = 0, wr = 0;
    Angle gimbal_yaw;
    void set_enable(const bool value) { enabled = value; }
    void set_mode(const Mode value) { mode = value; }
    void set_speed(const float x, const float y, const float w) { vx = x; vy = y; wr = w; }
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
    struct { float chassis_power_limit = 120; } robot_status;
    struct { float buffer_energy = 60; } power_heat_data;
};

struct VT13 {
    enum Mode { C, N, S };
    bool is_connect = false;
    Mode mode = N;
    float x = 0, y = 0, wheel = 0;
    struct { bool w = false, s = false, a = false, d = false, shift = false, ctrl = false; } key;
};

struct FSi6X {
    enum { UP, MID, DOWN };
    bool is_connect = false;
    int swc = MID;
    float x = 0, y = 0;
};

inline ChassisMatrix chassis;
inline SuperCap super_cap;
inline Referee referee;
inline VT13 vt13;
inline FSi6X fsi6x;
inline struct { bool is_connect = true; } yaw1, yaw2;
inline struct {
    struct { struct { Angle measure; } ecd; } yaw;
    struct { Angle measure; } yaw2;
} gimbal;
inline Nav nav(Cdc(2), {});
inline bool test_enable = true, test_active = true, test_fn = true;
inline bool sentry_enable() { return test_enable; }
inline bool sentry_active() { return test_active; }
inline bool sentry_in_game() { return false; }
inline bool rc_fn() { return test_fn; }
