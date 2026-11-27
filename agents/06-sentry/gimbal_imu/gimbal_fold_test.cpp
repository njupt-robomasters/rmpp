// GimbalFold IMU 闭环仿真：电机、被控对象与 IMU 由主机模型驱动
// 验证世界系跟踪、底盘转速抑制、小 yaw 限位、单独调 yaw2 速度环、离线保护与 pitch 限位
#include "robot/gimbal_fold.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

// 仿真时间，Timer 按它推进
static uint64_t now_us = 0;

float Timer::elapsed() const {
    return static_cast<float>(now_us - last_cycles) * 1e-6f;
}

float Timer::restart() {
    const float dt = elapsed();
    last_cycles = now_us;
    return dt;
}

static void check(const bool condition, const char* message) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        std::abort();
    }
}

// 输出按型号量程换算，反馈直接给出
class TestMotor : public Motor {
  private:
    const bool is_torque;

  public:
    explicit TestMotor(const bool is_torque, const float max_current, const float max_torque)
        : Motor({.can = Can(1), .timeout = 0.005f}, max_current, max_torque, 1), is_torque(is_torque) {}

    void set_output(const float output) override {
        if (is_torque)
            set_torque(output);
        else
            set_current(output);
    }

    void feedback(const float position, const float speed) {
        can_callback({.speed = speed, .angle = position});
    }
};

static const Pid::Config YAW1_ANGLE = {.kp = 7.0f, .max_out = 720 * deg_s};
static const Pid::Config YAW1_SPEED = {.kp = 4.0f, .ki = 0.3f, .max_i = 1.0f, .max_out = 7.0f, .err_fc = 10};
static const Pid::Config YAW2_ANGLE = {.kp = 15.0f, .max_out = 720 * deg_s};
static const Pid::Config YAW2_SPEED = {.kp = 2.0f, .ki = 0.1f, .max_i = 1.0f, .max_out = 8.5f, .err_fc = 10};
static const Pid::Config PITCH_ANGLE = {.kp = 20.0f, .max_out = 720 * deg_s};
static const Pid::Config PITCH_SPEED = {.kp = 0.50558f, .ki = 2.0f, .kd = 0.02f, .max_i = 0.4f, .max_out = 5.0f, .err_fc = 10};

static const Pid::Config ZERO_PID = {};

static GimbalFold::Config make_config(const Pid::Config* yaw1_angle, const Pid::Config* yaw1_speed, const Pid::Config* yaw2_angle) {
    return {
        .yaw1_angle_pid_config = yaw1_angle,
        .yaw1_speed_pid_config = yaw1_speed,
        .yaw2_angle_pid_config = yaw2_angle,
        .yaw2_speed_pid_config = &YAW2_SPEED,
        .yaw2_min = -40 * deg,
        .yaw2_max = 40 * deg,
        .pitch_angle_pid_config = &PITCH_ANGLE,
        .pitch_speed_pid_config = &PITCH_SPEED,
        .pitch_min = -0.1f,
        .pitch_max = 0.314f,
        .pitch1_position_pid_config = &PITCH_ANGLE,
        .pitch1_speed_pid_config = &PITCH_SPEED,
        .pitch1_min = -90 * deg,
        .pitch1_max = 0 * deg,
        .pitch1_gravity = {},
        .pitch2_gravity = {},
    };
}

// 被控对象：电机输出经一阶惯性得到各关节输出轴角速度，关节角积分得到 IMU 姿态
struct Sim {
    TestMotor yaw1{true, 20, 7}, yaw2{false, 8.5f, 4.1f}, pitch1{true, 20, 7}, pitch2{true, 20, 7};
    GimbalFold fold;

    // 输出单位 → 角速度，rad/s per Nm 或 per A；一阶时间常数，s
    float yaw1_gain = 3, yaw2_gain = 3, pitch_gain = 5;
    float yaw1_tau = 0.05f, yaw2_tau = 0.01f, pitch_tau = 0.02f;

    float th1 = 0, th2 = 0, pitch_ecd = 0;
    float w1 = 0, w2 = 0, pitch_speed = 0;
    float chassis = 0, chassis_rate = 0;
    bool is_yaw2_feedback = true;

    explicit Sim(const GimbalFold::Config& config)
        : fold({yaw1, yaw2, pitch1, pitch2}, config) {}

    float psi() const { return chassis + th1 + th2; }
    float phi() const { return pitch_ecd; }
    float psi_rate() const { return chassis_rate + w1 + w2; }

    void step(const float dt) {
        // 反馈：编码器角与输出轴角速度
        yaw1.feedback(th1, w1);
        if (is_yaw2_feedback)
            yaw2.feedback(th2, w2);
        pitch1.feedback(0, 0);
        pitch2.feedback(pitch_ecd, pitch_speed);

        // IMU：世界姿态与机体角速度
        const float yaw = psi(), pitch = phi(), yaw_rate = psi_rate();
        const Quaternionf q = euler_to_quaternion({.yaw = Angle(yaw), .pitch = Angle(pitch), .roll = Angle(0)});
        const Vector3f gyro(-std::sin(pitch) * yaw_rate, pitch_speed, std::cos(pitch) * yaw_rate);
        fold.update(q, gyro);

        // 被控对象与角度积分
        w1 += (yaw1.torque_ref * yaw1_gain - w1) * dt / yaw1_tau;
        w2 += (yaw2.current_ref * yaw2_gain - w2) * dt / yaw2_tau;
        pitch_speed += (pitch2.torque_ref * pitch_gain - pitch_speed) * dt / pitch_tau;
        th1 = Angle(th1 + w1 * dt);
        th2 = Angle(th2 + w2 * dt);
        pitch_ecd = Angle(pitch_ecd + pitch_speed * dt);
        chassis += chassis_rate * dt;
        now_us += static_cast<uint64_t>(dt * 1e6f);
    }

    void run(const float seconds, const float dt = 0.001f) {
        for (int i = 0; i < static_cast<int>(seconds / dt); i++)
            step(dt);
    }
};

// 使能后先跑两拍，轴进入闭环再设目标；首帧目标跟随实测
static void start(Sim& sim) {
    sim.fold.set_enable(true);
    sim.run(0.005f);
}

// 世界角跟踪：大 yaw 承担主要转角，小 yaw 吃掉残差并回中
static void test_track() {
    Sim sim(make_config(&YAW1_ANGLE, &YAW1_SPEED, &YAW2_ANGLE));
    start(sim);
    sim.fold.set_angle(0.8f, 0.2f);
    sim.run(2.0f);
    check(std::fabs(sim.psi() - 0.8f) < 5e-3f, "track: yaw 未收敛到目标");
    check(std::fabs(sim.phi() - 0.2f) < 1e-2f, "track: pitch 未收敛到目标");
    check(std::fabs(sim.th2) < 5 * deg, "track: 小 yaw 未回到中位");
    check(std::fabs(sim.fold.yaw.speed_ref) < 0.1f, "track: 静止时速度目标未归零");
    std::printf("track: yaw %.4f, pitch %.4f, yaw2 %.4f\n", sim.psi(), sim.phi(), float(sim.th2));
}

// 底盘匀速转动：头部保持世界朝向，大 yaw 反向补偿
static void test_chassis_spin() {
    Sim sim(make_config(&YAW1_ANGLE, &YAW1_SPEED, &YAW2_ANGLE));
    start(sim);
    sim.fold.set_angle(0.5f, 0.1f);
    sim.run(1.5f);
    const float ref = sim.psi();

    sim.chassis_rate = 3.0f;
    float max_err = 0;
    for (int i = 0; i < 1500; i++) {
        sim.step(0.001f);
        max_err = std::fmax(max_err, std::fabs(float(Angle(sim.psi() - ref))));
    }
    check(max_err < 0.02f, "spin: 底盘转动时头部世界角偏差过大");
    check(std::fabs(sim.th2) < 45 * deg, "spin: 小 yaw 超出限位");
    check(std::fabs(sim.w1 + 3.0f + sim.w2) < 0.2f, "spin: 大 yaw 未反向补偿底盘转速");
    std::printf("spin: max err %.4f, yaw1 speed %.3f, yaw2 %.4f\n", max_err, sim.w1, float(sim.th2));
}

// 速度指令：按世界系指令匀速转动
static void test_speed() {
    Sim sim(make_config(&YAW1_ANGLE, &YAW1_SPEED, &YAW2_ANGLE));
    start(sim);
    sim.fold.set_angle(0, 0);
    sim.run(1.5f);
    sim.fold.set_speed(1.0f, 0);
    sim.run(0.5f);

    const float start = sim.psi();
    sim.run(0.5f);
    const float rate = float(Angle(sim.psi() - start)) / 0.5f;
    check(std::fabs(rate - 1.0f) < 0.05f, "speed: 世界角速度未跟踪指令");
    std::printf("speed: measured %.4f rad/s\n", rate);
}

// 小 yaw 限位：大 yaw 慢于甩枪时，小 yaw 越限截断后由大 yaw 补足
static void test_yaw2_limit() {
    static const Pid::Config slow_angle = {.kp = 0.5f, .max_out = 60 * deg_s};
    static constexpr float yaw2_limit = 40 * deg;
    Sim sim(make_config(&slow_angle, &YAW1_SPEED, &YAW2_ANGLE));
    start(sim);
    sim.fold.set_angle(2.0f, 0);
    float max_yaw2 = 0;
    bool is_push_out = false;
    for (int i = 0; i < 10000; i++) {
        // 输出按踏入本拍前的编码器角计算，越限判断用同一时刻的角度
        const float yaw2_before = sim.th2;
        sim.step(0.001f);
        max_yaw2 = std::fmax(max_yaw2, std::fabs(sim.th2));
        if (yaw2_before > yaw2_limit && sim.yaw2.current_ref > 0)
            is_push_out = true;
        if (yaw2_before < -yaw2_limit && sim.yaw2.current_ref < 0)
            is_push_out = true;
    }
    check(!is_push_out, "limit: 越限后仍朝外输出");
    check(max_yaw2 < 50 * deg, "limit: 小 yaw 越过机械限位");
    check(std::fabs(sim.psi() - 2.0f) < 1e-2f, "limit: 大 yaw 未补足剩余转角");
    check(std::fabs(sim.th2) < 5 * deg, "limit: 收敛后小 yaw 未回中");
    std::printf("limit: max yaw2 %.2f deg, final %.4f\n", max_yaw2 / deg, float(sim.th2));
}

// 单独调 yaw2 速度环：大 yaw 与外环全零时，只有 IMU 角速度内环工作
static void test_yaw2_speed_only() {
    Sim sim(make_config(&ZERO_PID, &ZERO_PID, &ZERO_PID));
    start(sim);
    sim.fold.set_angle(0, 0);
    sim.run(0.2f);

    sim.w2 = 2.0f;
    sim.step(0.001f);
    check(sim.yaw2.current_ref < 0, "tune: IMU 角速度反馈符号错误");
    float max_yaw1 = 0;
    for (int i = 0; i < 500; i++) {
        sim.step(0.001f);
        max_yaw1 = std::fmax(max_yaw1, std::fabs(sim.yaw1.torque_ref));
    }
    check(max_yaw1 == 0, "tune: 大 yaw PID 为零时仍有输出");
    check(std::fabs(sim.psi_rate()) < 0.2f, "tune: 仅速度环未衰减头部角速度");
    std::printf("tune: head rate %.4f rad/s\n", sim.psi_rate());
}

// 离线：yaw2 断联时两个 yaw 输出零，恢复后重新闭环；pitch 不受影响
static void test_offline() {
    Sim sim(make_config(&YAW1_ANGLE, &YAW1_SPEED, &YAW2_ANGLE));
    start(sim);
    sim.fold.set_angle(0.4f, 0.1f);
    sim.run(1.5f);

    sim.is_yaw2_feedback = false;
    sim.run(0.05f);
    check(!sim.yaw2.is_connect, "offline: 断联状态未生效");
    check(sim.yaw1.torque_ref == 0 && sim.yaw2.current_ref == 0, "offline: yaw 轴仍有输出");
    check(sim.fold.yaw1.speed_ref == 0 && sim.fold.yaw.speed_ref == 0, "offline: yaw 速度目标未清零");

    sim.is_yaw2_feedback = true;
    sim.run(1.5f);
    check(std::fabs(sim.psi() - 0.4f) < 1e-2f, "offline: 恢复后未重新闭环");
    std::printf("offline: recovered psi %.4f\n", sim.psi());
}

// pitch 编码器限位：世界目标按 IMU 与编码器偏差换算后钳制，扫描端点取编码器系
static void test_pitch_limit() {
    Sim sim(make_config(&YAW1_ANGLE, &YAW1_SPEED, &YAW2_ANGLE));
    start(sim);
    sim.fold.set_angle(0, 0.8f);
    sim.run(2.0f);
    check(std::fabs(sim.phi() - 0.314f) < 1e-2f, "pitch: 未停在编码器上限");
    check(std::fabs(float(sim.fold.pitch.ecd_ref) - 0.314f) < 1e-3f, "pitch: 编码器系参考未限位");
    std::printf("pitch: %.4f, ecd ref %.4f\n", sim.phi(), float(sim.fold.pitch.ecd_ref));
}

int main() {
    test_track();
    test_chassis_spin();
    test_speed();
    test_yaw2_limit();
    test_yaw2_speed_only();
    test_offline();
    test_pitch_limit();
    std::printf("PASS\n");
}
