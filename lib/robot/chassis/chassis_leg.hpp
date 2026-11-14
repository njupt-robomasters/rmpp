#pragma once

#include "bsp/timer.hpp"
#include "math/eigen.hpp"
#include "math/unit.hpp"
#include "motor/dm8009.hpp"
#include "motor/m3508.hpp"
#include "pid/pid.hpp"

// 双轮腿底盘：五连杆 VMC + LQR 平衡，含跳跃与倒地自救
// 坐标：底盘机体系 X 前、Y 左、Z 上；双腿数组 [0] 左、[1] 右
// 姿态由调用方每周期传入底盘机体的 q 与 gyro，可来自本板 IMU 或云台换算的虚拟 IMU
class ChassisLeg {
  private:
    static constexpr float DT_MIN = 0.0001f, DT_MAX = 0.01f; // 控制周期限幅，s；避免首次调用和停顿后的大步长

  public:
    struct Config {
        // 质量与几何
        float mass = 0;                 // 整车质量，kg
        float spring_force = 0;         // 气弹簧标称推力，N
        float wheel_radius = 0;         // m
        float wheel_distance = 0;       // 左右轮距，m
        float active_link = 0;          // 主动杆长 AB=ED，m
        float passive_link = 0;         // 从动杆长 BC=DC，m
        float leg_min = 0, leg_max = 0; // 目标腿长范围，m；leg_min 也是自救站起与跳跃收腿腿长，leg_max 也是起跳腿长

        // 控制器，误差均为“目标减反馈”
        const Pid::Config* leg_length_pid; // 腿长 PD：m → 轴向力 N，伸腿为正；微分取 -d_L，不含目标变化
        const Pid::Config* leg_angle_pid;  // 相对腿角 PD：rad → 摆动力矩 N·m，后摆为正
        const Pid::Config* roll_pid;       // 横滚 PD：rad → 腿长修正 m；右侧低时输出为负，左腿缩短、右腿伸长

        // LQR 增益与误差限幅
        const float (*lqr_fit)[10][6]; // LQR 增益拟合系数 [4 输出][10 状态][6 项]，项序 1、Ll、Lr、Ll²、Ll·Lr、Lr²

        // LQR 目标与误差限幅
        float theta_ref = 0.07f;     // 目标腿倾角，rad，后摆为正
        float theta_err_max = 0.45f; // 腿倾角误差限幅，rad
        float pitch_err_max = 0.25f; // 俯仰误差限幅，rad
        float x_max = 0.3f;          // 位移限幅，m；重心偏移所需回位位移超出后会持续漂移

        // 倒地与自救
        float fall_theta_front = -63 * deg; // 相对腿角下限，rad，后摆为正；任一腿超出判定倒地
        float fall_theta_back = 63 * deg;   // 相对腿角上限，rad
        float rescue_stand_speed = 4.0f;    // 站起目标腿角速度，rad/s
        float rescue_flip_speed = 3.0f;     // 翻转目标腿角速度，rad/s

        // 跳跃
        float jump_takeoff_force = 350.0f;  // 单腿起跳力，N，伸腿为正
        float jump_retract_force = -350.0f; // 单腿收腿力，N，伸腿为正
        float jump_retract_time = 0.3f;     // 收腿保持时间，s
        float jump_buffer_length = 0.22f;   // 落地缓冲目标腿长，m
        float jump_buffer_time = 0.3f;      // 落地缓冲时间，s
    };

  private:
    // 倒地与自救
    enum class Fall {
        UPRIGHT, // 车体正向
        HEAD,    // 头部着地
        TAIL,    // 尾部着地
    };

    // 车体运动：姿态为传入 q 换算的标准 Z-Y-X 欧拉角，角速度取机体系 gyro；前向运动与 d_yaw_wheel 由轮腿运动学得出
    struct {
        float pitch = 0, d_pitch = 0; // rad、rad/s；低头为正（绕机体 +Y）；与外部工程 imu_pitch 同号
        float roll = 0, d_roll = 0;   // rad、rad/s；右侧低为正（绕机体 +X）；与外部工程 imu_roll 相反
        float d_yaw_gyro = 0;         // 陀螺仪航向角速度，rad/s；俯视逆时针为正（绕机体 +Z）；与外部工程 imu_d_yaw 同号
        float d_yaw_wheel = 0;        // 轮速航向角速度，rad/s，右侧前向速度较大（逆时针）为正
        float x = 0;                  // 前向位移积分，m，前进为正；有速度指令或动作期间清零
        float d_x = 0;                // 前向速度，m/s，前进为正
    } body;

    // 五连杆示意（站立姿态）：关节电机 A、E 同轴装于 O，A 驱动 AB、E 驱动 ED；C 为轮轴
    // 图中 x 向右、y 向下，杆方位角从 +x 向 +y（顺时针）为正；电机零位时 AB 指向 -x（phi1 = π）、ED 指向 +x（phi4 = 0）
    // 虚拟腿为 OC：L = |OC|，phi0 为 OC 方位角，正下方为 π/2
    //
    // 左腿：在车左侧面向关节电机输出轴看，车头在左
    //
    //   车头 <--                         --> 车尾
    //
    //                  O ------> +x       O：关节电机 A（left_back）、E（left_front）
    //    AB (phi1)    / \    ED (phi4)
    //                B   D
    //    BC (phi2)    \ /    DC (phi3)
    //                  C                  轮轴
    //                  |
    //                  v +y
    //
    // 右腿：在车右侧面向关节电机输出轴看，车头在右；与左腿镜像，同一套方位角定义
    //
    //   车尾 <--                         --> 车头
    //
    //                  O ------> +x       O：关节电机 A（right_front）、E（right_back）
    //    AB (phi1)    / \    ED (phi4)
    //                B   D
    //    BC (phi2)    \ /    DC (phi3)
    //                  C                  轮轴
    //                  |
    //                  v +y
    //
    // 单腿状态与输出；杆方位角从局部 +x 向 +y 为正，左右腿各自面向电机输出轴观察
    struct Leg {
        // 连杆角：由关节反馈换算
        float phi1 = 0, d_phi1 = 0; // 主动杆 AB（关节 A）方位角，rad、rad/s
        float phi4 = 0, d_phi4 = 0; // 主动杆 ED（关节 E）方位角，rad、rad/s
        float phi2 = 0, phi3 = 0;   // 从动杆 BC、DC 方位角，rad
        float phi_bod = 0;          // 主动杆夹角 ∠BOD = phi1 - phi4，rad，有符号
        float phi_obc = 0;          // 膝内角 ∠OBC，rad，[0, π]

        // 虚拟腿 OC
        float phi0 = 0, d_phi0 = 0; // 方位角，rad、rad/s，车体正下方为 π/2
        float L = 0, d_L = 0;       // 腿长 m；伸缩速度 m/s，伸腿为正

        // 腿角
        float rel_theta = 0;          // 相对底盘腿角，rad，车体正下方为零，后摆为正（左腿图中向右、右腿图中向左）
        float theta = 0, d_theta = 0; // 相对竖直腿倾角 = rel_theta + pitch，rad、rad/s，后摆为正

        // 前向速度分量
        float v_wheel = 0; // 车轮线速度，m/s，前进为正
        float v_swing = 0; // 腿摆动引起的前向速度补偿，m/s

        // 轴向前馈力
        float Fg = 0; // 单腿重力沿腿分量，N，非负
        float Fs = 0; // 气弹簧等效伸腿力，N，非负

        // 虚拟力：控制计算输出
        float Fn = 0; // 虚拟腿轴向力，N，伸腿为正
        float Tt = 0; // 虚拟腿摆动力矩，N·m，后摆为正

        // 电机力矩：由虚拟力映射，下发给电机
        float Ta = 0, Te = 0; // 关节力矩，N·m，电机正转为正，与 phi1、phi4 增大方向相反
        float Tw = 0;         // 轮端力矩，N·m，前进为正
    } leg[2];

    bool is_motor_ok = false; // 全部电机在线且关节已使能无报错；否则全部电机输出零

    // 质量与几何
    const float mass;           // 整车质量，kg
    const float spring_force;   // 气弹簧标称推力，N
    const float wheel_radius;   // m
    const float wheel_distance; // 左右轮距，m
    const float active_link;    // 主动杆长 AB=ED，m
    const float passive_link;   // 从动杆长 BC=DC，m
    const float leg_min;        // 目标腿长范围，m；leg_min 也是自救站起与跳跃收腿腿长，leg_max 也是起跳腿长
    const float leg_max;        // 目标腿长范围，m；leg_min 也是自救站起与跳跃收腿腿长，leg_max 也是起跳腿长

    // LQR 增益与误差限幅
    const float (*const lqr_fit)[10][6]; // LQR 增益拟合系数 [4 输出][10 状态][6 项]，项序 1、Ll、Lr、Ll²、Ll·Lr、Lr²
    const float theta_ref;               // 目标腿倾角，rad，后摆为正
    const float theta_err_max;           // 腿倾角误差限幅，rad
    const float pitch_err_max;           // 俯仰误差限幅，rad
    const float x_max;                   // 位移限幅，m；重心偏移所需回位位移超出后会持续漂移

    // 倒地与自救
    const float fall_theta_front;   // 相对腿角下限，rad，后摆为正；任一腿超出判定倒地
    const float fall_theta_back;    // 相对腿角上限，rad
    const float rescue_stand_speed; // 站起目标腿角速度，rad/s
    const float rescue_flip_speed;  // 翻转目标腿角速度，rad/s

    // 跳跃
    const float jump_takeoff_force; // 单腿起跳力，N，伸腿为正
    const float jump_retract_force; // 单腿收腿力，N，伸腿为正
    const float jump_retract_time;  // 收腿保持时间，s
    const float jump_buffer_length; // 落地缓冲目标腿长，m
    const float jump_buffer_time;   // 落地缓冲时间，s

    // 设备绑定
    struct Motors {
        DM8009 &left_front, &left_back, &right_back, &right_front; // 关节：面向输出轴逆时针为正；零位时 A 杆方位角 π、E 杆 0
        M3508 &left_wheel, &right_wheel;                           // 轮：转速与力矩均以车前进为正
    } motor;

    // 指令
    bool is_enable = false;
    float d_x_ref = 0, d_yaw_ref = 0;
    float leg_length_ref;
    bool allow_rescue = false;
    bool is_position = false; // 速度降到阈值内后开始位移积分
    bool trigger_jump = false, last_trigger_jump = false;

    struct {
        bool is_fallen = false;    // 倒地锁存，站起完成或失能时清除
        bool is_running = false;   // 自救运行中
        Fall fall = Fall::UPRIGHT; // 启动时的倒地方向
        int flip_step[2]{};        // 翻转阶段：0 进入翻转区间、1 继续翻转、2 完成
        int stand_step[2]{};       // 站起阶段：0 摆腿、1 收腿
        float target_theta[2]{};   // 目标相对腿角，rad，后摆为正
    } rescue;

    // 跳跃
    struct {
        bool is_active = false; // 跳跃运行中
        int step = 0;           // 0 起跳、1 收腿、2 收腿保持、3 落地缓冲
        float force = 0;        // 单腿跳跃力，N，伸腿为正
        float hold_time = 0;    // 阶段 2、3 已保持时间，s
    } jump;

    // 控制器与跨周期状态
    Pid leg_length_pid[2], leg_angle_pid[2], roll_pid;
    Timer update_timer;

  public:
    ChassisLeg(const Motors& motor, const Config& config);

    // 使能或失能全部电机
    void set_enable(bool is_enable);

    // 前向速度 m/s 前进为正；航向角速度 rad/s 俯视逆时针为正
    void set_speed(float d_x, float d_yaw);

    // 双腿共用目标腿长，m，限幅到 [leg_min, leg_max]
    void set_leg_length(float leg_length);

    // 倒地后允许启动自救；撤销不打断已运行的自救
    void set_allow_rescue(bool allow_rescue);

    // 跳跃触发，上升沿且空闲时启动
    void set_trigger_jump(bool trigger_jump);

    // 每个控制周期调用：q 为底盘机体到世界，gyro 为底盘机体系角速度 rad/s
    // 计算并设置全部电机力矩，发送由调用方完成
    void update(const Quaternionf& q, const Vector3f& gyro);

  private:
    // 反馈：电机在线状态、车体姿态、双腿几何与前馈力、轮速
    void update_feedback(const Quaternionf& q, const Vector3f& gyro);

    // 五连杆正运动学：由 phi1、phi4 及角速度更新虚拟腿几何与速度
    void solve_leg(Leg& leg) const;

    // 倒地检测与自救状态机
    void update_rescue(float dt);

    // 自救：站起为正向摆腿收腿；翻转完成后转入站起
    void rescue_stand(float dt);

    void rescue_flip(float side, float dt);

    // 跳跃状态机
    void update_jump(float dt);

    // 按失能、倒地、跳跃状态计算双腿虚拟力 Fn、Tt 与轮端力矩 Tw
    void calc_force(float dt);

    // 按目标腿长和横滚补偿计算双腿轴向力，含重力与弹簧补偿
    void balance_legs(float dt, float length);

    // LQR 平衡：按腿长插值增益，计算轮端力矩 Tw 与摆动力矩 Tt
    void calc_lqr();

    // 力映射：由 L、Fn 和机构极性的摆动力矩 Tt 更新 T_A、T_E
    void map_force(Leg& leg, float Tt) const;
};
