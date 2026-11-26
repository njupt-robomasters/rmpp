#pragma once

#include "bsp/timer.hpp"
#include "math/eigen.hpp"
#include "math/unit.hpp"
#include "motor/dm8009.hpp"
#include "motor/m3508.hpp"
#include "pid/pid.hpp"

// 双轮腿底盘：五连杆 VMC + LQR 平衡，含跳跃与倒地自救；LQR 输出按四路额定力矩与双轮净机械功率分配
// 坐标：底盘机体系 X 前、Y 左、Z 上；双腿数组 [0] 左、[1] 右
// 姿态由调用方每周期传入底盘机体的 q 与 gyro，可来自本板 IMU 或云台换算的虚拟 IMU
class ChassisLeg {
  private:
    struct Motors {
        DM8009 &left_front, &left_back, &right_back, &right_front; // 关节：面向输出轴逆时针为正；零位时 A 杆方位角 π、E 杆 0
        M3508 &left_wheel, &right_wheel;                           // 轮：转速与力矩均以车前进为正
    } motor;

  public:
    struct Config {
        // 质量与几何
        float mass = 22.75f;                    // 整车质量，kg
        float spring_force = 450.0f;            // 气弹簧标称推力，N
        float wheel_radius = 0.06f;             // 轮半径，m
        float wheel_distance = 0.488f;          // 左右轮距，m
        float active_link = 0.210f;             // 主动杆长 AB=ED，m
        float passive_link = 0.248f;            // 从动杆长 BC=DC，m
        float leg_min = 0.14f, leg_max = 0.33f; // 目标腿长范围，m；leg_min 也是自救站起与跳跃收腿腿长，leg_max 也是起跳腿长

        // 控制器，误差均为"目标减反馈"
        const Pid::Config* leg_length_pid; // 腿长 PD：m → 轴向力 N，伸腿为正；微分取 -d_L，不含目标变化
        const Pid::Config* leg_angle_pid;  // 相对腿角 PD：rad → 摆动力矩 N·m，后摆为正
        const Pid::Config* roll_pid;       // 横滚 PD：rad → 腿长修正 m；右侧低时输出为负，左腿缩短、右腿伸长

        // LQR 增益与目标
        const float (*lqr_fit)[10][6];     // LQR 增益拟合系数 [4 输出][10 状态][6 项]，项序 1、Ll、Lr、Ll²、Ll·Lr、Lr²
        const float (*lqr_posture_fit)[6]; // 两腿稳姿态的俯仰角速度增益拟合系数 [左摆、右摆][6 项]，项序同 lqr_fit；功率分配用
        const float* lqr_r;                // LQR 出力权重 R 对角 [左轮、右轮、左摆、右摆]，与 lqr_fit 同一组；功率分配用
        float theta_ref = 0.07f;           // 目标腿倾角，rad，后摆为正
        float x_max = 0.1f;                // 位移积分限幅，m
        float wheel_work_speed = 65.0f;    // 轮子额定转速，rad/s；直行目标上限 = 该值 × 轮半径，边走边转按较快一侧轮缘等比缩小

        // 倒地与自救
        float fall_theta_front = -63 * deg; // 相对腿角下限，rad，后摆为正；任一腿超出判定倒地
        float fall_theta_back = 63 * deg;   // 相对腿角上限，rad
        float rescue_stand_speed = 4.0f;    // 站起目标腿角速度，rad/s
        float rescue_flip_speed = 3.0f;     // 翻转目标腿角速度，rad/s

        // 跳跃
        float jump_takeoff_force = 350.0f;  // 单腿起跳力，N，伸腿为正
        float jump_retract_force = -350.0f; // 单腿收腿力，N，伸腿为正
        float jump_retract_time = 0.2f;     // 收腿保持时间，s
        float jump_buffer_length = 0.22f;   // 落地缓冲目标腿长，m
        float jump_buffer_time = 0.5f;      // 落地缓冲时间，s
    } const config;

  private:
    // 控制指令，由 set_* 写入
    bool allow_rescue = false; // 倒地后允许启动自救；撤销不打断已运行的自救
    bool trigger_jump = false; // 跳跃触发，上升沿且空闲时启动
    float vx = 0, wr = 0;      // 前向速度 m/s 前进为正；航向角速度目标 rad/s 逆时针为正，小陀螺为手动角速度、跟随为云台 yaw 目标速度前馈
    float gimbal_yaw = 0;      // 云台相对底盘航向，rad；无云台、断联或小陀螺时为 0，不追踪航向
    float leg_length;          // 双腿目标腿长，m
    float power_ref = 0;       // 双轮净机械功率上限 Σ Tw·ω，W；INFINITY 不限

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
        float axial_gain = 0;       // 轴向力到单关节力矩的系数 0.5·L·tan∠OCB，N·m/N，轴向力形成反向关节力矩

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

        // 虚拟力：VMC 控制计算输出
        float Fn = 0; // 虚拟腿轴向力，N，伸腿为正
        float Tt = 0; // 虚拟腿摆动力矩，N·m，后摆为正

        // 电机力矩：由虚拟力映射，下发给电机
        float Ta = 0, Te = 0; // 关节力矩，N·m，电机正转为正，与 phi1、phi4 增大方向相反
        float Tw = 0;         // 轮端力矩，N·m，前进为正
    } leg[2];

    bool is_motor_online = false; // 六电机反馈在线、四关节使能且无报错；任一不满足时全部输出零

  public:
    // 车体运动与位移积分，外部只读；姿态为传入 q 换算的标准 Z-Y-X 欧拉角，角速度取机体系 gyro，前向运动与 d_yaw_wheel 由轮腿运动学得出
    struct {
        float pitch = 0, d_pitch = 0; // rad、rad/s；低头为正（绕机体 +Y）；与外部工程 imu_pitch 同号
        float roll = 0, d_roll = 0;   // rad、rad/s；右侧低为正（绕机体 +X）；与外部工程 imu_roll 相反
        float d_yaw_gyro = 0;         // 陀螺仪航向角速度，rad/s；俯视逆时针为正（绕机体 +Z）
        float d_yaw_wheel = 0;        // 轮速航向角速度，rad/s，右侧前向速度较大（逆时针）为正
        float x = 0;                  // 前向位移积分，m，前进为正；有速度指令或动作期间清零
        float d_x = 0;                // 前向速度，m/s，前进为正
        bool is_position = false;     // 速度降到 0.1 m/s 内后开始位移积分
    } body;

    // 底盘使能状态，外部只读
    bool is_enable = false;

    // 倒地与自救，外部只读 state
    struct Rescue {
        // 自救阶段；UPRIGHT 为默认值 0，rescue = {} 可复位
        enum class State { UPRIGHT = 0, // 站立，正常控制
                           FALLEN,      // 已倒地，等待启动许可
                           RUNNING };   // 自救动作执行中

        // 倒地方向
        enum class FallDir { NORMAL, // 正向倒地，横滚 < 90°
                             HEAD,   // 头部着地，横滚 > 90° 且低头
                             TAIL }; // 尾部着地，横滚 > 90° 且抬头

        State state = State::UPRIGHT;  // 当前自救阶段；站起完成、失能或跳跃时复位
        FallDir dir = FallDir::NORMAL; // 启动时的倒地方向，RUNNING 状态下有效
        int flip_step[2]{};            // 翻转阶段：0 进入翻转区间、1 继续翻转、2 完成
        int stand_step[2]{};           // 站起阶段：0 摆腿、1 收腿
        float target_theta[2]{};       // 目标相对腿角，rad，后摆为正
    } rescue;

  private:
    // 跳跃；IDLE 为默认值 0，jump = {} 可复位
    struct Jump {
        enum class State { IDLE = 0, // 未激活
                           TAKEOFF,  // 全力伸腿，伸到 leg_max
                           RETRACT,  // 全力收腿，收到 leg_min
                           HOLD,     // 保持收腿腿长
                           BUFFER }; // 落地缓冲

        State state = State::IDLE; // 当前跳跃阶段
        float force = 0;           // 单腿跳跃力，N，伸腿为正
        float hold_time = 0;       // 阶段 HOLD、BUFFER 已保持时间，s
    } jump;

    // 功率：额定力矩分配后估计的双轮净机械功率 Σ Tw·ω，W；超过 power_ref 时修正
    float power_estimate = 0;

    // 跨周期状态
    bool last_trigger_jump = false; // 上拍跳跃触发状态，用于取上升沿；不复位，按住触发只跳一次

    // 控制器
    Pid leg_length_pid[2], leg_angle_pid[2], roll_pid;

    Timer update_timer;

  public:
    ChassisLeg(const Motors& motor, const Config& config);

    // 使能或失能全部电机
    void set_enable(bool is_enable);

    // 前向速度 m/s 前进为正；航向角速度目标 rad/s 俯视逆时针为正，小陀螺为手动角速度，跟随为云台 yaw 目标速度前馈
    // 两个目标按轮子额定转速乘轮半径等比限速，存储值不会超限
    void set_speed(float vx, float wr);

    // 云台相对底盘航向，rad；无云台、断联或小陀螺时传 0，0 即不追踪航向
    void set_gimbal_yaw(float gimbal_yaw);

    // 双腿共用目标腿长，m，限幅到 [leg_min, leg_max]
    void set_leg_length(float leg_length);

    // 倒地后允许启动自救；撤销不打断已运行的自救
    void set_allow_rescue(bool allow_rescue);

    // 跳跃触发，上升沿且空闲时启动
    void set_trigger_jump(bool trigger_jump);

    // 双轮净机械功率上限 Σ Tw·ω，W；通常取超电 power_chassis_ref，INFINITY 不限；关节功率与铜耗不计
    void set_power_ref(float power_ref);

    // 每个控制周期调用：q 为底盘机体到世界，gyro 为底盘机体系角速度 rad/s
    // 计算并设置全部电机力矩，发送由调用方完成
    void update(const Quaternionf& q, const Vector3f& gyro);

  private:
    // 反馈：电机、车体姿态、双腿几何、车体运动
    void update_motor();
    void update_attitude(const Quaternionf& q, const Vector3f& gyro);
    void update_leg(Leg& leg, float mirror);
    void update_motion(float dt);

    // 腿长控制：目标腿长加横滚补偿，输出双腿轴向力 Fn
    void control_leg_length(float dt, float length);

    // 倒地检测与自救状态机
    void update_rescue(float dt);

    // 自救：站起为正向摆腿收腿；翻转完成后转入站起
    void rescue_stand(float dt);
    void rescue_flip(float side, float dt);

    // 跳跃状态机
    void update_jump(float dt);

    // 按失能、倒地、跳跃状态计算双腿虚拟力 Fn、Tt 与轮端力矩 Tw，并映射成关节力矩 Ta、Te
    void update_force(float dt);

    // LQR 平衡：按目标求四路力矩，额定力矩分配与功率限制后写入轮端力矩 Tw 与摆动力矩 Tt
    void calc_lqr();

    // 四路额定区间：轮取电机力矩上限，腿摆取关节余量的一半
    void calc_torque_limit(float lower[4], float upper[4]) const;

    // 分配：姿态组优先，运动组按最大可行比例让位；四路写入 out
    void alloc_torque(const float base[4], const float motion[4], const float lower[4], const float upper[4], float out[4]) const;

    // 功率：双轮净机械功率超限时按 R 加权最短方向削减轮力矩，缺口由两腿摆动力矩补上
    void limit_power(const float terms[6], const float k[4][10], const float lower[4], const float upper[4], float out[4]);

    // 分配：四路满足 [lower, upper] 且比例不超过 1 时返回最大可行比例，无解返回 -1
    static float maximum_share(const float base[4], const float change[4], const float lower[4], const float upper[4]);

    // VMC 力映射：由轴向力系数与机构极性的摆动力矩 Tt 更新 Ta、Te
    void vmc_map(Leg& leg, float Tt) const;
};
