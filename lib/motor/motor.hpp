#pragma once

#include "controller/pid.hpp"

class Motor {
public:
    // 闭环模式
    enum pid_mode_e {
        OPEN_LOOP_MODE, // 开环模式，直接设置电流/扭矩
        SPEED_MODE,     // 速度模式
        ANGLE_MODE,     // 角度模式
    };

    // PID输出类型
    enum pid_type_e {
        CURRENT_TYPE, // PID输出电流
        TORQUE_TYPE,  // PID输出扭矩
    };

    bool is_enable = false; // 电机使能标志
    bool is_online = false; // 电机在线标识

    // 电流
    struct {
        UnitFloat<A> ref, measure, raw; // raw为电机反转处理前的值
    } current;

    // 扭矩
    struct {
        UnitFloat<Nm> ref, measure, raw; // raw为电机反转处理前的值
    } torque;

    // 转速
    struct {
        UnitFloat<rpm> ref, measure, raw; // raw为电机反转处理前的值
    } speed;

    // 角度（减速比!=1时，自动执行软件多圈记数）
    struct {
        Angle<deg> ref, measure, raw, last_raw; // raw为电机反转、安装偏移处理前的值
    } angle;

    // 设置电机使能/失能
    void SetEnable(bool is_enable);

    // 电机本体设置
    void SetReduction(float reduction); // 设置电机减速比
    void SetKt(const UnitFloat<>& Kt);  // 设置电机扭矩系数

    // 电机安装参数
    void SetInvert(bool is_invert);                                                                                         // 设置电机反转
    void SetOffset(const Angle<>& offset);                                                                                  // 设置电机安装偏移，限位模式下必须是电机运动范围中心位置
    void SetLimit(bool is_limit, const Angle<>& limit_min = 0 * default_unit, const Angle<>& limit_max = 0 * default_unit); // 设置电机限位

    // 设置PID
    void SetPID(pid_mode_e pid_mode, pid_type_e pid_type, PID::param_t* pid_param);

    // 设置CAN发送频率
    void SetCanSendFreq(const UnitFloat<>& can_send_freq);

    // 设置电流
    void SetCurrent(const UnitFloat<>& current);

    // 设置扭矩
    void SetTorque(const UnitFloat<>& torque);

    // 设置速度
    void SetSpeed(const UnitFloat<>& speed);

    // 设置角度
    void SetAngle(const Angle<>& angle, const UnitFloat<>& speed_ff = 0 * default_unit); // &&为广义引用，达到限位会修改传入的值

    // 需要在循环中调用，子类重写后要调用
    void OnLoop();

protected:
    static constexpr float TIMEOUT = 0.01f;

    // 电机本体参数
    float reduction = 1.0f;           // 电机减速比
    UnitFloat<Nm_A> Kt = 1.0f * Nm_A; // 力矩系数

    // 电机安装参数
    bool is_invert = false;          // 电机反转标志
    Angle<deg> offset;               // 电机角度偏移量（如果是限位模式，需要是电机运动范围的中心位置）
    bool is_limit = false;           // false-圆周模式，true-限位模式
    Angle<deg> limit_min, limit_max; // min-右限位（负值），max-左限位（正值），相对于offset的角度

    // PID相关
    pid_mode_e pid_mode = OPEN_LOOP_MODE; // 闭环模式
    pid_type_e pid_type = CURRENT_TYPE;   // PID输出类型
    PID pid;                              // PID控制器

    // CAN发送频率
    UnitFloat<Hz> can_send_freq = 1000.0f * Hz;

    BSP::Dwt dwt_can_send_freq; // 用于控制CAN报文发送频率

    // CAN接收回调，子类重写后要调用
    void callback();

private:
    BSP::Dwt dwt_online; // 用于电机掉线检测，也可以参考CAN接收频率
};