#pragma once

#include "algorithm/PID.hpp"

class Motor {
public:
    // 电机闭环模式
    enum control_mode_e {
        OPEN_LOOP_MODE,   // 开环电流/力矩控制
        SPEED_MODE,       // 速度闭环
        ANGLE_MODE,       // 角度闭环
        ANGLE_SPEED_MODE, // 角度-角度闭环（串级PID）
    };

    // PID输出类型
    enum pid_out_type_e {
        CURRENT_OUTPUT, // PID输出电流
        TORQUE_OUTPUT,  // PID输出扭矩
    };

    // 电机配置
    struct config_t {
        // CAN通信参数
        uint8_t can_port = -1;
        uint32_t master_id = 0, slave_id = 0;
        UnitFloat<> timeout = 100 * ms; // 电机断联超时时间

        // 电机本体参数
        float reduction = 1.0f;       // 减速比
        UnitFloat<> Kt = 1.0f * Nm_A; // 力矩系数
        UnitFloat<> R = 0.0f * Ohm;   // 相电阻（两相间电阻/2）

        // 电机安装参数
        bool is_invert = false;       // 电机反转标志
        Angle<> offset;               // 电机角度偏移量
        bool is_limit = false;        // false-圆周模式，true-限位模式
        Angle<> limit_min, limit_max; // min-右限位（负值），max-左限位（正值），相对于offset的角度

        // PID相关
        control_mode_e control_mode = OPEN_LOOP_MODE;    // 电机闭环模式
        pid_out_type_e pid_out_type = CURRENT_OUTPUT;    // PID输出类型
        const PID::config_t* speed_pid_config = nullptr; // 速度PID参数
        const PID::config_t* angle_pid_config = nullptr; // 角度PID参数
    } config;

    bool is_connect = false; // 电机连接状态
    bool is_enable = false;  // 电机使能状态

    // 电流
    struct {
        UnitFloat<A> ref, measure;
    } current;

    // 扭矩
    struct {
        UnitFloat<Nm> ref, measure;
    } torque;

    // 转速
    struct {
        UnitFloat<rpm> ref, outer, measure;
    } speed;

    // 角度（减速比!=1时，自动执行软件多圈记数）
    struct {
        Angle<deg> ref, measure;
    } angle;

    // 温度
    struct {
        UnitFloat<C> mos, motor;
    } temperature;

    Motor(const config_t& config);

    // 设置电机使能/失能
    void SetEnable(bool is_enable);

    // 设置电流
    void SetCurrent(const UnitFloat<>& current);

    // 设置扭矩
    void SetTorque(const UnitFloat<>& torque);

    // 设置速度
    void SetSpeed(const UnitFloat<>& speed);

    // 设置角度
    Angle<> SetAngle(const Angle<>& angle, const UnitFloat<>& speed_ff = 0 * default_unit);

    // 需要在循环中调用
    virtual void OnLoop();

    virtual void SendCanCmd() {}

protected:
    // 电机反转、偏移修正之前的值
    struct raw_t {
        UnitFloat<A> current;
        UnitFloat<Nm> torque;
        UnitFloat<rpm> speed;
        Angle<deg> angle;
    };

    PID speed_pid, angle_pid; // PID控制器

    BSP::Dwt dwt_connect; // 用于电机断联检测

    // CAN接收回调，子类重写后要调用
    void callback(const raw_t& raw);

private:
    raw_t raw;             // 用于debug校正电机
    Angle<deg> last_angle; // 用于多圈计数
    bool is_first_setangle = false;
};