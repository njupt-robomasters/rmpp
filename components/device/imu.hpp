#pragma once

#include "pid.hpp"

class IMU {
public:
    // 用于修正安装误差的参数
    struct param_t {
        float yaw = 0;
        float pitch = 0;
        float roll = 0;
        float scale[3] = {1, 1, 1};
        uint8_t flag = 1;
    };

    struct calib_t {
        float GxOFFSET = 0, GyOFFSET = 0, GzOFFSET = 0;
        float gNORM = 1;
    };

    float roll = 0;
    float pitch = 0;
    float yaw = 0;
    float yaw_total_angle = 0;

    IMU(const param_t &param, const calib_t &calib);

    void Init();

    void WaitReady() const;

    void Update();

private:
    // 可配置参数
    static constexpr float temperature_ref = 40; // 陀螺仪加热温度
    static constexpr float gravity[3] = {0, 0, 9.81f}; // 当地重力加速度
    static constexpr float accel_lpf = 0.0085f;

    static constexpr auto X = 0;
    static constexpr auto Y = 1;
    static constexpr auto Z = 2;

    static constexpr float xb[3] = {1, 0, 0};
    static constexpr float yb[3] = {0, 1, 0};
    static constexpr float zb[3] = {0, 0, 1};

    bool is_ready = false;

    param_t param;
    calib_t calib;

    // IMU原始数据
    float gyro[3]{}; // 角速度
    float accel[3]{}; // 加速度

    float q[4]{}; // 四元数

    float motion_accel_b[3]{}; // 机体坐标加速度
    float motion_accel_n[3]{}; // 绝对系加速度

    // 加速度在绝对系的向量表示
    float xn[3]{};
    float yn[3]{};
    float zn[3]{};

    float atanxz = 0;
    float atanyz = 0;

    // IMU加热
    PID::param_t temperature_pid_param = {.kp = 50, .ki = 1, .i_limit = 40, .max_out = 100};
    float temperature_measure = 0;
    PID temperature_pid;

    void IMU_Temperature_Ctrl();

    static void IMU_Param_Correction(param_t *param, float gyro[3], float accel[3]);

    static void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, const float *q);

    static void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, const float *q);

    static void QuaternionToEularAngle(const float *q, float *Yaw, float *Pitch, float *Roll);

    static void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);

    static void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt);
};
