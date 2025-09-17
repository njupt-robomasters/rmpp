#pragma once

#include "bsp.hpp"
#include "unit.hpp"
#include "pid.hpp"

class IMU {
public:
    // 陀螺仪安装方向
    struct dir_t {
        float yaw, pitch, roll; // 单位：角度
    };

    // 陀螺仪校准参数
    struct calib_t {
        float gx_offset = 0, gy_offset = 0, gz_offset = 0;
        float g_norm = 1;
    };

private:
    // IMU加热
    class Temperature_Control {
        static constexpr Unit<C> ref = 40.0f;
        Unit<C> measure;
        Unit<percentage> power;

        PID::param_t pid_param = {.kp = 0.5, .ki = 0.01, .max_i = 0.4, .max_out = 1};
        PID pid;

    public:
        Temperature_Control() : pid(pid_param) {}
        void Update();
    } temperature_control{};

    dir_t param;
    calib_t calib;

    volatile bool is_ready = false;

    BSP::Dwt dwt; // 维护dt

    // bmi088原始数据
    float gyro[3]{}; // 角速度
    float accel[3]{}; // 加速度

    static void DIR_Correction(const dir_t& dir, float gyro[3], float accel[3]);

    static void BodyFrameToEarthFrame(const float vecBF[3], float vecEF[3], const float q[4]);

    static void EarthFrameToBodyFrame(const float vecEF[3], float vecBF[3], const float q[4]);

    static void QuaternionToEularAngle(const float q[4], float& yaw, float& pitch, float& roll);

    static void EularAngleToQuaternion(float yaw, float pitch, float roll, float q[4]);

    static void QuaternionUpdate(float q[], float gx, float gy, float gz, float dt);

public:
    // 欧拉角
    Angle<deg> yaw, pitch, roll;
    Unit<deg> yaw_total_angle;

    // 四元数
    float q[4]{};

    IMU(const dir_t& param, const calib_t& calib);

    void Init();

    void Calibrate();

    void WaitReady();

    void Update();
};
