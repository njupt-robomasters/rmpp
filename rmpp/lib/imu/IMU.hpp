#pragma once

#include "algorithm/PID.hpp"
#include "lib/ekf/QuaternionEKF.hpp"
#include "lib/bmi088/bmi088.hpp"


class IMU {
public :

    static constexpr auto X = 0;
    static constexpr auto Y = 1;
    static constexpr auto Z = 2;

    // 安装方向参数
    struct dir_t {
        Angle<deg> yaw, pitch, roll;
    };
    // 校准参数
    struct calib_t {
        float gx_offset = 0, gy_offset = 0, gz_offset = 0;
        float g_norm = 1;
    };


    float gyro[3]{};  // 角速度
    float accel[3]{}; // 加速度

    // 四元数
    float q[4]{};

    // 欧拉角
    Angle<deg> yaw, pitch, roll;
    UnitFloat<deg> yaw_total_angle;

    // 增加一个温度（方便调试）
    UnitFloat<C> temperature;

    // 构造函数
    IMU(const dir_t& dir, calib_t& calib, const BMI088::Config& hw_config);

    // 校准陀螺仪
    void Calibrate();

    // 欧拉角转四元数
    static void EularAngleToQuaternion(const Angle<>& yaw, const Angle<>& pitch, const Angle<>& roll, float q[4]);

    // 四元数转欧拉角
    static std::tuple<Angle<>, Angle<>, Angle<>> QuaternionToEularAngle(const float q[4]);

    // 机体坐标系 -> 世界坐标系
    static void BodyFrameToEarthFrame(const float vecBF[3], float vecEF[3], const float q[4]);

    // 世界坐标系 -> 机体坐标系
    static void EarthFrameToBodyFrame(const float vecEF[3], float vecBF[3], const float q[4]);

    // 更新四元数
    static void QuaternionUpdate(float q[4], float gx, float gy, float gz, float dt);

    // 需要在循环中调用
    void OnLoop();

private:
    // imu加热
    class Temperature_Control {
    public:
        Temperature_Control() : pid(&pid_config) {}

        // 需要在循环中调用
        void OnLoop(const UnitFloat<C>& current_temp);

    private:
        static constexpr UnitFloat<> ref = 45.0f * C; // 目标温度
        UnitFloat<C> measure;
        UnitFloat<pct> power;

        PID::config_t pid_config = {
            .kp = 100.0f * (pct / C),
            .ki = 1.0f * (pct / C),
            .max_i = 40.0f * pct,
            .max_out = 100.0f * pct
        };
        PID pid;
    } temperature_control{};

    dir_t dir;     // 安装方向参数
    calib_t calib; // 校准参数

    bool is_init = false;  // 用于自动初始化陀螺仪

    BSP::Dwt dwt;

    // 创建IMU的EKF的实例
    QuaternionEKF ekf;


    BMI088 bmi088;

    // 初始化
    bool init();

    // 不用static
    void dirCorrect(const dir_t& dir, float gyro[3], float accel[3]);

    dir_t last_dir;
    bool dir_first_flag = true;
    float c_11 = 0, c_12 = 0, c_13 = 0;
    float c_21 = 0, c_22 = 0, c_23 = 0;
    float c_31 = 0, c_32 = 0, c_33 = 0;
};


