#pragma once

#include "algorithm/pid.hpp"
#include "bsp/pwm.hpp"
#include "bsp/spi.hpp"
#include "bsp/timer.hpp"
#include "math/angle.hpp"
#include "math/unit.hpp"
#include <tuple>

class Imu {
  public:
    static constexpr auto X = 0;
    static constexpr auto Y = 1;
    static constexpr auto Z = 2;

    struct Config {
        Spi accel;
        Spi gyro;
        Pwm heater;
    };

    // 安装方向参数
    struct Dir {
        Angle yaw, pitch, roll;
    };

    // 校准参数
    struct Calib {
        float gx_offset = 0, gy_offset = 0, gz_offset = 0;
        float g_norm = 1;
    };

    // 陀螺仪原始数据
    float gyro[3]{};  // rad/s
    float accel[3]{}; // m/s^2

    // 四元数
    float q[4]{};

    // 欧拉角
    Angle yaw, pitch, roll;
    float yaw_total_angle{}; // rad, unwrapped

    Imu(const Dir& dir, Calib& calib, const Config& config);

    // 校准陀螺仪，校准数据存放在calib成员变量
    void calibrate();

    // 欧拉角转四元数
    static void eular_angle_to_quaternion(Angle yaw, Angle pitch, Angle roll, float q[4]);

    // 四元数转欧拉角
    static std::tuple<Angle, Angle, Angle> quaternion_to_eular_angle(const float q[4]);

    // 机体坐标系 -> 世界坐标系
    static void body_frame_to_earth_frame(const float vecBF[3], float vecEF[3], const float q[4]);

    // 世界坐标系 -> 机体坐标系
    static void earth_frame_to_body_frame(const float vecEF[3], float vecBF[3], const float q[4]);

    // 更新四元数
    static void quaternion_update(float q[4], float gx, float gy, float gz, float dt);

    // 需要在循环中调用
    void on_loop();

  private:
    // imu加热
    class TemperatureControl {
      public:
        explicit TemperatureControl(Pwm heater) : pid_(&pid_config_), heater_(heater) {}

        // 需要在循环中调用
        void on_loop();

      private:
        static constexpr float REF = 45.0f; // Celsius
        float measure_{};
        float power_{};

        Pid::Config pid_config_ = {
            .kp = 100.0f * (pct),
            .ki = 1.0f * (pct),
            .max_i = 40.0f * pct,
            .max_out = 100.0f * pct};
        Pid pid_;
        Pwm heater_;
    } temperature_control_;

    Config config_;
    Dir dir_;     // 安装方向参数
    Calib calib_; // 校准参数

    bool is_init_ = false; // 用于自动初始化陀螺仪

    Timer timer_; // 用于计算dt

    // 初始化陀螺仪
    void init() const;

    // 安装方向修正
    static void dir_correct(const Dir& dir, float gyro[3], float accel[3]);
};
