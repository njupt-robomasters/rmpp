#pragma once

#include "bmi088.hpp"
#include "bsp/pwm.hpp"
#include "bsp/timer.hpp"
#include "math/angle.hpp"
#include "motor/pid.hpp"
#include "quaternion_ekf.hpp"
#include <tuple>

class Imu {
  public:
    struct Hardware {
        Spi accel;
        Spi gyro;
        Pwm heater;
    };

    struct Adjust {
        // 传感器到机体的固定旋转：Rz(yaw) Ry(pitch) Rx(roll)
        Angle yaw{}, pitch{}, roll{};
        float gyro_offset[3]{}; // 传感器坐标系零偏，rad/s
        float gravity = 9.81f;  // 标定时原始加速度模长，m/s²，必须 > 0
    };

    struct Control {
        float temperature = 45.0f; // °C
        Pid::Config temperature_pid{.kp = 1.0f, .ki = 0.01f, .max_i = 0.4f, .max_out = 1.0f};
        QuaternionEkf::Config ekf{};
    };

    struct Config {
        Hardware hardware;
        Adjust adjust{};
        Control control{};
    };

    enum class Calibration {
        IDLE,
        SAMPLING,
        SUCCEEDED,
        FAILED
    };
    static constexpr int X = 0, Y = 1, Z = 2;

    // 校准及安装修正后的机体系量；gyro 尚未减去 EKF 估计的剩余零偏
    float gyro[3]{};              // rad/s
    float accel[3]{};             // m/s²
    float q[4]{1, 0, 0, 0};       // [w, x, y, z]，机体到世界
    Angle yaw{}, pitch{}, roll{}; // Z-Y-X 欧拉角，rad
    float yaw_total_angle{};      // 累积偏航角，rad，不归一化
    float temperature{};          // °C
    float heater_duty{};          // 实际加热占空比，[0, 1]
    Calibration calibration = Calibration::IDLE;
    unsigned calibration_samples = 0;

    explicit Imu(const Config& config);
    // PID 借用内部配置，实例地址必须保持稳定
    Imu(const Imu&) = delete;
    Imu& operator=(const Imu&) = delete;

    const Config& config() const;
    void update();
    // 立即开始采样；由 update 推进，调用方负责热机和静止时机
    void start_calibration();

    // 右手系，roll-X、pitch-Y、yaw-Z；四元数转换接口要求单位四元数
    static void euler_to_quaternion(Angle yaw, Angle pitch, Angle roll, float q[4]);
    static std::tuple<Angle, Angle, Angle> quaternion_to_euler(const float q[4]); // yaw, pitch, roll
    // 向量转换支持输入输出使用同一数组
    static void body_to_world(const float body[3], float world[3], const float q[4]);
    static void world_to_body(const float world[3], float body[3], const float q[4]);
    static void integrate_quaternion(float q[4], float gx, float gy, float gz, float dt);

  private:
    Config config_;
    Bmi088 sensor_;
    QuaternionEkf ekf_;
    Pid temperature_pid_;
    Timer timer_;
    Timer calibration_timer_;
    bool initialized_ = false;
    float installation_q_[4]{};
    float calibration_mean_[4]{}; // 原始 gx、gy、gz、加速度模长
    float calibration_min_[4]{};
    float calibration_max_[4]{};
};
