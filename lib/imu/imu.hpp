#pragma once

#include "bmi088.hpp"
#include "bsp/pwm.hpp"
#include "bsp/timer.hpp"
#include "math/angle.hpp"
#include "pid/pid.hpp"
#include "quaternion_ekf.hpp"

class Imu {
  public:
    struct Config {
        struct Hardware {
            Bmi088 sensor;
            Pwm heater;
        } const hardware;

        struct Adjust {
            Quaternion orientation = Quaternion::Identity(); // 传感器到机体
            Vector3 gyro_offset = Vector3::Zero();           // 传感器坐标系零偏，rad/s
            float gravity = 9.81f;                           // 标定时原始加速度模长，m/s²
        } adjust;

        struct Control {
            float temperature = 45.0f; // °C
            Pid::Config temperature_pid_config = {.kp = 1.0f, .ki = 0.01f, .max_i = 0.4f, .max_out = 1.0f};
            QuaternionEkf::Config ekf;
        } control;
    } config;

    // 校准及安装修正后的机体系量
    Vector3 gyro = Vector3::Zero();        // rad/s
    Vector3 accel = Vector3::Zero();       // m/s²
    Quaternion q = Quaternion::Identity(); // 机体到世界
    Angle yaw, pitch, roll;                // Z-Y-X 欧拉角，rad
    float yaw_total_angle = 0.0f;          // 累积偏航角，rad，不归一化

    explicit Imu(const Config& config);

    void update();

    // 立即开始采样；由 update 推进，调用方负责热机和静止时机
    void start_calibration();

    // 右手系，roll-X、pitch-Y、yaw-Z；四元数转换接口要求单位四元数
    static Quaternion euler_to_quaternion(const EulerAngles& euler);
    static EulerAngles quaternion_to_euler(const Quaternion& q);
    static Vector3 body_to_world(const Vector3& body, const Quaternion& q);
    static Vector3 world_to_body(const Vector3& world, const Quaternion& q);

  private:
    bool initialized_ = false;
    Bmi088 sensor_;
    QuaternionEkf ekf_;

    // 用于陀螺仪加热
    Pid temperature_pid_;
    float temperature{}; // °C
    float heater_duty{}; // 实际加热占空比，[0, 1]

    // 用于陀螺仪校准
    enum class Calibration {
        IDLE,
        STARTING,
        SAMPLING,
        SUCCEEDED,
        FAILED
    } calibration_ = Calibration::IDLE;
    Timer calibration_timer_;
    uint32_t calibration_samples_ = 0;
    Vector4 calibration_mean_ = Vector4::Zero();
    Vector4 calibration_min_ = Vector4::Zero();
    Vector4 calibration_max_ = Vector4::Zero();

    void update_heater();
    void update_calibration();
};
