#include "imu.hpp"

#include "bsp/pwm.hpp"
#include "lib/bmi088/bmi088.h"
#include "lib/bmi088/bmi088_middleware.h"
#include "lib/ekf/QuaternionEKF.h"
#include "math/angle.hpp"
#include "math/unit.hpp"
#include <cmath>
#include <tuple>

Imu::Imu(const Dir& dir, Calib& calib, const Config& config) : temperature_control_(config.heater), config_(config), dir_(dir), calib_(calib) {
    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0);
}

void Imu::calibrate() {
    if (!config_.accel || !config_.gyro)
        return;
    if (is_init_ == false) {
        init();
        is_init_ = true;
    }

    BMI088_Calibrate();
    calib_ = {
        .gx_offset = BMI088.GyroOffset[X],
        .gy_offset = BMI088.GyroOffset[Y],
        .gz_offset = BMI088.GyroOffset[Z],
        .g_norm = BMI088.gNorm};
}

void Imu::on_loop() {
    if (!config_.accel || !config_.gyro)
        return;
    if (is_init_ == false) {
        init();
        is_init_ = true;
    }

    // 计算dt
    const float dt = timer_.update_dt();

    // 读取bmi088数据
    BMI088_Read(&BMI088);
    accel[X] = BMI088.Accel[X];
    accel[Y] = BMI088.Accel[Y];
    accel[Z] = BMI088.Accel[Z];
    gyro[X] = BMI088.Gyro[X];
    gyro[Y] = BMI088.Gyro[Y];
    gyro[Z] = BMI088.Gyro[Z];

    // 安装方向修正
    dir_correct(dir_, gyro, accel);

    // 核心函数，EKF更新四元数
    IMU_QuaternionEKF_Update(gyro[X], gyro[Y], gyro[Z],
                             accel[X], accel[Y], accel[Z],
                             dt);

    // 获取四元数
    memcpy(q, QEKF_INS.q, sizeof(QEKF_INS.q));

    // 获取欧拉角
    yaw = QEKF_INS.Yaw * deg;
    pitch = QEKF_INS.Pitch * deg;
    roll = QEKF_INS.Roll * deg;
    yaw_total_angle = QEKF_INS.YawTotalAngle * deg;

    // imu加热
    temperature_control_.on_loop();
}

void Imu::init() const {
    bmi088_bind(config_.accel, config_.gyro);
    BMI088_Init();
    BMI088_SetCalibrateParam(calib_.gx_offset, calib_.gy_offset, calib_.gz_offset, calib_.g_norm);
}

void Imu::dir_correct(const Dir& dir, float gyro[3], float accel[3]) {
    static Dir last_dir;
    static bool first_flag = true;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;

    if (first_flag ||
        dir.yaw != last_dir.yaw ||
        dir.pitch != last_dir.pitch ||
        dir.roll != last_dir.roll) {
        const float cos_yaw = arm_cos_f32(dir.yaw);
        const float cos_pitch = arm_cos_f32(dir.pitch);
        const float cos_roll = arm_cos_f32(dir.roll);
        const float sin_yaw = arm_sin_f32(dir.yaw);
        const float sin_pitch = arm_sin_f32(dir.pitch);
        const float sin_roll = arm_sin_f32(dir.roll);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll;
        c_12 = cos_pitch * sin_yaw;
        c_13 = cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch;
        c_21 = cos_yaw * sin_pitch * sin_roll - cos_roll * sin_yaw;
        c_22 = cos_yaw * cos_pitch;
        c_23 = -sin_yaw * sin_roll - cos_yaw * cos_roll * sin_pitch;
        c_31 = -cos_pitch * sin_roll;
        c_32 = sin_pitch;
        c_33 = cos_pitch * cos_roll;

        first_flag = false;
    }

    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; i++) {
        gyro_temp[i] = gyro[i];
    }
    gyro[X] = c_11 * gyro_temp[X] + c_12 * gyro_temp[Y] + c_13 * gyro_temp[Z];
    gyro[Y] = c_21 * gyro_temp[X] + c_22 * gyro_temp[Y] + c_23 * gyro_temp[Z];
    gyro[Z] = c_31 * gyro_temp[X] + c_32 * gyro_temp[Y] + c_33 * gyro_temp[Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; i++) {
        accel_temp[i] = accel[i];
    }
    accel[X] = c_11 * accel_temp[X] + c_12 * accel_temp[Y] + c_13 * accel_temp[Z];
    accel[Y] = c_21 * accel_temp[X] + c_22 * accel_temp[Y] + c_23 * accel_temp[Z];
    accel[Z] = c_31 * accel_temp[X] + c_32 * accel_temp[Y] + c_33 * accel_temp[Z];

    last_dir = dir;
}

void Imu::eular_angle_to_quaternion(Angle yaw, Angle pitch, Angle roll, float q[4]) {
    const float cos_pitch = arm_cos_f32(pitch / 2);
    const float cos_yaw = arm_cos_f32(yaw / 2);
    const float cos_roll = arm_cos_f32(roll / 2);
    const float sin_pitch = arm_sin_f32(pitch / 2);
    const float sin_yaw = arm_sin_f32(yaw / 2);
    const float sin_roll = arm_sin_f32(roll / 2);
    q[0] = cos_pitch * cos_roll * cos_yaw + sin_pitch * sin_roll * sin_yaw;
    q[1] = sin_pitch * cos_roll * cos_yaw - cos_pitch * sin_roll * sin_yaw;
    q[2] = sin_pitch * cos_roll * sin_yaw + cos_pitch * sin_roll * cos_yaw;
    q[3] = cos_pitch * cos_roll * sin_yaw - sin_pitch * sin_roll * cos_yaw;
}

std::tuple<Angle, Angle, Angle> Imu::quaternion_to_eular_angle(const float q[4]) {
    Angle yaw = std::atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    Angle pitch = std::atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
    Angle roll = std::asinf(2.0f * (q[0] * q[2] - q[1] * q[3]));
    return {yaw, pitch, roll};
}

void Imu::body_frame_to_earth_frame(const float vecBF[3], float vecEF[3], const float q[4]) {
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

void Imu::earth_frame_to_body_frame(const float vecEF[3], float vecBF[3], const float q[4]) {
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

void Imu::quaternion_update(float q[4], float gx, float gy, float gz, const float dt) {
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    const float qa = q[0];
    const float qb = q[1];
    const float qc = q[2];
    q[0] += -qb * gx - qc * gy - q[3] * gz;
    q[1] += qa * gx + qc * gz - q[3] * gy;
    q[2] += qa * gy - qb * gz + q[3] * gx;
    q[3] += qa * gz + qb * gy - qc * gx;
}

void Imu::TemperatureControl::on_loop() {
    measure_ = BMI088.Temperature;
    power_ = pid_.calculate(REF - measure_);
    heater_.set_duty(power_);
}
