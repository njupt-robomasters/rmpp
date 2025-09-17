#include "imu.hpp"
#include "bsp.hpp"
#include "bmi088.h"
#include "QuaternionEKF.h"

static constexpr auto X = 0;
static constexpr auto Y = 1;
static constexpr auto Z = 2;

void IMU::Temperature_Control::Update() {
    measure = BMI088.Temperature;
    const float temperature_err = ref - measure;
    power = pid.CalcPosition(temperature_err);
    BSP::IMUHeat::SetPower(power);
}

IMU::IMU(const dir_t& param, const calib_t& calib) : param(param), calib(calib) {}

void IMU::Init() {
    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0);
    BMI088_Init(&hspi1);
    BMI088_SetCalibrateParam(calib.gx_offset, calib.gy_offset, calib.gz_offset, calib.g_norm);
}

void IMU::Calibrate() {
    BMI088_Calibrate();
    calib = {
        .gx_offset = BMI088.GyroOffset[X],
        .gy_offset = BMI088.GyroOffset[Y],
        .gz_offset = BMI088.GyroOffset[Z],
        .g_norm = BMI088.gNorm
    };
}

void IMU::WaitReady() {
    is_ready = false;
    while (is_ready == false) {
        osDelay(1);
    }
}

void IMU::Update() {
    const float dt = dwt.GetDT();

    BMI088_Read(&BMI088);

    accel[X] = BMI088.Accel[X];
    accel[Y] = BMI088.Accel[Y];
    accel[Z] = BMI088.Accel[Z];
    gyro[X] = BMI088.Gyro[X];
    gyro[Y] = BMI088.Gyro[Y];
    gyro[Z] = BMI088.Gyro[Z];

    // 用于修正安装误差
    DIR_Correction(param, gyro, accel);

    // 核心函数,EKF更新四元数
    IMU_QuaternionEKF_Update(gyro[X], gyro[Y], gyro[Z], accel[X],
                             accel[Y], accel[Z], dt);

    memcpy(q, QEKF_INS.q, sizeof(QEKF_INS.q));

    yaw = QEKF_INS.Yaw * deg;
    pitch = QEKF_INS.Pitch * deg;
    roll = QEKF_INS.Roll * deg;
    yaw_total_angle = QEKF_INS.YawTotalAngle * deg;

    // imu加热
    temperature_control.Update();

    is_ready = true;
}

/**
 * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 *
 *
 * @param dir IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
void IMU::DIR_Correction(const dir_t& dir, float gyro[3], float accel[3]) {
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    static uint8_t flag = 1;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(dir.yaw - lastYawOffset) > 0.001f ||
        fabsf(dir.pitch - lastPitchOffset) > 0.001f ||
        fabsf(dir.roll - lastRollOffset) > 0.001f ||
        flag) {
        cosYaw = arm_cos_f32(dir.yaw / 57.295779513f);
        cosPitch = arm_cos_f32(dir.pitch / 57.295779513f);
        cosRoll = arm_cos_f32(dir.roll / 57.295779513f);
        sinYaw = arm_sin_f32(dir.yaw / 57.295779513f);
        sinPitch = arm_sin_f32(dir.pitch / 57.295779513f);
        sinRoll = arm_sin_f32(dir.roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        gyro_temp[i] = gyro[i];

    gyro[X] = c_11 * gyro_temp[X] +
        c_12 * gyro_temp[Y] +
        c_13 * gyro_temp[Z];
    gyro[Y] = c_21 * gyro_temp[X] +
        c_22 * gyro_temp[Y] +
        c_23 * gyro_temp[Z];
    gyro[Z] = c_31 * gyro_temp[X] +
        c_32 * gyro_temp[Y] +
        c_33 * gyro_temp[Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        accel_temp[i] = accel[i];

    accel[X] = c_11 * accel_temp[X] +
        c_12 * accel_temp[Y] +
        c_13 * accel_temp[Z];
    accel[Y] = c_21 * accel_temp[X] +
        c_22 * accel_temp[Y] +
        c_23 * accel_temp[Z];
    accel[Z] = c_31 * accel_temp[X] +
        c_32 * accel_temp[Y] +
        c_33 * accel_temp[Z];

    lastYawOffset = dir.yaw;
    lastPitchOffset = dir.pitch;
    lastRollOffset = dir.roll;
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void IMU::BodyFrameToEarthFrame(const float vecBF[3], float vecEF[3], const float q[4]) {
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

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void IMU::EarthFrameToBodyFrame(const float vecEF[3], float vecBF[3], const float q[4]) {
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

/**
 * @brief        Convert quaternion to eular angle
 */
void IMU::QuaternionToEularAngle(const float q[4], float& yaw, float& pitch, float& roll) {
    yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion
 */
void IMU::EularAngleToQuaternion(float yaw, float pitch, float roll, float q[4]) {
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    yaw /= 57.295779513f;
    pitch /= 57.295779513f;
    roll /= 57.295779513f;
    cosPitch = arm_cos_f32(pitch / 2);
    cosYaw = arm_cos_f32(yaw / 2);
    cosRoll = arm_cos_f32(roll / 2);
    sinPitch = arm_sin_f32(pitch / 2);
    sinYaw = arm_sin_f32(yaw / 2);
    sinRoll = arm_sin_f32(roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}

/**
 * @brief        Update quaternion
 */
void IMU::QuaternionUpdate(float q[4], float gx, float gy, float gz, float dt) {
    float qa, qb, qc;
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += -qb * gx - qc * gy - q[3] * gz;
    q[1] += qa * gx + qc * gz - q[3] * gy;
    q[2] += qa * gy - qb * gz + q[3] * gx;
    q[3] += qa * gz + qb * gy - qc * gx;
}
