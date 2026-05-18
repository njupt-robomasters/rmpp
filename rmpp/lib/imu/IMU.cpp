#include "IMU.hpp"
#include <tuple>
#include "bsp/bsp.hpp"
#include "lib/ekf/ekf_utils.hpp"

IMU::IMU(const dir_t& dir, calib_t& calib, const BMI088::Config& hw_config)
    : dir(dir), calib(calib), bmi088(hw_config) {
    ekf.Init(10.0f, 0.001f, 10000000.0f, 1.0f, 0.0f); // 需要修改的参数
}

void IMU::Calibrate() {
    if (is_init == false) {
        init();
        is_init = true;
    }

    bmi088.Calibrate();

    calib.gx_offset = bmi088.gyro_offset[X];
    calib.gy_offset = bmi088.gyro_offset[Y];
    calib.gz_offset = bmi088.gyro_offset[Z];
    calib.g_norm    = bmi088.g_norm;
}

void IMU::OnLoop() {
    if (is_init == false) {
        init();
        is_init = true;
    }
    // 计算dt
    const UnitFloat<> dt = dwt.UpdateDT();

    // 读取自己对象的 bmi088 数据（带单位）
    bmi088.Read();

    gyro[X] = bmi088.data.gyro[X].toFloat(rad_s);
    gyro[Y] = bmi088.data.gyro[Y].toFloat(rad_s);
    gyro[Z] = bmi088.data.gyro[Z].toFloat(rad_s);

    accel[X] = bmi088.data.accel[X].toFloat(m_ss);
    accel[Y] = bmi088.data.accel[Y].toFloat(m_ss);
    accel[Z] = bmi088.data.accel[Z].toFloat(m_ss);

    // 安装方向修正
    dirCorrect(dir, gyro, accel);

    // 核心函数，EKF更新四元数
    ekf.UpdateEKF(gyro[X], gyro[Y], gyro[Z],
                  accel[X], accel[Y], accel[Z],
                  dt.toFloat(s));

    // 获取四元数
    std::memcpy(q, ekf.q, sizeof(ekf.q));

    // 获取欧拉角
    yaw = ekf.Yaw * deg;
    pitch = ekf.Pitch * deg;
    roll = ekf.Roll * deg;
    yaw_total_angle = ekf.YawTotalAngle * deg;
    temperature = bmi088.data.temperature;

    // imu加热
    temperature_control.OnLoop(bmi088.data.temperature);
}

// 去掉了 const
void IMU::init() {
    bmi088.Init();

    bmi088.gyro_offset[X] = calib.gx_offset;
    bmi088.gyro_offset[Y] = calib.gy_offset;
    bmi088.gyro_offset[Z] = calib.gz_offset;
    bmi088.g_norm = calib.g_norm;
    bmi088.accel_scale = 9.81f / calib.g_norm;
}

// 不用static
void IMU::dirCorrect(const dir_t& dir, float gyro[3], float accel[3]) {
    if (dir_first_flag ||
        dir.yaw != last_dir.yaw ||
        dir.pitch != last_dir.pitch ||
        dir.roll != last_dir.roll) {

        const float cos_yaw = arm_cos_f32(dir.yaw.toFloat(rad));
        const float cos_pitch = arm_cos_f32(dir.pitch.toFloat(rad));
        const float cos_roll = arm_cos_f32(dir.roll.toFloat(rad));
        const float sin_yaw = arm_sin_f32(dir.yaw.toFloat(rad));
        const float sin_pitch = arm_sin_f32(dir.pitch.toFloat(rad));
        const float sin_roll = arm_sin_f32(dir.roll.toFloat(rad));

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

        dir_first_flag = false;
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

void IMU::EularAngleToQuaternion(const Angle<>& yaw, const Angle<>& pitch, const Angle<>& roll, float q[4]) {
    const float cos_pitch = arm_cos_f32(pitch.toFloat(rad) / 2);
    const float cos_yaw = arm_cos_f32(yaw.toFloat(rad) / 2);
    const float cos_roll = arm_cos_f32(roll.toFloat(rad) / 2);
    const float sin_pitch = arm_sin_f32(pitch.toFloat(rad) / 2);
    const float sin_yaw = arm_sin_f32(yaw.toFloat(rad) / 2);
    const float sin_roll = arm_sin_f32(roll.toFloat(rad) / 2);
    q[0] = cos_pitch * cos_roll * cos_yaw + sin_pitch * sin_roll * sin_yaw;
    q[1] = sin_pitch * cos_roll * cos_yaw - cos_pitch * sin_roll * sin_yaw;
    q[2] = sin_pitch * cos_roll * sin_yaw + cos_pitch * sin_roll * cos_yaw;
    q[3] = cos_pitch * cos_roll * sin_yaw - sin_pitch * sin_roll * cos_yaw;
}

std::tuple<Angle<>, Angle<>, Angle<>> IMU::QuaternionToEularAngle(const float q[4]) {
    Angle<deg> e_yaw = std::atan2(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * rad;
    Angle<deg> e_pitch = std::atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * rad;
    Angle<deg> e_roll = std::asin(MathUtils::abs_limit(-2.0f * (q[1] * q[3] - q[0] * q[2]), 1.0f)) * rad;
    return {e_yaw, e_pitch, e_roll};
}

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

void IMU::QuaternionUpdate(float q[4], float gx, float gy, float gz, const float dt) {
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


void IMU::Temperature_Control::OnLoop(const UnitFloat<C>& current_temp) {
    measure = current_temp;
    power = pid.Calculate(ref - measure);
    BSP::IMUHeat::SetPower(power.toFloat(ratio));
}