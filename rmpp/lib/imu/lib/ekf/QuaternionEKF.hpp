#pragma once

#include "KalmanFilter.hpp"


class QuaternionEKF : public Algorithm::KalmanFilter<6, 0, 3> {
public:
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float GyroBias[3] = {0};
    float Gyro[3] = {0};
    float Accel[3] = {0};
    float OrientationCosine[3] = {0};

    float Roll = 0.0f;
    float Pitch = 0.0f;
    float Yaw = 0.0f;
    float YawTotalAngle = 0.0f;

private:
    bool Initialized = false;
    bool ConvergeFlag = false;
    bool StableFlag = false;
    uint32_t ErrorCount = 0;
    uint32_t UpdateCount = 0;

    float accLPFcoef = 0.0f;
    float gyro_norm = 0.0f;
    float accl_norm = 0.0f;
    float AdaptiveGainScale = 1.0f;

    float Q1 = 0.0f;
    float Q2 = 0.0f;
    float R_noise = 0.0f;
    float dt_sec = 0.0f;

    arm_matrix_instance_f32 ChiSquare;
    float ChiSquare_Data[1] = {0};
    float ChiSquareTestThreshold = 1e-8f;
    float lambda = 1.0f;

    int16_t YawRoundCount = 0;
    float YawAngleLast = 0.0f;

    float IMU_QuaternionEKF_F_Backup[36] = {0};

public:
    QuaternionEKF();

    void Init(float process_noise1, float process_noise2, float measure_noise, float lambda_val, float lpf);

    void UpdateEKF(float gx, float gy, float gz, float ax, float ay, float az, float dt);

private:
    static void Observe_Callback(Algorithm::KalmanFilter<6, 0, 3>* kf);
    static void F_Linearization_Callback(Algorithm::KalmanFilter<6, 0, 3>* kf);
    static void SetH_Callback(Algorithm::KalmanFilter<6, 0, 3>* kf);
    static void xhatUpdate_Callback(Algorithm::KalmanFilter<6, 0, 3>* kf);
};