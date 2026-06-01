#include "QuaternionEKF.hpp"
#include "ekf_utils.hpp"

QuaternionEKF::QuaternionEKF() {
    std::memset(IMU_QuaternionEKF_F_Backup, 0, sizeof(IMU_QuaternionEKF_F_Backup));
    for (int i = 0; i < 6; i++) {
        IMU_QuaternionEKF_F_Backup[i * 6 + i] = 1.0f;
    }
}

void QuaternionEKF::Init(float process_noise1, float process_noise2, float measure_noise, float lambda_val, float lpf) {
    Initialized = true;
    Q1 = process_noise1;
    Q2 = process_noise2;
    R_noise = measure_noise;
    ChiSquareTestThreshold = 1e-8f;
    lambda = (lambda_val > 1.0f) ? 1.0f : lambda_val;
    accLPFcoef = lpf;

    arm_mat_init_f32(&ChiSquare, 1, 1, ChiSquare_Data);

    xhat_data[0] = 1.0f; xhat_data[1] = 0.0f;
    xhat_data[2] = 0.0f; xhat_data[3] = 0.0f;

    User_Func0_f = Observe_Callback;
    User_Func1_f = F_Linearization_Callback;
    User_Func2_f = SetH_Callback;
    User_Func3_f = xhatUpdate_Callback;

    SkipEq3 = true;
    SkipEq4 = true;

    std::memcpy(F_data, IMU_QuaternionEKF_F_Backup, sizeof(F_data));

    for (int i = 0; i < 6; i++) {
        P_data[i * 6 + i] = (i < 4) ? 100000.0f : 100.0f;
    }
}

void QuaternionEKF::UpdateEKF(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    if (!Initialized) Init(10, 0.001f, 10000000.0f, 1.0f, 0.0f);

    dt_sec = dt;

    Gyro[0] = gx - GyroBias[0];
    Gyro[1] = gy - GyroBias[1];
    Gyro[2] = gz - GyroBias[2];

    float halfgxdt = 0.5f * Gyro[0] * dt;
    float halfgydt = 0.5f * Gyro[1] * dt;
    float halfgzdt = 0.5f * Gyro[2] * dt;

    std::memcpy(F_data, IMU_QuaternionEKF_F_Backup, sizeof(F_data));

    F_data[1] = -halfgxdt; F_data[2] = -halfgydt; F_data[3] = -halfgzdt;
    F_data[6] = halfgxdt;  F_data[8] = halfgzdt;  F_data[9] = -halfgydt;
    F_data[12] = halfgydt; F_data[13] = -halfgzdt; F_data[15] = halfgxdt;
    F_data[18] = halfgzdt; F_data[19] = halfgydt; F_data[20] = -halfgxdt;

    if (UpdateCount == 0) {
        Accel[0] = ax; Accel[1] = ay; Accel[2] = az;
    }
    float lpf_factor = dt / (dt + accLPFcoef);
    Accel[0] += (ax - Accel[0]) * lpf_factor;
    Accel[1] += (ay - Accel[1]) * lpf_factor;
    Accel[2] += (az - Accel[2]) * lpf_factor;

    float accelInvNorm = MathUtils::invSqrt(Accel[0] * Accel[0] + Accel[1] * Accel[1] + Accel[2] * Accel[2]);
    MeasuredVector[0] = Accel[0] * accelInvNorm;
    MeasuredVector[1] = Accel[1] * accelInvNorm;
    MeasuredVector[2] = Accel[2] * accelInvNorm;

    bool valid[3] = {true, true, true};

    gyro_norm = 1.0f / MathUtils::invSqrt(Gyro[0] * Gyro[0] + Gyro[1] * Gyro[1] + Gyro[2] * Gyro[2]);
    accl_norm = 1.0f / accelInvNorm;
    StableFlag = (gyro_norm < 0.3f && accl_norm > 9.3f && accl_norm < 10.3f);

    Q_data[0] = Q1 * dt; Q_data[7] = Q1 * dt; Q_data[14] = Q1 * dt; Q_data[21] = Q1 * dt;
    Q_data[28] = Q2 * dt; Q_data[35] = Q2 * dt;
    R_data[0] = R_noise; R_data[4] = R_noise; R_data[8] = R_noise;

    Update(MeasuredVector, valid);

    q[0] = xhat_data[0]; q[1] = xhat_data[1];
    q[2] = xhat_data[2]; q[3] = xhat_data[3];
    GyroBias[0] = xhat_data[4];
    GyroBias[1] = xhat_data[5];
    GyroBias[2] = 0.0f;

    Yaw = std::atan2(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.2957795f;
    Pitch = std::atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.2957795f;
    Roll = std::asin(MathUtils::abs_limit(-2.0f * (q[1] * q[3] - q[0] * q[2]), 1.0f)) * 57.2957795f;

    if (Yaw - YawAngleLast > 180.0f) YawRoundCount--;
    else if (Yaw - YawAngleLast < -180.0f) YawRoundCount++;

    YawTotalAngle = 360.0f * YawRoundCount + Yaw;
    YawAngleLast = Yaw;
    UpdateCount++;
}

void QuaternionEKF::Observe_Callback(Algorithm::KalmanFilter<6, 0, 3>* kf) {}

void QuaternionEKF::F_Linearization_Callback(Algorithm::KalmanFilter<6, 0, 3>* kf) {
    auto* ekf = static_cast<QuaternionEKF*>(kf);

    float q0 = ekf->xhatminus_data[0], q1 = ekf->xhatminus_data[1];
    float q2 = ekf->xhatminus_data[2], q3 = ekf->xhatminus_data[3];

    float qInvNorm = MathUtils::invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    for (uint8_t i = 0; i < 4; i++) ekf->xhatminus_data[i] *= qInvNorm;

    float half_dt = ekf->dt_sec * 0.5f;
    ekf->F_data[4] = q1 * half_dt;   ekf->F_data[5] = q2 * half_dt;
    ekf->F_data[10] = -q0 * half_dt; ekf->F_data[11] = q3 * half_dt;
    ekf->F_data[16] = -q3 * half_dt; ekf->F_data[17] = -q0 * half_dt;
    ekf->F_data[22] = q2 * half_dt;  ekf->F_data[23] = -q1 * half_dt;

    ekf->P_data[28] /= ekf->lambda;
    ekf->P_data[35] /= ekf->lambda;

    if (ekf->P_data[28] > 10000.0f) ekf->P_data[28] = 10000.0f;
    if (ekf->P_data[35] > 10000.0f) ekf->P_data[35] = 10000.0f;
}

void QuaternionEKF::SetH_Callback(Algorithm::KalmanFilter<6, 0, 3>* kf) {
    float q0 = kf->xhatminus_data[0], q1 = kf->xhatminus_data[1];
    float q2 = kf->xhatminus_data[2], q3 = kf->xhatminus_data[3];

    std::memset(kf->H_data, 0, sizeof(float) * 18);
    kf->H_data[0] = -2.0f*q2; kf->H_data[1] =  2.0f*q3; kf->H_data[2] = -2.0f*q0; kf->H_data[3] =  2.0f*q1;
    kf->H_data[6] =  2.0f*q1; kf->H_data[7] =  2.0f*q0; kf->H_data[8] =  2.0f*q3; kf->H_data[9] =  2.0f*q2;
    kf->H_data[12]=  2.0f*q0; kf->H_data[13]= -2.0f*q1; kf->H_data[14]= -2.0f*q2; kf->H_data[15]=  2.0f*q3;
}

void QuaternionEKF::xhatUpdate_Callback(Algorithm::KalmanFilter<6, 0, 3>* kf) {
    auto* ekf = static_cast<QuaternionEKF*>(kf);

    ekf->MatStatus = arm_mat_trans_f32(&ekf->H, &ekf->HT);
    ekf->temp_matrix.numRows = ekf->H.numRows;
    ekf->temp_matrix.numCols = ekf->Pminus.numCols;
    ekf->MatStatus = arm_mat_mult_f32(&ekf->H, &ekf->Pminus, &ekf->temp_matrix);

    ekf->temp_matrix1.numRows = ekf->temp_matrix.numRows;
    ekf->temp_matrix1.numCols = ekf->HT.numCols;
    ekf->MatStatus = arm_mat_mult_f32(&ekf->temp_matrix, &ekf->HT, &ekf->temp_matrix1);

    ekf->S.numRows = ekf->R.numRows;
    ekf->S.numCols = ekf->R.numCols;
    ekf->MatStatus = arm_mat_add_f32(&ekf->temp_matrix1, &ekf->R, &ekf->S);

    ekf->MatStatus = arm_mat_inverse_f32(&ekf->S, &ekf->temp_matrix1);

    float q0 = ekf->xhatminus_data[0], q1 = ekf->xhatminus_data[1];
    float q2 = ekf->xhatminus_data[2], q3 = ekf->xhatminus_data[3];

    ekf->temp_vector.numRows = ekf->H.numRows;
    ekf->temp_vector.numCols = 1;
    ekf->temp_vector_data[0] = 2.0f * (q1 * q3 - q0 * q2);
    ekf->temp_vector_data[1] = 2.0f * (q0 * q1 + q2 * q3);
    ekf->temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    for (uint8_t i = 0; i < 3; i++) {
        ekf->OrientationCosine[i] = std::acos(std::abs(ekf->temp_vector_data[i]));
    }

    ekf->temp_vector1.numRows = ekf->z.numRows;
    ekf->temp_vector1.numCols = 1;
    ekf->MatStatus = arm_mat_sub_f32(&ekf->z, &ekf->temp_vector, &ekf->temp_vector1);

    ekf->temp_matrix.numRows = ekf->temp_vector1.numRows;
    ekf->temp_matrix.numCols = 1;
    ekf->MatStatus = arm_mat_mult_f32(&ekf->temp_matrix1, &ekf->temp_vector1, &ekf->temp_matrix);

    ekf->temp_vector.numRows = 1;
    ekf->temp_vector.numCols = ekf->temp_vector1.numRows;
    ekf->MatStatus = arm_mat_trans_f32(&ekf->temp_vector1, &ekf->temp_vector);
    ekf->MatStatus = arm_mat_mult_f32(&ekf->temp_vector, &ekf->temp_matrix, &ekf->ChiSquare);

    if (ekf->ChiSquare_Data[0] < 0.5f * ekf->ChiSquareTestThreshold) {
        ekf->ConvergeFlag = true;
    }

    if (ekf->ChiSquare_Data[0] > ekf->ChiSquareTestThreshold && ekf->ConvergeFlag) {
        if (ekf->StableFlag) ekf->ErrorCount++;
        else ekf->ErrorCount = 0;

        if (ekf->ErrorCount > 50) {
            ekf->ConvergeFlag = false;
            ekf->SkipEq5 = false;
        } else {
            std::memcpy(ekf->xhat_data, ekf->xhatminus_data, sizeof(float) * 6);
            std::memcpy(ekf->P_data, ekf->Pminus_data, sizeof(float) * 36);
            ekf->SkipEq5 = true;
            return;
        }
    } else {
        if (ekf->ChiSquare_Data[0] > 0.1f * ekf->ChiSquareTestThreshold && ekf->ConvergeFlag) {
            ekf->AdaptiveGainScale = (ekf->ChiSquareTestThreshold - ekf->ChiSquare_Data[0]) / (0.9f * ekf->ChiSquareTestThreshold);
        } else {
            ekf->AdaptiveGainScale = 1.0f;
        }
        ekf->ErrorCount = 0;
        ekf->SkipEq5 = false;
    }

    ekf->temp_matrix.numRows = ekf->Pminus.numRows;
    ekf->temp_matrix.numCols = ekf->HT.numCols;
    ekf->MatStatus = arm_mat_mult_f32(&ekf->Pminus, &ekf->HT, &ekf->temp_matrix);
    ekf->MatStatus = arm_mat_mult_f32(&ekf->temp_matrix, &ekf->temp_matrix1, &ekf->K);

    for (uint8_t i = 0; i < ekf->K.numRows * ekf->K.numCols; i++) {
        ekf->K_data[i] *= ekf->AdaptiveGainScale;
    }
    for (uint8_t i = 4; i < 6; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            ekf->K_data[i * 3 + j] *= ekf->OrientationCosine[i - 4] / 1.5707963f;
        }
    }

    ekf->temp_vector.numRows = ekf->K.numRows;
    ekf->temp_vector.numCols = 1;
    ekf->MatStatus = arm_mat_mult_f32(&ekf->K, &ekf->temp_vector1, &ekf->temp_vector);

    if (ekf->ConvergeFlag) {
        for (uint8_t i = 4; i < 6; i++) {
            if (ekf->temp_vector_data[i] >  1e-2f * ekf->dt_sec) ekf->temp_vector_data[i] =  1e-2f * ekf->dt_sec;
            if (ekf->temp_vector_data[i] < -1e-2f * ekf->dt_sec) ekf->temp_vector_data[i] = -1e-2f * ekf->dt_sec;
        }
    }

    ekf->temp_vector_data[3] = 0.0f;

    ekf->MatStatus = arm_mat_add_f32(&ekf->xhatminus, &ekf->temp_vector, &ekf->xhat);
}