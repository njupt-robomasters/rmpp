#pragma once

#include <cstdint>
#include <cstring>
#include "arm_math.h"

// DSP库的宏定义
#define mat arm_matrix_instance_f32
#define mat_status arm_status
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32

// // 兼容老代码的全局变量宏定义
// inline uint16_t sizeof_float = sizeof(float);
// inline uint16_t sizeof_double = sizeof(double);

namespace Algorithm {
    // X_DIM 状态维度，U_DIM 输入维度，Z_DIM 观测维度
    template <uint8_t X_DIM, uint8_t U_DIM, uint8_t Z_DIM>
    class KalmanFilter {
    private:
        // 计算最大维度，在编译期分配安全的缓存
        static constexpr uint8_t MAX_DIM = (X_DIM > Z_DIM) ? X_DIM : Z_DIM;
        static constexpr uint8_t SAFE_U_DIM = (U_DIM > 0) ? U_DIM : 1;

    public:
        // ================= 数据内存区 （舍弃malloc）=================
        float FilteredValue[X_DIM] = {0};        // 外部最终读取的平滑结果
        float MeasuredVector[Z_DIM] = {0};       // 原始传感器度数
        float ControlVector[SAFE_U_DIM] = {0};

        const uint8_t xhatSize = X_DIM;
        const uint8_t uSize = U_DIM;
        const uint8_t zSize = Z_DIM;

        uint8_t UseAutoAdjustment = 0;            // 是否使用动态维度调整
        uint8_t MeasurementValidNum = 0;          // 当前正常的传感器数量

        uint8_t MeasurementMap[Z_DIM] = {0};       // 传感器对应的状态变量
        float MeasurementDegree[Z_DIM] = {0};      // 观测矩阵的系数值
        float MatR_DiagonalElements[Z_DIM] = {0};  // 传感器的噪声协方差
        float StateMinVariance[X_DIM] = {0};       // 状态最小方差
        uint8_t temp[Z_DIM] = {0};                  // 索引缓存
        // 黄金五式的开启/跳过控制位
        uint8_t SkipEq1 = 0, SkipEq2 = 0, SkipEq3 = 0, SkipEq4 = 0, SkipEq5 = 0;

        mat xhat, xhatminus, u, z, P, Pminus, F, FT, B, H, HT, Q, R, K;
        mat S, temp_matrix, temp_matrix1, temp_vector, temp_vector1;
        mat_status MatStatus;

        // 回调函数绑定
        using FuncPtr = void (*)(KalmanFilter<X_DIM, U_DIM, Z_DIM>*);
        FuncPtr User_Func0_f = nullptr, User_Func1_f = nullptr, User_Func2_f = nullptr;
        FuncPtr User_Func3_f = nullptr, User_Func4_f = nullptr, User_Func5_f = nullptr, User_Func6_f = nullptr;

        float xhat_data[X_DIM] = {0};
        float xhatminus_data[X_DIM] = {0};
        float u_data[SAFE_U_DIM] = {0};
        float z_data[Z_DIM] = {0};
        float P_data[X_DIM * X_DIM] = {0};
        float Pminus_data[X_DIM * X_DIM] = {0};
        float F_data[X_DIM * X_DIM] = {0};
        float FT_data[X_DIM * X_DIM] = {0};
        float B_data[X_DIM * SAFE_U_DIM] = {0};
        float H_data[Z_DIM * X_DIM] = {0};
        float HT_data[X_DIM * Z_DIM] = {0};
        float Q_data[X_DIM * X_DIM] = {0};
        float R_data[Z_DIM * Z_DIM] = {0};
        float K_data[X_DIM * Z_DIM] = {0};

        float S_data[Z_DIM * Z_DIM] = {0};
        float temp_matrix_data[MAX_DIM * MAX_DIM] = {0};
        float temp_matrix_data1[MAX_DIM * MAX_DIM] = {0};
        float temp_vector_data[MAX_DIM] = {0};
        float temp_vector_data1[MAX_DIM] = {0};

        KalmanFilter() {
            Matrix_Init(&xhat, X_DIM, 1, xhat_data);
            Matrix_Init(&xhatminus, X_DIM, 1, xhatminus_data);
            if constexpr (U_DIM > 0) {
                Matrix_Init(&u, U_DIM, 1, u_data);
                Matrix_Init(&B, X_DIM, U_DIM, B_data);
            }
            Matrix_Init(&z, Z_DIM, 1, z_data);
            Matrix_Init(&P, X_DIM, X_DIM, P_data);
            Matrix_Init(&Pminus, X_DIM, X_DIM, Pminus_data);
            Matrix_Init(&F, X_DIM, X_DIM, F_data);
            Matrix_Init(&FT, X_DIM, X_DIM, FT_data);
            Matrix_Init(&H, Z_DIM, X_DIM, H_data);
            Matrix_Init(&HT, X_DIM, Z_DIM, HT_data);
            Matrix_Init(&Q, X_DIM, X_DIM, Q_data);
            Matrix_Init(&R, Z_DIM, Z_DIM, R_data);
            Matrix_Init(&K, X_DIM, Z_DIM, K_data);

            Matrix_Init(&S, Z_DIM, Z_DIM, S_data);
            Matrix_Init(&temp_matrix, X_DIM, X_DIM, temp_matrix_data);
            Matrix_Init(&temp_matrix1, X_DIM, X_DIM, temp_matrix_data1);
            Matrix_Init(&temp_vector, X_DIM, 1, temp_vector_data);
            Matrix_Init(&temp_vector1, X_DIM, 1, temp_vector_data1);
        }
        // 状态预测
        void xhatMinusUpdate() {
            if (!SkipEq1) {
                if constexpr (U_DIM > 0) {
                    temp_vector.numRows = X_DIM; temp_vector.numCols = 1;
                    MatStatus = Matrix_Multiply(&F, &xhat, &temp_vector);
                    temp_vector1.numRows = X_DIM; temp_vector1.numCols = 1;
                    MatStatus = Matrix_Multiply(&B, &u, &temp_vector1);
                    MatStatus = Matrix_Add(&temp_vector, &temp_vector1, &xhatminus);
                } else {
                    MatStatus = Matrix_Multiply(&F, &xhat, &xhatminus);
                }
            }
        }
        // 预测误差
        void PminusUpdate() {
            if (!SkipEq2) {
                MatStatus = Matrix_Transpose(&F, &FT);
                MatStatus = Matrix_Multiply(&F, &P, &Pminus);
                temp_matrix.numRows = Pminus.numRows; temp_matrix.numCols = FT.numCols;
                MatStatus = Matrix_Multiply(&Pminus, &FT, &temp_matrix);
                MatStatus = Matrix_Add(&temp_matrix, &Q, &Pminus);
            }
        }
        // 计算卡尔曼增益K
        bool SetK() {
            if (!SkipEq3) {
                MatStatus = Matrix_Transpose(&H, &HT);
                temp_matrix.numRows = H.numRows; temp_matrix.numCols = Pminus.numCols;
                MatStatus = Matrix_Multiply(&H, &Pminus, &temp_matrix);
                temp_matrix1.numRows = temp_matrix.numRows; temp_matrix1.numCols = HT.numCols;
                MatStatus = Matrix_Multiply(&temp_matrix, &HT, &temp_matrix1);
                S.numRows = R.numRows; S.numCols = R.numCols;
                MatStatus = Matrix_Add(&temp_matrix1, &R, &S);

                MatStatus = Matrix_Inverse(&S, &temp_matrix1);
                if (MatStatus != ARM_MATH_SUCCESS) return false; // 奇异矩阵保护

                temp_matrix.numRows = Pminus.numRows; temp_matrix.numCols = HT.numCols;
                MatStatus = Matrix_Multiply(&Pminus, &HT, &temp_matrix);
                MatStatus = Matrix_Multiply(&temp_matrix, &temp_matrix1, &K);  // K = P' * HT * inv(s)
            }
            return true;
        }
        // 融合修正
        void xhatUpdate() {
            if (!SkipEq4) {
                temp_vector.numRows = H.numRows; temp_vector.numCols = 1;
                MatStatus = Matrix_Multiply(&H, &xhatminus, &temp_vector);
                temp_vector1.numRows = z.numRows; temp_vector1.numCols = 1;
                MatStatus = Matrix_Subtract(&z, &temp_vector, &temp_vector1);
                temp_vector.numRows = K.numRows; temp_vector.numCols = 1;
                MatStatus = Matrix_Multiply(&K, &temp_vector1, &temp_vector);
                MatStatus = Matrix_Add(&xhatminus, &temp_vector, &xhat);
            }
        }
        // 收敛P
        void P_Update() {
            if (!SkipEq5) {
                temp_matrix.numRows = K.numRows; temp_matrix.numCols = H.numCols;
                temp_matrix1.numRows = temp_matrix.numRows; temp_matrix1.numCols = Pminus.numCols;
                MatStatus = Matrix_Multiply(&K, &H, &temp_matrix);
                MatStatus = Matrix_Multiply(&temp_matrix, &Pminus, &temp_matrix1);
                MatStatus = Matrix_Subtract(&Pminus, &temp_matrix1, &P);
            }
        }

        // ================= 大循环调用 =================
        // 用bool值is_valid 判断 ，原版是用0.0判断
        void Update(const float* modern_z = nullptr, const bool* is_valid = nullptr) {
            if (modern_z != nullptr && is_valid != nullptr) PrepareMeasurementModern(modern_z, is_valid);
            else PrepareMeasurementLegacy();

            if (User_Func0_f) User_Func0_f(this);

            xhatMinusUpdate();                    // 预测状态
            if (User_Func1_f) User_Func1_f(this);

            PminusUpdate();                        // 预测误差协方差
            if (User_Func2_f) User_Func2_f(this);
            // 只有当有传感器数据，且S矩阵求逆成功，才执行融合修正
            if ((MeasurementValidNum != 0 || UseAutoAdjustment == 0) && SetK()) {
                if (User_Func3_f) User_Func3_f(this);

                xhatUpdate();                      // 融合修正状态
                if (User_Func4_f) User_Func4_f(this);

                P_Update();                         // 收敛误差协方差
            } else {
                memcpy(xhat_data, xhatminus_data, sizeof(float) * X_DIM);
                memcpy(P_data, Pminus_data, sizeof(float) * X_DIM * X_DIM);
            }

            if (User_Func5_f) User_Func5_f(this);

            // 防过度收敛托底保护
            for (uint8_t i = 0; i < X_DIM; i++) {
                if (P_data[i * X_DIM + i] < StateMinVariance[i]) P_data[i * X_DIM + i] = StateMinVariance[i];
            }

            memcpy(FilteredValue, xhat_data, sizeof(float) * X_DIM);
            if (User_Func6_f) User_Func6_f(this);
        }

        float GetState(uint8_t index) const { return (index < X_DIM) ? xhat_data[index] : 0.0f; }
        void ResetState(uint8_t index, float value) { if (index < X_DIM) xhat_data[index] = value; }

    private:
        void PrepareMeasurementLegacy() {
            if (!UseAutoAdjustment) {
                memcpy(z_data, MeasuredVector, sizeof(float) * Z_DIM);
                MeasurementValidNum = Z_DIM;
            } else {
                MeasurementValidNum = 0;
                memset(R_data, 0, sizeof(float) * Z_DIM * Z_DIM);
                memset(H_data, 0, sizeof(float) * X_DIM * Z_DIM);
                for (uint8_t i = 0; i < Z_DIM; i++) {
                    if (MeasuredVector[i] != 0.0f) {
                        z_data[MeasurementValidNum] = MeasuredVector[i];
                        temp[MeasurementValidNum] = i;
                        H_data[X_DIM * MeasurementValidNum + MeasurementMap[i] - 1] = MeasurementDegree[i];
                        MeasurementValidNum++;
                    }
                }
                for (uint8_t i = 0; i < MeasurementValidNum; i++) {
                    R_data[i * MeasurementValidNum + i] = MatR_DiagonalElements[temp[i]];
                }
                H.numRows = MeasurementValidNum;   H.numCols = X_DIM;
                HT.numRows = X_DIM;              HT.numCols = MeasurementValidNum;
                R.numRows = MeasurementValidNum;   R.numCols = MeasurementValidNum;
                K.numRows = X_DIM;               K.numCols = MeasurementValidNum;
                z.numRows = MeasurementValidNum;   z.numCols = 1;
            }
            memset(MeasuredVector, 0, sizeof(float) * Z_DIM);
            if constexpr (U_DIM > 0) memcpy(u_data, ControlVector, sizeof(float) * U_DIM);
        }

        void PrepareMeasurementModern(const float* measure, const bool* is_valid) {
            if (!UseAutoAdjustment) {
                memcpy(z_data, measure, sizeof(float) * Z_DIM);
                MeasurementValidNum = Z_DIM;
                return;
            }
            MeasurementValidNum = 0;
            memset(R_data, 0, sizeof(float) * Z_DIM * Z_DIM);
            memset(H_data, 0, sizeof(float) * X_DIM * Z_DIM);
            for (uint8_t i = 0; i < Z_DIM; i++) {
                if (is_valid[i]) {
                    z_data[MeasurementValidNum] = measure[i];
                    temp[MeasurementValidNum] = i;
                    H_data[X_DIM * MeasurementValidNum + MeasurementMap[i] - 1] = MeasurementDegree[i];
                    MeasurementValidNum++;
                }
            }
            for (uint8_t i = 0; i < MeasurementValidNum; i++) {
                R_data[i * MeasurementValidNum + i] = MatR_DiagonalElements[temp[i]];
            }
            H.numRows = MeasurementValidNum;   H.numCols = X_DIM;
            HT.numRows = X_DIM;              HT.numCols = MeasurementValidNum;
            R.numRows = MeasurementValidNum;   R.numCols = MeasurementValidNum;
            K.numRows = X_DIM;               K.numCols = MeasurementValidNum;
            z.numRows = MeasurementValidNum;   z.numCols = 1;
        }
    };
}