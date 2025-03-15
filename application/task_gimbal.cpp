#include "task_gimbal.h"
#include "cmsis_os.h"
#include "main.h"
#include "tim.h"
#include <math.h>

#include "task_uart.h"
#include "pid.hpp"

// yaw参数（云台旋转）
#define YAW_MAX_SPEED 120.0f // yaw满杆量速度【单位：角度/s】
// 用于计算PID的参数
#define YAW_MAX_VOLTAGE 12.0f // 最大电压【单位：V（假设满电压24V）】
#define YAW_PMAX_ERR 10.0f    // 达到此误差后，P输出拉满【单位：角度】
#define YAW_D_RATIO 0.0f

// pitch参数（云台俯仰）
#define PTICH_MAX_SPEED 100.0f // pitch满杆量速度【单位：角度/s】
#define PITCH_MAX_ANGLE 50.0f  // pitch电机最大角度
// 用于计算PID的参数
#define PITCH_MAX_VOLTAGE 12.0f   // 最大电压【单位：V（假设满电压24V）】
#define PITCH_PMAX_ERR 5.0f       // 达到此误差后，P输出拉满【单位：角度】
#define PITCH_STEADY_CURRENT 0.6f // 稳态电流【单位：A】
#define PITCH_STEADY_ERR 20.0f    // 稳态误差（PID不加I时的误差）【单位：角度】
#define PITCH_STEADY_T 0.1f       // 达到稳态的时间常数【单位：s】
#define PITCH_D_RATIO 0.0f

// shoot参数（拨弹电机）
#define SHOOT_FREQ 1.0f                    // 弹丸发射频率（每秒多少发）
#define SHOOT_MAX_CURRENT 5.0f             // 最大电流【单位：A（2006电机额定电流3A，堵转电流27.3A，CAN通信最大支持10A）】
#define SHOOT_RPM (SHOOT_FREQ / 8 * 60.0f) // 拨弹电机速度【单位：rpm，减速后的速度】
#define SHOOT_PMAX_SPEED SHOOT_RPM         // 达到此速度后，P输出拉满【单位：角度/s】

// 常量
#define DT 0.001f         // 闭环控制周期【单位：ms】
#define M6002_RATIO 36.0f // 6002电机减速比

#define get_motor_measure(ptr, data)                                   \
    {                                                                  \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
        (ptr)->temperate = (data)[6];                                  \
    }

#define ecd_to_angle(ecd) (ecd / 8192.0f * 360.0f) // 编码器值转角度

// // 从CAN获取的电机状态
// typedef struct {
//     uint16_t ecd;
//     int16_t speed_rpm;
//     int16_t given_current;
//     uint8_t temperate;
// } motor_status_t;
//
// // 6020电机
// typedef struct {
//     motor_status_t status;  // 从CAN获取电机状态
//     pid_type_def angle_pid; // PID结构体
//     fp32 angle_now;         // 角度当前值【单位：角度】
//     fp32 angle_set;         // 角度目标值【单位：角度】
// } m6020_t;
//
// // 2006电机
// typedef struct {
//     motor_status_t status; // 从CAN获取电机状态
//     pid_type_def rpm_pid;  // PID结构体
//     fp32 rpm_now;          // 速度当前值【单位：rpm，减速后的速度】
//     fp32 rpm_set;          // 速度目标值【单位：rpm，减速后的速度】
// } m2006_t;
//
// m6020_t yaw, pitch;
// m2006_t shoot;
// fp32 pitch_offset;
//
// // 摩擦轮开关
// static void wheel_control(uint8_t enable) {
//     if (enable) {
//         HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//         HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//         HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//         HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//     } else {
//         HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//         HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
//         HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
//         HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
//     }
// }
//
// static float calc_angle_err(float now, float set) {
//     while (now < 0) {
//         now += 360.0f;
//     }
//     while (now > 360) {
//         now -= 360.0f;
//     }
//     while (set < 0) {
//         set += 360.0f;
//     }
//     while (set > 360) {
//         set -= 360.0f;
//     }
//     float err = fmodf(set - now + 540.0f, 360.0f) - 180.0f; // 最短路径误差
//     return err;
// }
//
// static void handle_rc() {
//     // 检查遥控器连接
//     if (rc_data.is_connected == 0) {
//         wheel_control(0);
//         shoot.rpm_set = 0;
//         return;
//     }
//
//     // pitch电机
//     pitch.angle_set += rc_data.pitch * PTICH_MAX_SPEED * DT;
//     if (pitch.angle_set > PITCH_MAX_ANGLE + pitch_offset)
//         pitch.angle_set = PITCH_MAX_ANGLE + pitch_offset;
//     if (pitch.angle_set < pitch_offset)
//         pitch.angle_set = pitch_offset;
//
//     // yaw电机
//     yaw.angle_set += (-rc_data.yaw) * YAW_MAX_SPEED * DT;
//     if (yaw.angle_set > 360)
//         yaw.angle_set -= 360;
//     if (yaw.angle_set < 0)
//         yaw.angle_set += 360;
//
//     // 摩擦轮、拨弹电机，受左拨杆控制
//     if (rc_data.left_switch == UP) { // 关闭：摩擦轮关、拨弹电机关
//         wheel_control(0);
//         shoot.rpm_set = 0;
//     } else if (rc_data.left_switch == MID) { // 准备：摩擦轮开，拨弹电机关
//         wheel_control(1);
//         shoot.rpm_set = 0;
//     } else if (rc_data.left_switch == DOWN) { // 发射：摩擦轮开，拨弹电机开
//         wheel_control(1);
//         shoot.rpm_set = SHOOT_RPM;
//     }
// }
//
// // 6020为舵电机，3508为轮电机
// static void send_current(fp32 yaw, fp32 pitch, fp32 shoot) {
//     CAN_TxHeaderTypeDef header;
//     uint8_t data[8];
//     uint32_t send_mail_box;
//
//     // 6020电机：电压控制，-25000~0~25000（假设满电压为24V）
//     int16_t yaw_val = yaw / 24.0f * 25000.0f;
//     int16_t pitch_val = pitch / 24.0f * 25000.0f;
//     // 2006电机（C610电调）：-10~0~10A => -10000~0~10000
//     int16_t shoot_val = shoot / 10.0f * 10000.0f;
//
//     header.StdId = 0X1FF;
//     header.IDE = CAN_ID_STD;
//     header.RTR = CAN_RTR_DATA;
//     header.DLC = 0x08;
//     data[0] = yaw_val >> 8; // YAW，6020电机，ID：5
//     data[1] = yaw_val;
//     data[2] = pitch_val >> 8; // PITCH，6020电机，ID：6
//     data[3] = pitch_val;
//     data[4] = shoot_val >> 8; // SHOOT，6002电机，ID：7
//     data[5] = shoot_val;
//     data[6] = 0;
//     data[7] = 0;
//
//     HAL_CAN_AddTxMessage(&hcan1, &header, data, &send_mail_box);
// }

[[noreturn]] void task_gimbal_entry(void const *argument) {
    // // 初始化PID
    // // yaw位置环
    // const fp32 YAW_KP = YAW_MAX_VOLTAGE / YAW_PMAX_ERR;
    // const fp32 YAW_KI = 0;
    // const fp32 YAW_KD = YAW_KP * YAW_D_RATIO;
    // PID_init(&yaw.angle_pid, PID_POSITION, (fp32[3]){YAW_KP, YAW_KI, YAW_KD}, YAW_MAX_VOLTAGE, 0);
    // // pitch位置环
    // const fp32 PITCH_KP = PITCH_MAX_VOLTAGE / PITCH_PMAX_ERR;
    // const fp32 PITCH_KI = PITCH_STEADY_CURRENT / PITCH_STEADY_ERR * DT / PITCH_STEADY_T;
    // const fp32 PITCH_KD = PITCH_KP * PITCH_D_RATIO;
    // PID_init(&pitch.angle_pid, PID_POSITION, (fp32[3]){PITCH_KP, PITCH_KI, PITCH_KD}, PITCH_MAX_VOLTAGE, PITCH_STEADY_CURRENT * 1.2f);
    // // shoot速度环
    // const fp32 SHOOT_KP = SHOOT_MAX_CURRENT / SHOOT_PMAX_SPEED;
    // const fp32 SHOOT_KI = 0;
    // const fp32 SHOOT_KD = 0;
    // PID_init(&shoot.rpm_pid, PID_POSITION, (fp32[3]){SHOOT_KP, SHOOT_KI, SHOOT_KD}, SHOOT_MAX_CURRENT, 0);
    //
    // // 等待收到电机数据
    // osDelay(500);
    // // 电机初始位置
    // yaw.angle_set = ecd_to_angle(yaw.status.ecd);
    // pitch.angle_set = ecd_to_angle(pitch.status.ecd);
    // pitch_offset = pitch.angle_set;

    while (true) {
        // handle_rc(); // 处理遥控器数据
        //
        // // 计算PID
        // // yaw位置环
        // fp32 yaw_err = calc_angle_err(yaw.angle_now, yaw.angle_set);
        // PID_calc(&yaw.angle_pid, 0, yaw_err);
        // // pitch位置环
        // fp32 pitch_err = calc_angle_err(pitch.angle_now, pitch.angle_set);
        // PID_calc(&pitch.angle_pid, 0, pitch_err);
        // // shoot速度环
        // PID_calc(&shoot.rpm_pid, shoot.rpm_now, shoot.rpm_set);
        //
        // // CAN发送电机电流
        // send_current(yaw.angle_pid.out, pitch.angle_pid.out, shoot.rpm_pid.out);

        osDelay(1);
    }
}

// void GIMBAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
//     CAN_RxHeaderTypeDef header;
//     uint8_t data[8];
//
//     HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data);
//
//     switch (header.StdId) {
//         case 0x205: // YAW，6020电机，ID：5
//             get_motor_measure(&yaw.status, data);
//         yaw.angle_now = ecd_to_angle(yaw.status.ecd);
//         break;
//         case 0x206: // PITCH，6020电机，ID：6
//             get_motor_measure(&pitch.status, data);
//         pitch.angle_now = ecd_to_angle(pitch.status.ecd);
//         break;
//         case 0x207: // SHOOT，6002电机，ID：7
//             get_motor_measure(&shoot.status, data);
//         shoot.rpm_now = (float)shoot.status.speed_rpm / M6002_RATIO;
//         break;
//     }
// }
