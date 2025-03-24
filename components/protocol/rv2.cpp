#include "rv2.hpp"
#include "bsp_cdc.h"
#include "mavlink.h"

void RV2::SendIMUData(const float roll, const float pitch, const float yaw) {
    mavlink_message_t msg;
    const int len = mavlink_msg_imu_data_pack(0, 0, &msg, roll, pitch, yaw);
    uint8_t buf[len];
    mavlink_msg_to_send_buffer(buf, &msg);
    BSP_CDC_Transmit(buf, len);
}

void RV2::SendRefereeData(const bool team_is_red, const float bullet_speed) {
    mavlink_message_t msg;
    const int len = mavlink_msg_referee_data_pack(0, 0, &msg, team_is_red, bullet_speed);
    uint8_t buf[len];
    mavlink_msg_to_send_buffer(buf, &msg);
    BSP_CDC_Transmit(buf, len);
}

void RV2::ParseStreamingData(const uint8_t *data, const uint32_t len) {
    mavlink_message_t msg;
    mavlink_status_t status;

    for (int i = 0; i < len; i++) {
        const uint8_t c = data[i];
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_GIMBAL_CMD: {
                    mavlink_gimbal_cmd_t gimbal_cmd;
                    mavlink_msg_gimbal_cmd_decode(&msg, &gimbal_cmd);

                    // 处理解析到的数据
                    pitch = gimbal_cmd.pitch;
                    yaw = gimbal_cmd.yaw;
                    is_locked = gimbal_cmd.is_locked;
                    fire_advise = gimbal_cmd.fire_advise;
                }
                break;

                default:
                    break;
            }
        }
    }
}
