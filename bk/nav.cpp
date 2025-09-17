#include "nav.hpp"
#include "mavlink.h"
#include "bsp_cdc.h"

void NAV::SendTargetData(const float x, const float y) {
    mavlink_message_t msg;
    const float x2 = y;
    const float y2 = -x;
    const int len = mavlink_msg_target_pack(0, 0, &msg, x2, y2);
    uint8_t buf[len];
    mavlink_msg_to_send_buffer(buf, &msg);
    BSP_CDC_Transmit(buf, len);
}

void NAV::ParseStreamingData(const uint8_t *data, const uint32_t len) {
    mavlink_message_t msg;
    mavlink_status_t status;
    for (int i = 0; i < len; i++) {
        const uint8_t c = data[i];
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_POSITION: {
                    mavlink_position_t position_msg;
                    mavlink_msg_position_decode(&msg, &position_msg);

                    // 处理解析到的数据
                    const float x = position_msg.x;
                    const float y = position_msg.y;

                    this->position_x = -y;
                    this->position_y = x;
                    this->position_yaw = position_msg.yaw;
                    break;
                }
                case MAVLINK_MSG_ID_CMD_VEL: {
                    mavlink_cmd_vel_t vel_cmd;
                    mavlink_msg_cmd_vel_decode(&msg, &vel_cmd);

                    // 处理解析到的数据
                    const float x = vel_cmd.x_vel;
                    const float y = vel_cmd.y_vel;

                    this->vel_x = -y;
                    this->vel_y = x;
                    this->vel_yaw = vel_cmd.z_vel;
                    break;
                }
                default:
                    break;
            }
        }
    }
}
