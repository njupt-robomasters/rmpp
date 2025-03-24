#include "nav.hpp"
#include "mavlink.h"
#include "bsp_cdc.h"


void NAV::SendTargetData(const float x, const float y, const float yaw) {
    mavlink_message_t msg;
    const int len = mavlink_msg_target_pack(0, 0, &msg, x, y);
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
                    this->position_x = position_msg.x;
                    this->position_y = position_msg.y;
                    this->position_yaw = position_msg.yaw;
                    break;
                }
                case MAVLINK_MSG_ID_CMD_VEL:{
                    mavlink_cmd_vel_t vel_cmd;
                    mavlink_msg_cmd_vel_decode(&msg,&vel_cmd);

                    this->vel_x =vel_cmd.x_vel;
                    this->vel_y =vel_cmd.y_vel;
                    this->vel_yaw =vel_cmd.z_vel;
                    break;
                }
                default:
                    break;
            }
        }
    }
}
