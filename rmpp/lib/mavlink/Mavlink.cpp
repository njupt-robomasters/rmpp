#include "Mavlink.hpp"

Mavlink::Mavlink() {
    // 注册CDC接收回调
    auto callback = [this](const uint8_t data[], const uint32_t size) {
        this->callback(data, size);
    };
    BSP::CDC::RegisterCallback(callback);
}

void Mavlink::OnLoop() {
    // 断联检测
    if (dwt_connect.GetDT() > CONNECT_TIMEOUT) {
        is_connect = false;
    }

    // 将三个包分摊在不同的时间点发送
    static uint8_t step = 0;
    if (dwt_send_freq.PollTimeout(1 / (SEND_FREQ * 3))) {
        switch (step) {
            case 0:
                sendTargetPosition();
                break;
            case 1:
                sendImu();
                break;
            case 2:
                sendReferee();
                break;
            default:
                break;;
        }
        step = (step + 1) % 3;
    }
}

void Mavlink::callback(const uint8_t data[], const uint32_t size) {
    // 逐字节解析MAVLink消息
    for (uint32_t i = 0; i < size; i++) {
        mavlink_message_t msg;
        mavlink_status_t status;
        if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &status)) {
            parse(msg);
            is_connect = true;
            dwt_connect.UpdateDT();
        }
    }
}

void Mavlink::parse(const mavlink_message_t& msg) {
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_auto_aim: {
            mavlink_auto_aim_t auto_aim;
            mavlink_msg_auto_aim_decode(&msg, &auto_aim);
            this->auto_aim = {
                .is_detect = (bool)auto_aim.is_detect,
                .yaw = auto_aim.yaw * deg,
                .pitch = auto_aim.pitch * deg,
                .is_fire = (bool)auto_aim.is_fire,
                .robot_id = auto_aim.robot_id,
                .wr = auto_aim.wr * rpm,
                .distance = auto_aim.distance * m,
            };
            break;
        }

        case MAVLINK_MSG_ID_ui: {
            mavlink_ui_t ui;
            mavlink_msg_ui_decode(&msg, &ui);
            this->ui = {
                .x0 = ui.x0,
                .y0 = ui.y0,
                .x1 = ui.x1,
                .y1 = ui.y1,
                .x2 = ui.x2,
                .y2 = ui.y2,
                .x3 = ui.x3,
                .y3 = ui.y3,
            };
            break;
        }

        case MAVLINK_MSG_ID_current_position: {
            mavlink_current_position_t current_position;
            mavlink_msg_current_position_decode(&msg, &current_position);
            this->current_position = {
                .x = current_position.x * m_s,
                .y = current_position.y * m_s,
                .yaw = current_position.yaw * deg
            };
            break;
        }

        case MAVLINK_MSG_ID_chassis_speed: {
            mavlink_chassis_speed_t chassis_speed;
            mavlink_msg_chassis_speed_decode(&msg, &chassis_speed);
            this->chassis_speed = {
                .vx = chassis_speed.vx * m,
                .vy = chassis_speed.vy * m,
            };
        }

        default:
            break;
    }
}

void Mavlink::send(const mavlink_message_t& msg) {
    // 序列化
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // 通过CDC发送
    BSP::CDC::Transmit(buf, len);
}

void Mavlink::sendImu() const {
    mavlink_message_t msg;
    mavlink_msg_imu_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &msg,
        imu.yaw.toFloat(deg),
        imu.pitch.toFloat(deg),
        imu.roll.toFloat(deg)
    );
    send(msg);
}

void Mavlink::sendReferee() const {
    mavlink_message_t msg;
    mavlink_msg_referee_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &msg,
        referee.is_red,
        referee.game_progress,
        referee.stage_remain_time,
        referee.bullet_speed.toFloat(m_s)
    );
    send(msg);
}

void Mavlink::sendTargetPosition() const {
    mavlink_message_t msg;
    mavlink_msg_target_position_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &msg,
        target_position.x.toFloat(m),
        target_position.y.toFloat(m)
    );
    send(msg);
}