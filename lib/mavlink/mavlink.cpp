#include "mavlink.hpp"
#include "bsp/bsp.hpp"
#include <cstring>

Mavlink::Mavlink() {
    // 注册CDC接收回调
    auto callback = [this](const uint8_t data[], const uint32_t size) {
        this->callback(data, size);
    };
    BSP::CDC::RegisterCallback(callback);
}

void Mavlink::OnLoop() {
    // 断联检测
    if (dwt_is_connected.GetDT() > CONNECT_TIMEOUT) {
        is_connected = false;
    }

    if (dwt_send_freq.GetDT() >= 1 / SEND_FREQ.toFloat(Hz)) {
        dwt_send_freq.UpdateDT();
        sendImu();
        sendReferee();
    }
}

void Mavlink::callback(const uint8_t data[], const uint32_t size) {
    // 逐字节解析MAVLink消息
    for (uint32_t i = 0; i < size; i++) {
        mavlink_message_t msg;
        mavlink_status_t status;
        if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &status)) {
            parse(msg);
            is_connected = true;
            dwt_is_connected.UpdateDT();
        }
    }
}

void Mavlink::parse(const mavlink_message_t& msg) {
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_aim: {
            mavlink_aim_t aim;
            mavlink_msg_aim_decode(&msg, &aim);
            this->aim = {
                .is_detected = (bool)aim.is_detected,
                .is_fire_advise = (bool)aim.is_fire_advise,
                .yaw = aim.yaw * deg,
                .pitch = aim.pitch * deg
            };
        }
        break;

        case MAVLINK_MSG_ID_ui: {
            mavlink_ui_t ui;
            mavlink_msg_ui_decode(&msg, &ui);
            this->ui = {
                .num = ui.num,
                .speed = ui.speed * rpm,
                .x0 = ui.x0,
                .y0 = ui.y0,
                .x1 = ui.x1,
                .y1 = ui.y1,
                .x2 = ui.x2,
                .y2 = ui.y2,
                .x3 = ui.x3,
                .y3 = ui.y3,
                .x4 = ui.x4,
                .y4 = ui.y4
            };
        }
        break;

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
        referee.bullet_speed.toFloat(m_s)
    );
    send(msg);
}