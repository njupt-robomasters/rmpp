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
    if (dwt_is_connected.GetDT() > CONNECT_TIMEOUT) {
        is_connect = false;
    }

    // // 发送频率控制
    // if (dwt_send_freq.PollTimeout(1 / SEND_FREQ)) {
    //     sendPos();
    //     sendImu();
    //     sendReferee();
    //
    // }
    static uint8_t step = 0;
    // 假设发送频率是 100Hz，我们将三个包分摊在不同的时间点发送
    // 这样上位机收到的数据流在时间上是均匀的
    if (dwt_send_freq.PollTimeout(1 / (SEND_FREQ * 3))) {
        switch(step) {
            case 0: sendPos();     break;
            case 1: sendImu();     break;
            case 2: sendReferee(); break;
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
                .is_detect = (bool)aim.is_detected,
                .is_fire_advise = (bool)aim.is_fire_advise,
                .yaw = aim.yaw * deg,
                .pitch = aim.pitch * deg
            };
            break;
        }

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
            break;
        }
        case MAVLINK_MSG_ID_nav_cmd_vel: {
            mavlink_nav_cmd_vel_t cmd_vel_t;
            mavlink_msg_nav_cmd_vel_decode(&msg, &cmd_vel_t);
            this->cmd_vel = {
                .vel_x = cmd_vel_t.vel_x * m_s,
                .vel_y = cmd_vel_t.vel_y * m_s
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
        referee.bullet_speed.toFloat(m_s)
    );
    send(msg);
}

void Mavlink::sendPos() const {
    mavlink_message_t msg;
    mavlink_msg_target_pose_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &msg,
        pos.pos_x.toFloat(m),
        pos.pos_y.toFloat(m)
    );
    send(msg);
}