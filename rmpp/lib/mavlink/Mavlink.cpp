#include "Mavlink.hpp"
#include "math/Conversions.hpp"

using math::deg;
using math::rpm;

Mavlink::Mavlink(const config_t& config) : config(config) {
    // 注册CDC接收回调
    auto callback = [this](const uint8_t data[], const uint32_t size) {
        this->callback(data, size);
    };
    BSP::CDC::RegisterCallback(callback);
}

void Mavlink::OnLoop() {
    // CDC串口断联检测
    if (dwt_cdc.GetDT() > config.timeout) {
        is_connect_cdc = false;
    }

    // 各报文断联检测
    if (dwt_auto_aim.GetDT() > config.timeout) {
        is_connect_vision = false;
        vision = {};
    }
    if (dwt_insta360.GetDT() > config.timeout) {
        is_connect_insta360 = false;
        insta360 = {};
    }
    if (dwt_current_position.GetDT() > config.timeout) {
        is_connect_current_position = false;
        current_position = {};
    }
    if (dwt_chassis_speed.GetDT() > config.timeout) {
        is_connect_chassis_speed = false;
        chassis_speed = {};
    }

    // 轮流发送
    if (dwt_send_freq.PollTimeout(1 / config.send_freq)) {
        switch (idx) {
            case 0:
                sendImu();
                break;
            case 1:
                sendReferee();
                break;
            case 2:
                sendTargetPosition();
                break;
            default:
                break;;
        }
        idx = (idx + 1) % 3;
    }
}

void Mavlink::callback(const uint8_t data[], const uint32_t size) {
    // 逐字节解析MAVLink消息
    for (uint32_t i = 0; i < size; i++) {
        mavlink_message_t msg;
        mavlink_status_t status;
        if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &status)) {
            dwt_cdc.UpdateDT();
            is_connect_cdc = true;
            parse(msg);
        }
    }
}

void Mavlink::parse(const mavlink_message_t& msg) {
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_auto_aim: {
            dwt_auto_aim.UpdateDT();
            is_connect_vision = true;
            mavlink_auto_aim_t auto_aim;
            mavlink_msg_auto_aim_decode(&msg, &auto_aim);
            const float yaw_deg = auto_aim.yaw;
            const float pitch_deg = auto_aim.pitch;
            const float wr_rpm = auto_aim.wr;
            this->vision = {
                .is_detect = (bool)auto_aim.is_detect,
                .yaw = yaw_deg * deg,
                .pitch = pitch_deg * deg,
                .is_fire = (bool)auto_aim.is_fire,
                .robot_id = auto_aim.robot_id,
                .wr = wr_rpm * rpm,
                .distance = auto_aim.distance,
            };
            break;
        }

        case MAVLINK_MSG_ID_insta360: {
            dwt_insta360.UpdateDT();
            is_connect_insta360 = true;
            mavlink_insta360_t ui;
            mavlink_msg_insta360_decode(&msg, &ui);
            const float a0_deg = ui.a0;
            const float a1_deg = ui.a1;
            const float a2_deg = ui.a2;
            this->insta360 = {
                .a0 = a0_deg * deg,
                .c0 = ui.c0,
                .a1 = a1_deg * deg,
                .c1 = ui.c1,
                .a2 = a2_deg * deg,
                .c2 = ui.c2,
            };
            break;
        }

        case MAVLINK_MSG_ID_current_position: {
            dwt_current_position.UpdateDT();
            is_connect_current_position = true;
            mavlink_current_position_t current_position;
            mavlink_msg_current_position_decode(&msg, &current_position);
            const float yaw_deg = current_position.yaw;
            this->current_position = {
                .x = current_position.x,
                .y = current_position.y,
                .yaw = yaw_deg * deg};
            break;
        }

        case MAVLINK_MSG_ID_chassis_speed: {
            dwt_chassis_speed.UpdateDT();
            is_connect_chassis_speed = true;
            mavlink_chassis_speed_t chassis_speed;
            mavlink_msg_chassis_speed_decode(&msg, &chassis_speed);
            this->chassis_speed = {
                .vx = chassis_speed.vx,
                .vy = chassis_speed.vy,
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
    const float yaw_deg = imu.yaw / deg;
    const float pitch_deg = imu.pitch / deg;
    const float roll_deg = imu.roll / deg;
    mavlink_msg_imu_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &msg,
        yaw_deg,
        pitch_deg,
        roll_deg);
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
        (uint16_t)referee.stage_remain_time,
        referee.bullet_speed);
    send(msg);
}

void Mavlink::sendTargetPosition() const {
    mavlink_message_t msg;
    mavlink_msg_target_position_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &msg,
        target_position.x,
        target_position.y);
    send(msg);
}
