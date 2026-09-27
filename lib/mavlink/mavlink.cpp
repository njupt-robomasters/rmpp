#include "mavlink.hpp"
#include "bsp/uart.hpp"
#include "math/unit.hpp"

Mavlink::Mavlink(const Config& config)
    : config(config) {
    // 注册 CDC 接收回调
    config.cdc.add_rx_callback([this](const uint8_t* data, std::size_t size) {
        this->callback(data, size);
    });
}

void Mavlink::update() {
    // 串口断联检测
    if (timer_cdc_.elapsed() > config.timeout) {
        timer_cdc_.restart();
        is_connect_cdc = false;
    }

    // 各报文断联检测
    if (timer_auto_aim_.elapsed() > config.timeout) {
        timer_auto_aim_.restart();
        is_connect_vision = false;
        vision = {};
    }
    if (timer_insta360_.elapsed() > config.timeout) {
        timer_insta360_.restart();
        is_connect_insta360 = false;
        insta360 = {};
    }
    if (timer_current_position_.elapsed() > config.timeout) {
        timer_current_position_.restart();
        is_connect_current_position = false;
        current_position = {};
    }
    if (timer_chassis_speed_.elapsed() > config.timeout) {
        timer_chassis_speed_.restart();
        is_connect_chassis_speed = false;
        chassis_speed = {};
    }

    // 轮流发送
    if (timer_send_freq_.elapsed() > 1 / config.send_freq) {
        timer_send_freq_.restart();
        switch (idx_) {
        case 0:
            send_imu();
            break;
        case 1:
            send_referee();
            break;
        case 2:
            send_target_position();
            break;
        default:
            break;
            ;
        }
        idx_ = (idx_ + 1) % 3;
    }
}

void Mavlink::callback(const uint8_t data[], const uint32_t size) {
    // 逐字节解析MAVLink消息
    for (uint32_t i = 0; i < size; i++) {
        mavlink_message_t msg;
        mavlink_status_t status;
        if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &status)) {
            timer_cdc_.restart();
            is_connect_cdc = true;
            parse(msg);
        }
    }
}

void Mavlink::parse(const mavlink_message_t& msg) {
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_auto_aim: {
        timer_auto_aim_.restart();
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
        timer_insta360_.restart();
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
        timer_current_position_.restart();
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
        timer_chassis_speed_.restart();
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

void Mavlink::send(const mavlink_message_t& msg) const {
    uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t size = mavlink_msg_to_send_buffer(tx_buf, &msg);
    config.cdc.send(tx_buf, size);
}

void Mavlink::send_imu() const {
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

void Mavlink::send_referee() const {
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

void Mavlink::send_target_position() const {
    mavlink_message_t msg;
    mavlink_msg_target_position_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &msg,
        target_position.x,
        target_position.y);
    send(msg);
}
