#include "referee.hpp"
#include "crc.hpp"
#include <cstring>

Referee::Referee(const Config& config)
    : config(config) {
    config.can.add_rx_callback([this](const CanFrame& frame) {
        if (frame.id == this->config.rx_id && !frame.is_extend && !frame.is_remote && frame.size <= 8)
            receive(frame.data, frame.size);
    });
}

void Referee::update() {
    if (connect_timer.elapsed() > config.timeout)
        is_connect = false;
}

void Referee::receive(const uint8_t* data, const size_t size) {
    // 正常帧内不会停顿 100 ms；丢失尾部后不让旧半包阻塞后续数据
    if (receive_timer.elapsed() > 0.1f)
        rx.clear();
    receive_timer.restart();

    for (size_t i = 0; i < size; i++) {
        if (rx.size == RX_CAPACITY)
            rx.clear();
        rx.push(data[i]);

        while (rx.size != 0) {
            if (rx[0] != 0xA5) {
                rx.pop(1);
                continue;
            }
            if (rx.size < 5)
                break;

            // 帧头：CRC8 校验前 4 字节，载荷超长视为错位
            uint8_t frame[FRAME_MAX];
            rx.peek(frame, 5);
            const size_t payload_size = frame[1] | (static_cast<size_t>(frame[2]) << 8);
            if (payload_size > PAYLOAD_MAX || !CRC8::verify(frame, 4, frame[4])) {
                rx.pop(1);
                continue;
            }

            const size_t frame_size = payload_size + 9;
            if (rx.size < frame_size)
                break;

            // 整帧复制为连续数组后 CRC16 校验并解包
            rx.peek(frame, frame_size);
            const uint16_t crc = frame[frame_size - 2] | (static_cast<uint16_t>(frame[frame_size - 1]) << 8);
            if (CRC16::verify(frame, frame_size - 2, crc)) {
                const uint16_t cmd = frame[5] | (static_cast<uint16_t>(frame[6]) << 8);
                parse(cmd, frame + 7, payload_size);
                is_connect = true;
                connect_timer.restart();
                rx.pop(frame_size);
            } else {
                // 保留后续字节重新寻找帧头，支持坏帧后紧接有效帧
                rx.pop(1);
            }
        }
    }
}

bool Referee::parse(const uint16_t cmd_id, const uint8_t* data, const size_t size) {
    switch (cmd_id) {
    case 0x0301: {
        // 子内容 ID、发送者、接收者各 2 B，内容至多 112 B
        if (size < 6)
            return false;
        const uint16_t id = data[0] | (data[1] << 8);
        if (id < 0x0200 || id > 0x02FF)
            return false;
        interaction.data_cmd_id = id;
        interaction.sender_id = data[2] | (data[3] << 8);
        interaction.receiver_id = data[4] | (data[5] << 8);
        interaction.size = static_cast<uint16_t>(size - 6);
        std::memset(interaction.user_data, 0, sizeof(interaction.user_data));
        std::memcpy(interaction.user_data, data + 6, size - 6);
        return true;
    }
    case 0x0001:
        if (size != sizeof(game_status))
            return false;
        std::memcpy(&game_status, data, size);
        return true;
    case 0x0002:
        if (size != sizeof(game_result))
            return false;
        std::memcpy(&game_result, data, size);
        return true;
    case 0x0003:
        if (size != sizeof(game_robot_hp))
            return false;
        std::memcpy(&game_robot_hp, data, size);
        return true;
    case 0x0101:
        if (size != sizeof(event_data))
            return false;
        std::memcpy(&event_data, data, size);
        return true;
    case 0x0104:
        if (size != sizeof(referee_warning))
            return false;
        std::memcpy(&referee_warning, data, size);
        return true;
    case 0x0105:
        if (size != sizeof(dart_info))
            return false;
        std::memcpy(&dart_info, data, size);
        return true;
    case 0x0201:
        if (size != sizeof(robot_status))
            return false;
        std::memcpy(&robot_status, data, size);
        return true;
    case 0x0202:
        if (size != sizeof(power_heat_data))
            return false;
        std::memcpy(&power_heat_data, data, size);
        return true;
    case 0x0203:
        if (size != sizeof(robot_pos))
            return false;
        std::memcpy(&robot_pos, data, size);
        return true;
    case 0x0204:
        if (size != sizeof(buff))
            return false;
        std::memcpy(&buff, data, size);
        return true;
    case 0x0206:
        if (size != sizeof(hurt_data))
            return false;
        std::memcpy(&hurt_data, data, size);
        hurt_sequence++;
        return true;
    case 0x0207:
        if (size != sizeof(shoot_data))
            return false;
        std::memcpy(&shoot_data, data, size);
        return true;
    case 0x0208:
        if (size != sizeof(projectile_allowance))
            return false;
        std::memcpy(&projectile_allowance, data, size);
        return true;
    case 0x0209:
        if (size != sizeof(rfid_status))
            return false;
        std::memcpy(&rfid_status, data, size);
        return true;
    case 0x020a:
        if (size != sizeof(dart_client_cmd))
            return false;
        std::memcpy(&dart_client_cmd, data, size);
        return true;
    case 0x020b:
        if (size != sizeof(ground_robot_position))
            return false;
        std::memcpy(&ground_robot_position, data, size);
        return true;
    case 0x020c:
        if (size != sizeof(radar_mark_data))
            return false;
        std::memcpy(&radar_mark_data, data, size);
        return true;
    case 0x020d:
        if (size != sizeof(sentry_info))
            return false;
        std::memcpy(&sentry_info, data, size);
        return true;
    case 0x020e:
        if (size != sizeof(radar_info))
            return false;
        std::memcpy(&radar_info, data, size);
        return true;
    case 0x0303:
        if (size != sizeof(map_command))
            return false;
        std::memcpy(&map_command, data, size);
        return true;
    default:
        return false;
    }
}
