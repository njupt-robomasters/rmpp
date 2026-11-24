#include "referee.hpp"
#include "crc.hpp"

Referee::Referee(const Config& config)
    : config(config) {
    // CAN：只收 rx_id 标准数据帧，载荷按序拼接
    config.can.add_rx_callback([this](const CanFrame& frame) {
        if (frame.id == this->config.rx_id && !frame.is_extend && !frame.is_remote && frame.size <= 8)
            callback(frame.data, frame.size);
    });
}

void Referee::update() {
    // 断联检测：数据保留最后一帧
    if (callback_timer.elapsed() > config.timeout)
        is_connect = false;
}

void Referee::callback(const uint8_t* data, const size_t size) {
    for (size_t index = 0; index < size; index++) {
        rx_fifo.push(data[index]);
        while (rx_fifo.size != 0) {
            // 帧头不符：丢弃 1 字节重新对齐
            if (rx_fifo[0] != 0xA5) {
                rx_fifo.pop(1);
                continue;
            }
            if (rx_fifo.size < 5)
                break;

            // 帧头 5 B：A5、载荷长度 2 B、包序号、CRC8；载荷超长或 CRC8 不符时丢弃 1 字节重新对齐
            uint8_t frame[FRAME_MAX];
            rx_fifo.peek(frame, 5);
            const size_t payload_size = frame[1] | (static_cast<size_t>(frame[2]) << 8u);
            if (payload_size > PAYLOAD_MAX || crc8_referee(frame, 4) != frame[4]) {
                rx_fifo.pop(1);
                continue;
            }
            const size_t frame_size = 5 + 2 + payload_size + 2;
            if (rx_fifo.size < frame_size)
                break;

            // 整帧：帧头、命令码 2 B、载荷、CRC16 2 B；CRC16 不符时丢弃 1 字节重新对齐
            rx_fifo.peek(frame, frame_size);
            const uint16_t crc = frame[frame_size - 2] | (static_cast<uint16_t>(frame[frame_size - 1]) << 8u);
            if (crc16_referee(frame, frame_size - 2) != crc) {
                rx_fifo.pop(1);
                continue;
            }
            rx_fifo.pop(frame_size);

            // 有效帧：按命令码解包
            const uint16_t cmd_id = frame[5] | (static_cast<uint16_t>(frame[6]) << 8u);
            parse(cmd_id, frame + 7, payload_size);
            is_connect = true;
            callback_timer.restart();
        }
    }
}

void Referee::parse(const uint16_t cmd_id, const uint8_t* data, const size_t size) {
    // 定长数据：长度与结构体一致才覆盖；0x0301 只收队友 0x0200~0x02FF 子内容
    switch (cmd_id) {
    case 0x0001:
        read(game_status, data, size);
        break;
    case 0x0002:
        read(game_result, data, size);
        break;
    case 0x0003:
        read(game_robot_hp, data, size);
        break;
    case 0x0101:
        read(event_data, data, size);
        break;
    case 0x0104:
        read(referee_warning, data, size);
        break;
    case 0x0105:
        read(dart_info, data, size);
        break;
    case 0x0201:
        read(robot_status, data, size);
        break;
    case 0x0202:
        read(power_heat_data, data, size);
        break;
    case 0x0203:
        read(robot_pos, data, size);
        break;
    case 0x0204:
        read(buff, data, size);
        break;
    case 0x0206:
        if (read(hurt_data, data, size))
            hurt_sequence++;
        break;
    case 0x0207:
        read(shoot_data, data, size);
        break;
    case 0x0208:
        read(projectile_allowance, data, size);
        break;
    case 0x0209:
        read(rfid_status, data, size);
        break;
    case 0x020A:
        read(dart_client_cmd, data, size);
        break;
    case 0x020B:
        read(ground_robot_position, data, size);
        break;
    case 0x020C:
        read(radar_mark_data, data, size);
        break;
    case 0x020D:
        read(sentry_info, data, size);
        break;
    case 0x020E:
        read(radar_info, data, size);
        break;
    case 0x0301: {
        // 子内容 ID、发送者、接收者各 2 B，内容至多 112 B
        if (size < 6)
            break;
        const uint16_t id = data[0] | (static_cast<uint16_t>(data[1]) << 8u);
        if (id < 0x0200 || id > 0x02FF)
            break;
        interaction.data_cmd_id = id;
        interaction.sender_id = data[2] | (static_cast<uint16_t>(data[3]) << 8u);
        interaction.receiver_id = data[4] | (static_cast<uint16_t>(data[5]) << 8u);
        interaction.size = static_cast<uint16_t>(size - 6);
        std::memset(interaction.user_data, 0, sizeof(interaction.user_data));
        std::memcpy(interaction.user_data, data + 6, size - 6);
        break;
    }
    case 0x0303:
        read(map_command, data, size);
        break;
    default:
        break;
    }
}
