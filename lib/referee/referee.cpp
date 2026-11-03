#include "referee.hpp"
#include "crc.hpp"
#include <algorithm>
#include <cstring>
#include <utility>

Referee::Referee(const Config& config)
    : config_(config) {
    if (const auto* can = std::get_if<Can>(&config_.bus)) {
        can->add_rx_callback([this](const CanFrame& frame) {
            if (frame.id == config_.rx_id && !frame.is_extend && !frame.is_remote && frame.size <= 8)
                receive(frame.data, frame.size);
        });
    } else {
        std::get<Uart>(config_.bus).add_rx_callback([this](const uint8_t* data, const size_t size) {
            receive(data, size);
        });
    }
}

void Referee::add_0311_callback(RxCallback callback) {
    callbacks_0311_.push_back(std::move(callback));
}

void Referee::update() {
    if (timer_connect_.elapsed() > config_.timeout)
        is_connected = false;
    if (tx_size_ == 0)
        return;

    // 链路忙时保留待发帧，下次重试
    if (const auto* can = std::get_if<Can>(&config_.bus)) {
        if (!can->can_send())
            return;
        const auto size = std::min<size_t>(8, tx_size_ - tx_pos_);
        CanFrame frame{.id = config_.tx_id, .size = static_cast<uint8_t>(size)};
        std::memcpy(frame.data, tx_buf_ + tx_pos_, size);
        can->send(frame);
        tx_pos_ += size;
    } else {
        const auto& uart = std::get<Uart>(config_.bus);
        if (!uart.can_send())
            return;
        uart.send(tx_buf_, tx_size_);
        tx_pos_ = tx_size_;
    }
    if (tx_pos_ == tx_size_) {
        tx_size_ = 0;
        tx_pos_ = 0;
    }
}

bool Referee::send(const uint16_t cmd_id, const uint8_t* data, const size_t size) {
    if (tx_size_ != 0 || size > 300 || (size != 0 && !data))
        return false;
    if (cmd_id == 0x0301 && (size < 6 || size > 118 || timer_interaction_.elapsed() < 1.0f / 30.0f))
        return false;
    tx_buf_[0] = 0xA5;
    tx_buf_[1] = static_cast<uint8_t>(size);
    tx_buf_[2] = static_cast<uint8_t>(size >> 8);
    tx_buf_[3] = seq_++;
    tx_buf_[4] = CRC8::check_sum(tx_buf_, 4);
    tx_buf_[5] = static_cast<uint8_t>(cmd_id);
    tx_buf_[6] = static_cast<uint8_t>(cmd_id >> 8);
    if (size != 0)
        std::memcpy(tx_buf_ + 7, data, size);
    const auto crc = CRC16::check_sum(tx_buf_, size + 7);
    tx_buf_[size + 7] = static_cast<uint8_t>(crc);
    tx_buf_[size + 8] = static_cast<uint8_t>(crc >> 8);
    tx_size_ = size + 9;
    if (cmd_id == 0x0301)
        timer_interaction_.restart();
    return true;
}

void Referee::receive(const uint8_t* data, const size_t size) {
    // 正常帧内不会停顿 100 ms；丢失尾部后不让旧半包阻塞后续数据
    if (timer_rx_.elapsed() > 0.1f)
        rx_size_ = 0;
    timer_rx_.restart();

    for (size_t i = 0; i < size; i++) {
        if (rx_size_ == sizeof(rx_buf_))
            rx_size_ = 0;
        rx_buf_[rx_size_++] = data[i];

        while (rx_size_ != 0) {
            if (rx_buf_[0] != 0xA5) {
                rx_size_--;
                std::memmove(rx_buf_, rx_buf_ + 1, rx_size_);
                continue;
            }
            if (rx_size_ < 5)
                break;

            const size_t payload_size = rx_buf_[1] | (static_cast<size_t>(rx_buf_[2]) << 8);
            if (payload_size > 300 || !CRC8::verify(rx_buf_, 4, rx_buf_[4])) {
                rx_size_--;
                std::memmove(rx_buf_, rx_buf_ + 1, rx_size_);
                continue;
            }

            const size_t frame_size = payload_size + 9;
            if (rx_size_ < frame_size)
                break;

            const uint16_t crc = rx_buf_[frame_size - 2] | (static_cast<uint16_t>(rx_buf_[frame_size - 1]) << 8);
            if (CRC16::verify(rx_buf_, frame_size - 2, crc)) {
                const uint16_t cmd = rx_buf_[5] | (static_cast<uint16_t>(rx_buf_[6]) << 8);
                deserialize(cmd, rx_buf_ + 7, payload_size);
                is_connected = true;
                timer_connect_.restart();
                rx_size_ -= frame_size;
                std::memmove(rx_buf_, rx_buf_ + frame_size, rx_size_);
            } else {
                // 保留后续字节重新寻找帧头，支持坏帧后紧接有效帧
                rx_size_--;
                std::memmove(rx_buf_, rx_buf_ + 1, rx_size_);
            }
        }
    }
}

bool Referee::deserialize(const uint16_t cmd_id, const uint8_t* data, const size_t size) {
    switch (cmd_id) {
    case 0x0301: {
        if (size < 6 || size > 118)
            return false;
        const uint16_t id = data[0] | (data[1] << 8);
        const auto content_size = size - 6;
        size_t expected = 0;
        switch (id) {
        case 0x0100:
            expected = 2;
            break;
        case 0x0101:
            expected = 15;
            break;
        case 0x0102:
            expected = 30;
            break;
        case 0x0103:
            expected = 75;
            break;
        case 0x0104:
            expected = 105;
            break;
        case 0x0110:
            expected = 45;
            break;
        case 0x0120:
            expected = 4;
            break;
        case 0x0121:
            expected = 8;
            break;
        default:
            if (id < 0x0200 || id > 0x02FF)
                return false;
            expected = content_size;
            break;
        }
        if (content_size != expected || content_size > sizeof(interaction.user_data))
            return false;
        interaction.data_cmd_id = id;
        interaction.sender_id = data[2] | (data[3] << 8);
        interaction.receiver_id = data[4] | (data[5] << 8);
        interaction.size = static_cast<uint16_t>(content_size);
        std::memset(interaction.user_data, 0, sizeof(interaction.user_data));
        switch (id) {
        case 0x0100:
            std::memcpy(&interaction.layer_delete, data + 6, sizeof(interaction.layer_delete));
            break;
        case 0x0101:
            std::memcpy(&interaction.figures[0], data + 6, sizeof(interaction.figures[0]));
            break;
        case 0x0102:
            std::memcpy(interaction.figures, data + 6, 2 * sizeof(interaction.figures[0]));
            break;
        case 0x0103:
            std::memcpy(interaction.figures, data + 6, 5 * sizeof(interaction.figures[0]));
            break;
        case 0x0104:
            std::memcpy(interaction.figures, data + 6, 7 * sizeof(interaction.figures[0]));
            break;
        case 0x0110:
            std::memcpy(&interaction.character, data + 6, sizeof(interaction.character));
            break;
        case 0x0120:
            std::memcpy(&interaction.sentry_command, data + 6, sizeof(interaction.sentry_command));
            break;
        case 0x0121:
            std::memcpy(&interaction.radar_command, data + 6, sizeof(interaction.radar_command));
            break;
        default:
            std::memcpy(interaction.user_data, data + 6, content_size);
            break;
        }
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
        hurt_received.restart();
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
    case 0x0302:
        if (size != sizeof(custom_robot_data.data))
            return false;
        std::memset(custom_robot_data.data, 0, sizeof(custom_robot_data.data));
        std::memcpy(custom_robot_data.data, data, size);
        custom_robot_data.size = static_cast<uint16_t>(size);
        return true;
    case 0x0303:
        if (size != sizeof(map_command))
            return false;
        std::memcpy(&map_command, data, size);
        return true;
    case 0x0305:
        if (size != sizeof(map_robot_data))
            return false;
        std::memcpy(&map_robot_data, data, size);
        return true;
    case 0x0306:
        if (size != sizeof(custom_client_data))
            return false;
        std::memcpy(&custom_client_data, data, size);
        return true;
    case 0x0307:
        if (size != sizeof(map_data))
            return false;
        std::memcpy(&map_data, data, size);
        return true;
    case 0x0308:
        if (size != sizeof(custom_info))
            return false;
        std::memcpy(&custom_info, data, size);
        return true;
    case 0x0309:
        if (size != sizeof(robot_custom_data.data))
            return false;
        std::memset(robot_custom_data.data, 0, sizeof(robot_custom_data.data));
        std::memcpy(robot_custom_data.data, data, size);
        robot_custom_data.size = static_cast<uint16_t>(size);
        return true;
    case 0x0310:
        if (size != sizeof(robot_custom_client_data.data))
            return false;
        std::memset(robot_custom_client_data.data, 0, sizeof(robot_custom_client_data.data));
        std::memcpy(robot_custom_client_data.data, data, size);
        robot_custom_client_data.size = static_cast<uint16_t>(size);
        return true;
    case 0x0311:
        if (size != sizeof(custom_client_command.data))
            return false;
        std::memset(custom_client_command.data, 0, sizeof(custom_client_command.data));
        std::memcpy(custom_client_command.data, data, size);
        custom_client_command.size = static_cast<uint16_t>(size);
        for (const auto& callback : callbacks_0311_)
            callback(data, size);
        return true;
    case 0x0a01:
        if (size != sizeof(radar_position))
            return false;
        std::memcpy(&radar_position, data, size);
        return true;
    case 0x0a02:
        if (size != sizeof(radar_hp))
            return false;
        std::memcpy(&radar_hp, data, size);
        return true;
    case 0x0a03:
        if (size != sizeof(radar_allowance))
            return false;
        std::memcpy(&radar_allowance, data, size);
        return true;
    case 0x0a04:
        if (size != sizeof(radar_team))
            return false;
        std::memcpy(&radar_team, data, size);
        return true;
    case 0x0a05:
        if (size != sizeof(radar_buff))
            return false;
        std::memcpy(&radar_buff, data, size);
        return true;
    case 0x0a06:
        if (size != sizeof(radar_key))
            return false;
        std::memcpy(&radar_key, data, size);
        return true;
    default:
        return false;
    }
}
