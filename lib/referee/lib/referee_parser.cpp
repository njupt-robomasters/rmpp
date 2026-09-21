#include "referee_parser.hpp"
#include "crc.hpp"
#include <cstring>

RefereeParser::RefereeParser(const Config& config) : config(config) {
    // 注册CAN回调
    rmpp::board.can(config.can_port).add_rx_callback([this](const rmpp::CanFrame& frame) {
        this->callback(this->config.can_port, frame.id, frame.data.data(), frame.size);
    });
}

void RefereeParser::on_loop() {
    // 断联检测
    if (timer_connect_.get_dt() > config.timeout) {
        is_connect = false;
    }
}

void RefereeParser::callback(const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    if (port != config.can_port)
        return;
    if (id != config.master_id)
        return;

    // 更新断联检测
    is_connect = true;
    timer_connect_.update_dt();

    // 逐字节解析
    for (int i = 0; i < dlc; i++) {
        parse_byte(data[i]);
    }
}

void RefereeParser::parse_byte(const uint8_t byte) {
    switch (step_) {
    case INIT: {
        if (byte == 0xA5) { // 找到SOF
            step_ = FRAME_HEADER;
            packet_[0] = byte;
            pos_ = 1;
        } else {
            pos_ = 0;
        }
    } break;

    case FRAME_HEADER: {
        packet_[pos_++] = byte;
        if (pos_ == 5) {
            if (CRC8::verify((const uint8_t*)&packet_.frame_header, 4, packet_.frame_header.crc8)) { // CRC8校验成功
                step_ = CMD_ID;
            } else { // CRC8校验失败，重新寻找SOF
                step_ = INIT;
            }
        }
    } break;

    case CMD_ID: {
        packet_[pos_++] = byte;
        if (pos_ == 5 + 2) {
            step_ = DATA;
        }
    } break;

    case DATA: {
        if (pos_ > sizeof(packet_) - 2) { // 数据长度超出缓冲区
            step_ = INIT;
        }
        packet_[pos_++] = byte;
        if (pos_ == 5 + 2 + packet_.frame_header.data_length) {
            step_ = FRAME_TAIL;
        }
    } break;

    case FRAME_TAIL: {
        packet_[pos_++] = byte;
        if (pos_ == 5 + 2 + packet_.frame_header.data_length + 2) {
            // CRC16校验
            const uint16_t crc16 = packet_[pos_ - 2] | (packet_[pos_ - 1] << 8);
            if (CRC16::verify((const uint8_t*)&packet_, pos_ - 2, crc16)) {
                deserialize(packet_.cmd_id, packet_.data, packet_.frame_header.data_length);
            }
            step_ = INIT;
        }
    } break;

    default: {
        step_ = INIT;
    } break;
    }
}

void RefereeParser::deserialize(const uint16_t cmd_id, const uint8_t data[], uint8_t size) {
    switch (cmd_id) {
    case 0x0001: { // 0x0001，比赛状态数据（1Hz）
        if (size != sizeof(game_status))
            break;
        memcpy(&game_status, data, sizeof(game_status));
        timer_game_status.update_dt();
        break;
    }
    case 0x0002: { // 0x0002，比赛结果数据（比赛结束触发发送）
        if (size != sizeof(game_result))
            break;
        memcpy(&game_result, data, sizeof(game_result));
        break;
    }
    case 0x0003: { // 0x0003，机器人血量数据（3Hz）
        if (size != sizeof(game_robot_hp))
            break;
        memcpy(&game_robot_hp, data, sizeof(game_robot_hp));
        timer_game_robot_hp.update_dt();
        break;
    }
    case 0x0101: { // 0x0101，场地事件数据（1Hz）
        if (size != sizeof(event_data))
            break;
        memcpy(&event_data, data, sizeof(event_data));
        timer_event_data.update_dt();
        break;
    }
    case 0x0104: { // 0x0104，裁判警告数据（1Hz，己方判罚/判负时触发发送）
        if (size != sizeof(referee_warning))
            break;
        memcpy(&referee_warning, data, sizeof(referee_warning));
        timer_referee_warning.update_dt();
        break;
    }
    case 0x0105: { // 0x0105，飞镖发射相关数据（1Hz）
        if (size != sizeof(dart_info))
            break;
        memcpy(&dart_info, data, sizeof(dart_info));
        timer_dart_info.update_dt();
        break;
    }
    case 0x0201: { // 0x0201，机器人性能体系数据（10Hz）
        if (size != sizeof(robot_status))
            break;
        memcpy(&robot_status, data, sizeof(robot_status));
        timer_robot_status.update_dt();
        break;
    }
    case 0x0202: { // 0x0202，实时底盘缓冲能量和射击热量数据（10Hz）
        if (size != sizeof(power_heat_data))
            break;
        memcpy(&power_heat_data, data, sizeof(power_heat_data));
        timer_power_heat_data.update_dt();
        break;
    }
    case 0x0203: { // 0x0203，机器人位置数据（1Hz）
        if (size != sizeof(robot_pos))
            break;
        memcpy(&robot_pos, data, sizeof(robot_pos));
        timer_robot_pos.update_dt();
        break;
    }
    case 0x0204: { // 0x0204，机器人增益和底盘能量数据（3Hz）
        if (size != sizeof(buff))
            break;
        memcpy(&buff, data, sizeof(buff));
        timer_buff.update_dt();
        break;
    }
    case 0x0206: { // 0x0206，伤害状态数据（伤害发生后发送）
        if (size != sizeof(hurt_data))
            break;
        memcpy(&hurt_data, data, sizeof(hurt_data));
        timer_hurt_data.update_dt();
        break;
    }
    case 0x0207: { // 0x0207，实时射击数据（弹丸发射后发送）
        if (size != sizeof(shoot_data))
            break;
        memcpy(&shoot_data, data, sizeof(shoot_data));
        break;
    }
    case 0x0208: { // 0x0208，允许发弹量（10Hz）
        if (size != sizeof(projectile_allowance))
            break;
        memcpy(&projectile_allowance, data, sizeof(projectile_allowance));
        timer_projectile_allowance.update_dt();
        break;
    }
    case 0x0209: { // 0x0209，机器人RFID模块状态（3Hz）
        if (size != sizeof(rfid_status))
            break;
        memcpy(&rfid_status, data, sizeof(rfid_status));
        timer_rfid_status.update_dt();
        break;
    }
    case 0x020A: { // 0x020A，飞镖选手端指令数据（3Hz）【服务器→己方飞镖机器人】
        if (size != sizeof(dart_client_cmd))
            break;
        memcpy(&dart_client_cmd, data, sizeof(dart_client_cmd));
        timer_dart_client_cmd.update_dt();
        break;
    }
    case 0x020B: { // 0x020B，地面机器人位置数据（1Hz）【服务器→己方哨兵机器人】
        if (size != sizeof(ground_robot_position))
            break;
        memcpy(&ground_robot_position, data, sizeof(ground_robot_position));
        timer_ground_robot_position.update_dt();
        break;
    }
    case 0x020C: { // 0x020C，雷达标记进度数据（1Hz）【服务器→己方雷达机器人】
        if (size != sizeof(radar_mark_data))
            break;
        memcpy(&radar_mark_data, data, sizeof(radar_mark_data));
        timer_radar_mark_data.update_dt();
        break;
    }
    case 0x020D: { // 0x020D，哨兵自主决策信息同步（1Hz）【服务器→己方哨兵机器人】
        if (size != sizeof(sentry_info))
            break;
        memcpy(&sentry_info, data, sizeof(sentry_info));
        timer_sentry_info.update_dt();
        break;
    }
    case 0x020E: { // 0x020E，雷达自主决策信息同步（1Hz）【服务器→己方雷达机器人】
        if (size != sizeof(radar_info))
            break;
        memcpy(&radar_info, data, sizeof(radar_info));
        timer_radar_info.update_dt();
        break;
    }
    default:
        break;
    }
}
