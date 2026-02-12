#include "RefereeParser.hpp"
#include "crc.hpp"

RefereeParser::RefereeParser(const config_t& config) : config(config) {
    // 注册CAN回调
    auto can_callback = [this](const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
        this->callback(port, id, data, dlc);
    };
    BSP::CAN::RegisterCallback(can_callback);
}

void RefereeParser::OnLoop() {
    // 断联检测
    if (dwt_connect.GetDT() > config.timeout) {
        is_connect = false;
    }
}

void RefereeParser::callback(const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    if (port != config.can_port) return;
    if (id != config.master_id) return;

    // 更新断联检测
    is_connect = true;
    dwt_connect.UpdateDT();

    // 逐字节解析
    for (int i = 0; i < dlc; i++) {
        parseByte(data[i]);
    }
}

void RefereeParser::parseByte(const uint8_t byte) {
    switch (step) {
        case INIT: {
            if (byte == 0xA5) { // 找到SOF
                step = FRAME_HEADER;
                packet[0] = byte;
                pos = 1;
            } else {
                pos = 0;
            }
        }
        break;

        case FRAME_HEADER: {
            packet[pos++] = byte;
            if (pos == 5) {
                if (CRC8::Verify((const uint8_t*)&packet.frame_header, 4, packet.frame_header.crc8)) { // CRC8校验成功
                    step = CMD_ID;
                } else { // CRC8校验失败，重新寻找SOF
                    step = INIT;
                }
            }
        }
        break;

        case CMD_ID: {
            packet[pos++] = byte;
            if (pos == 5 + 2) {
                step = DATA;
            }
        }
        break;

        case DATA: {
            if (pos > sizeof(packet) - 2) { // 数据长度超出缓冲区
                step = INIT;
            }
            packet[pos++] = byte;
            if (pos == 5 + 2 + packet.frame_header.data_length) {
                step = FRAME_TAIL;
            }
        }
        break;

        case FRAME_TAIL: {
            packet[pos++] = byte;
            if (pos == 5 + 2 + packet.frame_header.data_length + 2) {
                // CRC16校验
                const uint16_t crc16 = packet[pos - 2] | (packet[pos - 1] << 8);
                if (CRC16::Verify((const uint8_t*)&packet, pos - 2, crc16)) {
                    deserialize(packet.cmd_id, packet.data, packet.frame_header.data_length);
                }
                step = INIT;
            }
        }
        break;

        default: {
            step = INIT;
        }
        break;
    }
}

void RefereeParser::deserialize(const uint16_t cmd_id, const uint8_t data[], uint8_t size) {
    switch (cmd_id) {
        case 0x0001: { // 0x0001，比赛状态数据（1Hz）
            if (size != sizeof(game_status)) break;
            memcpy(&game_status, data, sizeof(game_status));
            dwt_game_status.UpdateDT();
            break;
        }
        case 0x0002: { // 0x0002，比赛结果数据（比赛结束触发发送）
            if (size != sizeof(game_result)) break;
            memcpy(&game_result, data, sizeof(game_result));
            break;
        }
        case 0x0003: { // 0x0003，机器人血量数据（3Hz）
            if (size != sizeof(game_robot_HP)) break;
            memcpy(&game_robot_HP, data, sizeof(game_robot_HP));
            dwt_game_robot_HP.UpdateDT();
            break;
        }
        case 0x0101: { // 0x0101，场地事件数据（1Hz）
            if (size != sizeof(event_data)) break;
            memcpy(&event_data, data, sizeof(event_data));
            dwt_event_data.UpdateDT();
            break;
        }
        case 0x0104: { // 0x0104，裁判警告数据（1Hz，己方判罚/判负时触发发送）
            if (size != sizeof(referee_warning)) break;
            memcpy(&referee_warning, data, sizeof(referee_warning));
            dwt_referee_warning.UpdateDT();
            break;
        }
        case 0x0105: { // 0x0105，飞镖发射相关数据（1Hz）
            if (size != sizeof(dart_info)) break;
            memcpy(&dart_info, data, sizeof(dart_info));
            dwt_dart_info.UpdateDT();
            break;
        }
        case 0x0201: { // 0x0201，机器人性能体系数据（10Hz）
            if (size != sizeof(robot_status)) break;
            memcpy(&robot_status, data, sizeof(robot_status));
            dwt_robot_status.UpdateDT();

            // 适配rmui
            extern int ui_self_id;
            ui_self_id = robot_status.robot_id;
            break;
        }
        case 0x0202: { // 0x0202，实时底盘缓冲能量和射击热量数据（10Hz）
            if (size != sizeof(power_heat_data)) break;
            memcpy(&power_heat_data, data, sizeof(power_heat_data));
            dwt_power_heat_data.UpdateDT();
            break;
        }
        case 0x0203: { // 0x0203，机器人位置数据（1Hz）
            if (size != sizeof(robot_pos)) break;
            memcpy(&robot_pos, data, sizeof(robot_pos));
            dwt_robot_pos.UpdateDT();
            break;
        }
        case 0x0204: { // 0x0204，机器人增益和底盘能量数据（3Hz）
            if (size != sizeof(buff)) break;
            memcpy(&buff, data, sizeof(buff));
            dwt_buff.UpdateDT();
            break;
        }
        case 0x0206: { // 0x0206，伤害状态数据（伤害发生后发送）
            if (size != sizeof(hurt_data)) break;
            memcpy(&hurt_data, data, sizeof(hurt_data));
            dwt_hurt_data.UpdateDT();
            break;
        }
        case 0x0207: { // 0x0207，实时射击数据（弹丸发射后发送）
            if (size != sizeof(shoot_data)) break;
            memcpy(&shoot_data, data, sizeof(shoot_data));
            break;
        }
        case 0x0208: { // 0x0208，允许发弹量（10Hz）
            if (size != sizeof(projectile_allowance)) break;
            memcpy(&projectile_allowance, data, sizeof(projectile_allowance));
            dwt_projectile_allowance.UpdateDT();
            break;
        }
        case 0x0209: { // 0x0209，机器人RFID模块状态（3Hz）
            if (size != sizeof(rfid_status)) break;
            memcpy(&rfid_status, data, sizeof(rfid_status));
            dwt_rfid_status.UpdateDT();
            break;
        }
        case 0x020A: { // 0x020A，飞镖选手端指令数据（3Hz）【服务器→己方飞镖机器人】
            if (size != sizeof(dart_client_cmd)) break;
            memcpy(&dart_client_cmd, data, sizeof(dart_client_cmd));
            dwt_dart_client_cmd.UpdateDT();
            break;
        }
        case 0x020B: { // 0x020B，地面机器人位置数据（1Hz）【服务器→己方哨兵机器人】
            if (size != sizeof(ground_robot_position)) break;
            memcpy(&ground_robot_position, data, sizeof(ground_robot_position));
            dwt_ground_robot_position.UpdateDT();
            break;
        }
        case 0x020C: { // 0x020C，雷达标记进度数据（1Hz）【服务器→己方雷达机器人】
            if (size != sizeof(radar_mark_data)) break;
            memcpy(&radar_mark_data, data, sizeof(radar_mark_data));
            dwt_radar_mark_data.UpdateDT();
            break;
        }
        case 0x020D: { // 0x020D，哨兵自主决策信息同步（1Hz）【服务器→己方哨兵机器人】
            if (size != sizeof(sentry_info)) break;
            memcpy(&sentry_info, data, sizeof(sentry_info));
            dwt_sentry_info.UpdateDT();
            break;
        }
        case 0x020E: { // 0x020E，雷达自主决策信息同步（1Hz）【服务器→己方雷达机器人】
            if (size != sizeof(radar_info)) break;
            memcpy(&radar_info, data, sizeof(radar_info));
            dwt_radar_info.UpdateDT();
            break;
        }
        default:
            break;
    }
}