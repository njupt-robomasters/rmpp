#include "referee.hpp"

#include <cstring>
#include "crc.hpp"

Referee::Referee() : can_parser(this), uart_parser(this) {
    auto can_callback = [this](const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
        this->can_callback(port, id, data, dlc);
    };
    BSP::CAN::RegisterCallback(can_callback);

    auto uart_callback = [this](const uint8_t data[], const uint16_t size) {
        this->uart_callback(data, size);
    };
    BSP::UART6::RegisterCallback(uart_callback);
}

void Referee::OnLoop() {
    if (dwt_can.GetDT() > TIMEOUT) {
        is_can_connected = false;
    }
    if (dwt_uart.GetDT() > TIMEOUT) {
        is_uart_connected = false;
    }
}

void Referee::can_callback(const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    if (port != 1) return;

    if (id != 0x00) return;

    is_can_connected = true;
    dwt_can.UpdateDT();

    for (int i = 0; i < dlc; i++) {
        can_parser.parseByte(data[i]);
    }
}

void Referee::uart_callback(const uint8_t data[], const uint16_t size) {
    is_uart_connected = true;
    dwt_uart.UpdateDT();

    for (int i = 0; i < size; i++) {
        uart_parser.parseByte(data[i]);
    }
}

void Referee::Parser::parseByte(const uint8_t byte) {
    switch (step) {
        case INIT: {
            if (byte == 0xA5) { // 找到SOF
                step = FRAME_HEADER;
                packet[0] = byte;
                len = 1;
            } else {
                len = 0;
            }
        }
        break;

        case FRAME_HEADER: {
            packet[len++] = byte;
            if (len == 5) {
                if (CRC8::Verify((const uint8_t*)&packet.frame_header, 4, packet.frame_header.crc8)) { // CRC8校验成功
                    step = CMD_ID;
                } else { // CRC8校验失败，重新寻找SOF
                    step = INIT;
                }
            }
        }
        break;

        case CMD_ID: {
            packet[len++] = byte;
            if (len == 5 + 2) {
                step = DATA;
            }
        }
        break;

        case DATA: {
            if (len > sizeof(packet) - 2) { // 数据长度超出缓冲区
                step = INIT;
            }
            packet[len++] = byte;
            if (len == 5 + 2 + packet.frame_header.data_length) {
                step = FRAME_TAIL;
            }
        }
        break;

        case FRAME_TAIL: {
            packet[len++] = byte;
            if (len == 5 + 2 + packet.frame_header.data_length + 2) {
                // CRC16校验
                const uint16_t crc16 = packet[len - 2] | (packet[len - 1] << 8);
                if (CRC16::Verify((const uint8_t*)&packet, len - 2, crc16)) {
                    referee->deserialize(packet.cmd_id, packet.data, packet.frame_header.data_length);
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

void Referee::deserialize(const uint16_t cmd_id, const uint8_t data[], uint8_t size) {
    switch (cmd_id) {
        case 0x0001: { // 0x0001，比赛状态数据（1Hz）
            if (size != sizeof(game_status)) break;
            memcpy(&game_status, data, sizeof(game_status));
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
            break;
        }
        case 0x0101: { // 0x0101，场地事件数据（1Hz）
            if (size != sizeof(event_data)) break;
            memcpy(&event_data, data, sizeof(event_data));
            break;
        }
        case 0x0104: { // 0x0104，裁判警告数据（1Hz，己方判罚/判负时触发发送）
            if (size != sizeof(referee_warning)) break;
            memcpy(&referee_warning, data, sizeof(referee_warning));
            break;
        }
        case 0x0105: { // 0x0105，飞镖发射相关数据（1Hz）
            if (size != sizeof(dart_info)) break;
            memcpy(&dart_info, data, sizeof(dart_info));
            break;
        }
        case 0x0201: { // 0x0201，机器人性能体系数据（10Hz）
            if (size != sizeof(robot_status)) break;
            memcpy(&robot_status, data, sizeof(robot_status));
            break;
        }
        case 0x0202: { // 0x0202，实时底盘缓冲能量和射击热量数据（10Hz）
            if (size != sizeof(power_heat_data)) break;
            memcpy(&power_heat_data, data, sizeof(power_heat_data));
            break;
        }
        case 0x0203: { // 0x0203，机器人位置数据（1Hz）
            if (size != sizeof(robot_pos)) break;
            memcpy(&robot_pos, data, sizeof(robot_pos));
            break;
        }
        case 0x0204: { // 0x0204，机器人增益和底盘能量数据（3Hz）
            if (size != sizeof(buff)) break;
            memcpy(&buff, data, sizeof(buff));
            break;
        }
        case 0x0206: { // 0x0206，伤害状态数据（伤害发生后发送）
            if (size != sizeof(hurt_data)) break;
            memcpy(&hurt_data, data, sizeof(hurt_data));
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
            break;
        }
        case 0x0209: { // 0x0209，机器人RFID模块状态（3Hz）
            if (size != sizeof(rfid_status)) break;
            memcpy(&rfid_status, data, sizeof(rfid_status));
            break;
        }
        case 0x020A: { // 0x020A，飞镖选手端指令数据（3Hz）【服务器→己方飞镖机器人】
            if (size != sizeof(dart_client_cmd)) break;
            memcpy(&dart_client_cmd, data, sizeof(dart_client_cmd));
            break;
        }
        case 0x020B: { // 0x020B，地面机器人位置数据（1Hz）【服务器→己方哨兵机器人】
            if (size != sizeof(ground_robot_position)) break;
            memcpy(&ground_robot_position, data, sizeof(ground_robot_position));
            break;
        }
        case 0x020C: { // 0x020C，雷达标记进度数据（1Hz）【服务器→己方雷达机器人】
            if (size != sizeof(radar_mark_data)) break;
            memcpy(&radar_mark_data, data, sizeof(radar_mark_data));
            break;
        }
        case 0x020D: { // 0x020D，哨兵自主决策信息同步（1Hz）【服务器→己方哨兵机器人】
            if (size != sizeof(sentry_info)) break;
            memcpy(&sentry_info, data, sizeof(sentry_info));
            break;
        }
        case 0x020E: { // 0x020E，雷达自主决策信息同步（1Hz）【服务器→己方雷达机器人】
            if (size != sizeof(radar_info)) break;
            memcpy(&radar_info, data, sizeof(radar_info));
            break;
        }
        case 0x0301: { // 0x0301，机器人交互数据（频率上限为30Hz）【多机通信接收】
            if (size != sizeof(robot_interaction_data_receive)) break;
            memcpy(&robot_interaction_data_receive, data, sizeof(robot_interaction_data_receive));
            break;
        }
        case 0x0303: { // 0x0303，选手端小地图交互数据（选手端触发发送）【选手端点击→服务器→发送方选择的己方机器人】
            if (size != sizeof(map_command)) break;
            memcpy(&map_command, data, sizeof(map_command));
            break;
        }
        case 0x0304: { // 0x0304（图传链路），键鼠遥控数据（30Hz）【选手端→选手端图传连接的机器人】
            if (size != sizeof(remote_control)) break;
            memcpy(&remote_control, data, sizeof(remote_control));
            break;
        }
        default:
            break;
    }
}