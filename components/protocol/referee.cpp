#include "referee.hpp"
#include <cstring>
#include "cmsis_os.h"

void Referee::ParsePacket(const uint8_t *packet, uint16_t packetSize) {
    if (packet[0] != 0xA5) {
        // SOF固定为0xA5
        return;
    }

    uint16_t data_length = (packet[2] << 8) | packet[1]; // 获取data_length
    uint8_t seq = packet[3]; // 包序号
    uint8_t crc8_received = packet[4]; // 收到的CRC8校验值

    // CRC8帧头校验
    uint8_t crc8_calculated = Get_CRC8_Check_Sum(packet, 4, CRC8_INIT); // 计算前4字节的CRC8
    if (crc8_calculated != crc8_received) {
        return;
    }
    //CRC16整包校验
    uint16_t crc16_received = (packet[packetSize - 2] << 8) | packet[packetSize - 1]; // 从frame_tail获取CRC16值
    uint16_t crc16_calculated = Get_CRC16_Check_Sum(packet, packetSize - 2, CRC_INIT); // 计算整个包的CRC16，不包括最后两个字节的CRC16

    //    if(!Verify_CRC16_Check_Sum(packet,packetSize)) {
    //            return;
    //    }

    //进行对应命令的解析
    uint16_t cmd_id = (packet[6] << 8) | packet[5];
    switch (cmd_id) {
        case 0x0201: {
            // 机器人性能体系数据
            if (data_length != 13) {
                return;
            }
            const uint8_t *dataField = &packet[7]; // 数据字段起始位置
            // 解析数据
            this->robot_id = dataField[0];
            uint8_t robotLevel = dataField[1];
            if (this->robot_id < 100) {
                this->team_is_red = 1;
            } else {
                this->team_is_red = 0;
            }
            this->robot_hp = (dataField[3] << 8) | dataField[2];
            this->shooter_cooling = (dataField[7] << 8) | dataField[8];
            this->shooter_heat_limit = (dataField[9] << 8) | dataField[8];
            this->chassis_power_limit = (dataField[11] << 8) | dataField[10];
            uint8_t powerManagement = dataField[12];
            this->power_gimbal = (powerManagement & 0x01);
            this->power_chassis = (powerManagement & 0x02);
            this->power_shooter = (powerManagement & 0x04);
            break;
        }
        case 0x0001: //比赛状态信息
        {
            if (data_length != 11) {
                return;
            }
            const uint8_t *dataField = &packet[7];
            uint8_t game_type_and_progress = dataField[0];
            uint8_t game_type = game_type_and_progress & 0x0F; // bit 0-3
            uint8_t game_progress = (game_type_and_progress >> 4) & 0x0F; // bit 4-7
            uint16_t stage_remain_time = (dataField[2] << 8) | dataField[1];
            if (game_progress == 4) {
                this->competition_is_started = 1; //比赛开始
            } else {
                this->competition_is_started = 0; //比赛未开始或者结束
            }
            break;
        }
        case 0x0202: // 实时底盘功率和枪口热量数据
        {
            if (data_length != 16) {
                return;
            }
            const uint8_t *dataField = &packet[7];
            uint16_t chassis_voltage = (dataField[1] << 8) | dataField[0];
            uint16_t chassis_current = (dataField[3] << 8) | dataField[2];
            uint32_t raw_chassis_power = (dataField[7] << 24) | (dataField[6] << 16) | (dataField[5] << 8) | dataField[
                                             4];
            float chassis_power = *reinterpret_cast<float *>(&raw_chassis_power);
            uint16_t buffer_energy = (dataField[9] << 8) | dataField[8];
            uint16_t shooter_17mm_1_barrel_heat = (dataField[11] << 8) | dataField[10];
            uint16_t shooter_17mm_2_barrel_heat = (dataField[13] << 8) | dataField[12];
            uint16_t shooter_42mm_barrel_heat = (dataField[15] << 8) | dataField[14];
            this->chassis_buffer_energy = buffer_energy;
            this->chassis_power_now = chassis_power;
            this->shooter_17mm_heat = shooter_17mm_1_barrel_heat;
            this->shooter_42mm_heat = shooter_42mm_barrel_heat;
            break;
        }
        case 0x0207: //弹丸速度
        {
            if (data_length != 7) {
                return;
            }
            const uint8_t *dataField = &packet[7];
            uint8_t bullet_type = dataField[0];
            uint8_t shooter_number = dataField[1];
            uint8_t launching_frequency = dataField[2];
            uint32_t raw_initial_speed = (dataField[6] << 24) | (dataField[5] << 16) | (dataField[4] << 8) | dataField[
                                             3];
            float initial_speed = *reinterpret_cast<float *>(&raw_initial_speed);
            this->shooter_bullet_speed = initial_speed;
            this->shooter_launching_frequency = launching_frequency;
            break;
        }
        case 0x0208: //允许发单量
        {
            if (data_length != 6) {
                // 结构体大小为6字节
                return;
            }
            const uint8_t *dataField = &packet[7];
            uint16_t projectile_allowance_17mm = (dataField[1] << 8) | dataField[0];
            uint16_t projectile_allowance_42mm = (dataField[3] << 8) | dataField[2];
            uint16_t remaining_gold_coin = (dataField[5] << 8) | dataField[4];
            this->shooter_projectile_allowance_17mm = projectile_allowance_17mm;
            this->shooter_projectile_allowance_42mm = projectile_allowance_42mm;
            break;
        }
        case 0x0304: {
            if (data_length != 12) {
                // 结构体大小为12字节
                return;
            }
            const uint8_t *dataField = &packet[7];
            // 解析数据，全部使用小端字节序
            int16_t mouse_x = (int16_t) ((dataField[1] << 8) | dataField[0]);
            int16_t mouse_y = (int16_t) ((dataField[3] << 8) | dataField[2]);
            int16_t mouse_z = (int16_t) ((dataField[5] << 8) | dataField[4]);
            int8_t left_button_down = (int8_t) dataField[6];
            int8_t right_button_down = (int8_t) dataField[7];
            uint16_t keyboard_value = (dataField[9] << 8) | dataField[8];
            uint16_t reserved = (dataField[11] << 8) | dataField[10];
            this->mouse_x = mouse_x;
            this->mouse_y = mouse_y;
            this->mouse_z = mouse_z;
            this->mouse_left_button_down = left_button_down;
            this->mouse_right_button_down = right_button_down;
            this->keyboard_value = keyboard_value;
            w = keyboard_value & (1<<0);
            s = keyboard_value & (1<<1);
            a = keyboard_value & (1<<2);
            d = keyboard_value & (1<<3);
            shift = keyboard_value & (1<<4);
            ctrl = keyboard_value & (1<<5);
            q = keyboard_value & (1<<6);
            e = keyboard_value & (1<<7);
            r = keyboard_value & (1<<8);
            f = keyboard_value & (1<<9);
            g = keyboard_value & (1<<10);
            z = keyboard_value & (1<<11);
            x = keyboard_value & (1<<12);
            c = keyboard_value & (1<<13);
            v = keyboard_value & (1<<14);
            b = keyboard_value & (1<<15);
            break;
        }
        case 0x0101: //中心增益点占领情况
        {
            const uint8_t *dataField = &packet[7];
            if (data_length != 4) {
                // 结构体大小为4字节
                return;
            }
            uint32_t event_data = (dataField[3] << 24) | (dataField[2] << 16) | (dataField[1] << 8) | dataField[0];
            uint32_t center_gain_status = (event_data >> 30) & 0x3;
            this->center_gain_status = center_gain_status;
            break;
        }


        default:
            break;
    }
}

void Referee::PhaseData(const uint8_t *data, uint16_t size) {
    if (size < 10) //太短
        return;
    for (int i = 0; i < size; i++) {
        if (data[i] == 0xA5) // 寻找帧头
        {
            if (size - i < 5) //剩下数据的不足以解析包头
            {
                return;
            }
            uint16_t data_length = (data[i + 2] << 8) | data[i + 1];
            uint16_t total_packet_size = 5 + data_length + 2; // frame_header(5) + data_length + frame_tail(CRC16, 2)

            if (size - i < total_packet_size) {
                // 数据不足，无法解析完整包
                return;
            }

            // 调用解析函数
            ParsePacket(&data[i], total_packet_size);
            i += total_packet_size - 1; // 跳过已处理的数据
        }
    }
}


//以下为CRC校验，不用管-------------------------------------------------------
//crc8 generator polynomial:G(x=x8+x5+x4+1

unsigned char Referee::Get_CRC8_Check_Sum(const uint8_t *pchMessage, unsigned int dwLength, unsigned char ucCRC8) {
    unsigned char ucIndex;
    while (dwLength--) {
        ucIndex = ucCRC8 ^ (*pchMessage++);
        ucCRC8 = CRC8_TAB[ucIndex];
    }
    return (ucCRC8);
}

/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result
*/
unsigned int Referee::Verify_CRC8_Check_Sum(const uint8_t *pchMessage, unsigned int dwLength) {
    unsigned char ucExpected = 0;
    if (pchMessage == nullptr || dwLength <= 2)
        return 0;
    ucExpected = Get_CRC8_Check_Sum(pchMessage, dwLength - 1, CRC8_INIT);
    return (ucExpected == pchMessage[dwLength - 1]);
}

/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result
*/
void Referee::Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength) {
    unsigned char ucCRC = 0;
    if ((pchMessage == 0 || (dwLength <= 2)))return;
    ucCRC = Get_CRC8_Check_Sum(pchMessage, dwLength - 1, CRC8_INIT);
    pchMessage[dwLength - 1] = ucCRC;
}

uint16_t Referee::Get_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC) {
    uint8_t chData;
    if (pchMessage == nullptr) {
        return 0xFFFF;
    }
    while (dwLength--) {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t) (wCRC) >> 8) ^ wCRC_Table[((uint16_t) (wCRC) ^ (uint16_t) (chData)) & 0x00ff];
    }
    return wCRC;
}

/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result
*/
uint32_t Referee::Verify_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength) {
    uint16_t wExpected = 0;
    if ((pchMessage == nullptr || (dwLength <= 2))) {
        return 0;
    }
    wExpected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC_INIT);
    return (((wExpected & 0xff) == pchMessage[dwLength - 2] && ((
                 (wExpected >> 8 & 0xff) == pchMessage[dwLength - 1]))));
}

/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result
*/
void Referee::Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength) {
    uint16_t wCRC = 0;
    if ((pchMessage == nullptr || (dwLength <= 2))) {
        return;
    }
}
