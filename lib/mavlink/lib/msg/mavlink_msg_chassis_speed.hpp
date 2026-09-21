// MESSAGE chassis_speed support class

#pragma once

namespace mavlink {
namespace msg {
namespace msg {

/**
 * @brief chassis_speed message
 *
 * chassis_speed(pc -> c, ref by yaw1)
 */
struct chassis_speed : mavlink::Message {
    static constexpr msgid_t MSG_ID = 6;
    static constexpr size_t LENGTH = 8;
    static constexpr size_t MIN_LENGTH = 8;
    static constexpr uint8_t CRC_EXTRA = 207;
    static constexpr auto NAME = "chassis_speed";


    float vx; /*< [m/s] x */
    float vy; /*< [m/s] y */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  vx: " << vx << std::endl;
        ss << "  vy: " << vy << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << vx;                            // offset: 0
        map << vy;                            // offset: 4
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> vx;                            // offset: 0
        map >> vy;                            // offset: 4
    }
};

} // namespace msg
} // namespace msg
} // namespace mavlink
