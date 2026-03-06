// MESSAGE nav_cmd_vel support class

#pragma once

namespace mavlink {
namespace msg {
namespace msg {

/**
 * @brief nav_cmd_vel message
 *
 * nav_cmd_vel(mini pc -> c board reference yaw1)
 */
struct nav_cmd_vel : mavlink::Message {
    static constexpr msgid_t MSG_ID = 20;
    static constexpr size_t LENGTH = 8;
    static constexpr size_t MIN_LENGTH = 8;
    static constexpr uint8_t CRC_EXTRA = 58;
    static constexpr auto NAME = "nav_cmd_vel";


    float vel_x; /*< [m_s] x */
    float vel_y; /*< [m_s] y */


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
        ss << "  vel_x: " << vel_x << std::endl;
        ss << "  vel_y: " << vel_y << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << vel_x;                         // offset: 0
        map << vel_y;                         // offset: 4
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> vel_x;                         // offset: 0
        map >> vel_y;                         // offset: 4
    }
};

} // namespace msg
} // namespace msg
} // namespace mavlink
