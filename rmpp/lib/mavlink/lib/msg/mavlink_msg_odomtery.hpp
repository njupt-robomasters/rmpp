// MESSAGE odomtery support class

#pragma once

namespace mavlink {
namespace msg {
namespace msg {

/**
 * @brief odomtery message
 *
 * current position and yaw(mini pc -> c)
 */
struct odomtery : mavlink::Message {
    static constexpr msgid_t MSG_ID = 22;
    static constexpr size_t LENGTH = 12;
    static constexpr size_t MIN_LENGTH = 12;
    static constexpr uint8_t CRC_EXTRA = 187;
    static constexpr auto NAME = "odomtery";


    float pos_x; /*< [m] x */
    float pos_y; /*< [m] y */
    float yaw; /*< [rad] yaw */


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
        ss << "  pos_x: " << pos_x << std::endl;
        ss << "  pos_y: " << pos_y << std::endl;
        ss << "  yaw: " << yaw << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << pos_x;                         // offset: 0
        map << pos_y;                         // offset: 4
        map << yaw;                           // offset: 8
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> pos_x;                         // offset: 0
        map >> pos_y;                         // offset: 4
        map >> yaw;                           // offset: 8
    }
};

} // namespace msg
} // namespace msg
} // namespace mavlink
