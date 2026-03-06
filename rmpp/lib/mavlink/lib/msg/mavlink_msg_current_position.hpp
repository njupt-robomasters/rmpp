// MESSAGE current_position support class

#pragma once

namespace mavlink {
namespace msg {
namespace msg {

/**
 * @brief current_position message
 *
 * current_position(pc -> c)
 */
struct current_position : mavlink::Message {
    static constexpr msgid_t MSG_ID = 5;
    static constexpr size_t LENGTH = 12;
    static constexpr size_t MIN_LENGTH = 12;
    static constexpr uint8_t CRC_EXTRA = 178;
    static constexpr auto NAME = "current_position";


    float x; /*< [m] x */
    float y; /*< [m] y */
    float yaw; /*< [deg] yaw */


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
        ss << "  x: " << x << std::endl;
        ss << "  y: " << y << std::endl;
        ss << "  yaw: " << yaw << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << x;                             // offset: 0
        map << y;                             // offset: 4
        map << yaw;                           // offset: 8
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> x;                             // offset: 0
        map >> y;                             // offset: 4
        map >> yaw;                           // offset: 8
    }
};

} // namespace msg
} // namespace msg
} // namespace mavlink
