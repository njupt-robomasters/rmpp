// MESSAGE target_position support class

#pragma once

namespace mavlink {
namespace msg {
namespace msg {

/**
 * @brief target_position message
 *
 * target_position(c -> pc)
 */
struct target_position : mavlink::Message {
    static constexpr msgid_t MSG_ID = 4;
    static constexpr size_t LENGTH = 8;
    static constexpr size_t MIN_LENGTH = 8;
    static constexpr uint8_t CRC_EXTRA = 2;
    static constexpr auto NAME = "target_position";


    float x; /*< [m] x */
    float y; /*< [m] y */


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

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << x;                             // offset: 0
        map << y;                             // offset: 4
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> x;                             // offset: 0
        map >> y;                             // offset: 4
    }
};

} // namespace msg
} // namespace msg
} // namespace mavlink
