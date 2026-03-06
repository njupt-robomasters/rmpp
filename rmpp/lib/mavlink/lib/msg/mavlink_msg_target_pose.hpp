// MESSAGE target_pose support class

#pragma once

namespace mavlink {
namespace msg {
namespace msg {

/**
 * @brief target_pose message
 *
 * target_pose(c board -> mini pc)
 */
struct target_pose : mavlink::Message {
    static constexpr msgid_t MSG_ID = 21;
    static constexpr size_t LENGTH = 8;
    static constexpr size_t MIN_LENGTH = 8;
    static constexpr uint8_t CRC_EXTRA = 63;
    static constexpr auto NAME = "target_pose";


    float pos_x; /*< [m] x */
    float pos_y; /*< [m] y */


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

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << pos_x;                         // offset: 0
        map << pos_y;                         // offset: 4
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> pos_x;                         // offset: 0
        map >> pos_y;                         // offset: 4
    }
};

} // namespace msg
} // namespace msg
} // namespace mavlink
