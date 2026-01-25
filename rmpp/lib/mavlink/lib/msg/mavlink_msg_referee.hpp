// MESSAGE referee support class

#pragma once

namespace mavlink {
namespace msg {
namespace msg {

/**
 * @brief referee message
 *
 * referee report(c board -> mini pc)
 */
struct referee : mavlink::Message {
    static constexpr msgid_t MSG_ID = 1;
    static constexpr size_t LENGTH = 5;
    static constexpr size_t MIN_LENGTH = 5;
    static constexpr uint8_t CRC_EXTRA = 20;
    static constexpr auto NAME = "referee";


    uint8_t is_red; /*<  is_red(1 is red, 0 is blue) */
    float bullet_speed; /*< [m/s] bullet_speed */


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
        ss << "  is_red: " << +is_red << std::endl;
        ss << "  bullet_speed: " << bullet_speed << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << bullet_speed;                  // offset: 0
        map << is_red;                        // offset: 4
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> bullet_speed;                  // offset: 0
        map >> is_red;                        // offset: 4
    }
};

} // namespace msg
} // namespace msg
} // namespace mavlink
