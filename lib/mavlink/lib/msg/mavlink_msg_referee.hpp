// MESSAGE referee support class

#pragma once

namespace mavlink {
namespace msg {
namespace msg {

/**
 * @brief referee message
 *
 * referee(c -> pc)
 */
struct referee : mavlink::Message {
    static constexpr msgid_t MSG_ID = 1;
    static constexpr size_t LENGTH = 8;
    static constexpr size_t MIN_LENGTH = 8;
    static constexpr uint8_t CRC_EXTRA = 9;
    static constexpr auto NAME = "referee";


    uint8_t is_red; /*<  is_red */
    uint8_t game_progress; /*<  game_progress(see robomaster serial manual) */
    uint16_t stage_remain_time; /*<  stage_remain_time(see robomaster serial manual) */
    float bullet_speed; /*< [m/s] bullet_speed(referee measure value) */


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
        ss << "  game_progress: " << +game_progress << std::endl;
        ss << "  stage_remain_time: " << stage_remain_time << std::endl;
        ss << "  bullet_speed: " << bullet_speed << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << bullet_speed;                  // offset: 0
        map << stage_remain_time;             // offset: 4
        map << is_red;                        // offset: 6
        map << game_progress;                 // offset: 7
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> bullet_speed;                  // offset: 0
        map >> stage_remain_time;             // offset: 4
        map >> is_red;                        // offset: 6
        map >> game_progress;                 // offset: 7
    }
};

} // namespace msg
} // namespace msg
} // namespace mavlink
