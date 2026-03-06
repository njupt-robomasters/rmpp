// MESSAGE auto_aim support class

#pragma once

namespace mavlink {
namespace msg {
namespace msg {

/**
 * @brief auto_aim message
 *
 * auto_aim(pc -> c)
 */
struct auto_aim : mavlink::Message {
    static constexpr msgid_t MSG_ID = 2;
    static constexpr size_t LENGTH = 19;
    static constexpr size_t MIN_LENGTH = 19;
    static constexpr uint8_t CRC_EXTRA = 152;
    static constexpr auto NAME = "auto_aim";


    uint8_t is_detect; /*<  is_detect */
    float yaw; /*< [deg] yaw(left is positive) */
    float pitch; /*< [deg] pitch(up is positive) */
    uint8_t is_fire; /*<  is_fire */
    uint8_t robot_id; /*<  robot_id(1-7, see robomaster manual) */
    float wr; /*< [rpm] rpm */
    float distance; /*< [m] distance */


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
        ss << "  is_detect: " << +is_detect << std::endl;
        ss << "  yaw: " << yaw << std::endl;
        ss << "  pitch: " << pitch << std::endl;
        ss << "  is_fire: " << +is_fire << std::endl;
        ss << "  robot_id: " << +robot_id << std::endl;
        ss << "  wr: " << wr << std::endl;
        ss << "  distance: " << distance << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << yaw;                           // offset: 0
        map << pitch;                         // offset: 4
        map << wr;                            // offset: 8
        map << distance;                      // offset: 12
        map << is_detect;                     // offset: 16
        map << is_fire;                       // offset: 17
        map << robot_id;                      // offset: 18
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> yaw;                           // offset: 0
        map >> pitch;                         // offset: 4
        map >> wr;                            // offset: 8
        map >> distance;                      // offset: 12
        map >> is_detect;                     // offset: 16
        map >> is_fire;                       // offset: 17
        map >> robot_id;                      // offset: 18
    }
};

} // namespace msg
} // namespace msg
} // namespace mavlink
