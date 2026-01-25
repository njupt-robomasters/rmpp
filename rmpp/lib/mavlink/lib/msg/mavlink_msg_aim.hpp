// MESSAGE aim support class

#pragma once

namespace mavlink {
namespace msg {
namespace msg {

/**
 * @brief aim message
 *
 * aim control(mini pc -> c board)
 */
struct aim : mavlink::Message {
    static constexpr msgid_t MSG_ID = 10;
    static constexpr size_t LENGTH = 10;
    static constexpr size_t MIN_LENGTH = 10;
    static constexpr uint8_t CRC_EXTRA = 133;
    static constexpr auto NAME = "aim";


    uint8_t is_detected; /*<  is_detected */
    uint8_t is_fire_advise; /*<  is_fire_advise */
    float yaw; /*< [deg] yaw */
    float pitch; /*< [deg] pitch */


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
        ss << "  is_detected: " << +is_detected << std::endl;
        ss << "  is_fire_advise: " << +is_fire_advise << std::endl;
        ss << "  yaw: " << yaw << std::endl;
        ss << "  pitch: " << pitch << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << yaw;                           // offset: 0
        map << pitch;                         // offset: 4
        map << is_detected;                   // offset: 8
        map << is_fire_advise;                // offset: 9
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> yaw;                           // offset: 0
        map >> pitch;                         // offset: 4
        map >> is_detected;                   // offset: 8
        map >> is_fire_advise;                // offset: 9
    }
};

} // namespace msg
} // namespace msg
} // namespace mavlink
