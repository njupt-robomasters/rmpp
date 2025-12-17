// MESSAGE imu support class

#pragma once

namespace mavlink {
namespace msg {
namespace msg {

/**
 * @brief imu message
 *
 * imu report(c board -> mini pc)
 */
struct imu : mavlink::Message {
    static constexpr msgid_t MSG_ID = 0;
    static constexpr size_t LENGTH = 12;
    static constexpr size_t MIN_LENGTH = 12;
    static constexpr uint8_t CRC_EXTRA = 178;
    static constexpr auto NAME = "imu";


    float yaw; /*< [deg] yaw */
    float pitch; /*< [deg] pitch */
    float roll; /*< [deg] roll */


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
        ss << "  yaw: " << yaw << std::endl;
        ss << "  pitch: " << pitch << std::endl;
        ss << "  roll: " << roll << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << yaw;                           // offset: 0
        map << pitch;                         // offset: 4
        map << roll;                          // offset: 8
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> yaw;                           // offset: 0
        map >> pitch;                         // offset: 4
        map >> roll;                          // offset: 8
    }
};

} // namespace msg
} // namespace msg
} // namespace mavlink
