// MESSAGE insta360 support class

#pragma once

namespace mavlink {
namespace msg {
namespace msg {

/**
 * @brief insta360 message
 *
 * insta360(pc -> c)
 */
struct insta360 : mavlink::Message {
    static constexpr msgid_t MSG_ID = 3;
    static constexpr size_t LENGTH = 24;
    static constexpr size_t MIN_LENGTH = 24;
    static constexpr uint8_t CRC_EXTRA = 134;
    static constexpr auto NAME = "insta360";


    float a0; /*< [deg] a0 */
    float c0; /*<  c0 */
    float a1; /*< [deg] a1 */
    float c1; /*<  c1 */
    float a2; /*< [deg] a2 */
    float c2; /*<  c2 */


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
        ss << "  a0: " << a0 << std::endl;
        ss << "  c0: " << c0 << std::endl;
        ss << "  a1: " << a1 << std::endl;
        ss << "  c1: " << c1 << std::endl;
        ss << "  a2: " << a2 << std::endl;
        ss << "  c2: " << c2 << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << a0;                            // offset: 0
        map << c0;                            // offset: 4
        map << a1;                            // offset: 8
        map << c1;                            // offset: 12
        map << a2;                            // offset: 16
        map << c2;                            // offset: 20
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> a0;                            // offset: 0
        map >> c0;                            // offset: 4
        map >> a1;                            // offset: 8
        map >> c1;                            // offset: 12
        map >> a2;                            // offset: 16
        map >> c2;                            // offset: 20
    }
};

} // namespace msg
} // namespace msg
} // namespace mavlink
