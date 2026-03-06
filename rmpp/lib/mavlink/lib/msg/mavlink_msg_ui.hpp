// MESSAGE ui support class

#pragma once

namespace mavlink {
namespace msg {
namespace msg {

/**
 * @brief ui message
 *
 * ui(pc -> c)
 */
struct ui : mavlink::Message {
    static constexpr msgid_t MSG_ID = 3;
    static constexpr size_t LENGTH = 32;
    static constexpr size_t MIN_LENGTH = 32;
    static constexpr uint8_t CRC_EXTRA = 149;
    static constexpr auto NAME = "ui";


    float x0; /*<  x0 */
    float y0; /*<  y0 */
    float x1; /*<  x1 */
    float y1; /*<  y1 */
    float x2; /*<  x2 */
    float y2; /*<  y2 */
    float x3; /*<  x3 */
    float y3; /*<  y3 */


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
        ss << "  x0: " << x0 << std::endl;
        ss << "  y0: " << y0 << std::endl;
        ss << "  x1: " << x1 << std::endl;
        ss << "  y1: " << y1 << std::endl;
        ss << "  x2: " << x2 << std::endl;
        ss << "  y2: " << y2 << std::endl;
        ss << "  x3: " << x3 << std::endl;
        ss << "  y3: " << y3 << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << x0;                            // offset: 0
        map << y0;                            // offset: 4
        map << x1;                            // offset: 8
        map << y1;                            // offset: 12
        map << x2;                            // offset: 16
        map << y2;                            // offset: 20
        map << x3;                            // offset: 24
        map << y3;                            // offset: 28
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> x0;                            // offset: 0
        map >> y0;                            // offset: 4
        map >> x1;                            // offset: 8
        map >> y1;                            // offset: 12
        map >> x2;                            // offset: 16
        map >> y2;                            // offset: 20
        map >> x3;                            // offset: 24
        map >> y3;                            // offset: 28
    }
};

} // namespace msg
} // namespace msg
} // namespace mavlink
