#include "cyber_dog_motor.hpp"

CyberDogMotor::CyberDogMotor(const Config& config, Protocol protocol)
    : MitMotor(apply_defaults(config, DEFAULT_CONFIG),
               {.max_speed = protocol.max_speed ? protocol.max_speed : DEFAULT_PROTOCOL.max_speed,
                .max_position = protocol.max_position ? protocol.max_position : DEFAULT_PROTOCOL.max_position}) {
    subscribe();
}

bool CyberDogMotor::send_can_cmd() {
    update_connection();
    ++send_count_;
    if (!is_connected || !is_enabled)
        return send_state(0xfd);
    // 保留每 100 次调用重新发送使能的节奏，包括提交失败的调用
    if (send_count_ % 100 == 0)
        return send_state(0xfc);
    return send_mit(CURRENT_OUTPUT);
}

void CyberDogMotor::receive(const CanFrame& frame) {
    if (!is_feedback_frame(frame) || frame.data[0] != config.protocol.slave_id)
        return;
    accept_mit_feedback(frame.data, CURRENT_OUTPUT);
}
