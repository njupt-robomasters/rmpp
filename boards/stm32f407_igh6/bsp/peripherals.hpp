#pragma once

#include "board/bytes.hpp"

namespace rmpp {
void timer_init();
void pwm_init();
void can_init();
void can_receive(int id, std::uint32_t fifo);
void can_error(int id);
void uart_init();
void uart_receive(int id, std::uint16_t size);
void uart_error(int id);
void cdc_receive(Bytes bytes);
} // namespace rmpp
