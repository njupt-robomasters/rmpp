#include "board/board.hpp"

namespace rmpp {
Can Board::can(int id) const { return Can{Can::find(id)}; }
Uart Board::uart(int id) const { return Uart{Uart::find(id)}; }
Cdc Board::cdc(int id) const { return Cdc{Cdc::find(id)}; }
Pwm Board::pwm(int timer, int channel) const { return Pwm{Pwm::find(timer, channel)}; }
Led Board::led(int id) const {
    return id == 1 ? Led{pwm(5, 3), pwm(5, 2), pwm(5, 1)} : Led{};
}
} // namespace rmpp
