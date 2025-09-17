#include "bsp.hpp"

void BSP::Init() {
    CAN::Init();
    CDC::Init();
    Dwt::Init();
    IMUHeat::Init();
    LED::Init();
    UART_RC::Init();
    UART_Video::Init();
}
