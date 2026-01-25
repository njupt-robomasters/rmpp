#include "bsp.hpp"

void BSP::Init() {
    CAN::Init();
    CDC::Init();
    Dwt::Init();
    IMUHeat::Init();
    LED::Init();
    UART3::Init();
    UART6::Init();
}