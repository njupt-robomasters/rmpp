#pragma once
#include <cstdint>

struct DwtRegisters {
    std::uint32_t CTRL = 0, CYCCNT = 0;
};

struct DebugRegisters {
    std::uint32_t DEMCR = 0;
};

// Null until main: global construction must not read these registers.
inline DwtRegisters* DWT = nullptr;
inline DebugRegisters* CoreDebug = nullptr;
inline std::uint32_t SystemCoreClock = 168000000;
inline std::uint32_t irq_mask = 0;

inline std::uint32_t __get_PRIMASK() {
    return irq_mask;
}

inline void __disable_irq() {
    irq_mask = 1;
}

inline void __set_PRIMASK(std::uint32_t mask) {
    irq_mask = mask;
}

inline constexpr std::uint32_t CoreDebug_DEMCR_TRCENA_Msk = 1;
inline constexpr std::uint32_t DWT_CTRL_CYCCNTENA_Msk = 1;

enum HAL_StatusTypeDef { HAL_OK,
                         HAL_ERROR,
                         HAL_BUSY };

inline std::uint32_t hal_tick = 0;

inline std::uint32_t HAL_GetTick() {
    return hal_tick;
}
