#pragma once

#include <cstddef>
#include <cstdint>

struct Bytes {
    const std::uint8_t* data = nullptr;
    std::size_t size = 0;
};
