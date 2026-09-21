#pragma once

#include <cstddef>
#include <cstdint>

namespace rmpp {
struct Bytes {
    const std::uint8_t* data = nullptr;
    std::size_t size = 0;
};
} // namespace rmpp
