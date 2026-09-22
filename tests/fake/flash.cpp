extern "C" {
#include "fal.h"
}
#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <cstring>

unsigned flash_reads = 0;
unsigned flash_writes = 0;
unsigned flash_erases = 0;
static std::array<std::uint8_t, 1024 * 1024> memory;

static int init() {
    memory.fill(0xff);
    return 0;
}

static int read(long offset, std::uint8_t* data, std::size_t size) {
    assert(offset >= 768 * 1024 && static_cast<std::size_t>(offset) + size <= memory.size());
    ++flash_reads;
    std::memcpy(data, memory.data() + offset, size);
    return size;
}

static int write(long offset, const std::uint8_t* data, std::size_t size) {
    assert(offset >= 768 * 1024 && static_cast<std::size_t>(offset) + size <= memory.size());
    ++flash_writes;
    for (std::size_t i = 0; i < size; ++i) {
        assert((memory[offset + i] & data[i]) == data[i]);
        memory[offset + i] &= data[i];
    }
    return size;
}

static int erase(long offset, std::size_t size) {
    assert(offset >= 768 * 1024 && static_cast<std::size_t>(offset) + size <= memory.size());
    assert(offset % (128 * 1024) == 0 && size % (128 * 1024) == 0);
    ++flash_erases;
    std::fill_n(memory.data() + offset, size, 0xff);
    return size;
}

extern "C" const struct fal_flash_dev stm32_onchip_flash = {
    "stm32_onchip",
    0x08000000,
    1024 * 1024,
    128 * 1024,
    {init, read, write, erase},
    8,
};
