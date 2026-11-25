#pragma once

#include "bsp/can.hpp"
#include "bsp/irq_lock.hpp"
#include "minipc/nav.hpp"
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

static uint64_t now_us = 1000000;
static uint32_t irq_depth = 0;
static int checks = 0;
static std::vector<Cdc::RxCallback> cdc_callbacks[4];
static std::vector<Can::RxCallback> can_callbacks;
static CanFrame last_can;

#define CHECK(condition) do { if (!(condition)) { \
    std::fprintf(stderr, "check failed at %s:%d: %s\n", __FILE__, __LINE__, #condition); \
    std::abort(); } checks++; } while (false)

static void near(const float actual, const float expected) {
    CHECK(std::isfinite(actual) && std::fabs(actual - expected) < 2e-5f);
}

float Timer::elapsed() const {
    return static_cast<float>(now_us - last_cycles) * 1e-6f;
}

float Timer::restart() {
    const float result = elapsed();
    last_cycles = now_us;
    return result;
}

IrqLock::IrqLock() : primask(irq_depth) {
    irq_depth++;
}

IrqLock::~IrqLock() {
    irq_depth = primask;
}

void Cdc::add_rx_callback(RxCallback callback) const {
    cdc_callbacks[id].push_back(std::move(callback));
}

void Can::add_rx_callback(RxCallback callback) const {
    can_callbacks.push_back(std::move(callback));
}

void Can::send(const CanFrame& frame) const {
    last_can = frame;
}

static void feed(const int channel, const uint8_t* bytes, const std::size_t size) {
    for (const auto& callback : cdc_callbacks[channel])
        callback(bytes, size);
    CHECK(irq_depth == 0);
}

// 与生产代码的逐位实现独立，生成查表结果；黄金帧另用固定字节核对
static uint16_t reference_crc(const uint8_t* bytes, const std::size_t size) {
    uint16_t table[256];
    for (unsigned i = 0; i < 256; i++) {
        unsigned value = i;
        for (int bit = 0; bit < 8; bit++) {
            if (value & 1)
                value = (value >> 1) ^ 0xA001;
            else
                value >>= 1;
        }
        table[i] = static_cast<uint16_t>(value);
    }
    uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < size; i++)
        crc = (crc >> 8) ^ table[(crc ^ bytes[i]) & 0xFF];
    return crc;
}

static std::array<uint8_t, 24> make_frame(
    const float vx, const float vy, const float yaw, const float x, const float y, const uint8_t is_through_hole) {
    std::array<uint8_t, 24> bytes{0xA3};
    bytes[1] = is_through_hole;
    const float values[] = {vx, vy, yaw, x, y};
    for (int i = 0; i < 5; i++) {
        uint32_t bits;
        std::memcpy(&bits, &values[i], 4);
        for (int j = 0; j < 4; j++)
            bytes[2 + 4 * i + j] = static_cast<uint8_t>(bits >> (8 * j));
    }
    const uint16_t crc = reference_crc(bytes.data(), 22);
    bytes[22] = static_cast<uint8_t>(crc);
    bytes[23] = static_cast<uint8_t>(crc >> 8);
    return bytes;
}
