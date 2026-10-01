// 主机测试桥接；直接包含实现以核验文件内固定矩阵运算，不增加固件测试接口
#include "imu/imu.hpp"
#include "imu/quaternion_ekf.cpp"
#include <memory>

static uint32_t clock_us = 0;
static unsigned transfers = 0;

struct Sensor {
    uint8_t registers[128]{};
    bool accel = false;
    bool fail = false;
};

struct Spi::Context {
    Sensor* sensor;
};

static Sensor sensors[8][2];
static std::unique_ptr<Imu> instances[8];

Spi::Spi(const int bus, const int chip_select) {
    static Context contexts[8][2];
    context_ = &contexts[bus][chip_select - 1];
    context_->sensor = &sensors[bus][chip_select - 1];
    auto& sensor = *context_->sensor;
    sensor.accel = chip_select == 1;
    sensor.registers[0] = sensor.accel ? 0x1E : 0x0F;
}

void Spi::transfer(const uint8_t* tx_buf, uint8_t* rx_buf, const std::size_t size) const {
    ++transfers;
    auto& sensor = *context_->sensor;
    std::memset(rx_buf, 0, size);
    if (sensor.fail)
        return;
    const int reg = tx_buf[0] & 0x7F;
    if ((tx_buf[0] & 0x80) == 0) {
        sensor.registers[reg] = tx_buf[1];
        return;
    }
    const unsigned prefix = sensor.accel ? 2 : 1;
    for (unsigned i = prefix; i < size; ++i)
        rx_buf[i] = sensor.registers[reg + i - prefix];
}

Pwm::Pwm(int, int) {}

void Pwm::set_duty(float) const {}

void Timer::delay(const float seconds) {
    clock_us += static_cast<uint32_t>(std::round(seconds * 1.0e6f));
}

float Timer::elapsed() {
    dt = std::clamp((clock_us - last_tick_) * 1.0e-6f, min_dt_, max_dt_);
    return dt;
}

float Timer::restart() {
    const float result = elapsed();
    last_tick_ = clock_us;
    return result;
}

extern "C" {
void* ekf_create(const float* values) {
    QuaternionEkf::Config config;
    if (values)
        config = {values[0], values[1], values[2], values[3], values[4], values[5]};
    return new QuaternionEkf(config);
}

void ekf_destroy(void* instance) {
    delete static_cast<QuaternionEkf*>(instance);
}

void ekf_step(void* instance, const float* gyro, const float* accel, const float dt, float* output) {
    auto& ekf = *static_cast<QuaternionEkf*>(instance);
    ekf.update(gyro, accel, dt);
    std::memcpy(output, ekf.q, sizeof(ekf.q));
    std::memcpy(output + 4, ekf.bias, sizeof(ekf.bias));
}

void ekf_reset_bias(void* instance) {
    static_cast<QuaternionEkf*>(instance)->reset_bias();
}

void convert_euler(const float* angles, float* q, float* roundtrip) {
    Imu::euler_to_quaternion(angles[0], angles[1], angles[2], q);
    const auto [yaw, pitch, roll] = Imu::quaternion_to_euler(q);
    roundtrip[0] = yaw;
    roundtrip[1] = pitch;
    roundtrip[2] = roll;
}

void rotate_vector(float* vector, const float* q, const bool inverse) {
    if (inverse)
        Imu::world_to_body(vector, vector, q);
    else
        Imu::body_to_world(vector, vector, q);
}

void integrate(float* q, const float* gyro, const float dt) {
    Imu::integrate_quaternion(q, gyro[0], gyro[1], gyro[2], dt);
}

void matrix_product(const float* left, const float* right, float* out, const bool transpose) {
    float a[6][6], b[6][6], c[6][6];
    std::memcpy(a, left, sizeof(a));
    std::memcpy(b, right, sizeof(b));
    if (transpose)
        multiply_transpose(a, b, c);
    else
        multiply(a, b, c);
    std::memcpy(out, c, sizeof(c));
}

void matrix_solve(const float* factor, const float* rhs, float* out) {
    float lower[3][3];
    std::memcpy(lower, factor, sizeof(lower));
    solve_cholesky(lower, rhs, out);
}

bool covariance_check(float* data) {
    float covariance[6][6];
    std::memcpy(covariance, data, sizeof(covariance));
    const bool valid = symmetrize_covariance(covariance);
    std::memcpy(data, covariance, sizeof(covariance));
    return valid;
}

void imu_create(const int index, const float* angles, const float* offset, const float gravity) {
    const unsigned before = transfers;
    instances[index] = std::make_unique<Imu>(Imu::Config{
        .hardware = {.accel = Spi(index, 1), .gyro = Spi(index, 2), .heater = Pwm(10, 1)},
        .adjust = {.yaw = angles[0], .pitch = angles[1], .roll = angles[2], .gyro_offset = {offset[0], offset[1], offset[2]}, .gravity = gravity},
    });
    if (transfers != before)
        std::abort();
}

void imu_sample(const int index, const int16_t* gyro, const int16_t* accel, const int16_t temperature, const bool fail) {
    auto& a = sensors[index][0];
    auto& g = sensors[index][1];
    g.fail = fail;
    for (int i = 0; i < 3; ++i) {
        a.registers[0x12 + 2 * i] = accel[i] & 0xFF;
        a.registers[0x13 + 2 * i] = (static_cast<uint16_t>(accel[i]) >> 8);
        g.registers[0x02 + 2 * i] = gyro[i] & 0xFF;
        g.registers[0x03 + 2 * i] = (static_cast<uint16_t>(gyro[i]) >> 8);
    }
    const auto raw_temperature = static_cast<uint16_t>(temperature) & 0x7FF;
    a.registers[0x22] = raw_temperature >> 3;
    a.registers[0x23] = (raw_temperature & 7) << 5;
}

void advance(const uint32_t microseconds) {
    clock_us += microseconds;
}

void imu_step(const int index, float* output) {
    auto& imu = *instances[index];
    imu.update();
    std::memcpy(output, imu.gyro, sizeof(imu.gyro));
    std::memcpy(output + 3, imu.accel, sizeof(imu.accel));
    std::memcpy(output + 6, imu.q, sizeof(imu.q));
    output[10] = imu.yaw;
    output[11] = imu.pitch;
    output[12] = imu.roll;
    output[13] = imu.yaw_total_angle;
    output[14] = imu.temperature;
    output[15] = imu.heater_duty;
    output[16] = static_cast<int>(imu.calibration);
    output[17] = imu.calibration_samples;
    std::memcpy(output + 18, imu.config().adjust.gyro_offset, sizeof(imu.config().adjust.gyro_offset));
    output[21] = imu.config().adjust.gravity;
}

void imu_calibrate(const int index) {
    instances[index]->start_calibration();
}
}
