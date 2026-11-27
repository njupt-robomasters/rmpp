#include "../../../app/jelly_sentry_debug/yaw_pid/profile.hpp"
#include <cassert>
#include <limits>

int main() {
    YawTestProfile profile;
    for (const float bad : {-1.0f, 26 * deg, std::numeric_limits<float>::quiet_NaN()})
        assert(!profile.set_profile({.amplitude = bad}));
    assert(!profile.set_profile({.wave = 3}));
    assert(!profile.set_profile({.frequency = 1}));
    assert(!profile.set_profile({.duration = 21}));
    assert(profile.set_profile({.wave = YawTestProfile::STEP}));
    bool has_positive = false, has_negative = false;
    for (unsigned i = 0; i < 7100; i++) {
        profile.update(0.001f);
        assert(std::fabs(profile.offset) <= 20 * deg);
        if (profile.elapsed > 1.1f && profile.elapsed < 2.9f) {
            assert(profile.offset == 20 * deg);
            has_positive = true;
        }
        if (profile.elapsed > 3.1f && profile.elapsed < 4.9f) {
            assert(profile.offset == -20 * deg);
            has_negative = true;
        }
    }
    assert(has_positive && has_negative && profile.is_finished && profile.offset == 0);
    assert(profile.set_profile({.wave = YawTestProfile::SINE, .amplitude = 25 * deg}));
    has_positive = has_negative = false;
    for (unsigned i = 0; i < 20100; i++) {
        profile.update(0.001f);
        assert(std::fabs(profile.offset) <= 25 * deg);
        has_positive |= profile.offset > 24 * deg;
        has_negative |= profile.offset < -24 * deg;
    }
    assert(has_positive && has_negative && profile.is_finished && profile.offset == 0);
}
