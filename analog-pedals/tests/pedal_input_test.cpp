#include "pedal_input.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

using namespace pedals;

static bool near(float a, float b, float eps = 0.001f) { return std::fabs(a - b) <= eps; }

static void deadzone_zeroes_rest_noise() {
    PedalSettings s;
    assert(trigger_to_pedal(1000, s) == 0.0f);
    assert(trigger_to_pedal(0, s) == 0.0f);
    assert(trigger_to_pedal(-5, s) == 0.0f);
}
static void full_press_is_one() { assert(near(trigger_to_pedal(32767, PedalSettings{}), 1.0f)); }
static void linear_midpoint() {
    PedalSettings s; // deadzone 0.05 -> live range 0.05..1, middle 0.525
    float raw = (0.05f + 1.0f) / 2.0f * 32767.0f;
    assert(near(trigger_to_pedal(static_cast<std::int16_t>(raw), s), 0.5f));
}
static void curve_two_squares() {
    PedalSettings s;
    s.deadzone = 0.0f;
    s.curve = 2.0f;
    assert(near(trigger_to_pedal(16384, s), 0.25f));
}
static void ramp_reaches_one_after_ramp_ms() {
    KeyRamp r;
    float v = 0;
    for (int i = 0; i < 4; ++i) v = r.step(true, 25, 100);
    assert(near(v, 1.0f));
    assert(near(r.step(true, 25, 100), 1.0f)); // clamped
}
static void ramp_zero_is_instant() {
    KeyRamp r;
    assert(r.step(true, 1, 0) == 1.0f);
    assert(r.step(false, 1, 0) == 0.0f);
}
static void ramp_is_capped_per_step() {
    KeyRamp r;
    assert(near(r.step(true, 5000, 100), 0.5f)); // 50 / 100
    KeyRamp q;
    assert(near(q.step(true, 5000, 200), 0.25f));
}
static void release_ramps_down() {
    KeyRamp r;
    r.step(true, 50, 50);
    assert(near(r.step(false, 25, 100), 0.75f));
    assert(near(r.step(false, 50, 100), 0.25f));
    assert(near(r.step(false, 50, 100), 0.0f));
}
static void combine_takes_max() {
    assert(combine(0.3f, 0.7f) == 0.7f);
    assert(combine(0.9f, 0.1f) == 0.9f);
}
static void fake_env_parses_and_rejects() {
    auto ok = parse_fake("0.5,0.3");
    assert(ok && near(ok->first, 0.5f) && near(ok->second, 0.3f));
    assert(parse_fake("0,1"));
    assert(!parse_fake(nullptr));
    assert(!parse_fake(""));
    assert(!parse_fake("0.5"));
    assert(!parse_fake("a,b"));
    assert(!parse_fake("0.5,"));
    assert(!parse_fake("1.5,0.2"));
    assert(!parse_fake("-0.1,0.2"));
    assert(!parse_fake("0.5,0.3,0.1"));
    assert(!parse_fake("0.5,0.3x"));
}

static PedalSettings parse(std::map<std::string, std::string> kv, std::vector<std::string> &warnings) {
    return parse_settings([&](const char *k) -> const char * { auto it = kv.find(k); return it == kv.end() ? nullptr : it->second.c_str(); },
                          [&](const std::string &w) { warnings.push_back(w); });
}
static void settings_valid_values_are_used() {
    std::vector<std::string> w;
    auto s = parse({{"deadzone", "0.2"}, {"curve", "1.5"}, {"ramp_ms", "250"}}, w);
    assert(near(s.deadzone, 0.2f) && near(s.curve, 1.5f) && s.ramp_ms == 250 && w.empty());
    s = parse({{"deadzone", "0"}, {"ramp_ms", "0"}}, w);
    assert(s.deadzone == 0.0f && s.ramp_ms == 0 && w.empty());
}
static void settings_missing_use_defaults_silently() {
    std::vector<std::string> w;
    auto s = parse({}, w);
    assert(near(s.deadzone, 0.05f) && near(s.curve, 1.0f) && s.ramp_ms == 100 && w.empty());
}
static void settings_invalid_log_once_and_default() {
    std::vector<std::string> w;
    auto s = parse({{"deadzone", "1"}, {"curve", "0"}, {"ramp_ms", "-5"}}, w);
    assert(near(s.deadzone, 0.05f) && near(s.curve, 1.0f) && s.ramp_ms == 100);
    assert(w.size() == 3);
    w.clear();
    s = parse({{"deadzone", "abc"}, {"curve", "-1"}, {"ramp_ms", "10x"}}, w);
    assert(near(s.deadzone, 0.05f) && near(s.curve, 1.0f) && s.ramp_ms == 100 && w.size() == 3);
}

int main() {
    settings_valid_values_are_used();
    settings_missing_use_defaults_silently();
    settings_invalid_log_once_and_default();
    deadzone_zeroes_rest_noise();
    full_press_is_one();
    linear_midpoint();
    curve_two_squares();
    ramp_reaches_one_after_ramp_ms();
    ramp_zero_is_instant();
    ramp_is_capped_per_step();
    release_ramps_down();
    combine_takes_max();
    fake_env_parses_and_rejects();
    std::puts("pedal_input_test: all passed");
    return 0;
}
