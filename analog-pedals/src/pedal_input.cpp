#include "pedal_input.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace pedals {

float trigger_to_pedal(std::int16_t raw, const PedalSettings &s) {
    float v = std::clamp(static_cast<float>(raw) / 32767.0f, 0.0f, 1.0f);
    if (v < s.deadzone) return 0.0f;
    float span = 1.0f - s.deadzone;
    v = span > 0.0f ? (v - s.deadzone) / span : 1.0f;
    return std::clamp(std::pow(v, s.curve), 0.0f, 1.0f);
}

float KeyRamp::step(bool held, int elapsed_ms, int ramp_ms) {
    float target = held ? 1.0f : 0.0f;
    if (ramp_ms <= 0) {
        value_ = target;
        return value_;
    }
    float delta = static_cast<float>(std::clamp(elapsed_ms, 0, 50)) / static_cast<float>(ramp_ms);
    if (value_ < target) value_ = std::min(target, value_ + delta);
    else value_ = std::max(target, value_ - delta);
    return value_;
}

float combine(float pad, float keys) { return std::max(pad, keys); }

static bool parse_unit(const char *&p, char terminator, float &out) {
    char *end = nullptr;
    float v = std::strtof(p, &end);
    if (end == p || *end != terminator || !(v >= 0.0f && v <= 1.0f)) return false;
    out = v;
    p = end;
    return true;
}

std::optional<std::pair<float, float>> parse_fake(const char *env) {
    if (!env) return std::nullopt;
    const char *p = env;
    float a = 0, b = 0;
    if (!parse_unit(p, ',', a)) return std::nullopt;
    ++p;
    if (!parse_unit(p, '\0', b)) return std::nullopt;
    return std::make_pair(a, b);
}

} // namespace pedals
