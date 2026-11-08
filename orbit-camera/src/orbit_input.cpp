#include "orbit_input.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace orbit {
namespace {
constexpr float kRadToDeg = 180.0f / 3.14159265358979f;

bool parse_float(const char *text, float *out) {
    if (text == nullptr || *text == '\0') return false;
    char *end = nullptr;
    const float v = std::strtof(text, &end);
    if (end == text || *end != '\0' || !std::isfinite(v)) return false;
    *out = v;
    return true;
}

float wrap180(float a) {  // (-180, 180]
    while (a > 180.0f) a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

void read_setting(const std::function<const char *(const char *)> &get, const std::function<void(const std::string &)> &warn,
                  const char *key, float lo, float hi, bool lo_inclusive, float *value) {
    const char *text = get(key);
    if (text == nullptr) return;
    float v;
    if (!parse_float(text, &v) || v > hi || v < lo || (!lo_inclusive && v == lo)) {
        warn(std::string("invalid ") + key + " = \"" + text + "\", keeping the default");
        return;
    }
    *value = v;
}
}  // namespace

Settings parse_settings(const std::function<const char *(const char *)> &get,
                        const std::function<void(const std::string &)> &warn) {
    Settings s;
    read_setting(get, warn, "deadzone", 0.0f, 1.0f, true, &s.deadzone);
    if (s.deadzone >= 1.0f) {  // 1 would make the stick unusable
        warn("invalid deadzone = \"1\", keeping the default");
        s.deadzone = Settings{}.deadzone;
    }
    read_setting(get, warn, "back_threshold", 0.0f, 1.0f, false, &s.back_threshold);
    read_setting(get, warn, "smooth_ms", 0.0f, 5000.0f, true, &s.smooth_ms);
    read_setting(get, warn, "pitch_deg", -45.0f, 45.0f, true, &s.pitch_deg);
    for (auto [key, field] : {std::pair{"invert_outside_chase", &s.invert_outside_chase}, std::pair{"invert_in_chase", &s.invert_in_chase}}) {
        const char *v = get(key);
        if (v == nullptr) continue;
        const std::string t = v;
        if (t == "true" || t == "1") *field = true;
        else if (t == "false" || t == "0") *field = false;
        else warn(std::string("invalid ") + key + " = \"" + t + "\", keeping the default");
    }
    return s;
}

bool is_full_chase_view(const float pivot[3], bool was_chase) {
    const float d = std::sqrt(pivot[0] * pivot[0] + pivot[1] * pivot[1] + pivot[2] * pivot[2]);
    return was_chase ? d >= 6.0f : d > 7.0f;
}

float view_yaw(float target, bool chase_view, const Settings &s) {
    if (!(chase_view ? s.invert_in_chase : s.invert_outside_chase) || target == 0.0f) return target;
    return -target;
}

float target_yaw(const Raw &raw, const Settings &s) {
    if (raw.back || raw.r3) return 180.0f;
    if (raw.left != raw.right) return raw.right ? 90.0f : -90.0f;
    if (raw.left && raw.right) return 0.0f;
    const float mag = std::hypot(raw.x, raw.y);
    if (mag < s.deadzone) return 0.0f;
    const float angle = std::atan2(raw.x, -raw.y) * kRadToDeg;
    if (raw.y <= s.back_threshold) return angle;
    const float w = std::min(1.0f, (raw.y - s.back_threshold) / (1.0f - s.back_threshold));
    const float rear = raw.x < 0.0f ? -180.0f : 180.0f;
    return angle + w * (rear - angle);
}

float YawSmoother::step(float target, float dt_ms, float tau_ms) {
    const float delta = wrap180(target - value_);
    if (tau_ms <= 0.0f || std::fabs(delta) < 0.05f) {
        value_ = target;
        return value_;
    }
    const float alpha = 1.0f - std::exp(-dt_ms / tau_ms);
    value_ = wrap180(value_ + delta * alpha);
    return value_;
}

std::optional<std::pair<float, float>> parse_fake(const char *env) {
    if (env == nullptr || *env == '\0') return std::nullopt;
    char *end = nullptr;
    const float yaw = std::strtof(env, &end);
    if (end == env || !std::isfinite(yaw) || std::fabs(yaw) > 360.0f) return std::nullopt;
    float pitch = 0.0f;
    if (*end == ',') {
        const char *p = end + 1;
        char *end2 = nullptr;
        pitch = std::strtof(p, &end2);
        if (end2 == p || *end2 != '\0' || !std::isfinite(pitch) || std::fabs(pitch) > 45.0f) return std::nullopt;
    } else if (*end != '\0') {
        return std::nullopt;
    }
    return std::make_pair(yaw, pitch);
}

}  // namespace orbit
