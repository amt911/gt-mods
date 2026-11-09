#include "rollover_logic.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>

namespace rollover {
namespace {
bool parse_float(const char *text, float *out) {
    if (text == nullptr || *text == '\0') return false;
    char *end = nullptr;
    const float v = std::strtof(text, &end);
    if (end == text || *end != '\0' || !std::isfinite(v)) return false;
    *out = v;
    return true;
}

void read_setting(const std::function<const char *(const char *)> &get, const std::function<void(const std::string &)> &warn,
                  const char *key, float lo, float hi, float *value) {
    const char *text = get(key);
    if (text == nullptr) return;
    float v;
    if (!parse_float(text, &v) || v < lo || v > hi) {
        warn(std::string("invalid ") + key + " = \"" + text + "\", keeping the default");
        return;
    }
    *value = v;
}
}  // namespace

Settings parse_settings(const std::function<const char *(const char *)> &get, const std::function<void(const std::string &)> &warn) {
    Settings s;
    read_setting(get, warn, "threshold", 0.1f, 50.0f, &s.threshold);
    read_setting(get, warn, "gain", 0.0f, 50.0f, &s.gain);
    read_setting(get, warn, "cap", 0.0f, 30.0f, &s.cap);
    read_setting(get, warn, "box_height", 0.5f, 3.0f, &s.box_height);
    read_setting(get, warn, "box_kick_cap", 0.1f, 20.0f, &s.box_kick_cap);
    if (const char *box = get("body_box")) {
        if (std::strcmp(box, "true") == 0) s.body_box = true;
        else if (std::strcmp(box, "false") == 0) s.body_box = false;
        else warn(std::string("invalid body_box = \"") + box + "\" (true or false), keeping true");
    }
    if (const char *style = get("style")) {
        if (std::strcmp(style, "realistic") == 0) s.style = Style::Realistic;
        else if (std::strcmp(style, "arcade") == 0) s.style = Style::Arcade;
        else warn(std::string("invalid style = \"") + style + "\" (realistic or arcade), keeping realistic");
    }
#if defined(ROLLOVER_SABOTAGE)
    s.sabotage = ROLLOVER_SABOTAGE;  // the in-game check's sabotage builds (it must fail with them)
#endif
    return s;
}

float effective_threshold(const Settings &s) {
    if (s.sabotage == 3) return 0.0f;
    return s.style == Style::Arcade ? s.threshold * kArcadeThresholdScale : s.threshold;
}

float effective_gain(const Settings &s) { return s.style == Style::Arcade ? s.gain * kArcadeGainScale : s.gain; }

float roll_rate_for(const Settings &s, float dv_lat) {
    if (!std::isfinite(dv_lat)) return 0.0f;
    const float excess = std::fabs(dv_lat) - effective_threshold(s);
    if (!(excess > 0.0f)) return 0.0f;
    return std::fmin(effective_gain(s) * excess, s.cap);
}

float Tracker::step(const float vel[3], const float right[3], const float pos[3]) {
    float out = 0.0f;
    bool ok = true;
    for (int i = 0; i < 3; ++i) ok = ok && std::isfinite(vel[i]) && std::isfinite(right[i]) && std::isfinite(pos[i]);
    if (!ok) {
        have_prev_ = false;
        left_ = 0;
        return 0.0f;
    }
    if (have_prev_) {
        const float dx = pos[0] - prev_pos_[0], dy = pos[1] - prev_pos_[1], dz = pos[2] - prev_pos_[2];
        const float dvx = vel[0] - prev_vel_[0], dvy = vel[1] - prev_vel_[1], dvz = vel[2] - prev_vel_[2];
        const bool moved = std::sqrt(dx * dx + dy * dy + dz * dz) > kJumpMeters;
        const bool jumped = std::sqrt(dvx * dvx + dvy * dvy + dvz * dvz) > kMaxDeltaV;
        if (moved || jumped) {
            left_ = 0;  // something else moved or stopped the car: no impact, and whatever was being held is stale
        } else {
            const float dv_lat = dvx * right[0] + dvy * right[1] + dvz * right[2];
            const float rate = roll_rate_for(s_, dv_lat);
            // the car rolls toward the side it hit: the velocity change points away from it
            const float signed_rate = dv_lat > 0.0f ? -rate : rate;
            if (rate > 0.0f && (left_ == 0 || std::fabs(signed_rate) >= std::fabs(target_) * static_cast<float>(left_) / kDriveSteps)) {
                target_ = signed_rate;
                left_ = kDriveSteps;
            }
        }
    }
    if (left_ > 0) {
        out = target_ * static_cast<float>(left_) / kDriveSteps;
        --left_;
    }
    for (int i = 0; i < 3; ++i) {
        prev_vel_[i] = vel[i];
        prev_pos_[i] = pos[i];
    }
    have_prev_ = true;
    return out;
}

float apply_rate(float current, float held, float cap) {
    if (!std::isfinite(current) || !std::isfinite(held) || held == 0.0f) return current;
    const float want = std::fmax(-cap, std::fmin(cap, held));  // never above the cap on our account
    // already rolling the same way at least as fast: leave it
    if (current * want > 0.0f && std::fabs(current) >= std::fabs(want)) return current;
    return want;
}

}  // namespace rollover
