#include "damage_logic.hpp"

#include <algorithm>
#include <cmath>

namespace damage {

namespace {
float clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

float hit_strength(float dv, const Params &p) {
    if (!(dv > p.threshold)) return 0.0f;  // also false for NaN
    return clamp((dv - p.threshold) * p.per_ms, 0.0f, 1.0f);
}

void add_hit(State &s, const float contact[3], float strength) {
    if (!(strength > 0.0f)) return;
    const float front = std::max(0.0f, contact[0]);
    const float rear = std::max(0.0f, -contact[0]);
    const float side = std::fabs(contact[1]);
    s.engine = clamp(s.engine + strength * front, 0.0f, 1.0f);
    s.drive = clamp(s.drive + strength * rear, 0.0f, 1.0f);
    const float pull = strength * (0.5f * front + side);
    s.align = clamp(s.align + (contact[1] < 0.0f ? -pull : pull), -1.0f, 1.0f);
}

float powertrain(const State &s) { return clamp(s.engine + s.drive, 0.0f, 1.0f); }

float throttle_scale(const State &s, const Params &p) { return clamp(1.0f - clamp(p.scale, 0.0f, 1.0f) * p.engine_loss * powertrain(s), 0.0f, 1.0f); }

float yaw_bias(const State &s, const Params &p) { return p.scale * p.yaw_bias * s.align; }

bool speed_cap(const State &s, const Params &p, float *cap) {
    const float pt = powertrain(s);
    if (!(p.scale > 0.0f) || !(pt > p.severe_start)) return false;
    const float t = clamp((pt - p.severe_start) / (1.0f - p.severe_start), 0.0f, 1.0f);
    *cap = p.cap_high + (p.cap_low - p.cap_high) * t;
    return true;
}

int percent(float v) { return static_cast<int>(std::lround(clamp(std::fabs(v), 0.0f, 1.0f) * 100.0f)); }

void repair(State &s) { s = State{}; }

}  // namespace damage
