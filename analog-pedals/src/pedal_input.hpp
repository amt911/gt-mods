// Pure input mapping for the analog-pedals mod: no SDL, no game access.
#pragma once

#include <cstdint>
#include <optional>
#include <utility>

namespace pedals {

struct PedalSettings {
    float deadzone = 0.05f;
    float curve = 1.0f;
    int ramp_ms = 100;
};

// Raw SDL trigger 0..32767 -> 0..1. Below `deadzone` -> 0, then rescaled to 0..1 and raised to `curve`.
float trigger_to_pedal(std::int16_t raw, const PedalSettings &s);

// Keyboard key as a pedal: moves towards 1 (held) or 0 at elapsed_ms / ramp_ms per step.
class KeyRamp {
  public:
    float step(bool held, int elapsed_ms, int ramp_ms);

  private:
    float value_ = 0.0f;
};

float combine(float pad, float keys);

// "0.5,0.3" -> {throttle, brake}; malformed or outside 0..1 -> nullopt.
std::optional<std::pair<float, float>> parse_fake(const char *env);

} // namespace pedals
