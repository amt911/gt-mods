// Pure input mapping for the analog-pedals mod: no SDL, no game access.
#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <optional>
#include <utility>

namespace pedals {

struct PedalSettings {
    float deadzone = 0.05f;
    float curve = 1.0f;
    int ramp_ms = 100;
};

// deadzone in [0,1), curve > 0, ramp_ms >= 0; a missing key keeps the default silently, an invalid value keeps the
// default and calls `warn` once for that key. `get` returns the mod.toml value or nullptr.
PedalSettings parse_settings(const std::function<const char *(const char *)> &get,
                             const std::function<void(const std::string &)> &warn);

// The game's own response to the analog input record (0x08952B68, docs/FACTS.md of gran-turismo-decomp), so the mod can write the
// record value whose response is the requested pedal. raw = s16 record value 0..4096 (x = raw/4096).
//   accelerator (+8, flag 0x02): y = x^8 (curve object 0x08CF2FDC: type 3, c = 1, linear term 0), pedal = y + (1 - y) * 0.1
//   brake (+0xA, flag 0x04):     pedal = x + (1 - x) * 0.1
// Both jump to at least 0.1 for any non-zero record, so a pedal below 0.1 cannot be produced: below 0.05 the inverse returns 0
// (released), between 0.05 and 0.1 the smallest press (raw 1, pedal 0.1).
float throttle_pedal_for_record(int raw);
float brake_pedal_for_record(int raw);
int throttle_record_for_pedal(float pedal);
int brake_record_for_pedal(float pedal);

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

// The record value to leave in a pedal field: `injected` (0..4096) from the mod, `existing` the game's (a digital press as
// 4096). `trigger_pressed` (the controller trigger is past its dead zone) makes the mod's value authoritative: the host
// also maps a trigger past ~25 % to a digital button, which must not turn a partial press into a full one. An `injected` of 0
// (trigger below the pedal floor) never overrides: the game's press stays.
int resolve_record(int injected, int existing, bool trigger_pressed);

// "0.5,0.3" -> {throttle, brake}; malformed or outside 0..1 -> nullopt.
std::optional<std::pair<float, float>> parse_fake(const char *env);

} // namespace pedals
