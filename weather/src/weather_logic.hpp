// Weather and time of day: the pure part (no game access). A grade is what the host's scene_grade takes: a per-channel tint, a brightness and a saturation
// (c' = clamp(mix(luma(c * tint), c * tint, saturation) * brightness), (1, 1, 1, 1, 1) = no grade).
#pragma once

#include <cstdint>

namespace weather {

struct Grade {
    float r = 1.f, g = 1.f, b = 1.f, brightness = 1.f, saturation = 1.f;
};
bool operator==(const Grade &a, const Grade &b);
bool neutral(const Grade &g);  // within 1e-4 of (1, 1, 1, 1, 1)

enum class Preset { Noon = 0, Afternoon = 1, Sunset = 2, Dusk = 3 };
constexpr int kPresetCount = 4;
const char *preset_name(Preset p);
bool parse_preset(const char *text, Preset *out);  // "noon", "afternoon", "sunset", "dusk" (case-insensitive)
Grade preset_grade(Preset p);

Grade mix(const Grade &a, const Grade &b, float t);  // t is clamped to 0..1
// The progressive mode: t in 0..1 walks noon -> afternoon -> sunset -> dusk in equal thirds, linear inside each.
Grade progressive_grade(float t);
float progress(std::uint64_t steps, std::uint64_t total);  // steps / total, clamped to 0..1; total 0 gives 1
// Two grades at once (tints and brightness multiply, saturations multiply).
Grade combine(const Grade &a, const Grade &b);

// Rain: 0 off, 1 light, 2 medium, 3 heavy. The rain also darkens and desaturates the picture.
constexpr int kRainLevels = 4;
Grade rain_grade(int level);
const char *rain_name(int level);

struct Streak {
    float x0, y0, x1, y1;  // the line from (x0, y0) to (x1, y1) in the 480 x 272 overlay space
    std::uint32_t rgba;
};
constexpr int kMaxStreaks = 260;
// The streaks of `frame` (any counter that grows by one per picture), deterministic: the same (level, frame, shift) gives the same streaks. `shift` (overlay pixels) moves them sideways
// as the camera turns (parallax). Returns how many streaks were written to `out` (at most `capacity`); 0 for level 0.
int rain_streaks(int level, std::uint32_t frame, float shift, Streak *out, int capacity);

// Grip reduction (off by default): the lateral part of a step's velocity change is reduced by `loss` (0..1) when it is a driving change (<= max_dv m/s in one step: a hit is left alone).
// `now` / `prev` are the velocity after / before the step, `right` the car's right row. Writes the adjusted velocity to `out`; returns true when it changed.
bool grip_adjust(const float now[3], const float prev[3], const float right[3], float loss, float max_dv, float out[3]);

}  // namespace weather
