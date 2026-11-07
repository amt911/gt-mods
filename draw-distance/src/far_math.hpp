// Pure logic of the draw-distance mod: no game access, no SDL.
#pragma once

#include <functional>
#include <string>

namespace drawdist {

constexpr float kDefaultScale = 2.5f;
constexpr float kMinScale = 1.0f;
constexpr float kMaxScale = 6.0f;

struct Settings {
    float far_scale = kDefaultScale;
};

// `far_scale` in [1, 6]: a number outside the range is clamped (warn once), a missing key keeps the default silently, text
// that is not a finite number keeps the default (warn once). `get` returns the mod.toml value or nullptr.
Settings parse_settings(const std::function<const char *(const char *)> &get, const std::function<void(const std::string &)> &warn);

// A scale from the environment (the in-game check's override): same rules as the setting, nullopt-like false when absent/invalid.
bool parse_scale(const char *text, float *out);

// The far plane to give the game for one call: far * scale, clamped scale, or `far` itself when the value is not a usable
// plane (not finite, <= near, i.e. nothing the mod understands) or the scale is the identity (so 1.0 writes nothing).
float scaled_far(float far_plane, float near_plane, float scale);

// Whether the call needs an edit at all (scaled_far(...) != far).
bool needs_edit(float far_plane, float near_plane, float scale);

// Clamp to [kMinScale, kMaxScale]; NaN -> default.
float clamp_scale(float scale);

} // namespace drawdist
