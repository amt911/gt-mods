// Input mapping of the orbit-camera mod: stick / keys -> target yaw, smoothing, settings. No SDL, no game access.
#pragma once

#include <functional>
#include <optional>
#include <string>
#include <utility>

namespace orbit {

struct Settings {
    float deadzone = 0.25f;        // stick magnitude below it = no look
    float back_threshold = 0.7f;   // stick pulled back (y, down positive) at least this far = rear view
    float smooth_ms = 80.0f;       // time constant of the swing; 0 = instant
    float pitch_deg = 0.0f;        // constant tilt added while orbiting, -45..45
};

// A missing key keeps the default silently; an invalid value keeps the default and calls `warn` once for that key.
Settings parse_settings(const std::function<const char *(const char *)> &get,
                        const std::function<void(const std::string &)> &warn);

struct Raw {
    float x = 0.0f, y = 0.0f;  // right stick, -1..1; y > 0 = pulled back (SDL: down)
    bool r3 = false;           // stick clicked
    bool left = false, right = false, back = false;  // keyboard
};

// Target yaw in degrees: 0 = no look. Keys: left -90, right +90 (both cancel), back 180 (wins). R3 or the stick pulled back
// past `back_threshold`: 180. Otherwise the stick's angle (atan2(x, -y): up 0, right 90, down 180) outside the dead zone.
float target_yaw(const Raw &raw, const Settings &s);

// First-order smoothing of the yaw towards a target, the short way round (+-180), exact at the end.
class YawSmoother {
  public:
    void reset(float yaw) { value_ = yaw; }
    float step(float target, float dt_ms, float tau_ms);
    float value() const { return value_; }

  private:
    float value_ = 0.0f;
};

// GT_MOD_CAMERA_FAKE="yaw[,pitch]" (degrees, yaw -360..360, pitch -45..45); malformed -> nullopt.
std::optional<std::pair<float, float>> parse_fake(const char *env);

}  // namespace orbit
