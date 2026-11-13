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
    bool invert_outside_chase = true;  // GT7: outside the chase view (bumper/cockpit/hood) the stick turns the other way
    bool invert_in_chase = true;       // the far chase view mirrored too (user, 2026-11-08)
};

// The settings with both invert flags flipped when `toggled` (the cheat device's `invert` action: stick mirroring on / off at run time).
Settings with_invert_toggle(Settings s, bool toggled);

// A missing key keeps the default silently; an invalid value keeps the default and calls `warn` once for that key.
Settings parse_settings(const std::function<const char *(const char *)> &get,
                        const std::function<void(const std::string &)> &warn);

struct Raw {
    float x = 0.0f, y = 0.0f;  // right stick, -1..1; y > 0 = pulled back (SDL: down)
    bool r3 = false;           // stick clicked
    bool left = false, right = false, back = false;  // keyboard
};

// Target yaw in degrees: 0 = no look. Keys: left -90, right +90 (both cancel), back 180 (wins). R3: 180. Otherwise, outside the
// dead zone, the stick's angle (atan2(x, -y): up 0, right 90, down 180). Pulled back past `back_threshold` the angle blends
// linearly towards the rear view (+-180, the side of x; w = (y - threshold) / (1 - threshold)), so there is no snap: 180 only at
// full back. Back-left and back-right are continuous with the rear view.
float target_yaw(const Raw &raw, const Settings &s);

// The yaw applied in the current view: mirrored (-target) in the far chase view when `invert_in_chase` is set and in the other views
// when `invert_outside_chase` is set (the user's GT7 behaviour: stick right turns the other way), otherwise `target`. 0 stays 0, 180
// stays the rear view.
float view_yaw(float target, bool chase_view, const Settings &s);

// The full third-person (far chase) view, told by the player car's distance from the eye (`pivot`, view space): ~9.2 there,
// <= ~5.3 in the near chase, cockpit, bumper and race-intro cameras (all measured; the active camera object is the same for the
// four driving views, so it cannot tell them apart). Hysteresis: enters above 7.0, leaves below 6.0 (`was_chase` = last answer).
bool is_full_chase_view(const float pivot[3], bool was_chase);

// "Is the player's view live": physics steps keep arriving. The mod's world-matrix hook runs for every object of every frame; a
// frame is counted by its orthographic (HUD) upload, which menus and pauses draw too. A step resets the count; the view is live
// until `max_frames` frames passed without a step (spec: 10), so the test is game-paced, not wall-clock (a slow machine at a few
// fps keeps the orbit).
class StepGate {
  public:
    explicit StepGate(int max_frames = 10) : max_frames_(max_frames) {}
    void on_step() { seen_ = true; frames_ = 0; }
    void on_frame() { if (frames_ <= max_frames_) ++frames_; }
    bool live() const { return seen_ && frames_ <= max_frames_; }

  private:
    int max_frames_;
    int frames_ = 0;
    bool seen_ = false;
};

// Nominal game time of one physics step, the smoothing's time base (game frames, not wall time).
constexpr float kStepMs = 1000.0f / 60.0f;

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
