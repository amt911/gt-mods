// Impact detection and roll impulse of the rollover mod. No game access: everything is driven by plain values so the unit test can
// run it.
#pragma once

#include <functional>
#include <string>

namespace rollover {

enum class Style { Realistic, Arcade };

constexpr float kArcadeThresholdScale = 0.5f;  // arcade: the threshold is this fraction of the setting ...
constexpr float kArcadeGainScale = 1.5f;       // ... and the gain this multiple
constexpr int kDriveSteps = 40;                // a hit holds its roll rate (decaying linearly) this many steps: the springs undo a one-step kick
constexpr float kJumpMeters = 3.0f;            // a car that moved this far in one step was moved by something else (a respawn, reset-to-track)
constexpr float kMaxDeltaV = 60.0f;            // a velocity change this big in one step is not an impact (a teleport, a new race)

struct Settings {
    Style style = Style::Realistic;
    float threshold = 3.0f;  // m/s of lateral velocity change in one step
    float gain = 2.5f;       // rad/s of roll rate per m/s above the threshold
    float cap = 12.0f;       // rad/s, largest roll rate requested
    bool body_box = true;    // the body box (bodybox.hpp): the tumbling car's body stops at the ground instead of sinking into it
    float box_height = 0.8f;  // m: the box's roof above the body origin (the game has no body height; a heuristic: roof ~1.2 m over the road)
    float box_kick_cap = 0.5f; // m/s: largest speed away from the ground the car keeps in a step where its body touches it (limits the hop)
    int sabotage = 0;        // check builds only: 1 = no flag (impulse only), 2 = flag only (no impulse), 3 = threshold 0 (impulse in normal driving), 4 = the flag also lifted for the flip state machine, 5 = stock physics (log only: no flag, no impulse written), 6 = the body box off (log still measures)
};

// A missing key keeps the default silently; an invalid value keeps the default and calls `warn` once for that key.
Settings parse_settings(const std::function<const char *(const char *)> &get, const std::function<void(const std::string &)> &warn);

float effective_threshold(const Settings &s);  // style applied
float effective_gain(const Settings &s);

// Roll rate (rad/s, always >= 0) a lateral velocity change of `dv_lat` m/s asks for: 0 up to the threshold, then gain * excess, never
// above the cap. Non-finite input gives 0.
float roll_rate_for(const Settings &s, float dv_lat);

// One call per physics step with the player's car. Detects a hard side impact and says what roll rate to hold.
class Tracker {
  public:
    explicit Tracker(const Settings &s) : s_(s) {}
    // vel: world velocity, right: the car's lateral axis (unit), pos: position. Returns the signed roll rate to hold this step (0 = none).
    // Positive = toward `right`: the car rolls toward the side it hit, i.e. opposite to the velocity change along `right`.
    float step(const float vel[3], const float right[3], const float pos[3]);
    void reset() { have_prev_ = false; left_ = 0; target_ = 0.0f; }  // no player car / a new race
    int steps_left() const { return left_; }

  private:
    Settings s_;
    bool have_prev_ = false;
    float prev_vel_[3] = {0, 0, 0};
    float prev_pos_[3] = {0, 0, 0};
    int left_ = 0;
    float target_ = 0.0f;  // signed, at the start of the drive
};

// The roll rate to write given the car's current one: the held rate when it is larger in its direction, else untouched (a hit never
// slows a roll that is already faster the same way); what is written never exceeds `cap` in magnitude.
float apply_rate(float current, float held, float cap);

}  // namespace rollover
