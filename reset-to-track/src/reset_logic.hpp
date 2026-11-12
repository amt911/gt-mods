// Pose ring, button and settings logic of the reset-to-track mod. No SDL, no game access: everything is driven by plain values
// so the unit test can run it.
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace reset {

// Nominal game time of one physics step (the game steps at 60 Hz): the time base of every duration setting.
constexpr float kStepMs = 1000.0f / 60.0f;
constexpr int kPoseEveryNSteps = 15;  // one pose into the ring this often (~0.25 s)
constexpr int kRingCapacity = 24;     // ~6 s of history
constexpr float kMinUpZ = 0.9f;       // "upright": the body's up vector z above this
constexpr float kMinSpeed = 5.0f;     // "moving": m/s above this
constexpr float kMaxYawRate = 0.6f;   // rad/s: a faster spin is not a pose to come back to
constexpr float kMinHeadingCos = 0.9f; // velocity . forward / speed: the car must be going where it points (no sliding sideways, no backwards)
constexpr int kMaxPoseAgeSteps = 480; // poses older than this (8 s) are never restored
constexpr float kJumpMeters = 15.0f;  // a car that moved this far in one step was moved by something else: the ring is stale
constexpr float kAheadMeters = 40.0f;      // default of `ahead_m`: a pose is preferred when the car's own path after it stays this long within kPathTolerance of the straight line along its heading
constexpr float kPathTolerance = 2.5f;     // m: lateral distance of the driven path from that line (a car's half width plus margin; the road is ~16 m wide)
constexpr float kMinAheadMeters = 15.0f;   // fallback: with no preferred pose, the one with the longest such run, at least this long (hairpins); below it nothing is restored
constexpr unsigned kFakeHeldSteps = 60;  // GT_MOD_RESET_FAKE holds the button this many steps

int steps_for_ms(float ms);  // ms / kStepMs rounded down (0 for less than one step)

struct Settings {
    float cooldown_ms = 1000.0f; // minimum time between two resets
    float rewind_ms = 2500.0f;   // restore a pose at least this old (the newest such one)
    float ahead_m = kAheadMeters; // length of the path after a pose that must stay on its straight line (8 - 100 m)
    int sabotage = 0;            // check builds only: 1 = rates not zeroed, 2 = newest pose, off-track included, 3 = no rewind (newest good pose), 5 = the straight-ahead rule off (old pick)
};

// A missing key keeps the default silently; an invalid value keeps the default and calls `warn` once for that key.
Settings parse_settings(const std::function<const char *(const char *)> &get,
                        const std::function<void(const std::string &)> &warn);

// The body state the mod saves and restores: position x/y/z, the three basis rows, the three Euler angles (Z-up).
constexpr int kPoseFloats = 15;
struct Pose {
    std::array<float, kPoseFloats> v{};
};

struct CarSample {
    int wheels_on_asphalt = 0;  // of 4 (contact record word +0x2C == 0)
    float up_z = 0.0f;
    float speed = 0.0f;         // m/s
    bool flag_bit2 = false;     // +0x74B bit 2
    float yaw_rate = 0.0f;      // rad/s, body yaw rate (+0x6F8)
    float heading_cos = 1.0f;   // velocity . forward / speed (1 when there is no speed to judge)
};

// All four wheels on asphalt, upright, moving, flag clear, not spinning, going where it points, all finite.
bool is_good(const CarSample &s);

// Fires once when the button has been held for `hold_steps` consecutive steps; must be released before it can fire again.
class HoldDetector {
  public:
    explicit HoldDetector(int hold_steps) : need_(hold_steps) {}
    bool update(bool held);

  private:
    int need_;
    int run_ = 0;
    bool fired_ = false;
};

// Poses with the step they were taken at, newest last, oldest dropped past `capacity`.
class PoseRing {
  public:
    explicit PoseRing(std::size_t capacity) : capacity_(capacity) {}
    void push(std::int64_t step, const Pose &pose, bool pinned = false);  // pinned: a restored pose, always eligible (the car is back there, no path after it yet)
    void clear() { items_.clear(); }
    std::size_t size() const { return items_.size(); }
    // The newest pose at least `min_age` steps old at `now`; when there is none, the oldest pose. Poses older than `max_age` steps
    // do not exist for this call; nullptr when nothing is left.
    const Pose *pick(std::int64_t now, int min_age, int max_age = 1 << 30) const;
    // The straight-ahead rule, from the car's own path (no road data needed: every pose in the ring has 4/4 wheels on asphalt, so the path
    // after a pose is asphalt). `run_after(i)` is how far the path after pose i stays within `tolerance` of the straight line along the pose's
    // heading (forward = (-sin yaw, cos yaw)); a gap in the ring (a step where the car was not in a good state) ends the run. pick_clear takes
    // the newest pose at least `min_age` old whose run is >= `ahead`; failing that, the one with the longest run if it is >= `floor`; else
    // nullptr (the button restores nothing). Poses older than `max_age` do not exist.
    const Pose *pick_clear(std::int64_t now, int min_age, int max_age, float ahead, float floor, float tolerance, float *run_out = nullptr) const;
    float run_after(std::size_t index, float tolerance) const;

  private:
    struct Item {
        std::int64_t step;
        Pose pose;
        bool pinned;
    };
    std::size_t capacity_;
    std::vector<Item> items_;
};

// One call per physics step with the player's car: records good poses, reads the button and says when to restore.
class Resetter {
  public:
    explicit Resetter(const Settings &s);
    // `current` is the car's pose now. Returns the pose to restore, or nothing.
    std::optional<Pose> step(const CarSample &sample, const Pose &current, bool button_held);
    void lose_car();  // no player car (menu, replay, new race): forget everything
    std::size_t recorded() const { return ring_.size(); }
    bool refused() const { return refused_; }  // the last step() call had a button press (past the cooldown) and no pose to restore
    float last_run() const { return last_run_; }  // the straight-ahead run of the last pick, m (-1 none)
    void invalidate() { ring_.clear(); have_prev_ = false; }  // something moved the car or paused the game for long: the ring is stale

  private:
    Settings s_;
    int rewind_steps_, cooldown_steps_;
    HoldDetector hold_;
    PoseRing ring_;
    bool refused_ = false;
    float last_run_ = -1.0f;
    std::int64_t now_ = 0;
    std::int64_t last_reset_ = -1;
    bool have_prev_ = false;
    float prev_pos_[3] = {0, 0, 0};
};

// GT_MOD_RESET_FAKE=<step>: the check holds the button from that step (counted in player-car steps) for kFakeHeldSteps steps.
std::optional<std::uint32_t> parse_fake(const char *env);
bool fake_held(std::uint32_t step, std::uint32_t from);

}  // namespace reset
