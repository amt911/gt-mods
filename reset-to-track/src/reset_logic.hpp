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
constexpr unsigned kFakeHeldSteps = 60;  // GT_MOD_RESET_FAKE holds the button this many steps

int steps_for_ms(float ms);  // ms / kStepMs rounded down (0 for less than one step)

struct Settings {
    float hold_ms = 500.0f;      // the button must be held this long (0 = fires on the press)
    float cooldown_ms = 1000.0f; // minimum time between two resets
    float rewind_ms = 2500.0f;   // restore a pose at least this old (the newest such one)
    int sabotage = 0;            // check builds only: 1 = rates not zeroed, 2 = newest pose, off-track included
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
};

// All four wheels on asphalt, upright, moving, flag clear, all finite.
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
    void push(std::int64_t step, const Pose &pose);
    void clear() { items_.clear(); }
    std::size_t size() const { return items_.size(); }
    // The newest pose at least `min_age` steps old at `now`; when there is none, the oldest pose; nullptr when empty.
    const Pose *pick(std::int64_t now, int min_age) const;

  private:
    struct Item {
        std::int64_t step;
        Pose pose;
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

  private:
    Settings s_;
    int rewind_steps_, cooldown_steps_;
    HoldDetector hold_;
    PoseRing ring_;
    std::int64_t now_ = 0;
    std::int64_t last_reset_ = -1;
};

// GT_MOD_RESET_FAKE=<step>: the check holds the button from that step (counted in player-car steps) for kFakeHeldSteps steps.
std::optional<std::uint32_t> parse_fake(const char *env);
bool fake_held(std::uint32_t step, std::uint32_t from);

}  // namespace reset
