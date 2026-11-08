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
constexpr int kTapReads = 3;          // pad reads a released short Select press is replayed for (the camera button's tap)
constexpr unsigned kFakeHeldSteps = 60;  // GT_MOD_RESET_FAKE holds the button this many steps

constexpr float kAheadMeters = 40.0f;      // default of `ahead_m`: a ring pose is eligible only if this much of the straight line ahead stays on asphalt
                                           // (a car restarted from rest covers ~38 m in 4 s: measured on the reset_multi route)
constexpr float kSideMeters = 1.5f;        // ... along three parallel lines: the centre and this far to each side (a wheel leaves the asphalt
                                           // before the car's centre does: half track 0.72 m)
constexpr float kLevelTolerance = 1.5f;    // ... and the ground there is within this of the car's level (not another road layer)
constexpr float kGroundBelowOrigin = 0.15f; // the car's origin sits about this far above the road

int steps_for_ms(float ms);  // ms / kStepMs rounded down (0 for less than one step)

struct Settings {
    float hold_ms = 500.0f;      // the button must be held this long (0 = fires on the press)
    float cooldown_ms = 1000.0f; // minimum time between two resets
    float ahead_m = kAheadMeters; // look-ahead distance of the rule (8 - 100)
    float rewind_ms = 2500.0f;   // restore a pose at least this old (the newest such one)
    int sabotage = 0;            // check builds only: 1 = rates not zeroed, 2 = newest pose, off-track included, 3 = no rewind (newest good pose),
                                 // 4 = the short press is not replayed, 5 = the look-ahead rule is off (every good pose is eligible)
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

// A point of the road: is it asphalt (the game's road query hit, surface word 0) and how high is the ground there?
struct RoadPoint {
    bool asphalt = false;  // hit and surface word 0
    float ground = 0.0f;   // ground height (Z-up world)
};
using RoadQuery = std::function<bool(float x, float y, RoadPoint *out)>;  // false: the query could not be made

// The look-ahead rule: the straight line from (x, y) along `yaw` (forward = (-sin yaw, cos yaw)), and the two lines kSideMeters to its
// sides, stay on asphalt, at the car's level (`z` = the car origin), for kAheadMeters. Samples every 4 m and at `ahead_m`. A query that fails or returns something that is not asphalt
// at that level makes the pose ineligible (fails safe).
bool ahead_on_asphalt(const RoadQuery &query, float x, float y, float z, float yaw, float ahead_m = kAheadMeters);

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
    // The newest pose at least `min_age` steps old at `now`; when there is none, the oldest pose. Poses older than `max_age` steps
    // do not exist for this call; nullptr when nothing is left.
    const Pose *pick(std::int64_t now, int min_age, int max_age = 1 << 30) const;

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
    std::optional<Pose> step(const CarSample &sample, const Pose &current, bool button_held, std::uint32_t hint0 = 0, std::uint32_t hint1 = 0);
    void lose_car();  // no player car (menu, replay, new race): forget everything
    std::size_t recorded() const { return ring_.size(); }
    void invalidate() { clear_ring(); have_prev_ = false; }  // something moved the car or paused the game for long: the ring is stale

    // Deferred mode (API v2): a good pose does not enter the ring at once; it waits in a queue until the vblank tick has run the
    // look-ahead rule on it (the road query may only be called from the tick). `accept` enters it (ignored when the ring was cleared
    // since); a pose the rule rejects is simply never accepted.
    struct Candidate {
        std::int64_t step;
        std::uint32_t epoch;
        Pose pose;
        std::uint32_t hint0 = 0, hint1 = 0;  // the game's road "hint" words (K+0x690/0x694) at that step: where the road query starts searching
    };
    void set_deferred(bool on) { deferred_ = on; }
    std::vector<Candidate> take_pending();
    void accept(const Candidate &c);

  private:
    Settings s_;
    int rewind_steps_, cooldown_steps_;
    HoldDetector hold_;
    PoseRing ring_;
    bool deferred_ = false;
    std::uint32_t epoch_ = 0;  // bumped whenever the ring is cleared: queued candidates of an older epoch are stale
    std::vector<Candidate> pending_;
    void clear_ring() { ring_.clear(); pending_.clear(); ++epoch_; }
    std::int64_t now_ = 0;
    std::int64_t last_reset_ = -1;
    bool have_prev_ = false;
    float prev_pos_[3] = {0, 0, 0};
};

// Select on the pad, seen by the game only as a tap. The pad-read hook feeds it once per read (one sample of the buffer): `sel` the
// physical Select, `racing` a live player car (steps keep arriving), `now` the player-car step. While racing a held Select is hidden;
// released before `hold_steps` it is replayed as a tap of kTapReads reads (the game's camera button keeps working), released after
// it nothing is replayed (the reset fires, the view does not change). Not racing: Select passes through untouched (menus).
class SelectGate {
  public:
    explicit SelectGate(int hold_steps) : hold_steps_(hold_steps) {}
    bool visible(bool sel, bool racing, std::int64_t now);  // does the game see Select on this read
    bool held() const { return held_; }                    // the physical Select is down while racing (feeds the reset's hold)

  private:
    int hold_steps_;
    bool held_ = false, long_ = false;
    std::int64_t press_ = 0;
    int tap_left_ = 0;
};

// GT_MOD_RESET_FAKE=<step>: the check holds the button from that step (counted in player-car steps) for kFakeHeldSteps steps.
std::optional<std::uint32_t> parse_fake(const char *env);
bool fake_held(std::uint32_t step, std::uint32_t from);

}  // namespace reset
