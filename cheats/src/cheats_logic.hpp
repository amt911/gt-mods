// The arithmetic of the cheats mod: time scaling of one car's step, the slow-motion setting and its labels. No game access: plain
// values in, plain values out, so the unit test can run it.
#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <array>
#include <functional>
#include <string>

namespace cheats {

// The kinematic state of one car body that the physics step integrates (offsets from K = word(car+8), docs/FACTS.md of the host
// repo, "Car body state"): position, the three basis rows, speed, velocity, angular rates and the Euler angles.
constexpr int kPos = 3, kRows = 9, kVel = 3, kRates = 5, kEuler = 3, kFloats = kPos + kRows + 1 + kVel + kRates + kEuler;
constexpr unsigned kOffsets[kFloats] = {0x10,  0x14,  0x18,                                                    // position
                                        0x1C,  0x20,  0x24,  0x28,  0x2C, 0x30, 0x34, 0x38, 0x3C,             // basis rows
                                        0x6C4,                                                                 // speed
                                        0x6CC, 0x6D0, 0x6D4,                                                   // velocity
                                        0x6E4, 0x6E8, 0x6F0, 0x6F4, 0x6F8,                                     // angular rates
                                        0x6FC, 0x700, 0x704};                                                  // Euler angles
constexpr int kIxPos = 0, kIxRows = 3, kIxSpeed = 12, kIxVel = 13, kIxRates = 16, kIxEuler = 21;

struct Kin {
    std::array<float, kFloats> v{};
};

// A car that moved farther than this in one step was moved by something else (reset-to-track, a respawn): never scaled.
constexpr float kJumpMeters = 8.0f;

// The state a step of `s` times the time would have produced, from the state before the step and the state after the full one:
// pre + s * (post - pre) field by field (explicit Euler is linear in dt), the Euler angles along the shortest way round (they wrap
// at +-pi), the basis rows re-orthonormalised (a blend of two rotations is not one). s = 1 returns `post` and s = 0 returns `pre`
// bit for bit; a car that jumped (kJumpMeters) returns `post` unchanged, and a field that is not a number before or after the step
// (some cars carry NaN in fields the step never uses) keeps the value the game wrote (post).
constexpr float kMaxScale = 1.5f;  // the largest step factor (extrapolation)
Kin scale_step(const Kin &pre, const Kin &post, float s);

enum class SlowMo { Off, Half, Quarter };
float factor(SlowMo m);                  // 1, 0.5, 0.25
const char *name(SlowMo m);              // "OFF", "0.5x", "0.25x"
SlowMo cycle(SlowMo m, int direction);   // Right / Enter (+1): OFF -> 0.5x -> 0.25x -> OFF; Left (-1) goes back
bool parse_slowmo(const char *text, SlowMo &out);  // "off", "0.5", "0.25" (and the names); false = unknown

// The factor of one car: the player's follows the slow motion; an AI car's too, or 0 when AI is frozen (it skips its step).
float car_factor(bool is_ai, SlowMo slow, bool freeze_ai);

// AI speed: the AI cars' step at a factor of the clock (like slow motion, AI only). Above 1 the step is extrapolated.
enum class AiSpeed { Normal, Half, ThreeQuarter, Fast };
float factor(AiSpeed s);                   // 1, 0.5, 0.75, 1.25
const char *name(AiSpeed s);               // "1x", "0.5x", "0.75x", "1.25x"
AiSpeed cycle(AiSpeed s, int direction);   // Right/Enter: 1x -> 0.5x -> 0.75x -> 1.25x -> 1x; Left back
// The factor of an AI car: 0 while frozen, else the AI speed times the slow motion.
float ai_factor(SlowMo slow, AiSpeed speed, bool freeze_ai);

// Nitro: while the throttle is at least kNitroThrottle, the player's velocity and speed grow by kNitroAccel (m/s^2) along the
// forward basis row, for one step of `dt` seconds.
constexpr float kNitroThrottle = 0.9f;
constexpr float kNitroAccel = 6.0f;
Kin nitro(const Kin &now, float throttle, float dt);

// Moon gravity: of the velocity a step lost along the car's up row (falling), only kMoonKeep is applied; rising is left alone.
constexpr float kMoonKeep = 0.3f;
Kin moon(const Kin &before, const Kin &now);

// Jump: kJumpSpeed m/s added along the car's up row, once.
constexpr float kJumpSpeed = 6.0f;
Kin jump(const Kin &now);

// The cheats in force at one physics step, packed in one byte (all off = 0): bits 0-1 slow motion, 2 freeze AI, 3-4 AI speed, 5 nitro,
// 6 moon gravity, 7 a jump on that step.
struct Controls {
    SlowMo slow = SlowMo::Off;
    bool freeze = false;
    AiSpeed ai = AiSpeed::Normal;
    bool nitro = false, moon = false, jump = false;
};
std::uint8_t pack(const Controls &c);
Controls unpack(std::uint8_t b);

// The replay re-simulates the race from the recorded pad stream, so the cheats must act there as they did live. The live race records one
// byte a step, indexed by the steps since the player's car first moved (a replay counts its steps from another point of the start countdown;
// the first movement is the same moment in both). Before that index the first byte applies, after the end of the recording nothing (0).
class Tape {
  public:
    void clear() { bytes_.clear(); }
    bool empty() const { return bytes_.empty(); }
    std::size_t size() const { return bytes_.size(); }
    void record(long step, std::uint8_t b);  // steps skipped since the last record keep the last byte; step < 0 is ignored
    std::uint8_t at(long step) const;

  private:
    std::vector<std::uint8_t> bytes_;
};

struct Settings {
    SlowMo slow = SlowMo::Off;
    bool freeze_ai = false;
    long from_step = 0;  // the cheats start at this player-car step (check aid; 0 = at once)
    int sabotage = 0;    // check builds only: 1 = slow motion does nothing, 2 = freeze does nothing, 3 = nothing recorded (the replay runs without cheats)
};
// Money: Enter/Right add 1,000,000, Left takes it away, clamped to [0, kMoneyCap]; the cell itself is the host's (API v3 money_get/set).
constexpr std::uint32_t kMoneyCap = 999999999u;
std::uint32_t money_after(std::uint32_t value, int direction);

// A copy of `savedata/UCES01245-GAMEDAT` into `savedata-backups/UCES01245-GAMEDAT-<stamp>` next to the savedata folder (not
// inside it: the game lists that folder). `where` gets the copy's path, or stays empty when there is no save yet (true: nothing
// to lose). False + `error` when the copy fails.
bool backup_save(const std::filesystem::path &savedata, long long stamp, std::string &where, std::string &error);

Settings parse_settings(const std::function<const char *(const char *)> &get, const std::function<void(const std::string &)> &warn);

}  // namespace cheats
