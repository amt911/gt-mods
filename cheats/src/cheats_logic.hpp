// The arithmetic of the cheats mod: time scaling of one car's step, the slow-motion setting and its labels. No game access: plain
// values in, plain values out, so the unit test can run it.
#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

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
Kin scale_step(const Kin &pre, const Kin &post, float s);

enum class SlowMo { Off, Half, Quarter };
float factor(SlowMo m);                  // 1, 0.5, 0.25
const char *name(SlowMo m);              // "OFF", "0.5x", "0.25x"
SlowMo cycle(SlowMo m, int direction);   // Right / Enter (+1): OFF -> 0.5x -> 0.25x -> OFF; Left (-1) goes back
bool parse_slowmo(const char *text, SlowMo &out);  // "off", "0.5", "0.25" (and the names); false = unknown

// The factor of one car: the player's follows the slow motion; an AI car's too, or 0 when AI is frozen (it skips its step).
float car_factor(bool is_ai, SlowMo slow, bool freeze_ai);

struct Settings {
    SlowMo slow = SlowMo::Off;
    bool freeze_ai = false;
    long from_step = 0;  // the cheats start at this player-car step (check aid; 0 = at once)
    int sabotage = 0;    // check builds only: 1 = slow motion does nothing, 2 = freeze does nothing
};
// Money (host repo FACTS "Money and cars"): the live balance is the word at 0x08F6AC00 of a script value whose high word is 0 and
// +8 word 0x00020100 (new game) or 0x01030000 (after a save or load); the game's cap is 999,999,999. Enter/Right add 1,000,000, Left takes it away, clamped to [0, cap].
constexpr std::uint32_t kMoneyCell = 0x08F6AC00u;
constexpr std::uint32_t kMoneyCap = 999999999u;
bool money_cell_valid(std::uint32_t value, std::uint32_t high, std::uint32_t type);
std::uint32_t money_after(std::uint32_t value, int direction);

// A copy of `savedata/UCES01245-GAMEDAT` into `savedata-backups/UCES01245-GAMEDAT-<stamp>` next to the savedata folder (not
// inside it: the game lists that folder). `where` gets the copy's path, or stays empty when there is no save yet (true: nothing
// to lose). False + `error` when the copy fails.
bool backup_save(const std::filesystem::path &savedata, long long stamp, std::string &where, std::string &error);

Settings parse_settings(const std::function<const char *(const char *)> &get, const std::function<void(const std::string &)> &warn);

}  // namespace cheats
