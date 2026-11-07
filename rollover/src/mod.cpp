// rollover: lets the player's car roll over (gran-turismo-decomp mod SDK, API 1).
//
// Why a stock GT PSP car cannot roll (docs/FACTS.md of the host repo, "rollover"): 0x0893E0FC clamps the car's two tilt angles (K+0x6FC,
// K+0x700) to +-0.611 rad (35 deg) and zeroes their rates unless the word at 0x08CF2F08 is non-zero; 0x0894CCB8 reads that word (its
// setter has no caller in the binary: a dormant switch). The suspension springs also pull a tilted car back, so a car needs (1) the flag
// and (2) a roll impulse to go over.
//
// (1) The flag is not written: the mod hooks the reader 0x0894CCB8 (six callers, all through the function table) and returns 1 while
// the mod is active. A mod switched off in the F9 menu never reaches its hooks, so the game reads the real word, 0, again: nothing to
// restore. One caller gets 0 instead: 0x08956930, which with the flag set starts a dormant "flipped car" state machine (byte +0x74B bits
// 1|2, countdown +0x74C = 120). It skips the contact/force pipeline while the car is on its side: the car then free-falls through the
// road and is pushed back up ~6 m when the machine lets go (measured, docs/FACTS.md). Without it the normal pipeline keeps the car on
// the road through the whole roll.
// (2) A pre-step hook on 0x0894DD9C (a0 = world; same player-car pick as reset-to-track and orbit-camera) watches the player car's
// world velocity between physics steps. K = word(car+8) is Z-up: velocity +0x6CC/6D0/6D4, forward axis = first basis row +0x1C/20/24,
// lateral axis = second row +0x28/2C/30, position +0x10/14/18, body roll rate +0x6F4 (positive tips the car toward the lateral axis). A lateral velocity change in ONE step above `threshold` is a hard side impact: the car
// is made to roll toward the side it hit with a rate `gain` * excess (at most `cap`), held for 40 steps with a linear decay (a one-step
// kick is undone by the springs: a 6 rad/s kick peaked at 0.34 rad). Normal driving changes the lateral velocity by ~0.1-0.2 m/s per
// step: nothing is written. A jump of the car (> 3 m in a step: reset-to-track, a respawn) or of its velocity (> 60 m/s) is not an
// impact and drops what was being held.
//
// GT_MOD_ROLLOVER_FAKE=<step>:<dv> adds <dv> m/s of lateral velocity change in the tracker's view at that player-car step (used by
// mods_together_check); GT_MOD_ROLLOVER_LOG=1 logs the car's state (see log_state).
#include "gt_mod/gt_mod_api.h"

#include "rollover_logic.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

namespace {

constexpr std::uint32_t kStep = 0x0894DD9C;       // a0 = world
constexpr std::uint32_t kFlagReader = 0x0894CCB8;  // returns the word at 0x08CF2F08
constexpr std::uint32_t kPlayerVtable = 0x08CC1F60;
constexpr std::uint32_t kEntryReplay = 0xA8, kEntryInput = 0x6C;
constexpr std::uint32_t kSlotSize = 0x2100, kCarInSlot = 0x2010;
constexpr std::uint32_t kBody = 8;
constexpr std::uint32_t kPos = 0x10, kForward = 0x1C, kLateral = 0x28, kUp = 0x34, kVel = 0x6CC, kRollRate = 0x6F4, kTiltA = 0x6FC, kTiltB = 0x700;
// Measured (docs/FACTS.md, rollover): a positive +0x6F4 tips the up vector toward the +0x28 row (the lateral axis); +0x1C is the forward axis.
constexpr float kRollSign = 1.0f;
// The flag's caller that starts the dormant flip state machine: ra after its `jal 0x0894CCB8` (0x08956950 + 8). That machine skips the
// car's contact/force pipeline while the car is on its side, so the car sinks through the road and pops back up (FACTS): it keeps getting 0.
constexpr std::uint32_t kFlipMachineRa = 0x08956958;
// With `--decomp on` the call site is C++ and ra is not the guest return address, so the flip machine is also caught by hooking its
// function (0x08956930): while it runs, its flag reads (it is the only reader that starts the machine) get 0.
constexpr std::uint32_t kFlipMachine = 0x08956930;

const GtModApi *g_api = nullptr;
GtMod *g_self = nullptr;
rollover::Settings g_settings;
std::unique_ptr<rollover::Tracker> g_tracker;
bool g_log = false;
std::uint32_t g_car = 0, g_car_steps = 0, g_impulses = 0, g_since_impulse = 0;
long g_fake_step = -1;
float g_fake_dv = 0.0f;
float g_fake_offset[3] = {0, 0, 0};
std::uint32_t g_ras[16], g_ra_count = 0;
bool g_in_flip_machine = false;
unsigned g_flip_hook_hits = 0;
float g_prev_vel[3] = {0, 0, 0};
bool g_have_prev_vel = false;

std::uint32_t find_player_car(GtModCtx *c, std::uint32_t world) {
    const GtModApi &a = *g_api;
    const std::uint32_t slots = a.read32(c, world + 4), controller = a.read32(c, world + 0x10);
    if (slots == 0 || controller == 0) return 0;
    const std::uint32_t list = a.read32(c, controller + 0xC);
    if (list == 0) return 0;
    const int count = static_cast<int>(a.read32(c, world + 8));
    std::uint32_t found = 0;
    int humans = 0;
    for (int i = 0; i < count && i < 32; ++i) {
        const int ix = static_cast<std::int8_t>(a.read8(c, list + 0x24 + i));
        if (ix < 0) continue;
        const std::uint32_t entry = a.read32(c, list + 4 + (2 * i + ix) * 4);
        if (entry == 0) continue;
        if (a.read32(c, entry) == kPlayerVtable && a.read8(c, entry + kEntryReplay) == 0 && a.read32(c, entry + kEntryInput) != 0) {
            ++humans;
            found = slots + i * kSlotSize + kCarInSlot;
        }
    }
    return humans == 1 ? found : 0;
}

// `[mod:rollover] st=<player-car step> imp=<impulses so far> since=<steps since the last, -1 none> pos vel up (the body's up vector) tilt=<+0x6FC,+0x700>
// rate=<+0x6F4> flag=<+0x74B>/<+0x74C> held=<steps left of the held roll rate>`
void log_state(GtModCtx *c, std::uint32_t body, float held) {
    const GtModApi &a = *g_api;
    char line[420];
    std::snprintf(line, sizeof line,
                  "st=%u imp=%u since=%d pos=%.2f,%.2f,%.2f vel=%.2f,%.2f,%.2f fwd=%.2f,%.2f,%.2f lat=%.2f,%.2f,%.2f up=%.3f,%.3f,%.3f tilt=%.3f,%.3f rate=%.2f flag=%02X/%d held=%d(%.2f) flipmachine_calls=%u",
                  g_car_steps, g_impulses, g_impulses == 0 ? -1 : static_cast<int>(g_since_impulse), a.readf(c, body + kPos), a.readf(c, body + kPos + 4),
                  a.readf(c, body + kPos + 8), a.readf(c, body + kVel), a.readf(c, body + kVel + 4), a.readf(c, body + kVel + 8), a.readf(c, body + kForward),
                  a.readf(c, body + kForward + 4), a.readf(c, body + kForward + 8), a.readf(c, body + kLateral), a.readf(c, body + kLateral + 4), a.readf(c, body + kLateral + 8), a.readf(c, body + kUp), a.readf(c, body + kUp + 4), a.readf(c, body + kUp + 8),
                  a.readf(c, body + kTiltA), a.readf(c, body + kTiltB), a.readf(c, body + kRollRate), a.read8(c, body + 0x74B),
                  static_cast<int>(static_cast<std::int16_t>(a.read16(c, body + 0x74C))), g_tracker->steps_left(), held, g_flip_hook_hits);
    a.log(g_self, line);
}

// The flag the game reads: 1 while the mod is active (the host bypasses this hook when the mod is off).
void on_flip_machine(GtModCtx *c) {
    ++g_flip_hook_hits;
    g_in_flip_machine = true;
    g_api->call_original(c);
    g_in_flip_machine = false;
}

void on_flag_read(GtModCtx *c) {
    const std::uint32_t ra = g_api->gpr(c, 31);
    g_api->call_original(c);
    if (g_log) {  // one line per distinct caller (ra), so the check can show which readers the game has and that the flip machine's got 0
        bool seen = false;
        for (std::uint32_t i = 0; i < g_ra_count; ++i) seen = seen || g_ras[i] == ra;
        if (!seen && g_ra_count < 16) {
            g_ras[g_ra_count++] = ra;
            char line[80];
            std::snprintf(line, sizeof line, "FLAGREAD ra=%08X", ra);
            g_api->log(g_self, line);
        }
    }
    g_api->set_gpr(c, 2, (ra == kFlipMachineRa || g_in_flip_machine) ? 0 : 1);
}

void on_step(GtModCtx *c) {
    const GtModApi &a = *g_api;
    const std::uint32_t car = find_player_car(c, a.gpr(c, 4));
    const std::uint32_t body = car != 0 ? a.read32(c, car + kBody) : 0;
    if (car == 0 || body == 0) {
        if (g_car != 0) g_tracker->reset();
        g_car = 0;
        g_car_steps = 0;
        g_have_prev_vel = false;
        a.call_original(c);
        return;
    }
    if (car != g_car) {  // a new race: the car's address changed
        g_tracker->reset();
        g_car = car;
        g_car_steps = 0;
        g_impulses = 0;
        g_have_prev_vel = false;
    }
    ++g_car_steps;
    ++g_since_impulse;
    float vel[3], right[3], fwd[3], pos[3];
    for (int i = 0; i < 3; ++i) {
        vel[i] = a.readf(c, body + kVel + 4 * i);
        right[i] = a.readf(c, body + kLateral + 4 * i);
        fwd[i] = a.readf(c, body + kForward + 4 * i);
        pos[i] = a.readf(c, body + kPos + 4 * i);
    }
    if (g_fake_step >= 0 && static_cast<long>(g_car_steps) == g_fake_step)  // the tracker sees a velocity jump that stays (a constant offset: no second jump)
        for (int i = 0; i < 3; ++i) g_fake_offset[i] = right[i] * g_fake_dv;
    for (int i = 0; i < 3; ++i) vel[i] += g_fake_offset[i];
    if (g_log && g_have_prev_vel) {  // diagnostics for tuning: every step whose lateral or forward velocity change is noticeable
        float dl = 0.0f, df = 0.0f;
        for (int i = 0; i < 3; ++i) {
            dl += (vel[i] - g_prev_vel[i]) * right[i];
            df += (vel[i] - g_prev_vel[i]) * fwd[i];
        }
        if (std::fabs(dl) > 0.8f || std::fabs(df) > 2.0f) {
            char line[140];
            std::snprintf(line, sizeof line, "DV st=%u lat=%.2f fwd=%.2f", g_car_steps, dl, df);
            a.log(g_self, line);
        }
    }
    for (int i = 0; i < 3; ++i) g_prev_vel[i] = vel[i];
    g_have_prev_vel = true;
    const bool was_idle = g_tracker->steps_left() == 0;
    float held = g_tracker->step(vel, right, pos);
    if (held != 0.0f && was_idle) {  // a hit started a drive
        ++g_impulses;
        g_since_impulse = 0;
        if (g_log) {
            char line[160];
            std::snprintf(line, sizeof line, "IMPULSE st=%u held=%.2f", g_car_steps, held);
            a.log(g_self, line);
        }
    }
    if (g_settings.sabotage == 2) held = 0.0f;  // sabotage (the check must fail): the flag without any impulse
    if (held != 0.0f) {
        const float rate = a.readf(c, body + kRollRate);
        const float want = rollover::apply_rate(rate, held * kRollSign, g_settings.cap);
        if (want != rate) a.writef(c, body + kRollRate, want);
    }
    if (g_log && (g_car_steps % 30 == 0 || (g_impulses != 0 && g_since_impulse < 150 && g_car_steps % 3 == 0))) log_state(c, body, held);
    a.call_original(c);
}

}  // namespace

extern "C" __attribute__((visibility("default"))) int gt_mod_init(const GtModApi *api, GtMod *self) {
    if (api == nullptr || api->version != GT_MOD_API_VERSION) return 1;
    g_api = api;
    g_self = self;
    g_settings = rollover::parse_settings([&](const char *k) { return api->config(self, k); }, [&](const std::string &w) { api->log(self, w.c_str()); });
    g_tracker = std::make_unique<rollover::Tracker>(g_settings);
    if (const char *log = std::getenv("GT_MOD_ROLLOVER_LOG")) g_log = *log != '\0' && std::string(log) != "0";
    if (const char *fake = std::getenv("GT_MOD_ROLLOVER_FAKE")) {
        float dv = 0.0f;
        long step = 0;
        if (std::sscanf(fake, "%ld:%f", &step, &dv) == 2 && step >= 0 && std::isfinite(dv)) {
            g_fake_step = step;
            g_fake_dv = dv;
        }
    }
    if (api->hook(self, kFlipMachine, on_flip_machine) != 0) return 1;
    if (g_settings.sabotage != 1 && api->hook(self, kFlagReader, on_flag_read) != 0) return 1;  // sabotage 1: the impulse without the flag
    return api->hook(self, kStep, on_step) != 0 ? 1 : 0;
}
