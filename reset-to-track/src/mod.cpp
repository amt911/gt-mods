// reset-to-track: one button puts the player's car back on the road (gran-turismo-decomp mod SDK, API 1).
//
// How it works (docs/FACTS.md of the host repo, "reset to track"): 0x0894DD9C(world) steps every car once per physics step. The
// player car is picked with the orbit-camera predicate (driver entry vtable 0x08CC1F60, +0xA8 == 0 live, +0x6C != 0 reads the pad,
// exactly one). Its body state K = word(car+8) is Z-up: position +0x10/14/18, basis rows +0x1C..+0x3C, velocity +0x6CC/6D0/6D4, speed
// +0x6C4, angular rates +0x6E4/6E8 (Euler) and +0x6F0/6F4/6F8 (body), Euler angles +0x6FC/700/704. The wheel contact records
// word(car+0x10) + 0x38*i hold the surface in word +0x2C: 0 on asphalt, 0x0300 off the track.
//
// Pre-step hook: while all four wheels are on asphalt, the car is upright (up.z > 0.9), moving (> 5 m/s) and +0x74B bit 2 is clear,
// a pose goes into a ring every 15 steps. When the button has been held 0.5 s, the newest pose at least 2.5 s old is written back and
// the car's velocity, speed and rates are zeroed (it restarts from rest), then the original step runs. With no button the hook only
// reads. A mod switched off in the F9 menu never reaches this code (the host passes the hook straight to the original).
//
// Straight-ahead rule (0.2.0): the car restarts from rest along the pose's heading, and a pose that points off the road (a diagonal crossing, a
// slide: 4/4 asphalt, upright, calm, aligned with its velocity, but heading at the edge) put it out of bounds again. The ring is searched for the
// newest old-enough pose whose straight line along its heading (and one car width to each side) is followed by the car's OWN path for `ahead_m`
// (every pose in the ring has 4/4 wheels on asphalt, so where the car drove is asphalt); with none, the pose with the longest such run, at least
// 15 m (hairpins); with none, nothing is restored (log line NOPOSE). No road query and no API v2: the game is never called.
//
// Button: pad Select held, or a keyboard key (`reset_keys`, default R). Select is read from the pad buffer the game itself reads: the
// import stub of sceCtrlReadBufferPositive (0x08C7ABF0, a0 = buffer, a1 = count, v0 = samples read; sample +4 = buttons, bit 0 =
// Select) is hooked, so the route's GT_PAD Select and the host's controller mapping both count. While racing a held Select is hidden
// from the game (it would cycle the camera view); a short press is replayed to the game as a tap on release, a hold of `hold_ms`
// or more fires the reset and the view does not change (reset_logic.hpp, SelectGate). GT_MOD_RESET_FAKE=<step> holds the button from
// that player-car step for 60 steps instead (mods_together_check); GT_MOD_RESET_LOG=1 logs the car's state (see log_state).
#include "gt_mod/gt_mod_api.h"

#include "input_source.hpp"
#include "reset_logic.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>

namespace {

constexpr std::uint32_t kStep = 0x0894DD9C;  // a0 = world
constexpr std::uint32_t kPadRead = 0x08C7ABF0;  // import stub sceCtrl::0x1F803938 (sceCtrlReadBufferPositive)
constexpr std::uint32_t kPlayerVtable = 0x08CC1F60;
constexpr std::uint32_t kEntryReplay = 0xA8, kEntryInput = 0x6C;
constexpr std::uint32_t kSlotSize = 0x2100, kCarInSlot = 0x2010;
constexpr std::uint32_t kBody = 8, kContacts = 0x10, kContactSize = 0x38, kContactSurface = 0x2C;
constexpr std::uint32_t kUpZ = 0x3C, kSpeed = 0x6C4, kFlags = 0x74B;
// position, basis rows, Euler angles: the pose that is saved and restored
constexpr std::uint32_t kPoseOffsets[reset::kPoseFloats] = {0x10, 0x14, 0x18, 0x1C, 0x20, 0x24, 0x28, 0x2C, 0x30, 0x34, 0x38, 0x3C, 0x6FC, 0x700, 0x704};
constexpr std::uint32_t kZeroed[] = {0x6C4, 0x6CC, 0x6D0, 0x6D4};                  // speed, velocity
constexpr std::uint32_t kZeroedRates[] = {0x6E4, 0x6E8, 0x6F0, 0x6F4, 0x6F8};       // angular rates
constexpr unsigned kLogWindow = 720;  // steps after a restore that the log follows

const GtModApi *g_api = nullptr;
GtMod *g_self = nullptr;
std::unique_ptr<reset::Resetter> g_resetter;
std::unique_ptr<reset::SelectGate> g_gate;
std::atomic<int> g_reads_since_step{1000};   // pad reads since the last physics step: the game is not racing when it grows
std::atomic<bool> g_select_held{false};      // the pad's Select, hidden from the game while racing
std::atomic<std::uint32_t> g_steps_atomic{0};
std::atomic<unsigned> g_pad_calls{0}, g_pad_ok{0}, g_pad_sel{0};  // diagnostics for the log
std::chrono::steady_clock::time_point g_last_step_wall;
constexpr int kRacingReads = 24;
constexpr double kWallGapSeconds = 2.0;      // no step for this long (the mod was switched off in F9, a long pause): the ring is stale
std::optional<std::uint32_t> g_fake;
bool g_log = false;
std::uint32_t g_car = 0;
unsigned g_car_steps = 0;       // steps of the current player car
unsigned g_since_restore = 0;   // 0 = none yet; else steps since the last restore + 1
unsigned g_restores = 0;
int g_sabotage = 0;

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

reset::CarSample read_sample(GtModCtx *c, std::uint32_t car, std::uint32_t body) {
    const GtModApi &a = *g_api;
    reset::CarSample s;
    const std::uint32_t contacts = a.read32(c, car + kContacts);
    if (contacts != 0)
        for (std::uint32_t i = 0; i < 4; ++i)
            if (a.read32(c, contacts + kContactSize * i + kContactSurface) == 0) ++s.wheels_on_asphalt;
    s.up_z = a.readf(c, body + kUpZ);
    s.speed = a.readf(c, body + kSpeed);
    s.flag_bit2 = (a.read8(c, body + kFlags) & 4) != 0;
    s.yaw_rate = a.readf(c, body + 0x6F8);
    // forward = (-sin yaw, cos yaw) in the Z-up frame (FACTS: velocity follows it); cos of the angle to the horizontal velocity
    const float vx = a.readf(c, body + 0x6CC), vy = a.readf(c, body + 0x6D0), yaw = a.readf(c, body + 0x704);
    const float vh = std::hypot(vx, vy);
    s.heading_cos = vh > 0.5f ? (-vx * std::sin(yaw) + vy * std::cos(yaw)) / vh : 1.0f;
    return s;
}

reset::Pose read_pose(GtModCtx *c, std::uint32_t body) {
    reset::Pose p;
    for (int i = 0; i < reset::kPoseFloats; ++i) p.v[i] = g_api->readf(c, body + kPoseOffsets[i]);
    return p;
}

void restore(GtModCtx *c, std::uint32_t body, const reset::Pose &p) {
    const GtModApi &a = *g_api;
    for (int i = 0; i < reset::kPoseFloats; ++i) a.writef(c, body + kPoseOffsets[i], p.v[i]);
    for (std::uint32_t o : kZeroed) a.writef(c, body + o, 0.0f);
    if (g_sabotage != 1)  // sabotage 1 (the check must fail): the car keeps the spin it had
        for (std::uint32_t o : kZeroedRates) a.writef(c, body + o, 0.0f);
}

// One line per call: `[mod:reset-to-track] t=<steps since restore, -1 before any> step=<n> wheels=<0..4> up_z spd pos yaw rates ring restores acc thr brk`
// (acc = accelerator K+0x530, thr = effective throttle K+0x534, brk = brake K+0x538, FACTS "accelerator and brake").
void log_state(GtModCtx *c, std::uint32_t car, std::uint32_t body) {
    const GtModApi &a = *g_api;
    const reset::CarSample s = read_sample(c, car, body);
    char line[400];
    std::snprintf(line, sizeof line, "t=%d step=%u wheels=%d up_z=%.3f spd=%.2f pos=%.2f,%.2f,%.2f yaw=%.3f rates=%.2f,%.2f,%.2f,%.2f,%.2f vel=%.2f,%.2f,%.2f fwdcos=%.2f ring=%zu restores=%u acc=%.2f thr=%.2f brk=%.2f pad=%u/%u/%u",
                  g_since_restore == 0 ? -1 : static_cast<int>(g_since_restore - 1), g_car_steps, s.wheels_on_asphalt, s.up_z, s.speed,
                  a.readf(c, body + 0x10), a.readf(c, body + 0x14), a.readf(c, body + 0x18), a.readf(c, body + 0x704), a.readf(c, body + 0x6E4),
                  a.readf(c, body + 0x6E8), a.readf(c, body + 0x6F0), a.readf(c, body + 0x6F4), a.readf(c, body + 0x6F8), a.readf(c, body + 0x6CC), a.readf(c, body + 0x6D0), a.readf(c, body + 0x6D4), s.heading_cos, g_resetter->recorded(), g_restores, a.readf(c, body + 0x530), a.readf(c, body + 0x534), a.readf(c, body + 0x538), g_pad_calls.load(), g_pad_ok.load(), g_pad_sel.load());
    a.log(g_self, line);
}

// Pad read: hide Select from the game while racing, replay a short press as a tap, remember whether it is held.
void on_pad_read(GtModCtx *c) {
    const GtModApi &a = *g_api;
    const std::uint32_t buffer = a.gpr(c, 4), count = a.gpr(c, 5);
    a.call_original(c);
    ++g_pad_calls;
    // The host fills the buffer before it can delay the reading thread to the next vblank (v0 is then not the count when the hook
    // sees it), so the samples are the `count` requested ones; only an error code (a bad buffer or count) is skipped.
    const std::uint32_t got = count;
    if (buffer == 0 || count == 0 || got > 64 || a.gpr(c, 2) >= 0x80000000u) return;
    constexpr std::uint32_t kSample = 16, kButtons = 4, kSelect = 1;
    const bool sel = (a.read32(c, buffer + (got - 1) * kSample + kButtons) & kSelect) != 0;
    ++g_pad_ok;
    if (sel) ++g_pad_sel;
    const bool racing = g_car != 0 && g_reads_since_step.fetch_add(1) < kRacingReads;
    const bool was_held = g_gate->held();
    bool show = g_gate->visible(sel, racing, g_steps_atomic.load());
    if (g_sabotage == 4 && !sel) show = false;  // sabotage (the in-game check must fail): the replayed tap is swallowed
    g_select_held = g_gate->held();
    if (g_log && was_held != g_gate->held()) {  // one line per press and release of a hidden Select (the in-game check reads them)
        char line[120];
        if (g_gate->held()) std::snprintf(line, sizeof line, "PADSEL down step=%u", g_steps_atomic.load());
        else std::snprintf(line, sizeof line, "PADSEL up step=%u tap=%d", g_steps_atomic.load(), show ? 1 : 0);
        g_api->log(g_self, line);
    }
    if (show == sel) return;
    for (std::uint32_t i = 0; i < got; ++i) {
        const std::uint32_t at = buffer + i * kSample + kButtons;
        const std::uint32_t v = a.read32(c, at);
        a.write32(c, at, show ? (v | kSelect) : (v & ~kSelect));
    }
}

void step_body(GtModCtx *c, bool *called) {
    const GtModApi &a = *g_api;
    g_reads_since_step = 0;
    const auto now = std::chrono::steady_clock::now();
    if (g_last_step_wall.time_since_epoch().count() != 0 && std::chrono::duration<double>(now - g_last_step_wall).count() > kWallGapSeconds)
        g_resetter->invalidate();
    g_last_step_wall = now;
    const std::uint32_t car = find_player_car(c, a.gpr(c, 4));
    const std::uint32_t body = car != 0 ? a.read32(c, car + kBody) : 0;
    if (car == 0 || body == 0) {
        if (g_car != 0) g_resetter->lose_car();
        g_car = 0;
        g_car_steps = 0;
        g_steps_atomic = 0;
        g_since_restore = 0;
        *called = true;
        a.call_original(c);
        return;
    }
    if (car != g_car) {  // a new race (the car's address changed)
        g_resetter->lose_car();
        g_car = car;
        g_car_steps = 0;
        g_since_restore = 0;
    }
    ++g_car_steps;
    g_steps_atomic = g_car_steps;
    const bool held = g_fake ? reset::fake_held(g_car_steps, *g_fake) : (g_select_held.load() || reset::button_held());
    std::optional<reset::Pose> pose = g_resetter->step(read_sample(c, car, body), read_pose(c, body), held);
    if (pose) {
        restore(c, body, *pose);
        ++g_restores;
        if (g_log) {
            char line[200];
            std::snprintf(line, sizeof line, "RESTORE step=%u pos=%.2f,%.2f,%.2f yaw=%.3f run=%.1f", g_car_steps, pose->v[0], pose->v[1], pose->v[2], pose->v[14], static_cast<double>(g_resetter->last_run()));
            a.log(g_self, line);
        }
    }
    if (g_resetter->refused()) {  // the button was pressed and no pose is eligible (none yet, or none with a straight run of at least 15 m)
        char line[140];
        std::snprintf(line, sizeof line, "NOPOSE step=%u: no pose with %.0f m of straight path in the ring (%zu poses), nothing restored", g_car_steps, static_cast<double>(reset::kMinAheadMeters), g_resetter->recorded());
        a.log(g_self, line);
    }
    if (pose) g_since_restore = 1;  // t = 0 at the restore step
    else if (g_since_restore != 0) ++g_since_restore;
    *called = true;
    a.call_original(c);
    if (g_log) {
        const bool window = g_since_restore != 0 && g_since_restore <= kLogWindow;
        if ((window && g_since_restore % 5 == 1) || (!window && g_car_steps % 30 == 0)) log_state(c, car, body);
    }
}

// A read outside PSP RAM throws (psprecomp::Error: a stale car pointer once a race is over); an exception out of a hook would abort the game, so it
// counts as "no player car" and the original step still runs exactly once.
void on_step(GtModCtx *c) {
    bool called = false;
    try {
        step_body(c, &called);
    } catch (...) {
        g_resetter->lose_car();
        g_car = 0;
        g_car_steps = 0;
        g_steps_atomic = 0;
        g_since_restore = 0;
        if (!called) g_api->call_original(c);
    }
}

}  // namespace

extern "C" __attribute__((visibility("default"))) int gt_mod_init(const GtModApi *api, GtMod *self) {
    if (api == nullptr || api->version != GT_MOD_API_VERSION) return 1;
    g_api = api;
    g_self = self;
    reset::Settings settings = reset::parse_settings([&](const char *k) { return api->config(self, k); },
                                                     [&](const std::string &w) { api->log(self, w.c_str()); });
    g_sabotage = settings.sabotage;
    g_resetter = std::make_unique<reset::Resetter>(settings);
    g_gate = std::make_unique<reset::SelectGate>(reset::steps_for_ms(settings.hold_ms));
    const char *keys = api->config(self, "reset_keys");
    reset::configure_keys(keys != nullptr ? keys : "R");
    g_fake = reset::parse_fake(std::getenv("GT_MOD_RESET_FAKE"));
    const char *log = std::getenv("GT_MOD_RESET_LOG");
    g_log = log != nullptr && *log != '\0' && std::string(log) != "0";
    if (api->hook(self, kStep, on_step) != 0) return 1;
    return api->hook(self, kPadRead, on_pad_read) != 0 ? 1 : 0;
}
