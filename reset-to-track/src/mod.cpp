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
// Button: the first SDL controller's Select held (BACK) or a keyboard key (`reset_keys`, default R). GT_MOD_RESET_FAKE=<step> holds
// it from that player-car step for 60 steps instead (the in-game check); GT_MOD_RESET_LOG=1 logs the car's state (see log_state).
#include "gt_mod/gt_mod_api.h"

#include "input_source.hpp"
#include "reset_logic.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>

namespace {

constexpr std::uint32_t kStep = 0x0894DD9C;  // a0 = world
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
    std::snprintf(line, sizeof line, "t=%d step=%u wheels=%d up_z=%.3f spd=%.2f pos=%.2f,%.2f,%.2f yaw=%.3f rates=%.2f,%.2f,%.2f,%.2f,%.2f ring=%zu restores=%u acc=%.2f thr=%.2f brk=%.2f",
                  g_since_restore == 0 ? -1 : static_cast<int>(g_since_restore - 1), g_car_steps, s.wheels_on_asphalt, s.up_z, s.speed,
                  a.readf(c, body + 0x10), a.readf(c, body + 0x14), a.readf(c, body + 0x18), a.readf(c, body + 0x704), a.readf(c, body + 0x6E4),
                  a.readf(c, body + 0x6E8), a.readf(c, body + 0x6F0), a.readf(c, body + 0x6F4), a.readf(c, body + 0x6F8), g_resetter->recorded(), g_restores, a.readf(c, body + 0x530), a.readf(c, body + 0x534), a.readf(c, body + 0x538));
    a.log(g_self, line);
}

void on_step(GtModCtx *c) {
    const GtModApi &a = *g_api;
    const std::uint32_t car = find_player_car(c, a.gpr(c, 4));
    const std::uint32_t body = car != 0 ? a.read32(c, car + kBody) : 0;
    if (car == 0 || body == 0) {
        if (g_car != 0) g_resetter->lose_car();
        g_car = 0;
        g_car_steps = 0;
        g_since_restore = 0;
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
    const bool held = g_fake ? reset::fake_held(g_car_steps, *g_fake) : reset::button_held();
    std::optional<reset::Pose> pose = g_resetter->step(read_sample(c, car, body), read_pose(c, body), held);
    if (pose) {
        restore(c, body, *pose);
        ++g_restores;
        if (g_log) {
            char line[200];
            std::snprintf(line, sizeof line, "RESTORE step=%u pos=%.2f,%.2f,%.2f yaw=%.3f", g_car_steps, pose->v[0], pose->v[1], pose->v[2], pose->v[14]);
            a.log(g_self, line);
        }
    }
    if (pose) g_since_restore = 1;  // t = 0 at the restore step
    else if (g_since_restore != 0) ++g_since_restore;
    a.call_original(c);
    if (g_log) {
        const bool window = g_since_restore != 0 && g_since_restore <= kLogWindow;
        if ((window && g_since_restore % 5 == 1) || (!window && g_car_steps % 30 == 0)) log_state(c, car, body);
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
    const char *keys = api->config(self, "reset_keys");
    reset::configure_keys(keys != nullptr ? keys : "R");
    g_fake = reset::parse_fake(std::getenv("GT_MOD_RESET_FAKE"));
    const char *log = std::getenv("GT_MOD_RESET_LOG");
    g_log = log != nullptr && *log != '\0' && std::string(log) != "0";
    return api->hook(self, kStep, on_step) != 0 ? 1 : 0;
}
