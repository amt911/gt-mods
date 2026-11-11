// cheats: race-time cheats for Gran Turismo PSP (gran-turismo-decomp mod SDK, API 2): slow motion and freeze AI, as rows of the F9 menu.
//
// How (docs/FACTS.md of the host repo, "Car body state" and "Cheats"): 0x0894DD9C(world) steps every car once per physics step
// (world+4 = slots of 0x2100 bytes, world+8 = car count, car = slot + 0x2010, K = word(car+8) the body state). The step has not run
// yet when the hook's call_original returns (with --decomp off the recompiled unit hands the body to the dispatcher after the hook
// returns; measured with a probe, docs/FACTS.md), so nothing can be read "after the step" there. The hook works on the NEXT call
// instead: at its entry the state of every car is the previous step's full result. For each car the mod keeps `base`, the state it
// left after the previous call, and writes back
//     base + f * (state_now - base)
// (explicit Euler is linear in dt: the same physics at a fraction of the clock; velocities change f times as slowly too, which a
// plain velocity multiplier would not do), over the kinematic fields of cheats_logic.hpp (position, basis rows, speed, velocity,
// angular rates, Euler angles). f = 0.5 / 0.25 for slow motion, 0 for an AI car while AI is frozen (it keeps the speed it had and
// goes on from it when the cheat is turned off). Nothing is written when f = 1 (frames are byte-identical to no mod). The picture
// drawn between two calls shows the car one full step ahead of `base`, a constant lead, so the motion on screen is f times as fast.
// It acts only while exactly one live human car exists (a race, never a replay, never two humans), never on a car that moved more
// than 8 m since the last call (a respawn, reset-to-track), restarts its bases after 8 vblanks without a call (the game's own count, not the wall clock) (the mod was off
// in F9) and leaves a slot whose pointers are unreadable alone.
//
// What is NOT slowed (the step hook cannot reach it): the game's clocks and the AI drivers' own state (timers, lap counters and the
// drivers' controllers run at the normal rate; a frozen AI car's driver keeps running against a car that does not move), the tyre
// and engine state outside the body (wheel spin, rpm), the sound, the camera. Why not step skipping (half of the steps do not run,
// everything slows consistently): the game steps at ~30 Hz, so 0.25x would draw 7 steps per second.
//
// The menu rows ("Slow motion", "Freeze AI") are the mod's API-2 menu rows: Right/Enter step forward, Left backward; the state is
// atomic (the host calls the row functions from other threads). GT_MOD_CHEATS_SLOWMO=<off|0.5|0.25>, GT_MOD_CHEATS_FREEZE_AI=1 and
// GT_MOD_CHEATS_FROM=<player-car step> set the initial state (used by tools/mod_cheats_check.sh); GT_MOD_CHEATS_LOG=1 logs the cars every
// GT_MOD_CHEATS_LOG_EVERY steps (default 10): `st=<step> slow=<OFF|0.5x|0.25x> freeze=<0|1> P=<player pos> spd acc A<i>=<AI pos>...`.
#include "gt_mod/gt_mod_api.h"

#include "cheats_logic.hpp"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <cstdlib>
#include <cstring>
#include <string>

namespace {

constexpr std::uint32_t kStep = 0x0894DD9C;  // a0 = world (a named hook point comes with API v3 step 3)
constexpr int kMaxCars = 8;
constexpr float kStepSeconds = 1.0f / 30.0f;  // the physics step runs at ~30 Hz (FACTS "Cheats")

const GtModApi *g_api = nullptr;
GtMod *g_self = nullptr;
cheats::Settings g_settings;
std::atomic<int> g_slow{0};      // cheats::SlowMo as an int
std::atomic<bool> g_freeze{false};
std::atomic<int> g_ai_speed{0};  // cheats::AiSpeed as an int
std::atomic<bool> g_nitro{false}, g_moon{false};
std::atomic<int> g_jump{0};      // pending jumps (Enter on the row)
unsigned g_jump_at = 0;          // GT_MOD_CHEATS_JUMP_AT=<step> (checks): one jump at that player-car step
bool g_log = false;
unsigned g_log_every = 10;  // GT_MOD_CHEATS_LOG_EVERY: steps between log lines
std::uint32_t g_player = 0, g_steps = 0;  // the player's car, steps seen with it

enum class Role { None, Human, Ai };

struct Car {
    int slot = 0;
    std::uint32_t body = 0;  // the API's car handle
    Role role = Role::None;
    GtCarState state{};      // the whole state at this call's entry (pedals included)
    cheats::Kin now;         // its kinematic part
};

// What the mod left in each slot's car after the previous call: the start of the step that has just run.
struct Base {
    std::uint32_t body = 0;
    bool valid = false;
    cheats::Kin kin;
};
Base g_base[kMaxCars];
std::uint64_t g_last_vblank = 0;     // the game's vblank count at the previous call (GtModApi::vblank_index)
constexpr std::uint64_t kVblankGap = 8;  // calls come every ~2 vblanks; a longer gap means the mod was switched off (or a long pause)

// The kinematic part of the API's car state, in cheats::Kin order (position, basis, speed, velocity, rates, Euler angles).
cheats::Kin kin_of(const GtCarState &s) {
    cheats::Kin k;
    int i = 0;
    for (float v : s.position) k.v[i++] = v;
    for (float v : s.basis) k.v[i++] = v;
    k.v[i++] = s.speed;
    for (float v : s.velocity) k.v[i++] = v;
    for (float v : s.rates) k.v[i++] = v;
    for (float v : s.euler) k.v[i++] = v;
    return k;
}
GtCarState with_kin(GtCarState s, const cheats::Kin &k) {
    int i = 0;
    for (float &v : s.position) v = k.v[i++];
    for (float &v : s.basis) v = k.v[i++];
    s.speed = k.v[i++];
    for (float &v : s.velocity) v = k.v[i++];
    for (float &v : s.rates) v = k.v[i++];
    for (float &v : s.euler) v = k.v[i++];
    return s;
}

// The race through the named SDK (API v3): fills cars[] and returns their count, or 0 when this is not a race with exactly
// one live human.
int read_world(GtModCtx *c, std::uint32_t world, Car *cars, std::uint32_t &player_body) {
    GtCar list[32];
    const std::uint32_t total = g_api->race_cars(g_self, c, world, list, 32);
    int n = 0, humans = 0;
    player_body = 0;
    for (std::uint32_t i = 0; i < total && i < 32 && n < kMaxCars; ++i) {
        if (list[i].index >= static_cast<std::uint32_t>(kMaxCars)) continue;
        Car &car = cars[n];
        if (g_api->car_state(g_self, c, list[i].handle, &car.state) != 0) continue;
        car.slot = static_cast<int>(list[i].index);
        car.body = list[i].handle;
        car.role = list[i].role == 1 ? Role::Human : Role::Ai;
        car.now = kin_of(car.state);
        if (car.role == Role::Human) {
            ++humans;
            player_body = car.body;
        }
        ++n;
    }
    return humans == 1 ? n : 0;
}

void log_cars(const Car *cars, int n, bool active) {  // slow/freeze read OFF/0 until GT_MOD_CHEATS_FROM has passed
    char line[420];
    int len = std::snprintf(line, sizeof line, "st=%u slow=%s freeze=%d", g_steps, active ? cheats::name(static_cast<cheats::SlowMo>(g_slow.load())) : "OFF", active && g_freeze.load() ? 1 : 0);
    for (int i = 0; i < n; ++i) {  // the player first, then the AI cars in slot order
        if (cars[i].role != Role::Human) continue;
        const GtCarState &s = cars[i].state;
        len += std::snprintf(line + len, sizeof line - len, " P=%.3f,%.3f,%.3f spd=%.3f acc=%.2f", s.position[0], s.position[1],
                             s.position[2], s.speed, s.throttle);
    }
    int ai = 0;
    for (int i = 0; i < n && len < static_cast<int>(sizeof line) - 60; ++i) {
        if (cars[i].role != Role::Ai) continue;
        const GtCarState &s = cars[i].state;
        len += std::snprintf(line + len, sizeof line - len, " A%d=%.3f,%.3f,%.3f", ai++, s.position[0], s.position[1], s.position[2]);
    }
    g_api->log(g_self, line);
}

void on_step(GtModCtx *c) {
    Car cars[kMaxCars];
    int n = 0;
    std::uint32_t player_body = 0;
    n = read_world(c, g_api->gpr(c, 4), cars, player_body);  // the host checks every pointer
    if (n == 0) {
        g_player = 0;
        g_steps = 0;
        for (Base &b : g_base) b.valid = false;
        g_api->call_original(c);
        return;
    }
    if (player_body != g_player) {  // a new race (the player's body changed)
        g_player = player_body;
        g_steps = 0;
        for (Base &b : g_base) b.valid = false;
    }
    const std::uint64_t vblank = g_api->vblank_index(g_self);
    if (g_last_vblank != 0 && vblank - g_last_vblank > kVblankGap)
        for (Base &b : g_base) b.valid = false;  // switched off in F9 for a while, or a long pause: the old bases are stale
    g_last_vblank = vblank;
    ++g_steps;
    const auto slow = static_cast<cheats::SlowMo>(g_slow.load());
    const bool freeze = g_freeze.load();
    const auto ai_speed = static_cast<cheats::AiSpeed>(g_ai_speed.load());
    const bool nitro = g_nitro.load(), moon = g_moon.load();
    if (g_jump_at != 0 && g_steps >= g_jump_at) { g_jump_at = 0; g_jump.store(1); }
    const bool jump = g_jump.exchange(0) > 0;
    const bool active = static_cast<long>(g_steps) > g_settings.from_step;
    for (int i = 0; i < n; ++i) {
        Base &base = g_base[cars[i].slot];
        const bool ai = cars[i].role == Role::Ai;
        float f = active ? (ai ? cheats::ai_factor(slow, ai_speed, freeze) : cheats::factor(slow)) : 1.0f;
        if (g_settings.sabotage == 1 && !(ai && freeze)) f = 1.0f;               // sabotage (the check must fail): slow motion does nothing
        if (g_settings.sabotage == 2 && ai && freeze) f = cheats::factor(slow);  // sabotage: freeze does nothing
        const bool fresh = !base.valid || base.body != cars[i].body;
        cheats::Kin out = cars[i].now;
        bool changed = false;
        if (!fresh && f != 1.0f) {  // the step that has just run, at f of the clock (f > 1 extrapolates: a faster AI)
            out = cheats::scale_step(base.kin, cars[i].now, f);
            changed = true;
        }
        if (!ai && active) {  // the player's own cheats
            if (moon && !fresh) { out = cheats::moon(base.kin, out); changed = true; }
            if (nitro && cars[i].state.throttle >= cheats::kNitroThrottle) { out = cheats::nitro(out, cars[i].state.throttle, kStepSeconds); changed = true; }
            if (jump) { out = cheats::jump(out); changed = true; }
        }
        if (changed) {
            const GtCarState next = with_kin(cars[i].state, out);
            g_api->car_set_state(g_self, c, cars[i].body, &next);
            cars[i].state = next;  // the log shows what the mod left, as before the port
        }
        base.valid = true;
        base.body = cars[i].body;
        base.kin = out;
    }
    if (g_log && g_steps % g_log_every == 0) log_cars(cars, n, active);
    g_api->call_original(c);
}

// The vblank tick exercises call_guest with a harmless read-only native: 0x0894CCB8, the 3-instruction reader of the dormant rollover flag word
// 0x08CF2F08 (host repo FACTS "Rollovers": nothing in the binary writes it, so it answers 0 unless the rollover mod's hook says otherwise). The log line
// `CALLGUEST status=<call_guest result> v0=<its v0>` (status 0, v0 0 expected) is what tools/mod_cheats_check.sh judges: call_guest works from the tick
// in a running game, through the same dispatcher as the game's own calls. Nothing else in the mod needs the tick.
constexpr std::uint32_t kFlagReader = 0x0894CCB8;
unsigned g_ticks = 0;
bool g_callguest = false;  // GT_MOD_CHEATS_CALLGUEST=1: verification only: the check's exercise of call_guest from the tick
// Money (cheats_logic.hpp): the row's key presses are queued and applied by the tick, which reads the balance cell every vblank (the
// label shows the cached value). Before the first change of a run the save folder is copied (cheats::backup_save); no copy, no change.
std::atomic<int> g_money_request{0};                    // +1 / -1 queued by the row, 0 none
std::atomic<std::uint32_t> g_money_seen{0xFFFFFFFFu};  // the balance at the last tick, all ones when the cell is not valid
bool g_money_backed_up = false;
bool g_money_backup_failed = false;
std::uint64_t g_money_test_at = 0;  // GT_MOD_CHEATS_MONEY_AT=<vblank>: press the money row (+1M) once at that vblank

void money_tick(GtModCtx *c) {
    std::uint32_t value = 0;
    const bool valid = g_api->money_get(g_self, c, &value) == 0;  // the host checks the cell (API v3)
    g_money_seen.store(valid ? value : 0xFFFFFFFFu);
    if (g_money_test_at != 0 && g_api->vblank_index(g_self) >= g_money_test_at) {  // GT_MOD_CHEATS_MONEY_AT (verification only)
        g_money_test_at = 0;
        g_money_request.store(1);
    }
    if (!valid) return;  // a queued press waits for the cell (loading, a menu without a game)
    const int request = g_money_request.exchange(0);
    if (request == 0) return;
    char line[200];
    if (!g_money_backed_up) {
        const char *dir = std::getenv("GT_SAVEDATA_DIR");
        std::string where, error;
        if (dir == nullptr || !cheats::backup_save(dir, static_cast<long long>(std::time(nullptr)), where, error)) {
            g_money_backup_failed = true;
            std::snprintf(line, sizeof line, "MONEY refused: %s", dir == nullptr ? "no GT_SAVEDATA_DIR" : error.c_str());
            g_api->log(g_self, line);
            return;
        }
        g_money_backed_up = true;
        g_money_backup_failed = false;
        std::snprintf(line, sizeof line, "MONEY save backed up to %s", where.empty() ? "(no save yet)" : where.c_str());
        g_api->log(g_self, line);
    }
    const std::uint32_t next = cheats::money_after(value, request);
    if (g_api->money_set(g_self, c, next) != 0) return;
    g_money_seen.store(next);
    std::snprintf(line, sizeof line, "ROW money -> %u (was %u)", next, value);
    g_api->log(g_self, line);
}

void on_tick(GtModCtx *c) {
    money_tick(c);
    if (!g_callguest || ++g_ticks % 300 != 30) return;
    std::uint32_t v0 = 0xFFFFFFFFu;
    const int status = g_api->call_guest(c, kFlagReader, nullptr, 0, &v0);
    char line[80];
    std::snprintf(line, sizeof line, "CALLGUEST status=%d v0=%u", status, v0);
    g_api->log(g_self, line);
}

void slow_label(GtMod *, char *out, std::uint32_t capacity) {
    std::snprintf(out, capacity, "Slow motion  %s", cheats::name(static_cast<cheats::SlowMo>(g_slow.load())));
}
void slow_activate(GtMod *, int direction) {
    const cheats::SlowMo next = cheats::cycle(static_cast<cheats::SlowMo>(g_slow.load()), direction);
    g_slow.store(static_cast<int>(next));
    char line[80];
    std::snprintf(line, sizeof line, "ROW slow_motion -> %s", cheats::name(next));
    g_api->log(g_self, line);
}
void freeze_label(GtMod *, char *out, std::uint32_t capacity) { std::snprintf(out, capacity, "Freeze AI    %s", g_freeze.load() ? "ON" : "OFF"); }
void freeze_activate(GtMod *, int) {
    const bool next = !g_freeze.load();
    g_freeze.store(next);
    char line[80];
    std::snprintf(line, sizeof line, "ROW freeze_ai -> %s", next ? "ON" : "OFF");
    g_api->log(g_self, line);
}

void money_label(GtMod *, char *out, std::uint32_t capacity) {
    const std::uint32_t seen = g_money_seen.load();
    if (g_money_backup_failed) std::snprintf(out, capacity, "Money        NO BACKUP");
    else if (seen == 0xFFFFFFFFu) std::snprintf(out, capacity, "Money        --");
    else std::snprintf(out, capacity, "Money        %u Cr +-1M", seen);
}
void money_activate(GtMod *, int direction) { g_money_request.store(direction < 0 ? -1 : 1); }

void ai_speed_label(GtMod *, char *out, std::uint32_t capacity) {
    std::snprintf(out, capacity, "AI speed     %s", cheats::name(static_cast<cheats::AiSpeed>(g_ai_speed.load())));
}
void ai_speed_activate(GtMod *, int direction) {
    g_ai_speed.store(static_cast<int>(cheats::cycle(static_cast<cheats::AiSpeed>(g_ai_speed.load()), direction)));
}
void nitro_label(GtMod *, char *out, std::uint32_t capacity) { std::snprintf(out, capacity, "Nitro        %s (full throttle)", g_nitro.load() ? "ON" : "OFF"); }
void nitro_activate(GtMod *, int) { g_nitro.store(!g_nitro.load()); }
void moon_label(GtMod *, char *out, std::uint32_t capacity) { std::snprintf(out, capacity, "Moon gravity %s", g_moon.load() ? "ON" : "OFF"); }
void moon_activate(GtMod *, int) { g_moon.store(!g_moon.load()); }
void jump_label(GtMod *, char *out, std::uint32_t capacity) { std::snprintf(out, capacity, "Jump         ENTER"); }
void jump_activate(GtMod *, int) { g_jump.store(1); }

}  // namespace

extern "C" __attribute__((visibility("default"))) int gt_mod_init(const GtModApi *api, GtMod *self) {
    if (api == nullptr || api->version < GT_MOD_API_VERSION_3) return 1;  // named SDK: money and cars (host repo MODS.md, API v3)
    g_api = api;
    g_self = self;
    g_settings = cheats::parse_settings([&](const char *k) { return api->config(self, k); }, [&](const std::string &w) { api->log(self, w.c_str()); });
    if (const char *e = std::getenv("GT_MOD_CHEATS_SLOWMO")) cheats::parse_slowmo(e, g_settings.slow);
    if (const char *e = std::getenv("GT_MOD_CHEATS_FREEZE_AI")) g_settings.freeze_ai = *e != '\0' && std::strcmp(e, "0") != 0;
    if (const char *e = std::getenv("GT_MOD_CHEATS_FROM")) g_settings.from_step = std::atol(e);
    if (const char *e = std::getenv("GT_MOD_CHEATS_CALLGUEST")) g_callguest = *e != '\0' && std::strcmp(e, "0") != 0;
    if (const char *e = std::getenv("GT_MOD_CHEATS_MONEY_AT")) g_money_test_at = std::strtoull(e, nullptr, 10);
    // check aids (tools/mod_cheats_check.sh): initial states of the new cheats
    if (const char *e = std::getenv("GT_MOD_CHEATS_AI_SPEED")) g_ai_speed.store(std::atoi(e) & 3);
    if (const char *e = std::getenv("GT_MOD_CHEATS_NITRO")) g_nitro.store(*e == '1');
    if (const char *e = std::getenv("GT_MOD_CHEATS_MOON")) g_moon.store(*e == '1');
    if (const char *e = std::getenv("GT_MOD_CHEATS_JUMP_AT")) g_jump_at = std::strtoul(e, nullptr, 10);
    if (const char *e = std::getenv("GT_MOD_CHEATS_LOG")) g_log = *e != '\0' && std::strcmp(e, "0") != 0;
    if (const char *e = std::getenv("GT_MOD_CHEATS_LOG_EVERY"); e != nullptr && std::atoi(e) > 0) g_log_every = static_cast<unsigned>(std::atoi(e));
    g_slow.store(static_cast<int>(g_settings.slow));
    g_freeze.store(g_settings.freeze_ai);
    if (api->add_menu_row(self, "slow_motion", slow_label, slow_activate) != 0) return 1;
    if (api->add_menu_row(self, "freeze_ai", freeze_label, freeze_activate) != 0) return 1;
    if (api->add_menu_row(self, "money", money_label, money_activate) != 0) return 1;
    if (api->add_menu_row(self, "ai_speed", ai_speed_label, ai_speed_activate) != 0) return 1;
    if (api->add_menu_row(self, "nitro", nitro_label, nitro_activate) != 0) return 1;
    if (api->add_menu_row(self, "moon", moon_label, moon_activate) != 0) return 1;
    if (api->add_menu_row(self, "jump", jump_label, jump_activate) != 0) return 1;
    if (api->on_vblank(self, on_tick) != 0) return 1;
    return api->hook(self, kStep, on_step) != 0 ? 1 : 0;
}
