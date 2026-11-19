// Mechanical damage (spec docs/superpowers/specs/2026-11-18-mechanical-damage-design.md in gran-turismo-decomp), mod API v5 (+ v4 menu entries), no guest address.
//
// The same impacts the deformation mod reads (`car_impact`, filtered by the same StepFilter) add damage to three parts by where the hit landed (damage_logic):
// front -> engine and steering alignment, rear -> drive, sides -> alignment. Each world step the effects are applied to the human's car (live or in its replay):
// the engine's gain per step is scaled down (bled before the step, like the cheats mod's nitro adds), a small yaw-rate bias pulls the car to one side, and a badly damaged powertrain caps the speed. Damage lasts for the race.
#include "gt_mod/gt_mod_api.h"

#include "damage_logic.hpp"
#include "deform_logic.hpp"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {
const GtModApi *g_api = nullptr;
GtMod *g_self = nullptr;
damage::Params g_params;
damage::State g_state;
deform::StepFilter g_filter;
float g_yaw_gain = 10.0f;  // how strongly the bias is applied per step (calibrated in the game: the yaw rate settles near the bias at full alignment damage)
bool g_log = false;
constexpr float kMaxGain = 0.5f;  // m/s per physics step: more than the engine can add (its own full-throttle gain is ~0.09 at the start, ~0.03 at speed)
float g_last_gain = 0.0f, g_prev_written_speed = 0.0f, g_prev_pedal = 0.0f;
bool g_have_prev = false;

std::uint32_t g_car = 0;
int g_role = 0;
std::uint64_t g_last_vblank = 0;
unsigned g_steps = 0;       // steps of this car since the race (or its replay) began
long g_moved_at = -1;       // step at which the accelerator first passed 0.5 (the car creeps before): the fake hit counts from there (the replay's countdown differs from the live one)
long g_fake_step = -1;      // GT_MOD_DAMAGE_FAKE=m<steps>:<dv>:<f|r|b|l>: a hit of dv m/s that many steps after the car moves, from the front, right, back or left
float g_fake_dv = 0.0f;
char g_fake_dir = 'f';
bool g_fake_done = false;
long g_repair_step = -1;    // GT_MOD_DAMAGE_REPAIR=m<steps>: the menu's repair, that many steps after the accelerator is pressed (the check has no keyboard)
bool g_repair_done = false;

// What the menu thread reads (percentages) and what it asks (repair).
std::atomic<int> g_pct_engine{0}, g_pct_drive{0}, g_pct_align{0};
std::atomic<bool> g_repair{false};

float setting(const char *key, float fallback) {
    const char *text = g_api->config(g_self, key);
    return text != nullptr && *text != '\0' ? static_cast<float>(std::atof(text)) : fallback;
}

void publish() {
    g_pct_engine.store(damage::percent(g_state.engine));
    g_pct_drive.store(damage::percent(g_state.drive));
    g_pct_align.store(damage::percent(g_state.align));
}

void new_race(const char *why) {
    if (g_log && (g_state.engine != 0.0f || g_state.drive != 0.0f || g_state.align != 0.0f)) g_api->log(g_self, "RESET damage cleared");
    g_state = damage::State{};
    g_filter.reset();
    g_steps = 0;
    g_last_gain = 0.0f;
    g_have_prev = false;
    g_moved_at = -1;
    g_fake_done = false;
    g_repair_done = false;
    publish();
    if (g_log) {
        char line[96];
        std::snprintf(line, sizeof line, "RACE %s", why);
        g_api->log(g_self, line);
    }
}

// Check build only (-DDAMAGE_SABOTAGE=1): the mod ignores the replay, as if damage were a live-only effect; the replay check must fail.
#if defined(DAMAGE_SABOTAGE) && DAMAGE_SABOTAGE == 1
constexpr bool skip_replay() { return true; }
#else
constexpr bool skip_replay() { return false; }
#endif

// The human's car of this world: live (role 1) or replayed (role 3). Exactly one, or none.
bool find_human(GtModCtx *ctx, std::uint32_t world, GtCar *out) {
    GtCar cars[32];
    const std::uint32_t n = g_api->race_cars(g_self, ctx, world, cars, 32);
    int found = 0;
    for (std::uint32_t i = 0; i < n && i < 32; ++i)
        if (cars[i].role == 1 || (cars[i].role == 3 && !skip_replay())) {
            *out = cars[i];
            ++found;
        }
    return found == 1;
}

void on_step(GtModCtx *ctx) {
    const std::uint32_t world = g_api->gpr(ctx, 4);
    GtCar human;
    if (!find_human(ctx, world, &human)) {
        g_api->call_original(ctx);
        return;
    }
    const std::uint64_t vblank = g_api->vblank_index(g_self);
    if (human.handle != g_car || human.role != g_role || (g_last_vblank != 0 && vblank > g_last_vblank + 120u)) {
        new_race(human.role == 3 ? "replay" : "live");
        g_car = human.handle;
        g_role = human.role;
    }
    g_last_vblank = vblank;
    if (g_repair.exchange(false)) {
        damage::repair(g_state);
        publish();
        if (g_log) g_api->log(g_self, "REPAIR");
    }
    GtCarState before;
    if (g_api->car_state(g_self, ctx, human.handle, &before) != 0) {
        g_api->call_original(ctx);
        return;
    }
    const float pedal_before = before.throttle;
    // Engine damage. The pedal float cannot be used (measured: a scaled +0x530 written before the step changes nothing; the drive chain reads the effective throttle it derives itself),
    // so the engine's contribution is bled the way the cheats mod's nitro adds to it: BEFORE the step the speed loses (1 - scale) x the gain the engine gave in the last step
    // (measured from the step's own start, so it does not compound), and the step then adds its gain on top: the net gain is `scale` times the undamaged one.
    // (the speed field is brought up to date at the START of a step, so a step's own gain shows in the NEXT step's first read: measured, before == after inside one hook call)
    {
        const float gain = before.speed - g_prev_written_speed;
        g_last_gain = g_prev_pedal > 0.05f && g_have_prev && gain > 0.0f && gain < kMaxGain ? gain : 0.0f;
    }
    const float scale = damage::throttle_scale(g_state, g_params);
    float written_speed = before.speed;
    if (scale < 1.0f && pedal_before > 0.05f && g_last_gain > 0.0f && before.speed > 0.5f) {
        const float bleed = (1.0f - scale) * g_last_gain;
        const float k = (before.speed - bleed) / before.speed;
        GtCarState w = before;
        w.speed = before.speed - bleed;
        for (float &v : w.velocity) v *= k;
        g_api->car_set_state(g_self, ctx, human.handle, &w);
        written_speed = w.speed;
    }
    g_prev_written_speed = written_speed;
    g_prev_pedal = pedal_before;
    g_have_prev = true;
    g_api->call_original(ctx);
    ++g_steps;
    GtCarState after;
    if (g_api->car_state(g_self, ctx, human.handle, &after) != 0) return;
    if (g_moved_at < 0 && pedal_before > 0.5f) g_moved_at = static_cast<long>(g_steps);
    if (g_repair_step >= 0 && !g_repair_done && g_moved_at >= 0 && static_cast<long>(g_steps) >= g_moved_at + g_repair_step) {
        g_repair_done = true;
        g_repair.store(true);  // applied at the start of the next step, like the menu's
    }
    GtCarImpact impact;
    const bool have_impact = g_api->car_impact(g_self, ctx, world, human.handle, &impact) == 0;
    const bool real_hit = have_impact && g_filter.accept(vblank, after.position, after.speed, impact.dv_magnitude, impact.yaw_rate_jump, impact.off_track != 0) &&
                          impact.dv_magnitude > g_params.threshold;
    float contact[3] = {0, 0, 0}, dv = 0.0f;
    bool hit = false;
    if (real_hit) {
        hit = true;
        dv = impact.dv_magnitude;
        std::memcpy(contact, impact.contact_dir, sizeof contact);
    } else if (g_fake_step >= 0 && !g_fake_done && g_moved_at >= 0 && static_cast<long>(g_steps) == g_moved_at + g_fake_step) {
        hit = true;
        g_fake_done = true;
        dv = g_fake_dv;
        contact[0] = g_fake_dir == 'f' ? 1.0f : (g_fake_dir == 'b' ? -1.0f : 0.0f);
        contact[1] = g_fake_dir == 'r' ? 1.0f : (g_fake_dir == 'l' ? -1.0f : 0.0f);
    }
    if (hit) {
        const float strength = damage::hit_strength(dv, g_params);
        damage::add_hit(g_state, contact, strength);
        publish();
        if (g_log) {
            char line[200];
            std::snprintf(line, sizeof line, "HIT step=%u dv=%.2f contact=(%.2f %.2f %.2f) strength=%.3f -> engine=%.3f drive=%.3f align=%.3f", g_steps, dv, contact[0], contact[1], contact[2],
                          strength, g_state.engine, g_state.drive, g_state.align);
            g_api->log(g_self, line);
        }
    }
    // the effects on the state the step produced
    GtCarState s = after;
    bool changed = false;
    const float bias = damage::yaw_bias(g_state, g_params);
    if (bias != 0.0f) {
        s.rates[4] += bias * g_yaw_gain / 60.0f;
        changed = true;
    }
    float cap = 0.0f;
    if (damage::speed_cap(g_state, g_params, &cap) && s.speed > cap) {
        const float k = cap / s.speed;
        s.speed = cap;
        for (float &v : s.velocity) v *= k;
        changed = true;
    }
    if (changed) g_api->car_set_state(g_self, ctx, human.handle, &s);
    if (g_log)
        std::fprintf(stderr, "[mod:mechanical-damage] S st=%u mv=%ld vb=%llu v=%.3f thr=%.3f/%.3f yaw=%.4f pos=%.2f,%.2f,%.2f role=%d bleed=%.4f gain=%.4f\n", g_steps, g_moved_at, static_cast<unsigned long long>(vblank), after.speed,
                     pedal_before, after.throttle, after.rates[4], after.position[0], after.position[1], after.position[2], human.role, written_speed - before.speed, g_last_gain);
}

// Menu (any thread): atomics only.
void label_fn(GtMod *, char *out, std::uint32_t capacity) {
    std::snprintf(out, capacity, "Damage: engine %d%%  drive %d%%  steering %d%% (Enter: repair)", g_pct_engine.load(), g_pct_drive.load(), g_pct_align.load());
}
void activate_fn(GtMod *, int direction) {
    if (direction == 0) g_repair.store(true);
}
void repair_action(GtMod *, GtModCtx *) { g_repair.store(true); }

void parse_fake(const char *text) {
    if (text == nullptr || *text != 'm') return;
    long steps = 0;
    float dv = 0.0f;
    char dir = 'f';
    if (std::sscanf(text + 1, "%ld:%f:%c", &steps, &dv, &dir) >= 2) {
        g_fake_step = steps;
        g_fake_dv = dv;
        g_fake_dir = dir;
    }
}
}  // namespace

extern "C" __attribute__((visibility("default"))) int gt_mod_init(const GtModApi *api, GtMod *self) {
    if (api->version < GT_MOD_API_VERSION_5) return 1;
    g_api = api;
    g_self = self;
    g_params.threshold = setting("threshold", g_params.threshold);
    g_params.per_ms = setting("per_ms", g_params.per_ms);
    g_params.engine_loss = setting("engine_loss", g_params.engine_loss);
    g_params.yaw_bias = setting("yaw_bias", g_params.yaw_bias);
    g_params.severe_start = setting("severe_start", g_params.severe_start);
    g_params.cap_high = setting("cap_high", g_params.cap_high);
    g_params.cap_low = setting("cap_low", g_params.cap_low);
    g_params.scale = setting("effect_scale", g_params.scale);
    g_yaw_gain = setting("yaw_gain", g_yaw_gain);
    g_log = std::getenv("GT_MOD_DAMAGE_LOG") != nullptr;
    parse_fake(std::getenv("GT_MOD_DAMAGE_FAKE"));
    if (const char *r = std::getenv("GT_MOD_DAMAGE_REPAIR")) {
        if (*r == 'm') g_repair_step = std::atol(r + 1);
    }
    if (api->hook_point(self, GT_HOOK_WORLD_STEP, on_step) != 0) return 1;
    if (api->version >= GT_MOD_API_VERSION_4) {
        if (api->add_entry(self, "Car", "damage", label_fn, activate_fn) != 0) return 1;
        if (api->add_action(self, "repair", "Repair damage", "Car", repair_action, "", "") != 0) return 1;
    }
    return 0;
}
