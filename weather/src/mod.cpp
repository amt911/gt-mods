// Weather and time of day (spec docs/superpowers/specs/2026-11-18-weather-mod-design.md in gran-turismo-decomp), mod API v8, no guest address.
//
// Time of day is a colour grade of the whole picture (scene_grade): a preset (noon / afternoon / sunset / dusk) or a progressive mode that walks through them over the race (counted in
// the human car's physics steps). Rain is visual: screen-space streaks drawn with the overlay API (seeded, deterministic, sideways with the car's yaw for parallax) and a darker,
// desaturated grade multiplied into the time of day's. Optional grip reduction (grip_loss, off by default) reduces the lateral part of each step's velocity change.
// Not done (spec): the game's own per-track fog / light tables (phase 2: not researched) and the lower far plane in rain.
// A grade of the whole picture also tints the HUD (FACTS "Scene grade"); in menus (no race) the grade is neutral.
//
// WEATHER_SABOTAGE (build flag, in-game check only): 1 = the grade is computed and logged but never applied, 2 = the rain draws nothing.
#include "gt_mod/gt_mod_api.h"

#include "weather_logic.hpp"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>

#ifndef WEATHER_SABOTAGE
#define WEATHER_SABOTAGE 0
#endif

namespace {
const GtModApi *g_api = nullptr;
GtMod *g_self = nullptr;
bool g_log = false;
float g_grip_loss = 0.f;
std::uint64_t g_progressive_steps = 18000;  // 5 minutes of race at 60 steps per second

constexpr int kProgressive = weather::kPresetCount;  // the menu's fifth mode
std::atomic<int> g_mode{0};   // 0..3 a preset, 4 progressive
std::atomic<int> g_rain{0};

constexpr std::uint64_t kLiveGap = 30, kResetGap = 120;  // vblanks without a step: no race on screen / a new race
std::atomic<std::uint64_t> g_last_step_vblank{0}, g_steps{0};
std::atomic<float> g_yaw_shift{0.f};  // overlay pixels: the car's yaw integrated, for the rain's parallax
std::uint32_t g_car = 0;
float g_prev_velocity[3] = {0, 0, 0};
bool g_have_prev = false;
weather::Grade g_applied;  // what scene_grade was last given (overlay thread)
bool g_applied_valid = false;
unsigned g_logged = 0;

float setting(const char *key, float fallback) {
    const char *text = g_api->config(g_self, key);
    return text != nullptr && *text != '\0' ? static_cast<float>(std::atof(text)) : fallback;
}

bool find_human(GtModCtx *ctx, std::uint32_t world, GtCar *out) {
    GtCar cars[32];
    const std::uint32_t n = g_api->race_cars(g_self, ctx, world, cars, 32);
    int found = 0;
    for (std::uint32_t i = 0; i < n && i < 32; ++i)
        if (cars[i].role == 1 || cars[i].role == 3) {
            *out = cars[i];
            ++found;
        }
    return found == 1;
}

void on_step(GtModCtx *ctx) {
    const std::uint32_t world = g_api->gpr(ctx, 4);
    GtCar human;
    const bool have = find_human(ctx, world, &human);
    GtCarState before;
    const bool read = have && g_api->car_state(g_self, ctx, human.handle, &before) == 0;
    g_api->call_original(ctx);
    if (!have) return;
    const std::uint64_t vb = g_api->vblank_index(g_self);
    if (human.handle != g_car || (g_last_step_vblank.load() != 0 && vb > g_last_step_vblank.load() + kResetGap)) {  // a new race (or its replay): the clock starts again
        g_car = human.handle;
        g_steps.store(0);
        g_have_prev = false;
        g_yaw_shift.store(0.f);
    }
    g_last_step_vblank.store(vb);
    g_steps.fetch_add(1);
    GtCarState after;
    if (!read || g_api->car_state(g_self, ctx, human.handle, &after) != 0) return;
    g_yaw_shift.store(g_yaw_shift.load() - after.rates[4] / 60.f * 200.f);  // the scene turns past the camera: ~200 overlay pixels per radian of yaw (half the picture per 1 rad), opposite to the turn
    if (g_grip_loss > 0.f && g_have_prev) {
        float out[3];
        if (weather::grip_adjust(after.velocity, g_prev_velocity, after.basis + 3, g_grip_loss, 1.5f, out)) {
            GtCarState s = after;
            const float k = std::sqrt(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);
            const float k0 = std::sqrt(after.velocity[0] * after.velocity[0] + after.velocity[1] * after.velocity[1] + after.velocity[2] * after.velocity[2]);
            for (int i = 0; i < 3; ++i) s.velocity[i] = out[i];
            if (k0 > 1e-3f) s.speed = after.speed * (k / k0);
            g_api->car_set_state(g_self, ctx, human.handle, &s);
            std::memcpy(g_prev_velocity, s.velocity, sizeof g_prev_velocity);
            return;
        }
    }
    std::memcpy(g_prev_velocity, after.velocity, sizeof g_prev_velocity);
    g_have_prev = true;
}

weather::Grade current_grade() {
    const int mode = g_mode.load();
    weather::Grade time = mode == kProgressive ? weather::progressive_grade(weather::progress(g_steps.load(), g_progressive_steps)) : weather::preset_grade(static_cast<weather::Preset>(mode));
    return weather::combine(time, weather::rain_grade(g_rain.load()));
}

void on_overlay(GtMod *, GtModCtx *) {
    const std::uint64_t now = g_api->vblank_index(g_self);
    const bool live = g_last_step_vblank.load() != 0 && now <= g_last_step_vblank.load() + kLiveGap;  // a race is on screen: the grade is neutral in menus
    const weather::Grade grade = live ? current_grade() : weather::Grade{};
    if (!g_applied_valid || !(grade == g_applied)) {
#if WEATHER_SABOTAGE != 1
        g_api->scene_grade(g_self, grade.r, grade.g, grade.b, grade.brightness, grade.saturation);
#endif
        g_applied = grade;
        g_applied_valid = true;
        if (g_log && g_logged < 4000) {
            ++g_logged;
            char line[200];
            std::snprintf(line, sizeof line, "GRADE vb=%llu steps=%llu mode=%d rain=%d rgb=%.4f,%.4f,%.4f bright=%.4f sat=%.4f", static_cast<unsigned long long>(now),
                          static_cast<unsigned long long>(g_steps.load()), g_mode.load(), g_rain.load(), grade.r, grade.g, grade.b, grade.brightness, grade.saturation);
            g_api->log(g_self, line);
        }
    }
#if WEATHER_SABOTAGE == 2
    return;
#endif
    const int rain = g_rain.load();
    if (!live || rain <= 0) return;
    weather::Streak streaks[weather::kMaxStreaks];
    const int n = weather::rain_streaks(rain, static_cast<std::uint32_t>(now), g_yaw_shift.load(), streaks, weather::kMaxStreaks);
    for (int i = 0; i < n; ++i) g_api->draw_line(g_self, streaks[i].x0, streaks[i].y0, streaks[i].x1, streaks[i].y1, 0.9f, streaks[i].rgba);
    if (g_log && g_logged < 4000 && now % 60 == 0) {
        char line[96];
        std::snprintf(line, sizeof line, "RAIN vb=%llu level=%d streaks=%d", static_cast<unsigned long long>(now), rain, n);
        g_api->log(g_self, line);
        ++g_logged;
    }
}

// Menu rows (the cheat device's Race page): any thread, atomics only.
void mode_label(GtMod *, char *out, std::uint32_t capacity) {
    const int m = g_mode.load();
    std::snprintf(out, capacity, "Time of day: %s", m == kProgressive ? "progressive" : weather::preset_name(static_cast<weather::Preset>(m)));
}
void mode_activate(GtMod *, int direction) {
    const int n = kProgressive + 1;
    g_mode.store(((g_mode.load() + (direction < 0 ? -1 : 1)) % n + n) % n);
}
void rain_label(GtMod *, char *out, std::uint32_t capacity) { std::snprintf(out, capacity, "Rain: %s", weather::rain_name(g_rain.load())); }
void rain_activate(GtMod *, int direction) {
    const int n = weather::kRainLevels;
    g_rain.store(((g_rain.load() + (direction < 0 ? -1 : 1)) % n + n) % n);
}
}  // namespace

extern "C" __attribute__((visibility("default"))) int gt_mod_init(const GtModApi *api, GtMod *self) {
    if (api->version < GT_MOD_API_VERSION_8) return 1;
    g_api = api;
    g_self = self;
    g_log = std::getenv("GT_MOD_WEATHER_LOG") != nullptr;
    const char *preset = std::getenv("GT_MOD_WEATHER_PRESET");
    if (preset == nullptr) preset = api->config(self, "preset");
    weather::Preset p = weather::Preset::Noon;
    if (preset != nullptr && std::strcmp(preset, "progressive") == 0) g_mode.store(kProgressive);
    else if (weather::parse_preset(preset, &p)) g_mode.store(static_cast<int>(p));
    const char *rain = std::getenv("GT_MOD_WEATHER_RAIN");
    const int r = rain != nullptr ? std::atoi(rain) : static_cast<int>(setting("rain", 0.f));
    g_rain.store(r < 0 ? 0 : (r >= weather::kRainLevels ? weather::kRainLevels - 1 : r));
    g_grip_loss = setting("grip_loss", 0.f);
    const float steps = setting("progressive_steps", 18000.f);
    g_progressive_steps = steps >= 60.f ? static_cast<std::uint64_t>(steps) : 60u;
    if (const char *e = std::getenv("GT_MOD_WEATHER_STEPS")) g_progressive_steps = static_cast<std::uint64_t>(std::atol(e)) >= 60u ? static_cast<std::uint64_t>(std::atol(e)) : 60u;
    if (api->hook_point(self, GT_HOOK_WORLD_STEP, on_step) != 0) return 1;
    if (api->on_overlay(self, on_overlay) != 0) return 1;
    if (api->add_entry(self, "Race", "weather_time", mode_label, mode_activate) != 0) return 1;
    if (api->add_entry(self, "Race", "weather_rain", rain_label, rain_activate) != 0) return 1;
    return 0;
}
