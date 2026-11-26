// Telemetry panel (spec docs/superpowers/specs/2026-11-18-telemetry-mod-design.md in gran-turismo-decomp), mod API v8, no guest address.
//
// A world-step hook samples the player's car (car_state) into a 300-step ring (about 5 s of game time); the overlay callback (once per vblank)
// copies the ring under a lock and draws the panel with draw_text / draw_rect / draw_line. Read-only: the mod writes nothing to the game, and
// with the panel hidden it draws nothing.
//
// Not done yet (needs an SDK call that does not exist): per-wheel slip and wheel speed (`car_wheels`, spec step 0).
//
// TELEMETRY_SABOTAGE (build flag, for the in-game check only): 1 = constant G, 2 = the overlay draws nothing, 3 = the speed digits are frozen,
// 4 = the throttle and brake bars are swapped.
#include "gt_mod/gt_mod_api.h"

#include "telemetry_logic.hpp"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>

#ifndef TELEMETRY_SABOTAGE
#define TELEMETRY_SABOTAGE 0
#endif

namespace {
const GtModApi *g_api = nullptr;
GtMod *g_self = nullptr;

std::mutex g_mutex;  // guards g_tracker and g_latest (the world-step hook writes, the overlay reads)
telemetry::Tracker g_tracker;
telemetry::Sample g_latest;
uint64_t g_steps = 0;
std::atomic<uint64_t> g_last_step_vblank{0};
std::atomic<bool> g_visible{true};
std::atomic<bool> g_live{false};  // the previous overlay pass drew the panel (for the log line when it changes)
bool g_log = false;
unsigned g_log_every = 1;  // GT_MOD_TELEMETRY_LOG=<n>: one STEP line per n world steps (1 = all)
bool g_mph = false;
float g_x = 8.f, g_y = 52.f;

// A step gap this long (in vblanks) means no race is running (menu, pause, results): the panel hides and the history starts over.
constexpr uint64_t kLiveGap = 30;
constexpr uint64_t kResetGap = 120;

float setting(const char *key, float fallback) {
    const char *text = g_api->config(g_self, key);
    return text != nullptr && *text != '\0' ? static_cast<float>(std::atof(text)) : fallback;
}
bool bool_setting(const char *key, bool fallback) {
    const char *text = g_api->config(g_self, key);
    if (text == nullptr || *text == '\0') return fallback;
    return std::strcmp(text, "false") != 0 && std::strcmp(text, "0") != 0;
}
void logline(const char *line) { g_api->log(g_self, line); }

void on_step(GtModCtx *ctx) {
    const uint32_t world = g_api->gpr(ctx, 4);
    g_api->call_original(ctx);  // read the state the step produced
    uint32_t car = 0;
    if (g_api->player_car(g_self, ctx, world, &car) != 0) return;
    GtCarState state;
    if (g_api->car_state(g_self, ctx, car, &state) != 0) return;
    const uint64_t vblank = g_api->vblank_index(g_self);
    telemetry::CarInput in;
    for (int i = 0; i < 3; ++i) {
        in.position[i] = state.position[i];
        in.velocity[i] = state.velocity[i];
        in.forward[i] = state.basis[i];
        in.right[i] = state.basis[3 + i];
    }
    in.speed = state.speed;
    in.throttle = state.throttle;
    in.brake = state.brake;
    telemetry::Sample s;
    uint64_t n;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (vblank > g_last_step_vblank.load() + kResetGap) g_tracker.reset();  // a new race, or back after a long pause
        s = g_tracker.feed(in);
#if TELEMETRY_SABOTAGE == 1
        s.g_long = 0.3f;  // a constant G: the "G follows braking" check must fail
        s.g_lat = 0.f;
#endif
        g_latest = s;
        n = ++g_steps;
    }
    g_last_step_vblank.store(vblank);
    if (g_log && n % g_log_every == 0) {
        char line[200];
        std::snprintf(line, sizeof line, "STEP n=%llu vblank=%llu speed=%.2f glong=%.3f glat=%.3f thr=%.2f brk=%.2f", static_cast<unsigned long long>(n),
                      static_cast<unsigned long long>(vblank), s.speed, s.g_long, s.g_lat, s.throttle, s.brake);
        logline(line);
    }
}

constexpr float kPanelW = 150.f, kPanelH = 128.f;
constexpr uint32_t kBg = 0x000000B4, kEdge = 0xFFFFFF40, kText = 0xFFFFFFFF, kDim = 0xFFFFFF90, kZero = 0xFFFFFF50;
constexpr uint32_t kSpeed = 0x4FC3F7FF, kGLong = 0xFFB300FF, kGLat = 0x66BB6AFF, kThrottle = 0x43A047FF, kBrake = 0xE53935FF;

void draw_graph(float x, float y, float w, float h, const telemetry::Ring &ring, telemetry::Range range, bool speed_series) {
    g_api->draw_rect(g_self, x, y, w, h, 0xFFFFFF18);
    if (!speed_series) {  // the zero line of the G graph
        const float zy = telemetry::graph_y(0.f, range, y, h);
        g_api->draw_line(g_self, x, zy, x + w, zy, 0.6f, kZero);
    }
    const size_t n = ring.size();
    if (n < 2) return;
    // every second sample: 150 segments are plenty for 150 logical pixels, and keep the pass small
    for (int series = 0; series < (speed_series ? 1 : 2); ++series) {
        float px = 0.f, py = 0.f;
        bool have = false;
        for (size_t i = 0; i < n; i += 2) {
            const telemetry::Sample &s = ring.at(i);
            const float v = speed_series ? s.speed : (series == 0 ? telemetry::shown_g(s.g_long) : telemetry::shown_g(s.g_lat));
            const float gx = telemetry::graph_x(i, n, ring.capacity(), x, w);
            const float gy = telemetry::graph_y(v, range, y, h);
            if (have) g_api->draw_line(g_self, px, py, gx, gy, 1.1f, speed_series ? kSpeed : (series == 0 ? kGLong : kGLat));
            px = gx;
            py = gy;
            have = true;
        }
    }
}

void on_overlay(GtMod *, GtModCtx *) {
#if TELEMETRY_SABOTAGE == 2
    return;
#endif
    const uint64_t now = g_api->vblank_index(g_self);
    const bool live = now <= g_last_step_vblank.load() + kLiveGap && g_last_step_vblank.load() != 0;
    const bool show = live && g_visible.load();
    if (show != g_live.exchange(show) && g_log) logline(show ? "PANEL shown" : "PANEL hidden");
    if (!show) return;
    telemetry::Ring ring(telemetry::kHistory);
    telemetry::Sample latest;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        ring = g_tracker.ring();
        latest = g_latest;
    }
    const float x = g_x, y = g_y;
    g_api->draw_rect(g_self, x, y, kPanelW, kPanelH, kBg);
    g_api->draw_rect(g_self, x, y, kPanelW, 1.f, kEdge);
    g_api->draw_rect(g_self, x, y + kPanelH - 1.f, kPanelW, 1.f, kEdge);

    char text[64];
    float speed = latest.speed * (g_mph ? 2.23694f : 3.6f);
#if TELEMETRY_SABOTAGE == 3
    speed = 123.f;
#endif
    std::snprintf(text, sizeof text, "%3d %s", static_cast<int>(std::lround(speed)), g_mph ? "mph" : "km/h");
    g_api->draw_text(g_self, x + 6.f, y + 3.f, 16.f, kText, text);
    std::snprintf(text, sizeof text, "G LON %+.2f  LAT %+.2f", telemetry::shown_g(latest.g_long), telemetry::shown_g(latest.g_lat));
    g_api->draw_text(g_self, x + 6.f, y + 21.f, 9.f, kText, text);

    // throttle and brake bars
    float thr = latest.throttle, brk = latest.brake;
#if TELEMETRY_SABOTAGE == 4
    std::swap(thr, brk);
#endif
    thr = std::fmin(1.f, std::fmax(0.f, thr));
    brk = std::fmin(1.f, std::fmax(0.f, brk));
    const float bx = x + 30.f, bw = kPanelW - 36.f;
    g_api->draw_text(g_self, x + 6.f, y + 33.f, 8.f, kDim, "THR");
    g_api->draw_rect(g_self, bx, y + 35.f, bw, 5.f, 0xFFFFFF20);
    if (thr > 0.f) g_api->draw_rect(g_self, bx, y + 35.f, bw * thr, 5.f, kThrottle);
    g_api->draw_text(g_self, x + 6.f, y + 43.f, 8.f, kDim, "BRK");
    g_api->draw_rect(g_self, bx, y + 45.f, bw, 5.f, 0xFFFFFF20);
    if (brk > 0.f) g_api->draw_rect(g_self, bx, y + 45.f, bw * brk, 5.f, kBrake);

    // graphs: speed, then G (longitudinal amber, lateral green), the last 5 s
    const float gx = x + 6.f, gw = kPanelW - 12.f;
    g_api->draw_text(g_self, gx, y + 53.f, 8.f, kDim, "SPEED 5 s");
    draw_graph(gx, y + 62.f, gw, 28.f, ring, telemetry::speed_range(ring, 40.f), true);
    g_api->draw_text(g_self, gx, y + 91.f, 8.f, kDim, "G LON / LAT");
    draw_graph(gx, y + 100.f, gw, 24.f, ring, telemetry::g_range(ring, 1.f), false);
}

// The cheat device's row and the hotkey: toggle the panel. Runs on the vblank tick.
void toggle_action(GtMod *, GtModCtx *) {
    const bool now = !g_visible.load();
    g_visible.store(now);
    if (g_log) logline(now ? "TOGGLE visible" : "TOGGLE hidden");
}
}  // namespace

extern "C" __attribute__((visibility("default"))) int gt_mod_init(const GtModApi *api, GtMod *self) {
    if (api->version < GT_MOD_API_VERSION_8) return 1;
    g_api = api;
    g_self = self;
    if (const char *log = std::getenv("GT_MOD_TELEMETRY_LOG")) {
        g_log = true;
        const long every = std::atol(log);
        g_log_every = every > 0 ? static_cast<unsigned>(every) : 1u;
    }
    const char *units = api->config(self, "units");
    g_mph = units != nullptr && std::strcmp(units, "mph") == 0;
    g_x = setting("x", g_x);
    g_y = setting("y", g_y);
    bool visible = bool_setting("visible", true);
    if (const char *env = std::getenv("GT_MOD_TELEMETRY_VISIBLE")) visible = std::strcmp(env, "0") != 0;
    g_visible.store(visible);
    if (api->hook_point(self, GT_HOOK_WORLD_STEP, on_step) != 0) return 1;
    if (api->on_overlay(self, on_overlay) != 0) return 1;
    return api->add_action(self, "toggle", "Telemetry panel", "Race", toggle_action, "T", "") != 0 ? 1 : 0;
}
