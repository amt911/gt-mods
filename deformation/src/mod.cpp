// Crash deformation (spec docs/superpowers/specs/2026-11-07-crash-deformation-design.md in gran-turismo-decomp), mod API v5, no guest address.
//
// How it works: the car-draw hook names each car's model (the container of its parts; every car has its own, FACTS "Car to model link"); a hit
// of the player car (velocity change of one step, `car_impact`) is queued; on the next vblank tick every vertex run of the player's model (model_runs:
// parts with several vertex arrays included) is read once, the vertices near the contact are pushed in (deform_logic) IN PLACE in the model, and
// the originals are kept. They are written back when no world step has run for a while (the race is over) and when the mod is switched off
// (on_switch_off). A model drawn by more than one car is never dented.
#include "gt_mod/gt_mod_api.h"

#include "deform_logic.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace {
const GtModApi *g_api = nullptr;
GtMod *g_self = nullptr;
deform::Params g_params;
uint32_t g_min_vertices = 100;
int g_axis[3] = {2, 0, 1};      // model axis (0 x, 1 y, 2 z) of the body's forward, right, up
float g_sign[3] = {1.f, 1.f, 1.f};

struct Part {
    uint32_t blob = 0;
    uint32_t run = 0;
    deform::Body body;
    std::vector<int16_t> written;  // what the mod last wrote, to restore only vertices still holding it
};
struct Hit {
    float contact[3];  // body frame (forward, right, up), unit: where on the body the hit landed
    float strength;
};

uint32_t g_world = 0;
uint32_t g_player = 0;                     // the player car's body handle, from the last world step
uint64_t g_last_step_vblank = 0;
std::map<uint32_t, std::set<uint32_t>> g_model_cars;  // model -> the cars seen drawing it
uint32_t g_player_model = 0;
std::vector<Hit> g_pending;
std::vector<Part> g_parts;                 // the player's vertex runs, dented in place
unsigned g_steps = 0;

float setting(const char *key, float fallback) {
    const char *text = g_api->config(g_self, key);
    return text != nullptr && *text != '\0' ? static_cast<float>(std::atof(text)) : fallback;
}
void axis_setting(const char *name, int slot, int fallback_axis) {
    const std::string key(name);
    const char *axis = g_api->config(g_self, (key + "_axis").c_str());
    g_axis[slot] = axis != nullptr && (*axis == 'x' || *axis == 'y' || *axis == 'z') ? *axis - 'x' : fallback_axis;
    g_sign[slot] = setting((key + "_sign").c_str(), 1.f) < 0.f ? -1.f : 1.f;
}
void logf(const char *fmt, unsigned a, unsigned b = 0) {
    char line[160];
    std::snprintf(line, sizeof line, fmt, a, b);
    g_api->log(g_self, line);
}

void on_step(GtModCtx *ctx) {
    g_world = g_api->gpr(ctx, 4);
    g_api->call_original(ctx);  // read the state the step produced
    ++g_steps;
    g_last_step_vblank = g_api->vblank_index(g_self);
    uint32_t car = 0;
    if (g_api->player_car(g_self, ctx, g_world, &car) != 0) return;
    g_player = car;
    GtCarImpact impact;
    if (g_api->car_impact(g_self, ctx, g_world, car, &impact) != 0) return;
    if (!deform::hit_detected(impact.dv_magnitude, g_params.threshold)) return;
    const float strength = deform::hit_strength(impact.dv_magnitude, g_params);
    char line[220];
    std::snprintf(line, sizeof line, "HIT step=%u vblank=%llu dv=%.2f body=(%.2f %.2f %.2f) contact=(%.2f %.2f %.2f) off_track=%d strength=%.3f", g_steps,
                  static_cast<unsigned long long>(g_last_step_vblank), impact.dv_magnitude, impact.dv_body[0], impact.dv_body[1], impact.dv_body[2],
                  impact.contact_dir[0], impact.contact_dir[1], impact.contact_dir[2], impact.off_track, strength);
    g_api->log(g_self, line);
    if (g_pending.size() < 16) g_pending.push_back({{impact.contact_dir[0], impact.contact_dir[1], impact.contact_dir[2]}, strength});
}

// The per-car view update: learn which model belongs to which car.
void on_car_draw(GtModCtx *ctx) {
    uint32_t car = 0, model = 0;
    if (g_api->draw_car_model(g_self, ctx, &car, &model) == 0) {
        g_model_cars[model].insert(car);
        if (car == g_player && g_player != 0u) {
            if (model != g_player_model) logf("PLAYER model=%08x", model);
            g_player_model = model;
        }
    }
    g_api->call_original(ctx);
}

void restore_all(GtModCtx *ctx) {
    unsigned restored = 0;
    for (Part &part : g_parts) {
        for (size_t v = 0; v < part.body.original.size() / 3u; ++v) {
            int16_t now[3];
            // only what the mod wrote: freed and reused memory is left alone
            if (g_api->run_vertex(g_self, ctx, part.blob, part.run, static_cast<uint32_t>(v), now) != 0) break;
            if (std::memcmp(now, &part.written[3 * v], 6) != 0 || std::memcmp(now, &part.body.original[3 * v], 6) == 0) continue;
            g_api->run_set_vertex(g_self, ctx, part.blob, part.run, static_cast<uint32_t>(v), &part.body.original[3 * v]);
            ++restored;
        }
    }
    if (!g_parts.empty()) logf("RESTORE runs=%u vertices=%u", static_cast<unsigned>(g_parts.size()), restored);
    g_parts.clear();
}

void restore_all_off(GtModCtx *ctx) {
    restore_all(ctx);
    g_pending.clear();
}

void release_all(GtModCtx *ctx) {
    restore_all(ctx);
    g_pending.clear();
    g_model_cars.clear();
    g_player_model = 0;
}

// Reads the player's vertex runs once (tick only).
bool capture(GtModCtx *ctx) {
    const auto cars = g_model_cars.find(g_player_model);
    if (cars == g_model_cars.end() || cars->second.size() != 1u) return false;  // shared by another car: never dent
    std::vector<GtVertexRun> runs(256);
    const uint32_t n = g_api->model_runs(g_self, ctx, g_player_model, runs.data(), static_cast<uint32_t>(runs.size()));
    for (uint32_t i = 0; i < n; ++i) {
        if (runs[i].count < g_min_vertices) continue;
        std::vector<int16_t> positions(3u * runs[i].count);
        bool ok = true;
        for (uint32_t v = 0; v < runs[i].count && ok; ++v) ok = g_api->run_vertex(g_self, ctx, runs[i].blob, runs[i].run, v, &positions[3u * v]) == 0;
        if (!ok) continue;
        Part part;
        part.blob = runs[i].blob;
        part.run = runs[i].run;
        part.body = deform::make_body(positions);
        part.written = positions;
        g_parts.push_back(std::move(part));
    }
    logf("CAPTURE runs=%u of=%u", static_cast<unsigned>(g_parts.size()), n);
    return !g_parts.empty();
}

void apply_hit(GtModCtx *ctx, const Hit &hit) {
    deform::Vec3 contact, inward;
    float c[3] = {0, 0, 0};
    for (int i = 0; i < 3; ++i) c[g_axis[i]] = g_sign[i] * hit.contact[i];  // body frame -> model axes
    // each part is normalised to the full s16 range in its own frame: the contact is on that frame's edge in the hit direction
    contact = {c[0] * 32767.f, c[1] * 32767.f, c[2] * 32767.f};
    inward = {-c[0], -c[1], -c[2]};
    unsigned moved_total = 0;
    for (Part &part : g_parts) {
        const std::vector<int16_t> before = part.body.positions;
        const size_t moved = deform::apply_dent(part.body, contact, inward, hit.strength, g_params);
        if (moved == 0) continue;
        moved_total += static_cast<unsigned>(moved);
        for (size_t v = 0; v < part.body.dented.size(); ++v)
            if (std::memcmp(&before[3 * v], &part.body.positions[3 * v], 6) != 0) {
                g_api->run_set_vertex(g_self, ctx, part.blob, part.run, static_cast<uint32_t>(v), &part.body.positions[3 * v]);
                std::memcpy(&part.written[3 * v], &part.body.positions[3 * v], 6);
            }
    }
    logf("DENT vertices=%u strength_milli=%u", moved_total, static_cast<unsigned>(hit.strength * 1000.f));
}

void on_tick(GtModCtx *ctx) {
    const uint64_t now = g_api->vblank_index(g_self);
    if (!g_parts.empty() || g_player_model != 0u) {
        if (g_last_step_vblank != 0 && now > g_last_step_vblank + 180u) {  // no world step for 3 s: the race is over
            release_all(ctx);
            g_last_step_vblank = 0;
            return;
        }
    }
    if (g_pending.empty() || g_player_model == 0u) return;
    if (g_parts.empty() && !capture(ctx)) {
        g_pending.clear();
        return;
    }
    for (const Hit &hit : g_pending) apply_hit(ctx, hit);
    g_pending.clear();
}
} // namespace

extern "C" __attribute__((visibility("default"))) int gt_mod_init(const GtModApi *api, GtMod *self) {
    if (api->version < GT_MOD_API_VERSION_5) return 1;
    g_api = api;
    g_self = self;
    g_params.threshold = setting("threshold", g_params.threshold);
    g_params.depth_per_ms = setting("depth_per_ms", g_params.depth_per_ms);
    g_params.radius = setting("radius", g_params.radius);
    g_params.cap = setting("cap", g_params.cap);
    g_min_vertices = static_cast<uint32_t>(setting("min_vertices", 100.f));
    axis_setting("forward", 0, 2);
    axis_setting("right", 1, 0);
    axis_setting("up", 2, 1);
    if (api->hook_point(self, GT_HOOK_WORLD_STEP, on_step) != 0) return 1;
    if (api->hook_point(self, GT_HOOK_CAR_DRAW, on_car_draw) != 0) return 1;
    if (api->on_vblank(self, on_tick) != 0) return 1;
    return api->on_switch_off(self, restore_all_off);
}
