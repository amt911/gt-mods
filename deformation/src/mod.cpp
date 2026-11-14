// Crash deformation (spec docs/superpowers/specs/2026-11-07-crash-deformation-design.md in gran-turismo-decomp), mod API v5, no guest address.
//
// How it works: the car-draw hook names each car's model (the container of its parts); a hit of the player car (velocity change of one step,
// `car_impact`) is queued; on the next vblank tick the player's part blobs are copied once (`mesh_copy`: copy-on-write, so an AI car of the
// same model would stay clean; a model shared by two cars is never dented), the copies are dented around the contact (deform_logic) and the
// mesh-draw hook redirects the originals to the copies. Switching the mod off stops the redirects (the originals were never touched);
// when no world step has run for a while (the race is over) the copies are released.
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
uint32_t g_redirects = 0;
int g_axis[3] = {2, 0, 1};      // model axis (0 x, 1 y, 2 z) of the body's forward, right, up
float g_sign[3] = {1.f, 1.f, 1.f};

struct Part {
    uint32_t copy = 0;
    deform::Body body;
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
std::map<uint32_t, Part> g_parts;          // original blob -> its dented copy
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
        if (car == g_player && g_player != 0u) g_player_model = model;
    }
    g_api->call_original(ctx);
}

void on_mesh_draw(GtModCtx *ctx) {
    if (!g_parts.empty()) {
        const uint32_t seen = g_api->draw_mesh_blob(ctx);
        const auto it = g_parts.find(seen);
        if (it != g_parts.end()) {
            g_api->draw_mesh_redirect(ctx, it->second.copy);
            if (++g_redirects == 1u) logf("REDIRECT first draw of a copy", 0);
        }
    }
    g_api->call_original(ctx);
}

void release_all(GtModCtx *ctx) {
    for (auto &part : g_parts) g_api->mesh_release(g_self, ctx, part.second.copy);
    if (!g_parts.empty()) logf("RELEASE parts=%u", static_cast<unsigned>(g_parts.size()));
    g_parts.clear();
    g_redirects = 0;
    g_pending.clear();
    g_model_cars.clear();
    g_player_model = 0;
}

// Copies the player's parts once (tick only: mesh_copy calls the game's allocator).
bool make_copies(GtModCtx *ctx) {
    const auto cars = g_model_cars.find(g_player_model);
    if (cars == g_model_cars.end() || cars->second.size() != 1u) return false;  // shared by another car: never dent
    uint32_t blobs[64];
    const uint32_t n = g_api->model_blobs(g_self, ctx, g_player_model, blobs, 64);
    for (uint32_t i = 0; i < n; ++i) {
        GtMeshInfo info;
        if (g_api->mesh_info(g_self, ctx, blobs[i], &info) != 0) continue;
        if (info.vertex_count < g_min_vertices) continue;
        Part part;
        if (g_api->mesh_copy(g_self, ctx, blobs[i], &part.copy) != 0) continue;
        // the copy's vertices are the originals' (a PRIM strip may repeat a vertex: each index is read and written on its own)
        std::vector<int16_t> positions(3u * info.vertex_count);
        bool ok = true;
        for (uint32_t v = 0; v < info.vertex_count && ok; ++v) ok = g_api->mesh_vertex(g_self, ctx, part.copy, v, &positions[3u * v]) == 0;
        if (!ok) {
            g_api->mesh_release(g_self, ctx, part.copy);
            continue;
        }
        part.body = deform::make_body(positions);
        g_parts[blobs[i]] = std::move(part);
    }
    logf("COPY parts=%u of=%u", static_cast<unsigned>(g_parts.size()), n);
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
    for (auto &entry : g_parts) {
        Part &part = entry.second;
        const std::vector<int16_t> before = part.body.positions;
        const size_t moved = deform::apply_dent(part.body, contact, inward, hit.strength, g_params);
        if (moved == 0) continue;
        moved_total += static_cast<unsigned>(moved);
        for (size_t v = 0; v < part.body.dented.size(); ++v)
            if (std::memcmp(&before[3 * v], &part.body.positions[3 * v], 6) != 0)
                g_api->mesh_set_vertex(g_self, ctx, part.copy, static_cast<uint32_t>(v), &part.body.positions[3 * v]);
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
    if (g_parts.empty() && !make_copies(ctx)) {
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
    if (api->hook_point(self, GT_HOOK_MESH_DRAW, on_mesh_draw) != 0) return 1;
    return api->on_vblank(self, on_tick);
}
