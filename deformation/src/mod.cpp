// Crash deformation (spec docs/superpowers/specs/2026-11-07-crash-deformation-design.md in gran-turismo-decomp), mod API v5, no guest address.
// v0.1.0: detects the player car's hits from the velocity change of the physics step and logs them. It does NOT change any mesh yet: the link
// between the player car and its body blobs is not found (FINDINGS "The real GE blob layout ... 2026-11-14"), and denting every large blob would
// dent AI cars and scenery. The dent logic (deform_logic) is tested and waits for that link.
#include "gt_mod/gt_mod_api.h"

#include "deform_logic.hpp"

#include <cstdio>
#include <cstdlib>

namespace {
const GtModApi *g_api = nullptr;
GtMod *g_self = nullptr;
deform::Params g_params;
uint32_t g_world = 0;
unsigned g_steps = 0;

float setting(const char *key, float fallback) {
    const char *text = g_api->config(g_self, key);
    return text != nullptr && *text != '\0' ? static_cast<float>(std::atof(text)) : fallback;
}

void on_step(GtModCtx *ctx) {
    g_world = g_api->gpr(ctx, 4);
    g_api->call_original(ctx);  // read the state the step produced
    ++g_steps;
    uint32_t car = 0;
    if (g_api->player_car(g_self, ctx, g_world, &car) != 0) return;
    GtCarImpact impact;
    if (g_api->car_impact(g_self, ctx, g_world, car, &impact) != 0) return;
    if (!deform::hit_detected(impact.dv_magnitude, g_params.threshold)) return;
    char line[200];
    std::snprintf(line, sizeof line, "HIT step=%u dv=%.2f body=(%.2f %.2f %.2f) contact=(%.2f %.2f %.2f) off_track=%d strength=%.3f", g_steps,
                  impact.dv_magnitude, impact.dv_body[0], impact.dv_body[1], impact.dv_body[2], impact.contact_dir[0], impact.contact_dir[1],
                  impact.contact_dir[2], impact.off_track, deform::hit_strength(impact.dv_magnitude, g_params));
    g_api->log(g_self, line);
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
    return api->hook_point(self, GT_HOOK_WORLD_STEP, on_step);
}
