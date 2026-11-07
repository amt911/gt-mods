// draw-distance: pushes the race world's far clip plane further away (gran-turismo-decomp mod SDK, API 1).
//
// The race world projection takes its depth range from func_08AF4B4C(matrix, near, far), called by func_0890E41C(self, ...) with
// near = word(self+4) and far = word(self+0xC) (1200 in the races seen; docs/FACTS.md "draw distance" of the host repo). The game
// does not cull by that plane: it submits the same geometry and the GE only clips, so a larger far shows scenery that was already
// there. The sky pass and the car reflection pass do not go through this function.
//
// Hook: 0x0890E41C. Multiply self+0xC by far_scale, run the original, put the original value back (scene data stays untouched).
// A disabled mod (F9 menu) never reaches this code: the loader's trampoline passes straight to the original.
#include "gt_mod/gt_mod_api.h"

#include "far_math.hpp"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace {

constexpr std::uint32_t kProjection = 0x0890E41C;  // a0 = self (scene fog/sky settings object)
constexpr std::uint32_t kNear = 4, kFar = 0xC;

const GtModApi *g_api = nullptr;
GtMod *g_self = nullptr;
drawdist::Settings g_settings;
bool g_log = false;
unsigned g_calls = 0, g_edits = 0;

std::uint32_t bits(float f) {
    std::uint32_t u;
    std::memcpy(&u, &f, sizeof u);
    return u;
}

void on_projection(GtModCtx *c) {
    const GtModApi &a = *g_api;
    const std::uint32_t self = a.gpr(c, 4);
    ++g_calls;
    if (self == 0) {
        a.call_original(c);
        return;
    }
    const float near_plane = a.readf(c, self + kNear), far_plane = a.readf(c, self + kFar);
    if (!drawdist::needs_edit(far_plane, near_plane, g_settings.far_scale)) {
        a.call_original(c);
        return;
    }
    const float edited = drawdist::scaled_far(far_plane, near_plane, g_settings.far_scale);
#if defined(DRAW_DISTANCE_SABOTAGE) && DRAW_DISTANCE_SABOTAGE == 1
    // sabotage: also scale the near plane (self+4); the in-game composition check must fail
    a.writef(c, self + kNear, near_plane * g_settings.far_scale);
#endif
    a.writef(c, self + kFar, edited);
    ++g_edits;
    a.call_original(c);
    // Restore only what we wrote: if the callee changed the value itself, that is the scene's own business.
    if (bits(a.readf(c, self + kFar)) == bits(edited)) a.writef(c, self + kFar, far_plane);
#if defined(DRAW_DISTANCE_SABOTAGE) && DRAW_DISTANCE_SABOTAGE == 1
    a.writef(c, self + kNear, near_plane);
#endif
    if (g_log && g_edits % 120 == 1) {
        char line[160];
        std::snprintf(line, sizeof line, "self=%08X near=%.3f far=%.1f -> %.1f calls=%u edits=%u", self, near_plane, far_plane, edited, g_calls, g_edits);
        a.log(g_self, line);
    }
}

}  // namespace

extern "C" __attribute__((visibility("default"))) int gt_mod_init(const GtModApi *api, GtMod *self) {
    g_api = api;
    g_self = self;
    g_settings = drawdist::parse_settings([&](const char *k) { return api->config(self, k); },
                                          [&](const std::string &w) { api->log(self, w.c_str()); });
    // The in-game check's override (several scales without rewriting mod.toml).
    float forced = 0;
    if (drawdist::parse_scale(std::getenv("GT_MOD_DRAWDIST_SCALE"), &forced)) g_settings.far_scale = forced;
    g_log = std::getenv("GT_MOD_DRAWDIST_LOG") != nullptr;
    char line[96];
    std::snprintf(line, sizeof line, "far_scale=%.3f", static_cast<double>(g_settings.far_scale));
    api->log(self, line);
    return api->hook(self, kProjection, on_projection);
}
