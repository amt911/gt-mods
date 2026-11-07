// draw-distance: pushes the race world's far clip plane further away (gran-turismo-decomp mod SDK, API 1).
//
// The race world projection gets its depth range from func_08AF4B4C(matrix, near, far), called by func_0890E41C(self, ...) (return
// addresses 0x0890E4B4 for the mode 0 path, 0x0890E4D0 for the other in --decomp off) with near = word(self+4) and far = word(self+0xC) (1200 in the
// races seen; docs/FACTS.md "draw distance" of the host repo). The game does not cull by that plane: it submits the same geometry
// and the GE only clips, so a larger far shows scenery that was already there. The sky pass and the car reflection pass (far 100,
// caller 0x0889C014) are other calls and are not touched.
//
// Why the hook is on 0x08AF4B4C and not on 0x0890E41C: in this runtime call_original returns before the rest of a function body
// runs (the body continues after the hook returned: observed, 0x0890E41C's call to 0x08AF4B4C ran after a hook's restore), so
// "scale self+0xC, call the original, restore" changes nothing. 0x08AF4B4C is a leaf that only writes the two depth terms
// m10 = -(far + near) / (far - near) and m14 = -2 far near / (far - near) into its matrix (a0) from f12 = near, f13 = far; float
// registers are not reachable from a hook, so the hook lets it run and then rewrites m10/m14 for far * far_scale (near is
// recovered from the terms). Nothing else is touched: scene data keeps its 1200, a disabled mod (F9 menu: the loader's trampoline
// passes straight to the original) leaves the game as it was, and far_scale 1.0 writes nothing.
#include "gt_mod/gt_mod_api.h"

#include "far_math.hpp"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {

constexpr std::uint32_t kDepth = 0x08AF4B4C;  // a0 = matrix, f12 = near, f13 = far
constexpr std::uint32_t kM10 = 0x28, kM14 = 0x38;

const GtModApi *g_api = nullptr;
GtMod *g_self = nullptr;
drawdist::Settings g_settings;
bool g_log = false;
unsigned g_calls = 0, g_edits = 0, g_nra = 0;
std::uint32_t g_ras[8];

bool env_set(const char *name) {
    const char *v = std::getenv(name);
    return v != nullptr && *v != '\0';
}

void on_depth(GtModCtx *c) {
    const GtModApi &a = *g_api;
    const std::uint32_t ra = a.gpr(c, 31), matrix = a.gpr(c, 4);
    a.call_original(c);
    if (matrix == 0) return;
    const float m10 = a.readf(c, matrix + kM10), m14 = a.readf(c, matrix + kM14);
    // The world pass is told by its depth range, not by the return address (0x0890E4B4 / 0x0890E4D0 in --decomp off, but another
    // address with --decomp on, where the decompiled caller reaches here through the dispatcher): the car reflection pass is
    // near 0.1 / far 100 (caller 0x0889C014); the race world is near ~1.3 / far ~1200 (is_world_depth).
    if (!drawdist::is_world_depth(m10, m14)) return;
    ++g_calls;
    if (g_log && g_nra < 8) {
        bool seen = false;
        for (unsigned i = 0; i < g_nra; ++i) seen = seen || g_ras[i] == ra;
        if (!seen) {
            g_ras[g_nra++] = ra;
            char line[96];
            std::snprintf(line, sizeof line, "world depth call from ra=%08X", ra);
            a.log(g_self, line);
        }
    }
    float n10 = 0, n14 = 0;
    if (!drawdist::rescale_depth(m10, m14, g_settings.far_scale, &n10, &n14)) return;
#if defined(DRAW_DISTANCE_SABOTAGE) && DRAW_DISTANCE_SABOTAGE == 1
    // sabotage: the near plane scaled too (x far_scale again; a mod that changes near must fail the in-game composition check)
    {
        const float k = g_settings.far_scale * 6.0f, near_plane = m14 / (m10 - 1.0f) * k, far_plane = m14 / (m10 + 1.0f) * g_settings.far_scale;
        n10 = -(far_plane + near_plane) / (far_plane - near_plane);
        n14 = -2.0f * far_plane * near_plane / (far_plane - near_plane);
    }
#endif
    a.writef(c, matrix + kM10, n10);
    a.writef(c, matrix + kM14, n14);
    ++g_edits;
    if (g_log && g_edits % 240 == 1) {
        char line[200];
        std::snprintf(line, sizeof line, "caller=%08X far %.1f -> %.1f near %.3f (calls=%u edits=%u)", ra, static_cast<double>(m14 / (m10 + 1.0f)),
                      static_cast<double>(n14 / (n10 + 1.0f)), static_cast<double>(n14 / (n10 - 1.0f)), g_calls, g_edits);
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
    g_log = env_set("GT_MOD_DRAWDIST_LOG");
    char line[96];
    std::snprintf(line, sizeof line, "far_scale=%.3f", static_cast<double>(g_settings.far_scale));
    api->log(self, line);
    return api->hook(self, kDepth, on_depth);
}
