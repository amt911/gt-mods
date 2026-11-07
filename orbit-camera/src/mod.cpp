// orbit-camera: right-stick orbit camera (Gran Turismo 7 style) for the player's car (gran-turismo-decomp mod SDK, API 1).
//
// How the game draws the view (docs/FACTS.md of the host repo, "orbit camera"): the GE never receives a view matrix; the game
// folds view x model on the CPU and uploads the product per object as the WORLD matrix: 0x08B59508 (a GU class method, this =
// a0 = 0x08D6CE00) calls 0x08B59210(this, 0x3A, ptr) with the 16 floats at word(this+0x2E0+0xD00), the projection being at
// word(this+0x2E0+0xD04) (m[11] = -1 for 3D, 0 for the HUD's orthographic one). 0x08B59210 writes the matrix into the GE list.
// The hook is on 0x08B59210 (see on_upload for why not on 0x08B59508): it rotates the world matrix around the player's car for a
// perspective upload, calls the original and restores the 16 floats, so nothing else sees the edit.
//
// Camera: S = the scene object of the race; word(S+0x1FA4) is its active camera (S itself in the chase view; another object in
// others), whose camera-to-world matrix sits at +0x10 (row vectors, row 3 = eye). S comes from code: 0x08928A80(self = S, ...) is
// the scene refresh that runs once per physics step; the hook remembers its a0. (The static word at 0x08D11F74 held S in a RAM dump
// but reads 0 during the race, so it is only logged.)
//
// Player-live predicate (same as analog-pedals): the driver entry with vtable 0x08CC1F60, byte +0xA8 == 0 (live, not replay) and
// +0x6C != 0 (reads the pad); exactly one. It is evaluated in the physics-step hook 0x0894DD9C; the upload hook only edits while
// a step ran within the last 10 frames (StepGate: frames are counted by the HUD's orthographic uploads, so it is game-paced and a slow
// machine does not flicker to vanilla; a menu, the pause screen, a garage or a replay stops the steps and the view is left alone).
// The orbit is the identity at yaw 0 and pitch 0: the mod then writes nothing.
#include "gt_mod/gt_mod_api.h"

#include "input_source.hpp"
#include "orbit_input.hpp"
#include "orbit_math.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <utility>

namespace {

constexpr std::uint32_t kStep = 0x0894DD9C;          // a0 = world
constexpr std::uint32_t kSceneRefresh = 0x08928A80;  // a0 = scene object S
constexpr std::uint32_t kUpload = 0x08B59210;        // a0 = GU object, a1 = command, a2 = matrix: the GE-list writer of 0x08B59508
constexpr std::uint32_t kWorldCommand = 0x3A;
constexpr std::uint32_t kGuObject = 0x08D6CE00;  // the graphics library context (this of 0x08B59508)
constexpr std::uint32_t kPlayerVtable = 0x08CC1F60;
constexpr std::uint32_t kEntryReplay = 0xA8, kEntryInput = 0x6C;
constexpr std::uint32_t kSlotSize = 0x2100, kCarInSlot = 0x2010;
constexpr std::uint32_t kSceneCamera = 0x1FA4, kCameraMatrix = 0x10;
constexpr std::uint32_t kGuBlock = 0x2E0, kGuProj = 0xD04;
constexpr std::uint32_t kSceneGlobal = 0x08D11F74;  // log only
constexpr int kStaleFrames = 10;  // spec: the step ran within the last 10 frames

const GtModApi *g_api = nullptr;
GtMod *g_self = nullptr;
orbit::Settings g_settings;
std::optional<std::pair<float, float>> g_fake;
bool g_log = false;

orbit::YawSmoother g_smoother;
float g_yaw = 0.0f;
std::uint32_t g_player_car = 0, g_scene = 0;
unsigned g_steps = 0, g_uploads = 0, g_edits = 0, g_rej_flags = 0, g_rej_persp = 0, g_rej_pivot = 0;
std::uint32_t g_seen[8];
unsigned g_nseen = 0;
orbit::StepGate g_gate(kStaleFrames);
float g_last_pivot[3] = {0, 0, 0};
bool g_chase_view = false;  // the full third-person view (orbit_input.hpp is_full_chase_view), evaluated each step

bool pivot_view(GtModCtx *c, float out[3]);

// Finds the live human car of `world`. Exactly one -> its car address, else 0.
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

void on_step(GtModCtx *c) {
    g_gate.on_step();
    ++g_steps;
    g_player_car = find_player_car(c, g_api->gpr(c, 4));
    float pv[3];
    const bool chase = pivot_view(c, pv) && orbit::is_full_chase_view(pv);
    if (g_log && chase != g_chase_view) {
        char line[160];
        std::snprintf(line, sizeof line, "view step=%u chase=%d pivot=%.2f,%.2f,%.2f", g_steps, chase ? 1 : 0, pv[0], pv[1], pv[2]);
        g_api->log(g_self, line);
    }
    g_chase_view = chase;
    if (g_player_car == 0) {
        g_smoother.reset(0.0f);
        g_yaw = 0.0f;
    } else if (g_fake) {
        g_yaw = orbit::view_yaw(g_fake->first, g_chase_view, g_settings);  // the check's fixed stick angle: exact, no swing
    } else {
        const float target = orbit::view_yaw(orbit::target_yaw(orbit::read_raw(), g_settings), g_chase_view, g_settings);
        g_yaw = g_smoother.step(target, orbit::kStepMs, g_settings.smooth_ms);
    }
    g_api->call_original(c);
    if (g_log && g_player_car != 0 && g_steps % 60 == 0) {
        char line[280];
        const std::uint32_t cam = g_scene != 0 ? g_api->read32(c, g_scene + kSceneCamera) : 0;
        std::snprintf(line, sizeof line, "step=%u yaw=%.2f chase=%d scene=%08X cam=%08X global=%08X pivot=%.2f,%.2f,%.2f uploads=%u edits=%u rej(flags/ortho/pivot)=%u/%u/%u", g_steps, g_yaw,
                      g_chase_view ? 1 : 0, g_scene, cam, g_api->read32(c, kSceneGlobal), g_last_pivot[0], g_last_pivot[1], g_last_pivot[2], g_uploads, g_edits, g_rej_flags, g_rej_persp, g_rej_pivot);
        g_api->log(g_self, line);
    }
}

void on_scene_refresh(GtModCtx *c) {
    const std::uint32_t s = g_api->gpr(c, 4);
    if (g_log && s != 0 && g_nseen < 8) {
        bool known = false;
        for (unsigned i = 0; i < g_nseen; ++i) known = known || g_seen[i] == s;
        if (!known) {
            g_seen[g_nseen++] = s;
            char line[160];
            std::snprintf(line, sizeof line, "scene refresh a0=%08X active camera word=%08X", s, g_api->read32(c, s + kSceneCamera));
            g_api->log(g_self, line);
        }
    }
    if (s != 0 && g_api->read32(c, s + kSceneCamera) != 0) g_scene = s;
    g_api->call_original(c);
}

// The player car's position in view space, from the active camera. false when anything is not there.
bool pivot_view(GtModCtx *c, float out[3]) {
    const GtModApi &a = *g_api;
    if (g_player_car == 0 || g_scene == 0) return false;
    const std::uint32_t cam = a.read32(c, g_scene + kSceneCamera);
    if (cam == 0) return false;
    const std::uint32_t cd = a.read32(c, g_player_car + 8);
    if (cd == 0) return false;
    float cm[16];
    for (int i = 0; i < 16; ++i) cm[i] = a.readf(c, cam + kCameraMatrix + 4 * i);
    for (int i = 0; i < 16; ++i)
        if (!std::isfinite(cm[i])) return false;
    float car[3];
    orbit::car_to_yup(a.readf(c, cd + 0x10), a.readf(c, cd + 0x14), a.readf(c, cd + 0x18), car);
    orbit::pivot_in_view(cm, car, out);
#if defined(ORBIT_CAMERA_SABOTAGE) && ORBIT_CAMERA_SABOTAGE == 1
    out[0] = out[1] = out[2] = 0.0f;  // pivot at the eye: the car swings out of the picture (the in-game check must fail)
#endif
    return std::isfinite(out[0]) && std::isfinite(out[1]) && std::isfinite(out[2]);
}

// 0x08B59210(this = a0, command = a1, data = a2): writes 12 words of a matrix upload (4 rows x 3 floats, float24) into the GE
// list; a1 = 0x3A is the world matrix. Hooked here and not at 0x08B59508 (the caller): the recompiled 0x08B59508 reaches this
// function by `ctx.pc = 0x08B59210; return;`, so the runtime runs it after 0x08B59508 returned, i.e. after a hook on 0x08B59508
// has already restored the matrix. 0x08B59210 is a leaf that reads a2 and returns, so the edit-call-restore is complete here.
void on_upload(GtModCtx *c) {
    const GtModApi &a = *g_api;
    // Only the GU context (a0 = 0x08D6CE00, FACTS): other callers of 0x08B59210 pass other objects, whose fields must not be read.
    if (a.gpr(c, 5) != kWorldCommand || a.gpr(c, 4) != kGuObject) { a.call_original(c); return; }
    const std::uint32_t wp = a.gpr(c, 6), pp = a.read32(c, a.gpr(c, 4) + kGuBlock + kGuProj);
    const bool perspective = pp != 0 && orbit::is_perspective(a.readf(c, pp + 4 * 11));
    if (pp != 0 && !perspective) g_gate.on_frame();  // the HUD's orthographic upload: once per frame, menus and pauses draw it too
    const float pitch = g_fake ? g_fake->second : g_settings.pitch_deg;
    if (!perspective || wp == 0 || g_player_car == 0 || !g_gate.live() || !orbit::needs_edit(g_yaw, g_yaw != 0.0f ? pitch : 0.0f)) {
        a.call_original(c);
        return;
    }
    ++g_uploads;
    float pv[3];
    if (!pivot_view(c, pv)) { ++g_rej_pivot; a.call_original(c); return; }
    g_last_pivot[0] = pv[0]; g_last_pivot[1] = pv[1]; g_last_pivot[2] = pv[2];
    float m[16], n[16];
    for (int i = 0; i < 16; ++i) m[i] = a.readf(c, wp + 4 * i);
    orbit::orbit_matrix(m, pv, g_yaw, g_yaw != 0.0f ? pitch : 0.0f, n);
    ++g_edits;
    for (int i = 0; i < 16; ++i) a.writef(c, wp + 4 * i, n[i]);
    a.call_original(c);
    for (int i = 0; i < 16; ++i) a.writef(c, wp + 4 * i, m[i]);
}

}  // namespace

extern "C" __attribute__((visibility("default"))) int gt_mod_init(const GtModApi *api, GtMod *self) {
    if (api == nullptr || api->version != GT_MOD_API_VERSION) return 1;
    g_api = api;
    g_self = self;
    g_settings = orbit::parse_settings([&](const char *k) { return api->config(self, k); },
                                       [&](const std::string &w) { api->log(self, w.c_str()); });
    orbit::configure_keys(api->config(self, "look_left_keys"), api->config(self, "look_right_keys"), api->config(self, "look_back_keys"));
    g_fake = orbit::parse_fake(std::getenv("GT_MOD_CAMERA_FAKE"));
    const char *log = std::getenv("GT_MOD_CAMERA_LOG");
    g_log = log != nullptr && *log != '\0' && std::string(log) != "0";
    if (api->hook(self, kStep, on_step) != 0) return 1;
    if (api->hook(self, kSceneRefresh, on_scene_refresh) != 0) return 1;
    if (api->hook(self, kUpload, on_upload) != 0) return 1;
    return 0;
}
