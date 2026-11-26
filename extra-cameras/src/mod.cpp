// Extra cameras (spec docs/superpowers/specs/2026-11-18-extra-cameras-design.md in gran-turismo-decomp), mod API v9, no guest address.
//
// New race views (wheel, roof, TV) selected from the cheat device (category Camera: the row "Extra view" and the action "Next extra view"), and a helicopter
// camera for replays ("Replay camera" row). Each physics step (GT_HOOK_WORLD_STEP) the human's car (live: role 1, replay: role 3) is read with car_state and
// car_dimensions; the camera function (GT_HOOK_CAMERA, once per layer of the scene renderer for the player camera) replaces the pose with camera_set.
// Left out of this version (spec): extending the game's own SELECT cycle and the replay's camera button (the game's view state is not researched), the near-plane
// adjustment, and the orbit-camera migration to these accessors.
#include "gt_mod/gt_mod_api.h"

#include "cam_logic.hpp"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>

namespace {
const GtModApi *g_api = nullptr;
GtMod *g_self = nullptr;
cams::Params g_params;
bool g_log = false;
int g_sabotage = 0;  // -DCAMS_SABOTAGE=1: the pose is never set (the offsets are ignored): the check must fail

std::atomic<int> g_view{0};          // cams::View
std::atomic<bool> g_heli{false};     // the replay helicopter camera
constexpr int kGameViews = 4, kExtraViews = 3;  // the game's race list (chase, bumper, cockpit, hood) and ours (Wheel, Roof, TV)
std::atomic<int> g_game_pos{0};      // the game's own position in its list, as last seen
std::atomic<int> g_virtual{-1};      // the SELECT cycle's virtual position (cam_logic cycle_*); -1 = not seen yet
std::atomic<unsigned> g_cycle_events{0};
constexpr float kStep = 1.0f / 60.0f;
constexpr std::uint64_t kStaleVblanks = 300;  // the physics stops while the game is paused: the last pose stays in force this long, then the game's camera returns (menus, garage)
std::uint64_t g_last_step_vblank = 0;

struct Snapshot {
    bool valid = false;
    cams::Car car;
    float front_axle = 1.3f, track = 1.5f;
    bool replay = false;
    std::uint64_t vblank = 0;
    float position[3] = {0, 0, 0};
};
std::mutex g_mutex;
Snapshot g_snap;
cams::TvCamera g_tv;
cams::Smoother g_heli_pos;
std::uint32_t g_car_handle = 0;
unsigned g_frame_logs = 0;


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
    // The world is a0: read it BEFORE call_original. With `--decomp on` the decompiled step runs inside call_original and leaves other values in a0 (measured: the snapshot was never taken, so the
    // camera function never set a pose); the recompiled build returns from call_original before the body has run, which is why it worked with `--decomp off`.
    const std::uint32_t world = g_api->gpr(ctx, 4);
    g_api->call_original(ctx);
    GtCar human;
    if (!find_human(ctx, world, &human)) return;
    GtCarState st;
    GtCarDimensions dim;
    if (g_api->car_state(g_self, ctx, human.handle, &st) != 0) return;
    const bool have_dim = g_api->car_dimensions(g_self, ctx, world, human.handle, &dim) == 0;
    const std::uint64_t vb = g_api->vblank_index(g_self);
    std::lock_guard<std::mutex> lock(g_mutex);
    for (int i = 0; i < 3; ++i)
        if (!std::isfinite(st.position[i])) return;  // a NaN would poison the smoother for good
    for (int i = 0; i < 9; ++i)
        if (!std::isfinite(st.basis[i])) return;
    if (g_last_step_vblank != 0 && vb > g_last_step_vblank + 120u) {  // the steps stopped for a while (a retry, a new race): start the points again
        g_tv.reset();
        g_heli_pos.reset();
    }
    g_last_step_vblank = vb;
    const cams::Vec3 previous = g_snap.car.position;
    g_snap.car = cams::car_from(st.position, st.basis);
    std::memcpy(g_snap.position, st.position, sizeof g_snap.position);
    if (have_dim) {
        g_snap.front_axle = dim.front_axle;
        g_snap.track = dim.track;
    }
    if (human.handle != g_car_handle || human.role == 3) {  // a new race: the points start again (a replay re-simulates, the smoother follows it)
        if (human.handle != g_car_handle || g_snap.replay != (human.role == 3)) {
            g_tv.reset();
            g_heli_pos.reset();
        }
        g_car_handle = human.handle;
    }
    if (g_snap.valid && cams::jumped(previous, g_snap.car.position, 15.0f)) {
        g_tv.reset();
        g_heli_pos.reset();
    }
    g_snap.replay = human.role == 3;
    g_snap.valid = true;
    g_snap.vblank = vb;
    g_heli_pos.step(cams::heli_target(g_snap.car, g_params), kStep, g_params.heli_smooth_s);
}

// GT_HOOK_VIEW_CYCLE: the game applying its view list (0x089274C8), which a SELECT press does after moving the list position by one (with wrap): position != previous. The mod follows the presses
// with a virtual position that has the extra views after the game's four, and changes the position the game applies: while an extra view is shown the game stays on its first view (chase).
// Not in a replay (its list is another one).
void on_view_cycle(GtModCtx *ctx) {
    std::uint32_t pos = 0, prev = 0, n = 0;
    const bool ok = g_sabotage != 2 && g_api->replay_active(g_self, ctx) == 0 &&  // check build (-DCAMS_SABOTAGE=2): the presses are not followed
                    g_api->view_cycle(g_self, ctx, &pos, &prev, &n) == 0 && static_cast<int>(n) == kGameViews;
    static unsigned applies = 0;
    if (g_log && applies < 24) {
        ++applies;
        char line[160];
        std::snprintf(line, sizeof line, "APPLY n=%u ok=%d pos=%u prev=%u count=%u ra=%08x", applies, ok ? 1 : 0, pos, prev, n, g_api->gpr(ctx, 31));
        g_api->log(g_self, line);
    }
    if (ok && pos != prev) {
        int v = g_virtual.load();
        if (v < 0 || (v < kGameViews && v != static_cast<int>(prev))) v = static_cast<int>(prev);  // first sight, or the game moved by itself: follow it
        const int dir = cams::cycle_step(static_cast<int>(prev), static_cast<int>(pos), kGameViews);
        if (dir != 0) {
            v = cams::cycle_advance(v, dir, kGameViews, kExtraViews);
            const std::uint32_t want = static_cast<std::uint32_t>(cams::cycle_game_position(v, kGameViews));
            if (want != pos && g_api->view_cycle_set(g_self, ctx, want) == 0) pos = want;
            g_virtual.store(v);
            g_view.store(static_cast<int>(cams::cycle_view(v, kGameViews)));
            g_game_pos.store(static_cast<int>(pos));
            const unsigned events = g_cycle_events.fetch_add(1) + 1;
            if (g_log) {
                char line[160];
                std::snprintf(line, sizeof line, "CYCLE press=%u dir=%d game %u->%u virtual=%d view=%s", events, dir, prev, pos, v, cams::view_name(cams::cycle_view(v, kGameViews)));
                g_api->log(g_self, line);
            }
        }
    }
    g_api->call_original(ctx);
}

// The in-game check's evidence: where the game draws the player's car, in view space (the translation row of the part's world matrix), against where the pose put it.
constexpr float kDrawPick = 2.5f;  // m: a car-like draw this close (per component) to where the pose puts the player's car is taken for it
cams::Vec3 g_expected_view{0, 0, 0};
std::atomic<bool> g_have_expected{false};
void on_mesh_draw(GtModCtx *ctx) {
    if (g_log && g_have_expected.load()) {
        static unsigned logged = 0;
        float m[12];
        if (logged < 400 && g_api->draw_matrix(g_self, ctx, m) == 0) {
            // The player's car: its parts are drawn from the model container early on and from VRAM copies later (FACTS "VRAM copies of the model parts"), so the blob alone does not tell. Every part of a car is
            // drawn with the same matrix, whose three rows are the box's half extents (0.4..3.5 m each); of those, the draw that lands within DRAW_PICK of where the pose puts the car is the player's.
            const float l0 = std::sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]), l1 = std::sqrt(m[3] * m[3] + m[4] * m[4] + m[5] * m[5]), l2 = std::sqrt(m[6] * m[6] + m[7] * m[7] + m[8] * m[8]);
            const bool car_like = l0 > 0.4f && l0 < 3.5f && l1 > 0.4f && l1 < 3.5f && l2 > 0.4f && l2 < 3.5f;
            const bool near_pose = std::fabs(m[9] - g_expected_view.x) <= kDrawPick && std::fabs(m[10] - g_expected_view.y) <= kDrawPick && std::fabs(m[11] - g_expected_view.z) <= kDrawPick;
            if (car_like && near_pose) {
                ++logged;
                std::fprintf(stderr, "[mod:extra-cameras] DRAW seen=(%.3f %.3f %.3f) expected=(%.3f %.3f %.3f)\n", m[9], m[10], m[11], g_expected_view.x, g_expected_view.y, g_expected_view.z);
            }
        }
    }
    g_api->call_original(ctx);
}

// GT_MOD_CAMERA_DEBUG=1: why the camera function did or did not set a pose (diagnosis of `--decomp on`).
struct CamStats {
    unsigned calls = 0, invalid = 0, stale = 0, replay = 0, off = 0, set_failed = 0, set_ok = 0;
};
CamStats g_stats;
void print_stats(std::uint64_t now, std::uint64_t snap_vb) {
    if (g_log && std::getenv("GT_MOD_CAMERA_DEBUG") != nullptr && g_stats.calls % 1200 == 0)
        std::fprintf(stderr, "[mod:extra-cameras] CAMDBG calls=%u invalid=%u stale=%u replay=%u off=%u set_failed=%u set_ok=%u now=%llu snap_vb=%llu view=%d\n", g_stats.calls, g_stats.invalid, g_stats.stale,
                     g_stats.replay, g_stats.off, g_stats.set_failed, g_stats.set_ok, static_cast<unsigned long long>(now), static_cast<unsigned long long>(snap_vb), g_view.load());
}

void on_camera(GtModCtx *ctx) {  // GT_HOOK_CAMERA: never call_original here (the host does)
    Snapshot snap;
    cams::Pose pose;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        snap = g_snap;
    }
    const std::uint64_t now = g_api->vblank_index(g_self);
    ++g_stats.calls;
    print_stats(now, snap.vblank);
    if (!snap.valid) { ++g_stats.invalid; return; }
    if (now > snap.vblank + kStaleVblanks) { ++g_stats.stale; return; }  // no race running (a pause shorter than ~5 s keeps the view): the game's own camera
    const bool replay = g_api->replay_active(g_self, ctx) != 0;
    const cams::View view = static_cast<cams::View>(g_view.load());
    const char *name;
    if (replay) {
        ++g_stats.replay;
        if (!g_heli.load()) return;
        std::lock_guard<std::mutex> lock(g_mutex);
        if (!g_heli_pos.has_value()) return;
        pose = cams::look_at(g_heli_pos.step(cams::heli_target(snap.car, g_params), 0.0f, g_params.heli_smooth_s), snap.car.position, snap.car.forward);
        name = "heli";
    } else if (view == cams::View::Wheel) {
        pose = cams::wheel_pose(snap.car, snap.front_axle, snap.track, g_params);
        name = "wheel";
    } else if (view == cams::View::Roof) {
        pose = cams::roof_pose(snap.car, g_params);
        name = "roof";
    } else if (view == cams::View::Tv) {
        std::lock_guard<std::mutex> lock(g_mutex);
        pose = g_tv.pose(snap.car, g_params);
        name = "tv";
    } else {
        ++g_stats.off;
        return;
    }
    float eye[3] = {pose.eye.x, pose.eye.y, pose.eye.z};
    // check build (-DCAMS_SABOTAGE=1): the offset is ignored, i.e. the pose is computed and logged but never handed to the game
    if (g_sabotage != 1 && g_api->camera_set(g_self, ctx, eye, pose.basis) != 0) {
        ++g_stats.set_failed;
        return;
    }
    ++g_stats.set_ok;
    if (g_log) {
        // the car's body origin in view space as this pose sees it (OpenGL view space: x right, y up, -z forward)
        const cams::Vec3 d = snap.car.position - cams::Vec3{eye[0], eye[1], eye[2]};
        const cams::Vec3 f{pose.basis[0], pose.basis[1], pose.basis[2]}, r{pose.basis[3], pose.basis[4], pose.basis[5]}, u{pose.basis[6], pose.basis[7], pose.basis[8]};
        g_expected_view = {cams::dot(r, d), cams::dot(u, d), -cams::dot(f, d)};
        g_have_expected.store(true);
        static std::uint64_t last_logged = 0;
        if (now != last_logged && g_frame_logs < 4000) {
            last_logged = now;
            ++g_frame_logs;
            std::fprintf(stderr, "[mod:extra-cameras] CAM vb=%llu view=%s replay=%d eye=%.3f,%.3f,%.3f fwd=%.3f,%.3f,%.3f car=%.3f,%.3f,%.3f carfwd=%.3f,%.3f,%.3f up=%.3f\n",
                         static_cast<unsigned long long>(now), name, replay ? 1 : 0, eye[0], eye[1], eye[2], pose.basis[0], pose.basis[1], pose.basis[2], snap.car.position.x, snap.car.position.y,
                         snap.car.position.z, snap.car.forward.x, snap.car.forward.y, snap.car.forward.z, pose.basis[8]);
        }
    }
}

// Menu (any thread): atomics only.
void view_label(GtMod *, char *out, std::uint32_t capacity) { std::snprintf(out, capacity, "Extra view: %s", cams::view_name(static_cast<cams::View>(g_view.load()))); }
void view_activate(GtMod *, int direction) {
    const cams::View next = cams::next_view(static_cast<cams::View>(g_view.load()), direction < 0 ? -1 : 1);
    g_view.store(static_cast<int>(next));
    g_virtual.store(cams::cycle_virtual(next, kGameViews, g_game_pos.load()));  // the SELECT cycle continues from here
}
void heli_label(GtMod *, char *out, std::uint32_t capacity) { std::snprintf(out, capacity, "Replay camera: %s", g_heli.load() ? "Helicopter" : "Game"); }
void heli_activate(GtMod *, int) { g_heli.store(!g_heli.load()); }
void next_view_action(GtMod *, GtModCtx *) { view_activate(nullptr, 1); }

cams::View parse_view(const char *text) {
    if (text == nullptr) return cams::View::Off;
    const std::string s(text);
    if (s == "wheel") return cams::View::Wheel;
    if (s == "roof") return cams::View::Roof;
    if (s == "tv") return cams::View::Tv;
    return cams::View::Off;
}
}  // namespace

extern "C" __attribute__((visibility("default"))) int gt_mod_init(const GtModApi *api, GtMod *self) {
    if (api->version < GT_MOD_API_VERSION_9) return 1;
    g_api = api;
    g_self = self;
#if defined(CAMS_SABOTAGE)
    g_sabotage = CAMS_SABOTAGE;
#endif
    g_params.wheel_up = setting("wheel_up", g_params.wheel_up);
    g_params.wheel_out = setting("wheel_out", g_params.wheel_out);
    g_params.wheel_toe_deg = setting("wheel_toe_deg", g_params.wheel_toe_deg);
    g_params.roof_up = setting("roof_up", g_params.roof_up);
    g_params.roof_forward = setting("roof_forward", g_params.roof_forward);
    g_params.roof_pitch_deg = setting("roof_pitch_deg", g_params.roof_pitch_deg);
    g_params.tv_ahead = setting("tv_ahead", g_params.tv_ahead);
    g_params.tv_side = setting("tv_side", g_params.tv_side);
    g_params.tv_height = setting("tv_height", g_params.tv_height);
    g_params.tv_pass = setting("tv_pass", g_params.tv_pass);
    g_params.heli_back = setting("heli_back", g_params.heli_back);
    g_params.heli_up = setting("heli_up", g_params.heli_up);
    g_params.heli_smooth_s = setting("heli_smooth_s", g_params.heli_smooth_s);
    g_log = std::getenv("GT_MOD_CAMS_LOG") != nullptr;
    const char *forced = std::getenv("GT_MOD_CAMS_VIEW");  // the checks' choice (there is no keyboard): wheel | roof | tv | heli
    if (forced != nullptr && std::string(forced) == "heli") g_heli.store(true);
    else g_view.store(static_cast<int>(parse_view(forced != nullptr ? forced : api->config(self, "start_view"))));
    g_virtual.store(g_view.load() == 0 ? -1 : cams::cycle_virtual(static_cast<cams::View>(g_view.load()), kGameViews, 0));
    if (api->hook_point(self, GT_HOOK_WORLD_STEP, on_step) != 0) return 1;
    if (api->hook_point(self, GT_HOOK_CAMERA, on_camera) != 0) return 1;
    if (api->hook_point(self, GT_HOOK_VIEW_CYCLE, on_view_cycle) != 0) return 1;
    if (g_log) {
        if (api->hook_point(self, GT_HOOK_MESH_DRAW, on_mesh_draw) != 0) return 1;
    }
    if (api->add_entry(self, "Camera", "extra_view", view_label, view_activate) != 0) return 1;
    if (api->add_entry(self, "Camera", "replay_camera", heli_label, heli_activate) != 0) return 1;
    if (api->add_action(self, "next_view", "Next extra view", "Camera", next_view_action, "", "") != 0) return 1;
    return 0;
}
