// Best-lap ghost in any mode (spec docs/superpowers/specs/2026-11-18-ghost-any-mode-design.md in gran-turismo-decomp), mod API v9, no guest address.
//
// Each physics step the human's car (live, or in its replay) is recorded; a lap ends when the car crosses its start line (ghost_logic LapLine: the point where it first moved, crossed forward
// after being away: the game's own lap event was not found, FACTS "Lap line"); the best lap (fewest steps) is kept and, on the next laps, drawn as a translucent ghost at the pose the car had at the same
// lap time. DEVIATION FROM THE SPEC: the ghost is a box with the car's size drawn as overlay lines (draw_line) projected with the player camera (camera_get + camera_projection in a camera
// function), NOT the car's model: `draw_car_pose` (the extra car pass) is not in API 8 (#667 merged without it: FACTS "Car draw replay: WORKS" is a throwaway probe). The ghost is visual only.
// Settings: show, persist (best laps saved per start line and car under <GT_MODS_DIR>/ghost-any-mode/), the box size. The cheat device's Race page has a row and a "clear" action.
//
// GHOST_SABOTAGE (build flag, in-game check only): 1 = the ghost is drawn at the CURRENT pose instead of the recorded one (the "ghost lags the player" check must fail).
#include "gt_mod/gt_mod_api.h"

#include "ghost_logic.hpp"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>

#ifndef GHOST_SABOTAGE
#define GHOST_SABOTAGE 0
#endif

namespace {
const GtModApi *g_api = nullptr;
GtMod *g_self = nullptr;
bool g_log = false, g_persist = false;
ghost::Box g_box;
std::string g_dir;

std::mutex g_mutex;  // guards everything below the line (the step hook writes, the overlay and the camera function read)
ghost::LapLine g_line;
ghost::Recorder g_rec;
ghost::Pose g_ghost, g_player;
bool g_ghost_valid = false;
ghost::Camera g_camera;
std::uint64_t g_camera_vblank = 0, g_last_step_vblank = 0, g_steps = 0;
std::uint32_t g_car = 0;
int g_role = 0;
bool g_gap_pending = false;
float g_last_position[3] = {0, 0, 0};
long g_moved_at = -1;
long g_fake_every = -1;  // GT_MOD_GHOST_FAKE_LAP=m<steps>: a lap boundary every that many steps after the car first moves (the checks cannot drive a lap)
float g_wheelbase = 2.5f;
std::string g_loaded_file;

std::atomic<bool> g_show{true};
std::atomic<std::uint32_t> g_best_steps{0};
std::atomic<bool> g_clear{false};
constexpr std::uint64_t kLiveGap = 30, kResetGap = 120;
constexpr float kRestartJump = 15.f;

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

void format_time(std::uint32_t steps, char *out, std::size_t n) {
    const double s = steps / 60.0;
    std::snprintf(out, n, "%d'%05.2f", static_cast<int>(s / 60.0), std::fmod(s, 60.0));
}

std::string lap_path() {
    if (g_dir.empty() || !g_line.has_line()) return std::string();
    return g_dir + "/ghost-any-mode/" + ghost::lap_file_name(g_line.origin(), g_line.heading(), g_wheelbase);
}

void load_best() {  // g_mutex held
    const std::string path = lap_path();
    if (!g_persist || path.empty() || path == g_loaded_file || g_rec.has_best()) return;
    g_loaded_file = path;
    std::ifstream in(path, std::ios::binary);
    if (!in) return;
    std::stringstream ss;
    ss << in.rdbuf();
    ghost::Lap lap;
    if (ghost::parse_lap(ss.str(), &lap)) {
        g_rec.set_best(std::move(lap));
        g_best_steps.store(g_rec.best().steps());
        if (g_log) {
            char line[160];
            std::snprintf(line, sizeof line, "LOADED %s steps=%u", path.c_str(), g_rec.best().steps());
            g_api->log(g_self, line);
        }
    }
}

void save_best() {  // g_mutex held
    const std::string path = lap_path();
    if (!g_persist || path.empty()) return;
    std::error_code ec;
    std::filesystem::create_directories(g_dir + "/ghost-any-mode", ec);
    std::ofstream out(path + ".tmp", std::ios::binary | std::ios::trunc);
    if (!out) return;
    const std::string bytes = ghost::serialize_lap(g_rec.best());
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    out.close();
    std::filesystem::rename(path + ".tmp", path, ec);
}

void on_step(GtModCtx *ctx) {
    g_api->call_original(ctx);
    const std::uint32_t world = g_api->gpr(ctx, 4);
    GtCar human;
    if (!find_human(ctx, world, &human)) return;
    GtCarState st;
    if (g_api->car_state(g_self, ctx, human.handle, &st) != 0) return;
    for (int i = 0; i < 3; ++i)
        if (!std::isfinite(st.position[i]) || !std::isfinite(st.velocity[i])) return;
    GtCarDimensions dim;
    const bool have_dim = g_api->car_dimensions(g_self, ctx, world, human.handle, &dim) == 0;
    const std::uint64_t vb = g_api->vblank_index(g_self);
    std::lock_guard<std::mutex> lock(g_mutex);
    if (have_dim && dim.wheelbase > 0.5f) g_wheelbase = dim.wheelbase;
    if (g_clear.exchange(false)) {
        g_rec.clear_all();
        g_best_steps.store(0);
        g_ghost_valid = false;
    }
    // a new race (the car changed, or the steps stopped and the car was put back at the start): the line and the current lap start again; the best lap of the session stays
    if (g_last_step_vblank != 0 && vb > g_last_step_vblank + kResetGap) g_gap_pending = true;
    bool restart = human.handle != g_car || human.role != g_role;
    if (g_gap_pending) {
        g_gap_pending = false;
        const float dx = st.position[0] - g_last_position[0], dy = st.position[1] - g_last_position[1], dz = st.position[2] - g_last_position[2];
        if (std::sqrt(dx * dx + dy * dy + dz * dz) > kRestartJump) restart = true;
    }
    if (restart) {
        g_line.reset();
        g_rec.clear_current();
        g_moved_at = -1;
        g_steps = 0;
        g_car = human.handle;
        g_role = human.role;
        g_ghost_valid = false;
    }
    g_last_step_vblank = vb;
    std::memcpy(g_last_position, st.position, sizeof g_last_position);
    ++g_steps;
    ghost::Pose pose;
    std::memcpy(pose.position, st.position, sizeof pose.position);
    std::memcpy(pose.basis, st.basis, sizeof pose.basis);
    g_player = pose;
    g_rec.push(pose);
    const bool had_line = g_line.has_line();
    bool crossed = g_line.step(st.position, st.velocity);
    if (!had_line && g_line.has_line()) {
        g_moved_at = static_cast<long>(g_steps);
        g_rec.clear_current();  // the lap starts where the car first moved
        g_rec.push(pose);
        load_best();
    }
    if (g_fake_every > 0 && g_moved_at >= 0 && static_cast<long>(g_steps) > g_moved_at && (static_cast<long>(g_steps) - g_moved_at) % g_fake_every == 0) crossed = true;
    if (crossed) {
        const std::uint32_t steps = g_rec.current_steps();
        const bool better = g_rec.lap_done(g_fake_every > 0 ? 30u : 300u);
        if (better) {
            g_best_steps.store(g_rec.best().steps());
            save_best();
        }
        if (g_log) {
            char line[160];
            std::snprintf(line, sizeof line, "LAP n=%u steps=%u best=%u new_best=%d", g_rec.laps(), steps, g_rec.best().steps(), better ? 1 : 0);
            g_api->log(g_self, line);
        }
    }
    // the ghost: the best lap's pose at this lap time (laps after the best one; the lap that set it is not shown against itself)
    const std::uint32_t lap_step = g_rec.current_steps() > 0 ? g_rec.current_steps() - 1 : 0;
    ghost::Pose gp;
    g_ghost_valid = g_rec.laps() >= 1 && ghost::ghost_pose(g_rec.best(), lap_step, &gp);
    if (g_ghost_valid) {
#if GHOST_SABOTAGE == 1
        gp = pose;
#endif
        g_ghost = gp;
        if (g_log && lap_step % 10 == 0) {
            char line[220];
            std::snprintf(line, sizeof line, "GHOST lapstep=%u pose=%.3f,%.3f,%.3f player=%.3f,%.3f,%.3f", lap_step, gp.position[0], gp.position[1], gp.position[2], pose.position[0], pose.position[1],
                          pose.position[2]);
            g_api->log(g_self, line);
        }
    }
    if (g_log && g_rec.laps() == 0 && g_rec.current_steps() % 10 == 1) {
        char line[160];
        std::snprintf(line, sizeof line, "REC lapstep=%u pos=%.3f,%.3f,%.3f", g_rec.current_steps() - 1, pose.position[0], pose.position[1], pose.position[2]);
        g_api->log(g_self, line);
    }
}

// GT_HOOK_CAMERA: the player camera of this frame (never changes it).
void on_camera(GtModCtx *ctx) {
    ghost::Camera cam;
    if (g_api->camera_get(g_self, ctx, cam.eye, cam.basis) != 0) return;
    if (g_api->camera_projection(g_self, ctx, &cam.focal_x, &cam.focal_y) != 0) return;
    std::lock_guard<std::mutex> lock(g_mutex);
    g_camera = cam;
    g_camera_vblank = g_api->vblank_index(g_self);
}

void on_overlay(GtMod *, GtModCtx *) {
    if (!g_show.load()) return;
    ghost::Pose pose;
    ghost::Camera cam;
    std::uint64_t cam_vb, last_vb;
    std::uint32_t best;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (!g_ghost_valid) return;
        pose = g_ghost;
        cam = g_camera;
        cam_vb = g_camera_vblank;
        last_vb = g_last_step_vblank;
        best = g_rec.best().steps();
    }
    const std::uint64_t now = g_api->vblank_index(g_self);
    if (now > last_vb + kLiveGap || cam_vb == 0 || now > cam_vb + 3 || cam.focal_x <= 0.f) return;  // no race on screen, or no player camera this frame
    float c[8][3], px[8], py[8];
    bool ok[8];
    ghost::box_corners(pose, g_box, c);
    float minx = 1e9f, miny = 1e9f, maxx = -1e9f, maxy = -1e9f;
    for (int i = 0; i < 8; ++i) {
        ok[i] = ghost::project(cam, c[i], &px[i], &py[i]);
        if (ok[i]) {
            minx = std::fmin(minx, px[i]);
            maxx = std::fmax(maxx, px[i]);
            miny = std::fmin(miny, py[i]);
            maxy = std::fmax(maxy, py[i]);
        }
    }
    for (const auto &e : ghost::kBoxEdges)
        if (ok[e[0]] && ok[e[1]]) g_api->draw_line(g_self, px[e[0]], py[e[0]], px[e[1]], py[e[1]], 1.4f, 0x66E0FF90u);
    if (ok[0] && ok[1]) {  // the lap time of the ghost above its front edge
        char text[24];
        format_time(best, text, sizeof text);
        g_api->draw_text(g_self, 0.5f * (px[0] + px[1]) - 14.f, 0.5f * (py[0] + py[1]) - 14.f, 8.f, 0x66E0FFB0u, text);
    }
    if (g_log && now % 30 == 0) {
        char line[200];
        std::snprintf(line, sizeof line, "BOX vb=%llu bbox=%.1f,%.1f,%.1f,%.1f corners=%d", static_cast<unsigned long long>(now), minx, miny, maxx, maxy, static_cast<int>(ok[0]) + ok[1] + ok[2] + ok[3] + ok[4] + ok[5] + ok[6] + ok[7]);
        g_api->log(g_self, line);
    }
}

// Menu (any thread): atomics only.
void show_label(GtMod *, char *out, std::uint32_t capacity) {
    const std::uint32_t best = g_best_steps.load();
    char t[24] = "-";
    if (best > 0) format_time(best, t, sizeof t);
    std::snprintf(out, capacity, "Ghost: %s (best lap %s)", g_show.load() ? "ON" : "OFF", t);
}
void show_activate(GtMod *, int) { g_show.store(!g_show.load()); }
void clear_action(GtMod *, GtModCtx *) { g_clear.store(true); }
}  // namespace

extern "C" __attribute__((visibility("default"))) int gt_mod_init(const GtModApi *api, GtMod *self) {
    if (api->version < GT_MOD_API_VERSION_9) return 1;
    g_api = api;
    g_self = self;
    g_log = std::getenv("GT_MOD_GHOST_LOG") != nullptr;
    const char *show = api->config(self, "show");
    g_show.store(show == nullptr || std::strcmp(show, "false") != 0);
    if (const char *hide = std::getenv("GT_MOD_GHOST_HIDE")) g_show.store(std::strcmp(hide, "1") != 0);
    const char *persist = api->config(self, "persist");
    g_persist = persist != nullptr && std::strcmp(persist, "true") == 0;
    if (const char *dir = std::getenv("GT_MODS_DIR")) g_dir = dir;
    g_box.length = setting("box_length", g_box.length);
    g_box.width = setting("box_width", g_box.width);
    g_box.height = setting("box_height", g_box.height);
    g_box.lift = setting("box_lift", g_box.lift);
    if (const char *fake = std::getenv("GT_MOD_GHOST_FAKE_LAP")) {
        if (*fake == 'm') g_fake_every = std::atol(fake + 1);
    }
    if (api->hook_point(self, GT_HOOK_WORLD_STEP, on_step) != 0) return 1;
    if (api->hook_point(self, GT_HOOK_CAMERA, on_camera) != 0) return 1;
    if (api->on_overlay(self, on_overlay) != 0) return 1;
    if (api->add_entry(self, "Race", "ghost", show_label, show_activate) != 0) return 1;
    if (api->add_action(self, "clear", "Clear ghost lap", "Race", clear_action, "", "") != 0) return 1;
    return 0;
}
