// The realistic style's glue (src/realistic.cpp) against a fake game behind a fake GtModApi: a car on a flat road, a "game step" that only integrates the
// velocity, the mod's hooks called as the host would. Checks the takeover on a hit, that the car tumbles and comes to rest or is handed back, that no hull
// point sinks, and that quiet driving writes nothing.
#include "realistic.hpp"

#include "physics_core.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

static int g_fail = 0;
#define CHECK(c)                                                    \
    do {                                                            \
        if (!(c)) {                                                 \
            std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); \
            ++g_fail;                                               \
        }                                                           \
    } while (0)

struct GtModCtx { int unused; };
struct GtMod { int unused; };

namespace {
struct Game {
    GtCarState s{};
    int pose_writes = 0, tilt_on = 0, tilt_calls = 0;
    float ground_h = 0.0f;
    std::vector<std::string> logs;
    GtHookFn step_hook = nullptr;
    bool wheels_loaded = true, no_car = false;
    float extra_dv_up = 0.0f;  // what grounded wheel records make the game's step add along the ground normal
    int role = 1;
} G;

void set_euler_identity(GtCarState &s) {
    const float rows[9] = {1, 0, 0, 0, -1, 0, 0, 0, 1};  // forward +x, right -y, up +z (left-handed)
    std::memcpy(s.basis, rows, sizeof rows);
}

int hook_point(GtMod *, GtHookPoint p, GtHookFn fn) {
    if (p == GT_HOOK_WORLD_STEP) G.step_hook = fn;
    return 0;
}
void call_original(GtModCtx *) {}
std::uint32_t gpr(GtModCtx *, unsigned) { return 0x09000000u; }
std::uint32_t race_cars(GtMod *, GtModCtx *, std::uint32_t, GtCar *out, std::uint32_t) {
    if (G.no_car) return 0;
    out[0] = GtCar{0x09300000u, 0, G.role}; return 1; }
int car_state(GtMod *, GtModCtx *, std::uint32_t, GtCarState *out) { *out = G.s; return 0; }
int car_dimensions(GtMod *, GtModCtx *, std::uint32_t, std::uint32_t, GtCarDimensions *out) { *out = GtCarDimensions{2.442f, 1.434f, 0.879f, -1.563f, 1.783f}; return 0; }
int car_wheel_contacts(GtMod *, GtModCtx *, std::uint32_t, std::uint32_t, GtWheelContact out[4]) {
    for (int i = 0; i < 4; ++i)
        out[i] = GtWheelContact{G.s.position[0] + (i < 2 ? 0.9f : -1.5f), G.s.position[1] + (i % 2 ? 0.7f : -0.7f), G.ground_h, 0, 1, G.wheels_loaded ? 3000.0f : 0.0f};
    return 0;
}
int car_set_pose(GtMod *, GtModCtx *, std::uint32_t, const float p[3], const float b[9], const float v[3], const float *) {
    ++G.pose_writes;
    for (int i = 0; i < 3; ++i) { G.s.position[i] = p[i]; G.s.velocity[i] = v[i]; }
    std::memcpy(G.s.basis, b, 9 * sizeof(float));
    G.s.speed = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    return 0;
}
int tilt_free(GtMod *, GtModCtx *, std::uint32_t, int on) { G.tilt_on = on; ++G.tilt_calls; return 0; }
void logf_(GtMod *, const char *m) { G.logs.push_back(m); }

GtModApi make_api() {
    GtModApi a{};
    a.version = 7;
    a.hook_point = hook_point;
    a.call_original = call_original;
    a.gpr = gpr;
    a.race_cars = race_cars;
    a.car_state = car_state;
    a.car_dimensions = car_dimensions;
    a.car_wheel_contacts = car_wheel_contacts;
    a.car_set_pose = car_set_pose;
    a.tilt_free = tilt_free;
    a.log = logf_;
    return a;
}

// The game's step on whatever pose it finds: position += velocity * dt, gravity only when the car is not "on its wheels" (the wheels carry it on a level car)
void game_step() {
    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 3; ++i) G.s.position[i] += G.s.velocity[i] * dt;
    if (G.s.basis[8] < 0.9f) G.s.velocity[2] -= 9.81f * dt;  // tilted: the wheels no longer hold it up
    G.s.velocity[2] += G.extra_dv_up;
}

bool has_log(const char *needle) {
    for (const std::string &l : G.logs) if (l.find(needle) != std::string::npos) return true;
    return false;
}
}  // namespace


static int count_logs(const char *needle) {
    int n = 0;
    for (const std::string &l : G.logs) n += l.find(needle) != std::string::npos ? 1 : 0;
    return n;
}

// Starts a simulation by a hit and returns; the car is then at rest-ish on its side or wheels.
static void start_hit(GtModCtx *ctx, const GtModApi &api, GtMod &mod, const rollover::Settings &set) {
    G.no_car = true;
    if (G.step_hook) G.step_hook(ctx);
    const int role = G.role;
    G = Game{};
    G.role = role;
    realistic::init(&api, &mod, set, true);
    G.s.position[0] = 100; G.s.position[1] = 50; G.s.position[2] = 0.40f;
    set_euler_identity(G.s);
    G.s.velocity[0] = 30.0f;
    G.s.speed = 30.0f;
    for (int i = 0; i < 40; ++i) { G.step_hook(ctx); game_step(); }
    G.s.velocity[0] -= 14.0f;
    G.s.velocity[1] += 8.3f;
}

static void test_review_cases(GtModCtx *ctx, const GtModApi &api, GtMod &mod, const rollover::Settings &set) {
    // 6: a vertical push from grounded wheel records while simulating is not a wall: no impulse
    start_hit(ctx, api, mod, set);
    G.extra_dv_up = 0.0f;
    for (int i = 0; i < 20; ++i) { G.step_hook(ctx); game_step(); }
    const int hits_before = count_logs("(in the simulation)");
    G.extra_dv_up = 3.0f;  // a "landing" every step, along the ground normal
    for (int i = 0; i < 10; ++i) { G.step_hook(ctx); game_step(); }
    CHECK(count_logs("(in the simulation)") == hits_before);
    // a push ALONG the ground (a wall) is an impulse
    G.extra_dv_up = 0.0f;
    G.s.velocity[0] += 4.0f;
    G.step_hook(ctx);
    CHECK(count_logs("(in the simulation)") > hits_before);
    // 5: the game turned the car (reset-to-track at the same place): up row far from the written one: the car goes back to the game
    const int writes = G.pose_writes;
    const float rows[9] = {0, 1, 0, 1, 0, 0, 0, 0, -1};  // upside down
    std::memcpy(G.s.basis, rows, sizeof rows);
    G.step_hook(ctx);
    CHECK(G.tilt_on == 0 && count_logs("RESET") == 1 && G.pose_writes == writes);
    // 7: live and replay agree: the same hit on the same state gives the same simulated poses
    std::vector<std::string> live, replay;
    start_hit(ctx, api, mod, set);
    for (int i = 0; i < 80; ++i) { G.step_hook(ctx); game_step(); }
    for (const std::string &l : G.logs) if (l.find("SIM ") != std::string::npos) live.push_back(l.substr(l.find("pos=")));
    G.role = 3;
    start_hit(ctx, api, mod, set);
    for (int i = 0; i < 80; ++i) { G.step_hook(ctx); game_step(); }
    for (const std::string &l : G.logs) if (l.find("SIM ") != std::string::npos) replay.push_back(l.substr(l.find("pos=")));
    CHECK(!live.empty() && live.size() == replay.size());
    bool same = live.size() == replay.size();
    for (std::size_t i = 0; same && i < live.size(); ++i) same = live[i].substr(0, live[i].find(" kd=")) == replay[i].substr(0, replay[i].find(" kd="));
    CHECK(same);
    G.role = 1;
}

int main() {
    GtModApi api = make_api();
    GtMod mod;
    rollover::Settings set;
    CHECK(realistic::init(&api, &mod, set, true) == 0 && G.step_hook != nullptr);
    GtModCtx ctx{};
    G = Game{};
    // (G was reset: re-register through init once more with the same fake)
    CHECK(realistic::init(&api, &mod, set, true) == 0);
    G.s.position[0] = 100; G.s.position[1] = 50; G.s.position[2] = 0.40f;
    set_euler_identity(G.s);
    G.s.velocity[0] = 30.0f;
    G.s.speed = 30.0f;
    // quiet driving: 300 steps, nothing written
    for (int i = 0; i < 300; ++i) { G.step_hook(&ctx); game_step(); }
    CHECK(G.pose_writes == 0 && G.tilt_calls == 0);
    // a barrier hit: -8.3 m/s along the car's right row (-y here: right = -y), 14 m/s of braking, in ONE step
    G.s.velocity[0] -= 14.0f;
    G.s.velocity[1] += 8.3f;  // pushed toward +y = the car's left, away from a wall on its right
    int steps = 0;
    bool handed = false;
    float lowest_com = 1e9f;
    for (; steps < 1500; ++steps) {
        G.step_hook(&ctx);
        if (G.tilt_calls > 0 && G.tilt_on == 0) { handed = true; }
        game_step();
        lowest_com = std::fmin(lowest_com, G.s.position[2]);
        if (handed) break;
        if (steps > 700) break;
    }
    CHECK(has_log("TAKEOVER"));
    CHECK(G.pose_writes > 50);
    CHECK(G.tilt_calls >= 1);
    for (const std::string &l : G.logs) CHECK(l.find("nan") == std::string::npos);
    // the car never sank through the road: the lowest hull point stays near the ground (SIM lines: lowest=)
    float worst = 1e9f;
    for (const std::string &l : G.logs) {
        const std::size_t at = l.find("lowest=");
        if (l.find("SIM ") != std::string::npos && at != std::string::npos) worst = std::fmin(worst, std::strtof(l.c_str() + at + 7, nullptr));
    }
    float min_upz = 1.0f, max_w = 0.0f;
    for (const std::string &l : G.logs) {
        const std::size_t u = l.find("up.z=");
        const std::size_t w = l.find(" w=");
        if (l.find("SIM ") == std::string::npos || u == std::string::npos) continue;
        min_upz = std::fmin(min_upz, std::strtof(l.c_str() + u + 5, nullptr));
        float wx = 0, wy = 0, wz = 0;
        std::sscanf(l.c_str() + w, " w=%f,%f,%f", &wx, &wy, &wz);
        max_w = std::fmax(max_w, std::sqrt(wx * wx + wy * wy + wz * wz));
    }
    std::printf("steps=%d handed=%d worst_lowest=%.3f pose_writes=%d min_up.z=%.2f max_w=%.2f\n", steps, handed ? 1 : 0, worst, G.pose_writes, min_upz, max_w);
    CHECK(worst > -0.05f);
    // ---- a drop of 2 m (the check's GT_MOD_ROLLOVER_DROP): bounce within 1.2 e^2 h, slides to rest, handed back ----
    {
        setenv("GT_MOD_ROLLOVER_DROP", "m5:2.0", 1);
        G.no_car = true;
        G.step_hook(&ctx);  // no player car (the race ended): the mod forgets the first scenario
        G = Game{};
        CHECK(realistic::init(&api, &mod, set, true) == 0);
        G.s.position[0] = 100; G.s.position[1] = 50; G.s.position[2] = 0.40f;
        set_euler_identity(G.s);
        G.s.velocity[0] = 4.0f;
        G.s.speed = 4.0f;
        bool handed2 = false;
        int n = 0;
        for (; n < 1200 && !handed2; ++n) {
            G.step_hook(&ctx);
            if (G.tilt_calls > 1 && G.tilt_on == 0) handed2 = true;
            game_step();
        }
        std::vector<float> lowest;
        for (const std::string &l : G.logs) {
            const std::size_t at = l.find("lowest=");
            if (l.find("SIM ") != std::string::npos && at != std::string::npos && l.find("n=") != std::string::npos) lowest.push_back(std::strtof(l.c_str() + at + 7, nullptr));
        }
        CHECK(!lowest.empty() && lowest.front() > 1.8f);
        std::size_t hit = 0;
        while (hit < lowest.size() && lowest[hit] > 0.02f) ++hit;
        float apex = 0, prev = lowest.empty() ? 0 : lowest[hit < lowest.size() ? hit : 0];
        bool rising = false;
        for (std::size_t i = hit + 1; i < lowest.size(); ++i) {
            if (lowest[i] > prev + 1e-4f) rising = true;
            if (rising) {
                if (lowest[i] < prev - 1e-4f) break;
                apex = std::fmax(apex, lowest[i]);
            }
            prev = lowest[i];
        }
        const float e2h = 0.15f * 0.15f * lowest.front();
        std::printf("drop: handed=%d steps=%d h0=%.2f first rebound apex=%.4f (1.2 e^2 h = %.4f)\n", handed2 ? 1 : 0, n, lowest.front(), apex, 1.2f * e2h);
        CHECK(handed2);
        CHECK(apex <= 1.2f * e2h && apex >= 0.5f * e2h);
    }
    test_review_cases(&ctx, api, mod, set);
    if (g_fail == 0) std::printf("realistic_test: all passed\n");
    return g_fail == 0 ? 0 : 1;
}
