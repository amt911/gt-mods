#include "realistic.hpp"

#include "physics_core.hpp"
#include "takeover.hpp"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace realistic {
namespace {

using namespace rollphys;

constexpr double kDt = 1.0 / 60.0;     // the game's physics step (FACTS "Car body state": 60 per second of game time)
constexpr float kDefaultRest = 0.35f;  // m: the body origin's height over the road on its wheels at speed when the wheel records cannot say (measured 2026-11-18)

const GtModApi *g_api = nullptr;
GtMod *g_self = nullptr;
rollover::Settings g_set;
bool g_log = false;

struct Fake {  // measurement aids for the in-game checks: GT_MOD_ROLLOVER_FAKE=m<steps>:<dv> (a hit), GT_MOD_ROLLOVER_DROP=m<steps>:<height> (a drop)
    long hit_step = -1;
    float hit_dv = 0;
    long drop_step = -1;
    float drop_height = 0;
} g_fake;

struct State {
    std::uint32_t car = 0, world = 0;
    int role = 0;
    unsigned steps = 0;
    long moved_at = -1;
    bool have_prev = false;
    float prev_pos[3] = {0, 0, 0}, prev_vel[3] = {0, 0, 0}, prev_rows[9] = {1, 0, 0, 0, -1, 0, 0, 0, 1};
    // the simulation
    bool sized = false;
    Hull hull;
    Params params;
    Body body;
    PlaneGround ground;
    bool have_ground_slope = false;
    float rest_height = kDefaultRest;
    float wrote_pos[3] = {0, 0, 0}, wrote_vel[3] = {0, 0, 0}, wrote_rows[9] = {1, 0, 0, 0, -1, 0, 0, 0, 1};
    bool wrote = false;
    unsigned sim_steps = 0, takeovers = 0, sim_hits = 0;
    double max_up = 0;
};
State g;
Supervisor *g_sup = nullptr;

void reset_state() {
    if (g_sup != nullptr) g_sup->reset();
    g = State{};
}

void logf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void logf(const char *fmt, ...) {
    char line[420];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);
    g_api->log(g_self, line);
}

// The player's car, live or in its replay (a replay re-simulates the race: the mod must act there as it did live). Exactly one, else none.
bool find_car(GtModCtx *c, std::uint32_t world, std::uint32_t *car, int *role) {
    GtCar cars[32];
    const std::uint32_t n = g_api->race_cars(g_self, c, world, cars, 32);
    int humans = 0;
    for (std::uint32_t i = 0; i < n && i < 32; ++i)
        if (cars[i].role == 1 || cars[i].role == 3) {
            ++humans;
            *car = cars[i].handle;
            *role = cars[i].role;
        }
    return humans == 1;
}

V3 v3(const float a[3]) { return {a[0], a[1], a[2]}; }
double up_z_of(const float rows[9]) { return rows[8]; }

// The ground under the car from the four wheel records (the heights stay valid while the car is on its roof, FACTS "Rollover body box"): the mean height
// where the wheels are, and a slope fitted while the four points are spread (upright); the last slope is kept while they are not.
bool update_ground(const GtWheelContact wc[4], double up_z) {
    float xs[4], ys[4], hs[4];
    int n = 0;
    bool off_track = false;
    for (int i = 0; i < 4; ++i) {
        if (!wc[i].valid) continue;
        xs[n] = wc[i].x;
        ys[n] = wc[i].y;
        hs[n] = wc[i].ground_height;
        off_track = off_track || wc[i].surface != 0;
        ++n;
    }
    if (n == 0) return false;
    double mx = 0, my = 0, mh = 0;
    for (int i = 0; i < n; ++i) { mx += xs[i]; my += ys[i]; mh += hs[i]; }
    mx /= n; my /= n; mh /= n;
    if (n == 4 && up_z > 0.9) {
        double sxx = 0, sxy = 0, syy = 0, sxh = 0, syh = 0;
        for (int i = 0; i < 4; ++i) {
            const double dx = xs[i] - mx, dy = ys[i] - my, dh = hs[i] - mh;
            sxx += dx * dx; sxy += dx * dy; syy += dy * dy; sxh += dx * dh; syh += dy * dh;
        }
        const double det = sxx * syy - sxy * sxy;
        if (det > 1e-3) {
            g.ground.gx = (sxh * syy - syh * sxy) / det;
            g.ground.gy = (syh * sxx - sxh * sxy) / det;
            g.have_ground_slope = true;
        }
    }
    g.ground.x0 = mx; g.ground.y0 = my; g.ground.h0 = mh;
    g.ground.mu = off_track ? g_set.friction * 5.0 / 7.0 : g_set.friction;
    return true;
}

// The origin's height over the road, from the state at takeover: while the car is level and at least three wheel records have ground under it, its height over their mean
// (measured 2026-11-18: ~0.35 m driving, 0.15 m standing on the grid), else the fixed default. No history: live and replay take over from the same state and agree.
float rest_from_wheels(const GtCarState &s, const GtWheelContact wc[4]) {
    if (s.basis[8] < 0.9f) return kDefaultRest;
    double mh = 0;
    int n = 0;
    for (int i = 0; i < 4; ++i) if (wc[i].valid) { mh += wc[i].ground_height; ++n; }
    if (n < 3) return kDefaultRest;
    const float now = static_cast<float>(s.position[2] - mh / n);
    return now > 0.2f && now < 0.6f ? now : kDefaultRest;
}

// The hull from the car record (the game has no body size: the proportions are those of the Peugeot 206's model box, FACTS "Car draw matrix": 3.83 x 1.78 x
// 1.44 m on a 2.442 m wheelbase and a 1.434 m track) and the measured height of the origin over the road.
void size_hull(const GtCarDimensions &d) {
    g.hull.hl = std::clamp(0.78 * d.wheelbase, 1.4, 2.6);
    g.hull.hw = std::clamp(0.5 * d.track + 0.17, 0.7, 1.1);
    g.hull.cx = std::fabs((d.front_axle + d.rear_axle) / 2) < 1.5 ? (d.front_axle + d.rear_axle) / 2 : 0.0;
    g.hull.zb = -g.rest_height;
    g.hull.zt = g.hull.zb + std::clamp(0.59 * d.wheelbase, 1.1, 1.9);
    g.sized = true;
    if (g_log) logf("HULL hl=%.2f hw=%.2f zb=%.2f zt=%.2f cx=%.3f rest=%.3f", g.hull.hl, g.hull.hw, g.hull.zb, g.hull.zt, g.hull.cx, g.rest_height);
}

void write_pose(GtModCtx *c, std::uint32_t car, bool send) {
    float rows[9], pos[3], vel[3];
    rows_from_quat(g.body.q, rows);
    const V3 o = origin_of(g.hull, g.body), vo = velocity_at_origin(g.hull, g.body);
    pos[0] = static_cast<float>(o.x); pos[1] = static_cast<float>(o.y); pos[2] = static_cast<float>(o.z);
    vel[0] = static_cast<float>(vo.x); vel[1] = static_cast<float>(vo.y); vel[2] = static_cast<float>(vo.z);
    const float w[3] = {static_cast<float>(g.body.w.x), static_cast<float>(g.body.w.y), static_cast<float>(g.body.w.z)};
    if (send) g_api->car_set_pose(g_self, c, car, pos, rows, vel, w);
    for (int i = 0; i < 3; ++i) { g.wrote_pos[i] = pos[i]; g.wrote_vel[i] = vel[i]; }  // what the game should find at the next step (check sabotage 15 does not send it)
    for (int i = 0; i < 9; ++i) g.wrote_rows[i] = rows[i];
    g.wrote = true;
}

void begin_simulation(GtModCtx *c, const GtCarState &s, Reason why, const float fake_dv[3], const GtCarDimensions &dims) {
    size_hull(dims);  // the floor as measured while driving
    ++g.takeovers;
    g_api->tilt_free(g_self, c, g.car, 1);
    const double dt = kDt;
    const V3 w = angular_velocity_from_rows(g.prev_rows, s.basis, dt);
    const Quat q = quat_from_rows(s.basis);
    V3 v = v3(s.velocity);
    V3 dv;
    if (why == Reason::Hit && g.have_prev) {  // the car before the hit, then the hit as an impulse at its contact point
        dv = v - v3(g.prev_vel);
        v = v3(g.prev_vel);
    }
    if (fake_dv != nullptr) { dv = v3(fake_dv); v = v3(s.velocity); }
    g.body = body_from_origin(g.hull, v3(s.position), q, v, w);
    if (len(dv) > 1e-6) {
        const V3 p = hit_point(g.hull, g.body, dv, g_set.hit_height);
        apply_impulse(g.hull, g.params, g.body, p, dv * g.params.mass);
        if (g_log) logf("HIT st=%u dv=%.2f,%.2f,%.2f at=%.2f,%.2f,%.2f reason=%d", g.steps, dv.x, dv.y, dv.z, p.x - g.body.com.x, p.y - g.body.com.y, p.z - g.body.com.z, static_cast<int>(why));
    }
    g.sim_steps = 0;
    g.sim_hits = 0;
    g.wrote = false;
    if (g_log) logf("TAKEOVER st=%u reason=%d v=%.2f,%.2f,%.2f w=%.2f,%.2f,%.2f up.z=%.3f", g.steps, static_cast<int>(why), v.x, v.y, v.z, w.x, w.y, w.z, up_z_of(s.basis));
}

void on_step(GtModCtx *c) {
    const std::uint32_t world = g_api->gpr(c, 4);
    std::uint32_t car = 0;
    int role = 0;
    if (!find_car(c, world, &car, &role)) {
        if (g.car != 0) {
            if (g_sup->simulating()) g_api->tilt_free(g_self, c, g.car, 0);
            reset_state();
        }
        g_api->call_original(c);
        return;
    }
    if (car != g.car || role != g.role) {  // a new race, or its replay starting
        if (g.car != 0 && g_sup->simulating()) g_api->tilt_free(g_self, c, g.car, 0);
        if (g_log && car == g.car) logf(role == 3 ? "REPLAY start" : "REPLAY end");
        reset_state();
        g.car = car;
        g.role = role;
    }
    g.world = world;
    ++g.steps;
    GtCarState s;
    GtWheelContact wc[4];
    GtCarDimensions dims;
    if (g_api->car_state(g_self, c, car, &s) != 0 || g_api->car_wheel_contacts(g_self, c, world, car, wc) != 0 || g_api->car_dimensions(g_self, c, world, car, &dims) != 0) {
        g_api->call_original(c);
        return;
    }
    const double speed2 = double(s.velocity[0]) * s.velocity[0] + double(s.velocity[1]) * s.velocity[1] + double(s.velocity[2]) * s.velocity[2];
    if (g.moved_at < 0 && speed2 > 1.0) g.moved_at = g.steps;
    if (!g.sized) {
        g.params.mass = g_set.mass;
        g.params.restitution = g_set.restitution;
        g.params.friction_scale = g_set.sabotage == 14 ? 0.0 : 1.0;
        size_hull(dims);
    }
    // the ground plane is frozen while the simulation drives (the wheel records move with the tumbling car and the valid set changes step to step); it is taken again from
    // the first game step after a handback or reset
    const bool have_ground = g_sup->simulating() ? true : update_ground(wc, s.basis[8]);
    const double jump = g.have_prev ? std::sqrt(std::pow(double(s.position[0]) - g.prev_pos[0], 2) + std::pow(double(s.position[1]) - g.prev_pos[1], 2) + std::pow(double(s.position[2]) - g.prev_pos[2], 2)) : 0.0;
    // fake hit / drop (the checks)
    const long since = g.moved_at < 0 ? -1 : static_cast<long>(g.steps) - g.moved_at;
    bool fake_hit = g_fake.hit_step >= 0 && since == g_fake.hit_step;
    bool fake_drop = g_fake.drop_step >= 0 && since == g_fake.drop_step;

    if (!g_sup->simulating()) {
        GameObs o;
        o.valid = g.have_prev && jump < 3.0;
        if (o.valid) {
            double dv2 = 0;
            for (int i = 0; i < 3; ++i) dv2 += double(s.velocity[i] - g.prev_vel[i]) * (s.velocity[i] - g.prev_vel[i]);
            o.dv = std::sqrt(dv2);
        }
        o.up_z = s.basis[8];
        o.w = o.valid ? len(angular_velocity_from_rows(g.prev_rows, s.basis, kDt)) : 0.0;
        o.loaded_wheels = 0;
        for (const GtWheelContact &w : wc) o.loaded_wheels += (w.valid && w.load > 0) ? 1 : 0;
        float fake_vec[3] = {0, 0, 0};
        Event e = Event::None;
        if (g_set.sabotage != 16 && have_ground) {
            if (fake_hit) {  // a lateral hit of fake_dv along the car's right row (as the old mod's fake did)
                for (int i = 0; i < 3; ++i) fake_vec[i] = s.basis[3 + i] * g_fake.hit_dv;
                g_sup->force_takeover(Reason::Hit);
                e = Event::Takeover;
            } else if (fake_drop) {
                g_sup->force_takeover(Reason::Hit);
                e = Event::Takeover;
            } else {
                e = g_sup->update_game(o);
            }
        }
        if (e == Event::Takeover) {
            GtCarState s2 = s;
            g.rest_height = rest_from_wheels(s, wc);  // before the fake drop moves the car: deterministic from the state at takeover (live and replay agree)
            if (fake_drop) s2.position[2] += g_fake.drop_height;
            begin_simulation(c, s2, g_sup->reason(), fake_hit ? fake_vec : nullptr, dims);
        }
    } else if (g.wrote) {
        // the game's own step changed the velocity we wrote: a barrier hit. Only the part ALONG the ground is taken: the part along the ground normal is the game's wheels and
        // springs (and gravity) answering a car that lands or rests on records that read grounded, not a wall
        V3 dv{s.velocity[0] - g.wrote_vel[0], s.velocity[1] - g.wrote_vel[1], s.velocity[2] - g.wrote_vel[2]};
        const V3 gn = g.ground.at(g.body.com.x, g.body.com.y).normal;
        dv = dv - gn * dot(dv, gn);
        if (len(dv) >= g_set.sim_hit_dv && g_set.sabotage != 16) {
            ++g.sim_hits;
            const V3 p = hit_point(g.hull, g.body, dv, g_set.hit_height);
            apply_impulse(g.hull, g.params, g.body, p, dv * g.params.mass);
            if (g_log) logf("HIT st=%u dv=%.2f,%.2f,%.2f (in the simulation) n=%u", g.steps, dv.x, dv.y, dv.z, g.sim_hits);
        }
    }

    if (g_sup->simulating()) {
        if (g_set.sabotage == 13) g.params.restitution = 1.0;
        StepInfo info = step(g.hull, g.params, g.ground, g.body, kDt);
        ++g.sim_steps;
        float rows[9];
        rows_from_quat(g.body.q, rows);
        SimObs so;
        so.up_z = rows[8];
        so.w = len(g.body.w);
        so.bottom_contacts = info.bottom_contacts;
        so.rows_jump = g.wrote ? std::fmax(std::fmax(std::fabs(s.basis[6] - g.wrote_rows[6]), std::fabs(s.basis[7] - g.wrote_rows[7])), std::fabs(s.basis[8] - g.wrote_rows[8])) : 0.0;
        so.jump = g.wrote ? std::sqrt(std::pow(double(s.position[0]) - g.wrote_pos[0], 2) + std::pow(double(s.position[1]) - g.wrote_pos[1], 2) + std::pow(double(s.position[2]) - g.wrote_pos[2], 2)) : 0.0;
        const Event e = g_sup->update_sim(so);
        if (g_log) {
            const V3 o = origin_of(g.hull, g.body);
            const V3 I = inertia_diag(g.hull, g.params.mass), wb = rotate_inverse(g.body.q, g.body.w);
            const double energy = g.params.mass * g.params.gravity * g.body.com.z + 0.5 * g.params.mass * dot(g.body.v, g.body.v) + 0.5 * (I.x * wb.x * wb.x + I.y * wb.y * wb.y + I.z * wb.z * wb.z);
            // kd: how far the game's own step left the car from the pose written last (the game integrates the velocity once more and its suspension acts): the pose the game reports at this step
            const double kd = g.wrote ? so.jump : 0.0, kup = g.wrote ? std::sqrt(std::pow(double(s.basis[6]) - g.wrote_rows[6], 2) + std::pow(double(s.basis[7]) - g.wrote_rows[7], 2) + std::pow(double(s.basis[8]) - g.wrote_rows[8], 2)) : 0.0;
            logf("SIM st=%u n=%u pos=%.3f,%.3f,%.3f vel=%.2f,%.2f,%.2f w=%.2f,%.2f,%.2f up.z=%.3f lowest=%.3f contacts=%d/%d ground=%.3f E=%.0f com.z=%.3f kd=%.4f,%.4f", g.steps, g.sim_steps,
                 o.x, o.y, o.z, g.body.v.x, g.body.v.y, g.body.v.z, g.body.w.x, g.body.w.y, g.body.w.z, so.up_z, info.lowest, info.contacts, info.bottom_contacts, g.ground.h0, energy,
                 g.body.com.z, kd, kup);
        }
        if (e == Event::Handback || e == Event::Reset) {
            if (e == Event::Handback) write_pose(c, car, true);  // the final pose with zero rates; the game's suspension settles it
            g_api->tilt_free(g_self, c, car, 0);
            g.wrote = false;
            if (g_log) logf("%s st=%u after=%u sim steps", e == Event::Handback ? "HANDBACK" : "RESET", g.steps, g.sim_steps);
        } else {
            write_pose(c, car, g_set.sabotage != 15);  // 15 (check build): the simulation runs but the pose never reaches the game
        }
    }
    if (g_log && !g_sup->simulating() && g.steps % 30 == 0)
        logf("ST st=%u pos=%.3f,%.3f,%.3f vel=%.2f,%.2f,%.2f speed=%.2f up.z=%.3f since=%ld", g.steps, s.position[0], s.position[1], s.position[2], s.velocity[0], s.velocity[1], s.velocity[2],
             s.speed, s.basis[8], since);
    for (int i = 0; i < 3; ++i) { g.prev_pos[i] = s.position[i]; g.prev_vel[i] = s.velocity[i]; }
    for (int i = 0; i < 9; ++i) g.prev_rows[i] = s.basis[i];
    g.have_prev = true;
    g_api->call_original(c);
}

}  // namespace

int init(const GtModApi *api, GtMod *self, const rollover::Settings &settings, bool log) {
    g_api = api;
    g_self = self;
    g_set = settings;
    g_log = log;
    static Supervisor sup{TakeoverParams{}};
    TakeoverParams tp;
    tp.hit_dv = settings.takeover_dv;
    tp.sim_hit_dv = settings.sim_hit_dv;
    sup = Supervisor(tp);
    g_sup = &sup;
    if (const char *fake = std::getenv("GT_MOD_ROLLOVER_FAKE")) {  // "m<steps>:<dv>"
        float dv = 0;
        long step = 0;
        if (std::sscanf(fake[0] == 'm' ? fake + 1 : fake, "%ld:%f", &step, &dv) == 2 && step >= 0 && std::isfinite(dv)) { g_fake.hit_step = step; g_fake.hit_dv = dv; }
    }
    if (const char *drop = std::getenv("GT_MOD_ROLLOVER_DROP")) {  // "m<steps>:<height>"
        float h = 0;
        long step = 0;
        if (std::sscanf(drop[0] == 'm' ? drop + 1 : drop, "%ld:%f", &step, &h) == 2 && step >= 0 && std::isfinite(h)) { g_fake.drop_step = step; g_fake.drop_height = h; }
    }
    if (api->hook_point(self, GT_HOOK_WORLD_STEP, on_step) != 0) return 1;
    // No switch-off function (it would need a vblank tick, which makes the host filter the pad): a mod switched off in the menu stops being called (its tilt_free is
    // dropped by the host too) and, switched on again, finds the car far from the pose it wrote last: that is a reset, and the car goes back to the game.
    return 0;
}

}  // namespace realistic
