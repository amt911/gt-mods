// Unit tests of src/takeover.hpp: the takeover triggers, the handback rule, the reset detection, the angular velocity from a basis change, the hit point.
#include "takeover.hpp"

#include <cstdio>

using namespace rollphys;
static int g_fail = 0;
#define CHECK(c)                                                    \
    do {                                                            \
        if (!(c)) {                                                 \
            std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); \
            ++g_fail;                                               \
        }                                                           \
    } while (0)

static void test_triggers() {
    TakeoverParams p;
    {
        Supervisor s(p);
        GameObs o;
        for (int i = 0; i < 600; ++i) CHECK(s.update_game(o) == Event::None);  // normal driving
        o.dv = 0.25;
        CHECK(s.update_game(o) == Event::None);  // springs, kerbs
        o.dv = 4.9;
        CHECK(s.update_game(o) == Event::None);
        o.dv = 8.3;
        CHECK(s.update_game(o) == Event::Takeover && s.reason() == Reason::Hit && s.simulating());
        CHECK(s.update_game(o) == Event::None);  // already simulating
    }
    {
        Supervisor s(p);
        GameObs o;
        o.up_z = 0.4;
        CHECK(s.update_game(o) == Event::Takeover && s.reason() == Reason::Flipped);
    }
    {
        Supervisor s(p);
        GameObs o;
        o.loaded_wheels = 0;
        o.w = 1.0;  // all wheels up but hardly spinning: a jump over a crest, not a tumble
        for (int i = 0; i < 50; ++i) CHECK(s.update_game(o) == Event::None);
        o.w = 3.0;
        for (int i = 0; i < p.airborne_steps - 1; ++i) CHECK(s.update_game(o) == Event::None);
        CHECK(s.update_game(o) == Event::Takeover && s.reason() == Reason::Airborne);
    }
    {
        Supervisor s(p);
        GameObs o;
        o.loaded_wheels = 0;
        o.w = 3.0;
        for (int i = 0; i < 3; ++i) s.update_game(o);
        o.loaded_wheels = 2;  // a wheel touches: the count starts again
        s.update_game(o);
        o.loaded_wheels = 0;
        for (int i = 0; i < p.airborne_steps - 1; ++i) CHECK(s.update_game(o) == Event::None);
    }
    {
        Supervisor s(p);
        GameObs o;
        o.dv = 20;
        o.valid = false;  // the first sample of a car, a teleport: nothing is judged
        CHECK(s.update_game(o) == Event::None);
    }
}

static void test_handback() {
    TakeoverParams p;
    Supervisor s(p);
    s.force_takeover(Reason::Hit);
    SimObs o;
    o.up_z = 0.9;
    o.bottom_contacts = 4;
    o.w = 0.3;
    for (int i = 0; i < p.handback_steps - 1; ++i) CHECK(s.update_sim(o) == Event::None);
    o.w = 0.8;  // not calm: the count starts again
    CHECK(s.update_sim(o) == Event::None);
    o.w = 0.3;
    for (int i = 0; i < p.handback_steps - 1; ++i) CHECK(s.update_sim(o) == Event::None);
    CHECK(s.update_sim(o) == Event::Handback && !s.simulating());
    // upside down at rest: never handed back
    s.force_takeover(Reason::Flipped);
    o.up_z = -0.9;
    for (int i = 0; i < 1000; ++i) CHECK(s.update_sim(o) == Event::None);
    // on its side
    o.up_z = 0.1;
    for (int i = 0; i < 1000; ++i) CHECK(s.update_sim(o) == Event::None);
    // upright but held by fewer than 3 floor points (balanced on a wall)
    o.up_z = 0.95;
    o.bottom_contacts = 2;
    for (int i = 0; i < 1000; ++i) CHECK(s.update_sim(o) == Event::None);
}

static void test_reset() {
    TakeoverParams p;
    Supervisor s(p);
    s.force_takeover(Reason::Hit);
    SimObs o;
    o.up_z = -0.5;
    o.jump = 1.0;  // a fast car moves ~1 m per step: not a reset
    CHECK(s.update_sim(o) == Event::None);
    o.jump = 130.0;  // reset-to-track put the car 130 m back
    CHECK(s.update_sim(o) == Event::Reset && !s.simulating());
    CHECK(s.update_sim(o) == Event::None);  // nothing more once the game has the car again
}

static void test_angular_velocity() {
    // a rotation of 0.1 rad about world z in 1/60 s: w_z = 6 rad/s
    const double th = 0.1 / 60.0 * 1.0;
    (void)th;
    const double a = 0.1;
    float before[9] = {1, 0, 0, 0, -1, 0, 0, 0, 1};  // forward x, right -y (left-handed basis), up z
    float after[9] = {float(std::cos(a)), float(std::sin(a)), 0, float(std::sin(a)), float(-std::cos(a)), 0, 0, 0, 1};
    const V3 w = angular_velocity_from_rows(before, after, 1.0 / 60.0);
    CHECK(std::fabs(w.z - 0.1 * 60.0) < 0.05 && std::fabs(w.x) < 1e-6 && std::fabs(w.y) < 1e-6);
    // about x (roll): up row tips toward -y
    float rolled[9] = {1, 0, 0, 0, float(-std::cos(a)), float(-std::sin(a)), 0, float(-std::sin(a)), float(std::cos(a))};
    const V3 w2 = angular_velocity_from_rows(before, rolled, 1.0 / 60.0);
    CHECK(std::fabs(std::fabs(w2.x) - 6.0) < 0.05 && std::fabs(w2.z) < 1e-6);
    // identical basis: no rotation
    CHECK(len(angular_velocity_from_rows(before, before, 1.0 / 60.0)) < 1e-12);
}

static void test_hit_point() {
    Hull h;
    Body b = body_from_origin(h, {0, 0, 1}, Quat{}, {}, {});
    // a push toward -y (the car's right side is -y in the hull frame: dv < 0 in y means the hit came from +y = left): lands on the +y face
    const V3 p = hit_point(h, b, {0, -8.0, 0}, 0.2);
    CHECK(std::fabs(p.y - (b.com.y + h.hw)) < 1e-9);
    const double hh = 0.5 * (h.zt - h.zb);
    CHECK(std::fabs(p.z - (b.com.z + (0.2 * 2 - 1) * hh)) < 1e-9);  // low on the face
    const V3 q = hit_point(h, b, {-8.0, 0, 0}, 0.5);  // pushed backward: a front hit
    CHECK(std::fabs(q.x - (b.com.x + h.hl)) < 1e-9 && std::fabs(q.z - b.com.z) < 1e-9);
    const V3 up = hit_point(h, b, {0, 0, 9.0}, 0.2);  // pushed up: from below
    CHECK(up.z < b.com.z);
}

int main() {
    test_triggers();
    test_handback();
    test_reset();
    test_angular_velocity();
    test_hit_point();
    if (g_fail == 0) std::printf("takeover_test: all passed\n");
    return g_fail == 0 ? 0 : 1;
}
