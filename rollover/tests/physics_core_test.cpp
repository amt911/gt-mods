// Unit tests of the rigid-body core (src/physics_core.hpp): free fall, rest, tipping, friction, restitution, spin decay, frame conversions.
#include "physics_core.hpp"

#include <algorithm>
#include <cstdio>
#include <initializer_list>
#include <cstdlib>

using namespace rollphys;

static int g_fail = 0;
#define CHECK(c)                                                              \
    do {                                                                      \
        if (!(c)) {                                                           \
            std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c);           \
            ++g_fail;                                                         \
        }                                                                     \
    } while (0)
#define NEAR(a, b, tol) CHECK(std::fabs((a) - (b)) <= (tol))

static const double kDt = 1.0 / 60.0;

static Quat axis_angle(V3 axis, double a) {
    const double s = std::sin(a / 2);
    return normalized({std::cos(a / 2), axis.x * s, axis.y * s, axis.z * s});
}
static Body make_body(const Hull &h, V3 origin, Quat q, V3 v = {}, V3 w = {}) { return body_from_origin(h, origin, q, v, w); }
static double lowest_point(const Hull &h, const Body &b, const PlaneGround &g) {
    V3 pl[kHullPoints];
    bool bot[kHullPoints];
    hull_points(h, pl, bot);
    double lo = 1e30;
    for (int i = 0; i < kHullPoints; ++i) {
        const V3 pw = b.com + rotate(b.q, pl[i]);
        lo = std::min(lo, pw.z - g.at(pw.x, pw.y).height);
    }
    return lo;
}

static void test_free_fall() {
    Hull h;
    Params p;
    p.lin_damping = 0;
    PlaneGround g;
    g.h0 = -1000;
    Body b = make_body(h, {0, 0, 10}, Quat{});
    for (int i = 0; i < 60; ++i) step(h, p, g, b, kDt);
    NEAR(b.com.z - com_offset(h).z, 10 - 0.5 * 9.81, 0.15);  // semi-implicit Euler: a 1.6 % error in one second
    NEAR(b.v.z, -9.81, 0.02);
    NEAR(b.v.x, 0, 1e-9);
}

static void test_flat_drop_rests() {
    Hull h;
    Params p;
    PlaneGround g;
    Body b = make_body(h, {0, 0, 0.5}, Quat{});
    for (int i = 0; i < 60 * 5; ++i) step(h, p, g, b, kDt);
    const V3 at = b.com;
    StepInfo si;
    for (int i = 0; i < 60 * 10; ++i) si = step(h, p, g, b, kDt);
    NEAR(len(b.com - at), 0.0, 0.001);  // no creep
    CHECK(si.bottom_contacts >= 4);
    CHECK(rotate(b.q, {0, 0, 1}).z > 0.9999);
    NEAR(lowest_point(h, b, g), 0.0, 0.01);
    CHECK(si.asleep);
}

static void test_dropped_on_a_corner_tips_over() {
    Hull h;
    Params p;
    PlaneGround g;
    // 50 degrees of roll and 20 of pitch: a roof corner lands first
    Quat q = axis_angle({1, 0, 0}, 50 * M_PI / 180) * axis_angle({0, 1, 0}, 20 * M_PI / 180);
    Body b = make_body(h, {0, 0, 1.5}, normalized(q));
    StepInfo si;
    for (int i = 0; i < 60 * 12; ++i) si = step(h, p, g, b, kDt);
    CHECK(si.asleep);
    // rests on a face: its normal is a hull axis within 3 degrees of vertical
    const V3 axes[3] = {rotate(b.q, {1, 0, 0}), rotate(b.q, {0, 1, 0}), rotate(b.q, {0, 0, 1})};
    double best = 0;
    for (const V3 &a : axes) best = std::max(best, std::fabs(a.z));
    CHECK(best > std::cos(3 * M_PI / 180));
    CHECK(si.contacts >= 3);
    NEAR(lowest_point(h, b, g), 0.0, 0.01);
}

static void test_slide_stops_by_friction() {
    Hull h;
    Params p;
    p.lin_damping = 0;
    p.ang_damping = 0;
    PlaneGround g;
    g.mu = 0.7;
    Body b = make_body(h, {0, 0, 0.001}, Quat{}, {10, 0, 0});
    const double x0 = b.com.x;
    for (int i = 0; i < 60 * 6; ++i) step(h, p, g, b, kDt);
    const double expected = 10.0 * 10.0 / (2 * 0.7 * 9.81);
    NEAR(b.com.x - x0, expected, 0.12 * expected);
    CHECK(len(b.v) < 0.05);
    // without friction it keeps sliding
    Params nf = p;
    nf.friction_scale = 0;
    Body c = make_body(h, {0, 0, 0.001}, Quat{}, {10, 0, 0});
    for (int i = 0; i < 60 * 3; ++i) step(h, nf, g, c, kDt);
    CHECK(c.v.x > 9.0);
}

static double bounce_apex(double e, double drop) {
    Hull h;
    Params p;
    p.restitution = e;
    p.lin_damping = 0;
    p.ang_damping = 0;
    PlaneGround g;
    const double rest_z = -h.zb;  // origin height when the floor touches
    Body b = make_body(h, {0, 0, rest_z + drop}, Quat{});
    bool bounced = false;
    double apex = 0;
    for (int i = 0; i < 60 * 4; ++i) {
        step(h, p, g, b, kDt);
        if (b.v.z > 0.05) bounced = true;
        if (bounced) apex = std::max(apex, origin_of(h, b).z - rest_z);
    }
    return apex;
}
static void test_restitution_bounce() {
    for (double e : {0.15, 0.4}) {
        const double apex = bounce_apex(e, 2.0), want = e * e * 2.0;
        std::printf("bounce e=%.2f apex=%.4f want=%.4f\n", e, apex, want);
        CHECK(apex <= 1.2 * want);
        CHECK(apex >= 0.8 * want);
    }
}

static void test_spin_decays_and_no_gyro_blowup() {
    Hull h;
    Params p;
    PlaneGround g;
    g.h0 = -1000;
    Body b = make_body(h, {0, 0, 2000}, Quat{}, {}, {0, 0, 10});  // about the vertical = the hull's z axis
    p.gravity = 0;
    for (int i = 0; i < 60 * 10; ++i) step(h, p, g, b, kDt);
    NEAR(len(b.w), 10.0 * std::exp(-0.12 * 10), 0.3);
    // tumbling about the intermediate axis does not gain energy
    Body t = make_body(h, {0, 0, 2000}, Quat{}, {}, {0.5, 6.0, 0.4});
    const V3 I = inertia_diag(h, p.mass);
    auto energy = [&](const Body &x) {
        const V3 wb = rotate_inverse(x.q, x.w);
        return 0.5 * (I.x * wb.x * wb.x + I.y * wb.y * wb.y + I.z * wb.z * wb.z);
    };
    const double e0 = energy(t);
    for (int i = 0; i < 60 * 10; ++i) {
        step(h, p, g, t, kDt);
        CHECK(energy(t) <= e0 * 1.02);
    }
}

static void test_impulse_makes_spin() {
    Hull h;
    Params p;
    Body b = make_body(h, {0, 0, 5}, Quat{});
    const V3 point = b.com + V3{0, 0, 0.4};  // above the centre of mass
    apply_impulse(h, p, b, point, V3{0, -1300.0 * 5.0, 0});  // a side hit: 5 m/s of lateral velocity
    NEAR(b.v.y, -5.0, 1e-9);
    CHECK(b.w.x > 0.1);  // a push toward -y above the centre of mass tips the top toward -y: a rotation about +x (right-hand rule)
}

static void test_rows_roundtrip() {
    srand(7);
    for (int i = 0; i < 2000; ++i) {
        const V3 axis = V3{rand() / double(RAND_MAX) - 0.5, rand() / double(RAND_MAX) - 0.5, rand() / double(RAND_MAX) - 0.5};
        if (len(axis) < 1e-3) continue;
        const Quat q = axis_angle(axis * (1.0 / len(axis)), (rand() / double(RAND_MAX)) * 2 * M_PI - M_PI);
        float rows[9];
        rows_from_quat(q, rows);
        const V3 f{rows[0], rows[1], rows[2]}, r{rows[3], rows[4], rows[5]}, u{rows[6], rows[7], rows[8]};
        NEAR(dot(cross(f, r), u), -1.0, 1e-5);  // left-handed, as the game's basis
        const Quat q2 = quat_from_rows(rows);
        float rows2[9];
        rows_from_quat(q2, rows2);
        for (int k = 0; k < 9; ++k) NEAR(rows[k], rows2[k], 1e-5);
    }
}

static void test_origin_roundtrip() {
    Hull h;
    h.cx = 0.3;
    const Quat q = normalized({0.8, 0.3, -0.2, 0.4});
    const V3 o{10, -4, 3}, v{3, 1, -2}, w{0.2, -0.5, 1.1};
    const Body b = body_from_origin(h, o, q, v, w);
    const V3 o2 = origin_of(h, b), v2 = velocity_at_origin(h, b);
    NEAR(len(o - o2), 0.0, 1e-9);
    NEAR(len(v - v2), 0.0, 1e-9);
}

static void test_no_pop_on_a_slope() {
    Hull h;
    Params p;
    PlaneGround g;
    g.gx = 0.05;  // 3 degrees
    g.mu = 0.7;
    Body b = make_body(h, {0, 0, 0.9}, axis_angle({0, 1, 0}, -std::atan(0.05)));
    for (int i = 0; i < 60 * 8; ++i) step(h, p, g, b, kDt);
    CHECK(len(b.v) < 0.05);  // friction holds it on a gentle slope
}

int main() {
    test_free_fall();
    test_flat_drop_rests();
    test_dropped_on_a_corner_tips_over();
    test_slide_stops_by_friction();
    test_restitution_bounce();
    test_spin_decays_and_no_gyro_blowup();
    test_impulse_makes_spin();
    test_rows_roundtrip();
    test_origin_roundtrip();
    test_no_pop_on_a_slope();
    if (g_fail == 0) std::printf("physics_core_test: all passed\n");
    return g_fail == 0 ? 0 : 1;
}
