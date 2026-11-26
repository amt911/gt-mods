#include "telemetry_logic.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

using namespace telemetry;

static bool near(float a, float b, float eps = 1e-3f) { return std::fabs(a - b) <= eps; }

static CarInput car_at(float x, float vx, float vy = 0.f) {
    CarInput c{};
    c.position[0] = x;
    c.velocity[0] = vx;
    c.velocity[1] = vy;
    c.forward[0] = 1.f;
    c.right[1] = 1.f;
    c.speed = std::fabs(vx);
    return c;
}

int main() {
    // G from velocity change: 1 G forward = 9.81 m/s^2 over one 1/60 s step
    {
        const float v0[3] = {10, 0, 0}, v1[3] = {10 + kGravity / 60.f, 0, 0}, f[3] = {1, 0, 0}, r[3] = {0, 1, 0};
        const GForce g = g_from_velocity_change(v0, v1, f, r, kStepSeconds);
        assert(near(g.longitudinal, 1.f) && near(g.lateral, 0.f));
        // braking is negative, a push to the right is positive lateral
        const float v2[3] = {10 - 2 * kGravity / 60.f, 3 * kGravity / 60.f, 0};
        const GForce b = g_from_velocity_change(v0, v2, f, r, kStepSeconds);
        assert(near(b.longitudinal, -2.f) && near(b.lateral, 3.f));
        // the up axis is left out: a vertical velocity change is no G
        const float v3[3] = {10, 0, 5};
        const GForce u = g_from_velocity_change(v0, v3, f, r, kStepSeconds);
        assert(near(u.longitudinal, 0.f) && near(u.lateral, 0.f));
        // a rotated body: forward along y
        const float f2[3] = {0, 1, 0}, r2[3] = {-1, 0, 0}, v4[3] = {10, kGravity / 60.f, 0};
        const GForce rot = g_from_velocity_change(v0, v4, f2, r2, kStepSeconds);
        assert(near(rot.longitudinal, 1.f) && near(rot.lateral, 0.f));
    }
    // smoother: average of the last 4, partial windows average what there is
    {
        Smoother s(4);
        assert(near(s.push(4.f), 4.f));
        assert(near(s.push(0.f), 2.f));
        s.push(0.f);
        assert(near(s.push(0.f), 1.f));
        assert(near(s.push(0.f), 0.f));  // the 4 left out
        s.reset();
        assert(near(s.push(8.f), 8.f));
        Smoother one(0);  // window clamps to 1
        assert(near(one.push(3.f), 3.f) && near(one.push(5.f), 5.f));
    }
    // ring: keeps the newest N, oldest first
    {
        Ring r(3);
        assert(r.size() == 0 && r.capacity() == 3);
        for (int i = 1; i <= 5; ++i) {
            Sample s;
            s.speed = static_cast<float>(i);
            r.push(s);
        }
        assert(r.size() == 3);
        assert(near(r.at(0).speed, 3.f) && near(r.at(1).speed, 4.f) && near(r.at(2).speed, 5.f));
        r.clear();
        assert(r.size() == 0);
        Sample s;
        s.speed = 9.f;
        r.push(s);
        assert(r.size() == 1 && near(r.at(0).speed, 9.f));
    }
    // tracker: steady drive = 0 G; braking over 8 steps shows negative G once smoothed; the first step is 0
    {
        Tracker t;
        float x = 0.f, v = 30.f;
        for (int i = 0; i < 10; ++i) {
            x += v / 60.f;
            const Sample s = t.feed(car_at(x, v));
            assert(near(s.g_long, 0.f) && near(s.g_lat, 0.f));
        }
        Sample last;
        for (int i = 0; i < 8; ++i) {
            v -= 1.f * kGravity / 60.f;  // 1 G of braking
            x += v / 60.f;
            last = t.feed(car_at(x, v));
        }
        assert(near(last.g_long, -1.f, 1e-2f));
        // the ring holds them all
        assert(t.ring().size() == 18);
        // a teleport (reset) gives 0 G and does not smear the jump into the average
        const Sample j = t.feed(car_at(x + 500.f, 0.f));
        assert(near(j.g_long, 0.f));
        const Sample after = t.feed(car_at(x + 500.f, 0.f));
        assert(near(after.g_long, 0.f));
        t.reset();
        assert(t.ring().size() == 0);
    }
    // a constant 1.5 m/s^2 lateral push reads as +lateral G and no longitudinal
    {
        Tracker t;
        float vy = 0.f, x = 0.f;
        Sample s;
        for (int i = 0; i < 12; ++i) {
            vy += 1.5f / 60.f;
            x += 20.f / 60.f;
            s = t.feed(car_at(x, 20.f, vy));
        }
        assert(near(s.g_lat, 1.5f / kGravity, 1e-2f) && near(s.g_long, 0.f, 1e-3f));
    }
    // graph scaling
    {
        Ring r(10);
        Sample s;
        s.speed = 33.f;
        s.g_long = -2.4f;
        r.push(s);
        Range sr = speed_range(r, 40.f);
        assert(near(sr.lo, 0.f) && near(sr.hi, 40.f));  // peak rounds up to 40, which is the floor
        s.speed = 61.f;
        r.push(s);
        sr = speed_range(r, 40.f);
        assert(near(sr.hi, 80.f));
        Range gr = g_range(r, 1.f);
        assert(near(gr.lo, -3.f) && near(gr.hi, 3.f));
        Ring empty(4);
        gr = g_range(empty, 1.f);
        assert(near(gr.lo, -1.f) && near(gr.hi, 1.f));
        s.g_lat = 80.f;  // a wall hit: clamped
        r.push(s);
        assert(near(g_range(r, 1.f).hi, kShownGLimit));
        // y mapping: hi at the top, lo at the bottom, clamped
        const Range g{-2.f, 2.f};
        assert(near(graph_y(2.f, g, 10.f, 40.f), 10.f));
        assert(near(graph_y(-2.f, g, 10.f, 40.f), 50.f));
        assert(near(graph_y(0.f, g, 10.f, 40.f), 30.f));
        assert(near(graph_y(99.f, g, 10.f, 40.f), 10.f));
        assert(near(graph_y(-99.f, g, 10.f, 40.f), 50.f));
        assert(near(graph_y(1.f, Range{1.f, 1.f}, 10.f, 40.f), 50.f));  // an empty range does not divide by zero
        // x: newest at the right edge, a full ring starts at the left edge
        assert(near(graph_x(9, 10, 10, 100.f, 90.f), 190.f));
        assert(near(graph_x(0, 10, 10, 100.f, 90.f), 100.f));
        assert(near(graph_x(0, 5, 10, 100.f, 90.f), 150.f));
        assert(near(shown_g(-50.f), -kShownGLimit) && near(shown_g(1.5f), 1.5f));
    }
    std::puts("telemetry_test: ok");
    return 0;
}
