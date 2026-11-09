#include "bodybox.hpp"

#include <cassert>
#include <cstdio>

using namespace bodybox;

static void rotate(State &s, float dt) {  // rotate the rows by w*dt (Rodrigues)
    const float th = len(s.w) * dt;
    if (th < 1e-9f) return;
    const V3 a = s.w * (1.0f / len(s.w));
    auto R = [&](V3 v) { return v * std::cos(th) + cross(a, v) * std::sin(th) + a * (dot(a, v) * (1 - std::cos(th))); };
    s.f = R(s.f); s.l = R(s.l); s.u = R(s.u);
}

// A free box under gravity at the physics step (60 Hz). The wheels' support is not modelled, so the floor corners stay active.
static State drop(float tilt, V3 w, V3 v, float gx, int steps, float *worst_clear) {
    Box b;
    Params pr;
    pr.soft = 1.0f; pr.correct = 0.8f; pr.max_up_speed = 1e9f;  // the rigid prototype response (the shipped defaults are soft and capped: tested below)
    pr.tilt_active_cos = 2.0f;
    Ground g;
    g.gx = gx;
    State s;
    s.p = {0, 0, 2.0f}; s.v = v; s.w = w;
    s.f = {1, 0, 0}; s.l = {0, 1, 0}; s.u = {0, 0, 1};
    State t = s; t.w = {1, 0, 0}; t.w = t.w * (tilt / (1.0f / 60.0f)); rotate(t, 1.0f / 60.0f);
    s.f = t.f; s.l = t.l; s.u = t.u;
    *worst_clear = 1e9f;
    for (int i = 0; i < steps; ++i) {
        s.v.z -= 9.8f / 60.0f;
        s.p = s.p + s.v * (1.0f / 60.0f);
        rotate(s, 1.0f / 60.0f);
        const Result r = resolve(s, b, g, pr);
        *worst_clear = std::min(*worst_clear, r.min_clear);
    }
    return s;
}

int main() {
    float worst;
    // upside down and on its side, dropped from 2 m: comes to rest, no jitter, never deeper than a few cm / a few steps' worth
    State s = drop(3.14159f, {0, 0, 0}, {0, 0, 0}, 0.0f, 1200, &worst);
    assert(len(s.v) < 0.05f && len(s.w) < 0.05f && worst > -0.1f);
    s = drop(1.5708f, {0, 0, 0}, {0, 0, 0}, 0.0f, 1200, &worst);
    assert(len(s.v) < 0.05f && len(s.w) < 0.05f && worst > -0.15f);
    // a tumble at 12 rad/s and 20 m/s: rests too
    s = drop(0.3f, {12, 0, 0}, {-20, 0, 0}, 0.0f, 1200, &worst);
    assert(len(s.v) < 0.05f && len(s.w) < 0.05f && worst > -0.3f);
    // a slope with friction holds it
    s = drop(0.3f, {12, 0, 0}, {-20, 0, 0}, 0.15f, 1200, &worst);
    assert(len(s.v) < 0.05f && worst > -0.3f);
    // nothing touching: nothing changes
    State t; t.p = {0, 0, 3}; t.f = {1, 0, 0}; t.l = {0, 1, 0}; t.u = {0, 0, 1}; t.v = {5, 0, -1}; t.w = {0, 1, 0};
    const State before = t;
    Ground g; Box b; Params pr;
    const Result r = resolve(t, b, g, pr);
    assert(r.contacts == 0 && r.push == 0 && t.v.x == before.v.x && t.v.z == before.v.z && t.p.z == before.p.z);
    // an upright car with its floor above the ground and only the roof tested: no response even deep in suspension travel
    Ground gg; gg.h0 = 2.95f;  // ground just under the floor (zb = -0.03 -> floor at 2.97)
    State u = before; u.v = {0, 0, -1}; u.w = {0, 0, 0};
    const Result r2 = resolve(u, b, gg, pr);
    assert(r2.contacts == 0 && u.v.z == -1.0f);
    // the shipped defaults (soft, position push 1.0, kick cap 3 m/s): the same drops come to rest, and a roof hitting at a lever arm
    // does not launch the car
    {
        Box sb; Params sp; Ground sg;
        sp.tilt_active_cos = 2.0f;
        State k;
        k.p = {0, 0, 1.2f}; k.f = {1, 0, 0}; k.l = {0, 1, 0}; k.u = {0, 0, -1}; k.v = {-15, 0, -1}; k.w = {9, 0, 0};  // upside down, rolling at 9 rad/s
        float peak = 0.0f, worst_depth = 0.0f;
        for (int i = 0; i < 1200; ++i) {
            k.v.z -= 9.8f / 60.0f;
            k.p = k.p + k.v * (1.0f / 60.0f);
            rotate(k, 1.0f / 60.0f);
            const Result r = resolve(k, sb, sg, sp);
            worst_depth = std::min(worst_depth, r.min_clear);
            peak = std::max(peak, k.p.z);
        }
        assert(len(k.v) < 2.0f && peak < 2.5f && worst_depth > -0.6f);  // a soft contact keeps sliding a while on its roof (friction scales with the soft impulse)
        // the cap itself: in a step with a contact the centre keeps at most max_up_speed away from the ground (0.3 m/s here; a 6 m/s drop at 12 rad/s gives 0.6 uncapped)
        State q; q.p = {0, 0, 0.7f}; q.f = {1, 0, 0}; q.l = {0, 1, 0}; q.u = {0, 0, -1}; q.v = {0, 0, -6}; q.w = {12, 0, 0};
        Params cp; cp.soft = 1.0f; cp.max_up_speed = 0.3f; cp.tilt_active_cos = 2.0f;
        resolve(q, sb, sg, cp);
        assert(q.v.z <= 0.3f + 1e-4f);
        Params nocap = cp; nocap.max_up_speed = 1e9f;
        State q2; q2.p = {0, 0, 0.7f}; q2.f = {1, 0, 0}; q2.l = {0, 1, 0}; q2.u = {0, 0, -1}; q2.v = {0, 0, -6}; q2.w = {12, 0, 0};
        resolve(q2, sb, sg, nocap);
        assert(q2.v.z > q.v.z + 0.05f);
    }
    std::puts("bodybox_test: ok");
    return 0;
}
