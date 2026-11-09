// Body box vs ground for the rollover mod. Pure maths (no game access): the unit test and the mod both include it.
// Frame: the game's physics frame, Z up. Rows f (forward), l (lateral, the car's right), u (up) are unit vectors in world coordinates.
#pragma once

#include <algorithm>
#include <cmath>

namespace bodybox {

struct V3 {
    float x = 0, y = 0, z = 0;
};
inline V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 operator*(V3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline float len(V3 a) { return std::sqrt(dot(a, a)); }

struct Box {
    float hl = 2.0f, hw = 0.85f;  // half length (forward) and half width (lateral)
    float zb = -0.03f, zt = 0.8f;  // floor and roof along the up axis, relative to the body origin
    float cx = 0.0f;               // box centre along the forward axis, relative to the body origin
};

// Ground plane: height h0 at (x0, y0) and slope gradient (gx, gy): z = h0 + gx (x - x0) + gy (y - y0).
struct Ground {
    float x0 = 0, y0 = 0, h0 = 0, gx = 0, gy = 0;
    float at(float x, float y) const { return h0 + gx * (x - x0) + gy * (y - y0); }
    V3 normal() const {
        const V3 n{-gx, -gy, 1.0f};
        return n * (1.0f / len(n));
    }
};

struct Params {
    float restitution = 0.15f;   // applied only above `bounce_speed` (a resting contact must not bounce)
    float bounce_speed = 2.0f;   // m/s of approach below which the contact is inelastic
    float soft = 0.25f;          // fraction of the approach speed removed per impulse (1 = rigid; <1 lets the position push finish the job: less bounce)
    float friction = 0.55f;      // Coulomb coefficient (roof or side on tarmac/grass)
    float skin = 0.01f;          // m: contact starts this deep
    float correct = 1.0f;        // fraction of the depth removed per step
    float max_correct = 0.35f;   // m per step
    float tilt_active_cos = 0.77f;  // floor corners only count when up.n is below this (~40 deg); the roof corners always count
    float max_up_speed = 0.5f;   // m/s: largest speed away from the ground the centre keeps in a step with a body contact
};

struct State {
    V3 p, v, w;      // position, world velocity, world angular velocity
    V3 f, l, u;      // basis rows
};

struct Result {
    float min_clear = 0.0f;  // lowest box corner above the ground before the response (negative = inside); all 8 corners
    float min_clear_roof = 0.0f;
    int contacts = 0;
    float max_depth = 0.0f;  // deepest penetrating corner counted by the response
    float push = 0.0f;       // position correction applied (m)
    float dv = 0.0f;         // largest velocity change applied (m/s)
};

inline V3 corner(const State &s, const Box &b, int k) {
    const float a = (k & 1) ? b.hl : -b.hl, c = (k & 2) ? b.hw : -b.hw, h = (k & 4) ? b.zt : b.zb;
    return s.p + s.f * (b.cx + a) + s.l * c + s.u * h;
}

// Inertia (unit mass, box body axes f, l, u).
inline V3 inertia(const Box &b) {
    const float L = 2 * b.hl, W = 2 * b.hw, H = b.zt - b.zb;
    return {(W * W + H * H) / 12.0f, (L * L + H * H) / 12.0f, (L * L + W * W) / 12.0f};
}

// One response for the box in state `s` against `g`. Mutates p, v, w.
inline Result resolve(State &s, const Box &b, const Ground &g, const Params &pr, bool apply = true) {
    Result res;
    const V3 n = g.normal();
    const bool tilted = dot(s.u, n) < pr.tilt_active_cos;
    const V3 inertia_body = inertia(b);
    res.min_clear = res.min_clear_roof = 1e9f;
    float depth[8];
    for (int k = 0; k < 8; ++k) {
        const V3 c = corner(s, b, k);
        depth[k] = g.at(c.x, c.y) - c.z;  // vertical depth; the push uses it along n (close enough for a few degrees of slope)
        res.min_clear = std::min(res.min_clear, -depth[k]);
        if (k & 4) res.min_clear_roof = std::min(res.min_clear_roof, -depth[k]);
    }
    if (!apply) return res;
    // sequential impulses: every penetrating corner, a few iterations (one corner at a time rocks a box that rests on four)
    const int passes = pr.soft < 1.0f ? 1 : 8;  // a soft contact is one partial impulse: repeating it would remove the whole speed anyway
    for (int pass = 0; pass < passes; ++pass) {
        bool any = false;
        for (int k = 0; k < 8; ++k) {
            if (!(k & 4) && !tilted) continue;
            const V3 cp = corner(s, b, k);
            const float d = g.at(cp.x, cp.y) - cp.z;
            if (d <= pr.skin) continue;
            const V3 r = cp - s.p;
            const V3 vc = s.v + cross(s.w, r);
            const float vn = dot(vc, n);
            if (pass == 0 && d > res.max_depth) { res.contacts++; res.max_depth = d; }
            auto angular_term = [&](V3 dir) {  // (r x dir) . I^-1 (r x dir)
                const V3 rn = cross(r, dir);
                const float a2 = dot(rn, s.f), c2 = dot(rn, s.l), d2 = dot(rn, s.u);
                return a2 * a2 / inertia_body.x + c2 * c2 / inertia_body.y + d2 * d2 / inertia_body.z;
            };
            auto apply_impulse = [&](V3 imp) {
                s.v = s.v + imp;
                const V3 torque = cross(r, imp);
                const float a2 = dot(torque, s.f) / inertia_body.x, c2 = dot(torque, s.l) / inertia_body.y, d2 = dot(torque, s.u) / inertia_body.z;
                s.w = s.w + s.f * a2 + s.l * c2 + s.u * d2;
                res.dv = std::max(res.dv, len(imp));
            };
            if (vn >= 0.0f) continue;
            any = true;
            const float e = (pass == 0 && -vn > pr.bounce_speed) ? pr.restitution : 0.0f;
            const float j = -(1.0f + e) * vn * pr.soft / (1.0f + angular_term(n));
            apply_impulse(n * j);
            const V3 vc2 = s.v + cross(s.w, r);
            const V3 vt = vc2 - n * dot(vc2, n);
            const float vtl = len(vt);
            if (vtl > 1e-4f) {
                const V3 td = vt * (1.0f / vtl);
                const float jt = std::min(pr.friction * (j / std::max(pr.soft, 0.05f)), vtl / (1.0f + angular_term(td)));  // friction follows the rigid normal impulse: a soft contact must not let the roof slide on
                apply_impulse(td * (-jt));
            }
        }
        if (!any) break;
    }
    // cap the rise: in a step where the body touches the ground the centre may not move away from it faster than max_up_speed (the roof corners
    // of a spinning car hop it 1.5 m up without the cap). Never without a contact: a car in the air is the game's.
    if (res.contacts > 0) {
        const float vn_after = dot(s.v, n);
        if (vn_after > pr.max_up_speed) {
            s.v = s.v - n * (vn_after - pr.max_up_speed);
            res.dv = std::max(res.dv, vn_after - pr.max_up_speed);
        }
    }
    // position: remove a fraction of the deepest penetration along the normal
    float deepest = 0.0f;
    for (int k = 0; k < 8; ++k) {
        if (!(k & 4) && !tilted) continue;
        const V3 c = corner(s, b, k);
        deepest = std::max(deepest, g.at(c.x, c.y) - c.z);
    }
    if (deepest > pr.skin) {
        const float push = std::min((deepest - pr.skin) * pr.correct, pr.max_correct);
        s.p = s.p + n * push;
        res.push = push;
        if (res.contacts == 0) { res.contacts = 1; res.max_depth = deepest; }
    }
    return res;
}

}  // namespace bodybox
