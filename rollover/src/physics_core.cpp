#include "physics_core.hpp"

#include <algorithm>

namespace rollphys {

Quat normalized(Quat q) {
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (!(n > 1e-12)) return Quat{};
    return {q.w / n, q.x / n, q.y / n, q.z / n};
}
Quat operator*(Quat a, Quat b) {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
V3 rotate(Quat q, V3 v) {
    const V3 u{q.x, q.y, q.z};
    const V3 t = cross(u, v) * 2.0;
    return v + t * q.w + cross(u, t);
}
V3 rotate_inverse(Quat q, V3 v) { return rotate({q.w, -q.x, -q.y, -q.z}, v); }
Quat integrate(Quat q, V3 w, double dt) {
    const Quat dq = Quat{0, w.x, w.y, w.z} * q;
    return normalized({q.w + 0.5 * dt * dq.w, q.x + 0.5 * dt * dq.x, q.y + 0.5 * dt * dq.y, q.z + 0.5 * dt * dq.z});
}

Quat quat_from_rows(const float rows[9]) {
    // columns of the rotation: forward, left (= -right), up
    const double m[3][3] = {{rows[0], -rows[3], rows[6]}, {rows[1], -rows[4], rows[7]}, {rows[2], -rows[5], rows[8]}};  // m[r][c]
    Quat q;
    const double tr = m[0][0] + m[1][1] + m[2][2];
    if (tr > 0) {
        const double s = std::sqrt(tr + 1.0) * 2;
        q = {0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s};
    } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
        const double s = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2;
        q = {(m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s};
    } else if (m[1][1] > m[2][2]) {
        const double s = std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2;
        q = {(m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s};
    } else {
        const double s = std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2;
        q = {(m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s};
    }
    return normalized(q);
}
void rows_from_quat(Quat q, float rows[9]) {
    const V3 f = rotate(q, {1, 0, 0}), l = rotate(q, {0, 1, 0}), u = rotate(q, {0, 0, 1});
    const V3 r = -l;
    const V3 out[3] = {f, r, u};
    for (int i = 0; i < 3; ++i) {
        rows[3 * i] = static_cast<float>(out[i].x);
        rows[3 * i + 1] = static_cast<float>(out[i].y);
        rows[3 * i + 2] = static_cast<float>(out[i].z);
    }
}

GroundSample PlaneGround::at(double x, double y) const {
    GroundSample s;
    s.height = h0 + gx * (x - x0) + gy * (y - y0);
    const V3 n{-gx, -gy, 1.0};
    s.normal = n * (1.0 / len(n));
    s.friction = mu;
    return s;
}

V3 com_offset(const Hull &h) { return {h.cx, 0, 0.5 * (h.zb + h.zt)}; }

void hull_points(const Hull &h, V3 out[kHullPoints], bool bottom[kHullPoints]) {
    const double hh = 0.5 * (h.zt - h.zb);
    int n = 0;
    for (int sx = -1; sx <= 1; sx += 2)
        for (int sy = -1; sy <= 1; sy += 2)
            for (int sz = -1; sz <= 1; sz += 2) {
                out[n] = {sx * h.hl, sy * h.hw, sz * hh};
                bottom[n++] = sz < 0;
            }
    for (int sy = -1; sy <= 1; sy += 2)
        for (int sz = -1; sz <= 1; sz += 2) {
            out[n] = {0, sy * h.hw, sz * hh};
            bottom[n++] = sz < 0;
        }
}

Body body_from_origin(const Hull &h, V3 origin, Quat q, V3 v_at_origin, V3 w) {
    Body b;
    b.q = normalized(q);
    const V3 r = rotate(b.q, com_offset(h));
    b.com = origin + r;
    b.w = w;
    b.v = v_at_origin + cross(w, r);
    return b;
}
V3 origin_of(const Hull &h, const Body &b) { return b.com - rotate(b.q, com_offset(h)); }
V3 velocity_at_origin(const Hull &h, const Body &b) { return b.v - cross(b.w, rotate(b.q, com_offset(h))); }

V3 inertia_diag(const Hull &h, double m) {
    const double lx = 2 * h.hl, ly = 2 * h.hw, lz = h.zt - h.zb;
    return {m / 12.0 * (ly * ly + lz * lz), m / 12.0 * (lx * lx + lz * lz), m / 12.0 * (lx * lx + ly * ly)};
}

namespace {

constexpr double kTouch = 0.01;  // m: a hull point this close to the ground counts as touching in StepInfo

V3 inv_inertia_mul(const V3 &I, const Quat &q, V3 t) {
    const V3 l = rotate_inverse(q, t);
    return rotate(q, {l.x / I.x, l.y / I.y, l.z / I.z});
}

struct Contact {
    V3 r;           // from the centre of mass to the point (world)
    V3 n;           // ground normal
    V3 t1, t2;      // tangents
    double depth;   // penetration (m, > 0 inside)
    double mu;
    double bounce;  // target normal speed after the response
    double ln = 0, l1 = 0, l2 = 0;  // accumulated impulses
    bool bottom;
};

double eff_mass_inv(const V3 &I, const Quat &q, double invm, V3 r, V3 d) {
    const V3 a = inv_inertia_mul(I, q, cross(r, d));
    return invm + dot(cross(a, r), d);
}

}  // namespace

void apply_impulse(const Hull &h, const Params &p, Body &b, V3 point, V3 J) {
    const V3 I = inertia_diag(h, p.mass);
    b.v = b.v + J * (1.0 / p.mass);
    b.w = b.w + inv_inertia_mul(I, b.q, cross(point - b.com, J));
    b.rest_steps = 0;
}

StepInfo step(const Hull &h, const Params &p, const Ground &g, Body &b, double dt_full) {
    StepInfo info;
    const V3 I = inertia_diag(h, p.mass);
    const double invm = 1.0 / p.mass;
    V3 pl[kHullPoints];
    bool bottom[kHullPoints];
    hull_points(h, pl, bottom);
    const int subs = std::max(1, p.substeps);
    const double dt = dt_full / subs;
    for (int sub = 0; sub < subs; ++sub) {
        // forces: gravity, gyroscopic torque (body frame), damping
        b.v.z -= p.gravity * dt;
        {
            const V3 wb = rotate_inverse(b.q, b.w);
            const V3 Iw{I.x * wb.x, I.y * wb.y, I.z * wb.z};
            const V3 tq = -cross(wb, Iw);
            const V3 dwb{tq.x / I.x * dt, tq.y / I.y * dt, tq.z / I.z * dt};
            b.w = rotate(b.q, wb + dwb);
        }
        b.v = b.v * std::max(0.0, 1.0 - p.lin_damping * dt);
        b.w = b.w * std::max(0.0, 1.0 - p.ang_damping * dt);
        // positions
        b.com = b.com + b.v * dt;
        b.q = integrate(b.q, b.w, dt);
        // contacts at the new pose
        Contact cs[kHullPoints];
        int nc = 0;
        for (int i = 0; i < kHullPoints; ++i) {
            const V3 r = rotate(b.q, pl[i]);
            const V3 pw = b.com + r;
            const GroundSample gs = g.at(pw.x, pw.y);
            const double d = dot(pw - V3{pw.x, pw.y, gs.height}, gs.normal);
            if (d >= 0) continue;
            Contact c;
            c.r = r;
            c.n = gs.normal;
            c.depth = -d;
            c.mu = gs.friction * p.friction_scale;
            c.bottom = bottom[i];
            const V3 axis = std::fabs(c.n.x) < 0.9 ? V3{1, 0, 0} : V3{0, 1, 0};
            c.t1 = cross(c.n, axis);
            c.t1 = c.t1 * (1.0 / len(c.t1));
            c.t2 = cross(c.n, c.t1);
            const double vn = dot(b.v + cross(b.w, r), c.n);
            c.bounce = vn < -p.bounce_speed ? -p.restitution * vn : 0.0;
            info.max_depth = std::max(info.max_depth, c.depth);
            cs[nc++] = c;
        }
        if (nc == 0) {
            b.rest_steps = 0;
            continue;
        }
        // velocity solve
        for (int it = 0; it < p.iterations; ++it) {
            for (int i = 0; i < nc; ++i) {
                Contact &c = cs[i];
                const V3 dirs[2] = {c.t1, c.t2};
                double *acc[2] = {&c.l1, &c.l2};
                for (int k = 0; k < 2; ++k) {  // friction first, clamped by the normal impulse so far
                    const double vt = dot(b.v + cross(b.w, c.r), dirs[k]);
                    const double kt = eff_mass_inv(I, b.q, invm, c.r, dirs[k]);
                    double dl = -vt / kt;
                    const double lim = c.mu * c.ln;
                    const double nl = std::clamp(*acc[k] + dl, -lim, lim);
                    dl = nl - *acc[k];
                    *acc[k] = nl;
                    apply_impulse(h, p, b, b.com + c.r, dirs[k] * dl);
                }
                const double vn = dot(b.v + cross(b.w, c.r), c.n);
                const double kn = eff_mass_inv(I, b.q, invm, c.r, c.n);
                double dl = (c.bounce - vn) / kn;
                const double nl = std::max(0.0, c.ln + dl);
                dl = nl - c.ln;
                c.ln = nl;
                apply_impulse(h, p, b, b.com + c.r, c.n * dl);
            }
        }
        // position correction: a separate pseudo-velocity solve, so it adds nothing to the real velocity (no energy gain)
        V3 vp, wp;
        for (int it = 0; it < 4; ++it) {
            for (int i = 0; i < nc; ++i) {
                const Contact &c = cs[i];
                const double pen = c.depth - p.slop;
                if (pen <= 0) continue;
                const double target = std::min(p.max_correct / dt_full, p.baumgarte * pen / dt_full * subs);
                const double vn = dot(vp + cross(wp, c.r), c.n);
                const double kn = eff_mass_inv(I, b.q, invm, c.r, c.n);
                const double dl = std::max(0.0, (target - vn) / kn);
                vp = vp + c.n * (dl * invm);
                wp = wp + inv_inertia_mul(I, b.q, cross(c.r, c.n * dl));
            }
        }
        b.com = b.com + vp * dt;
        b.q = integrate(b.q, wp, dt);
        // rest: a body held by >= 3 contacts and moving less than the threshold stops
        if (nc >= 3 && len(b.v) < p.sleep_speed && len(b.w) < p.sleep_speed) {
            b.v = V3{};
            b.w = V3{};
            ++b.rest_steps;
        } else {
            b.rest_steps = 0;
        }
    }
    // what the step left: hull points within kTouch of the ground
    info.lowest = 1e30;
    for (int i = 0; i < kHullPoints; ++i) {
        const V3 pw = b.com + rotate(b.q, pl[i]);
        const GroundSample gs = g.at(pw.x, pw.y);
        const double d = dot(pw - V3{pw.x, pw.y, gs.height}, gs.normal);
        info.lowest = std::min(info.lowest, d);
        if (d < kTouch) {
            ++info.contacts;
            if (bottom[i]) ++info.bottom_contacts;
        }
    }
    info.asleep = b.rest_steps > 0;
    return info;
}

}  // namespace rollphys
