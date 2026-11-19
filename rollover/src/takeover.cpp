#include "takeover.hpp"

#include <algorithm>

namespace rollphys {

void Supervisor::reset() {
    sim_ = false;
    reason_ = Reason::None;
    air_ = calm_ = 0;
}

Event Supervisor::update_game(const GameObs &o) {
    if (sim_) return Event::None;
    if (!o.valid) {
        air_ = 0;
        return Event::None;
    }
    air_ = (o.loaded_wheels == 0 && o.w > p_.airborne_w) ? air_ + 1 : 0;
    Reason why = Reason::None;
    if (o.dv >= p_.hit_dv) why = Reason::Hit;
    else if (o.up_z < p_.flipped_up_z) why = Reason::Flipped;
    else if (air_ >= p_.airborne_steps) why = Reason::Airborne;
    if (why == Reason::None) return Event::None;
    sim_ = true;
    reason_ = why;
    calm_ = 0;
    air_ = 0;
    return Event::Takeover;
}

Event Supervisor::update_sim(const SimObs &o) {
    if (!sim_) return Event::None;
    if (o.jump > p_.reset_jump || o.rows_jump > p_.reset_rows) {
        reset();
        return Event::Reset;
    }
    const bool calm = o.up_z > p_.handback_up_z && o.bottom_contacts >= p_.handback_contacts && o.w < p_.handback_w;
    calm_ = calm ? calm_ + 1 : 0;
    if (calm_ >= p_.handback_steps) {
        reset();
        return Event::Handback;
    }
    return Event::None;
}

V3 angular_velocity_from_rows(const float b[9], const float a[9], double dt) {
    // for each row r: dr = w x r (small step): w = sum over rows of (r x dr) / (sum |r|^2 ...) ; rows are orthonormal: w = 0.5 sum r_i x dr_i / dt
    V3 w;
    for (int i = 0; i < 3; ++i) {
        const V3 r{b[3 * i], b[3 * i + 1], b[3 * i + 2]};
        const V3 rr{a[3 * i], a[3 * i + 1], a[3 * i + 2]};
        w = w + cross(r + rr, rr - r) * 0.25;  // midpoint form: exact for a rotation about a fixed axis
    }
    return w * (1.0 / (dt > 0 ? dt : 1.0));
}

V3 hit_point(const Hull &h, const Body &b, V3 dv, double height_fraction) {
    const double hh = 0.5 * (h.zt - h.zb);
    const V3 local = rotate_inverse(b.q, -dv);  // the direction the push came from, hull frame
    const double lx = std::fabs(local.x) / h.hl, ly = std::fabs(local.y) / h.hw, lz = std::fabs(local.z) / hh;
    V3 p;
    const double fz = (height_fraction * 2.0 - 1.0) * hh;  // -hh .. +hh around the centre of mass
    if (lz > lx && lz > ly) {
        p = {0, 0, local.z > 0 ? hh : -hh};
    } else if (lx >= ly) {
        p = {local.x > 0 ? h.hl : -h.hl, 0, fz};
        if (std::fabs(local.y) > 1e-9) p.y = std::clamp(local.y / std::fabs(local.x) * h.hl, -h.hw, h.hw);
    } else {
        p = {0, local.y > 0 ? h.hw : -h.hw, fz};
        if (std::fabs(local.x) > 1e-9) p.x = std::clamp(local.x / std::fabs(local.y) * h.hw, -h.hl, h.hl);
    }
    return b.com + rotate(b.q, p);
}

}  // namespace rollphys
