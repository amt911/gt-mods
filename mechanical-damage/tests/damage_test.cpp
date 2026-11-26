#include "damage_logic.hpp"

#include <cassert>
#include <algorithm>
#include <cmath>

using namespace damage;

static bool near(float a, float b, float eps = 1e-5f) { return std::fabs(a - b) <= eps; }

int main() {
    Params p;
    // below the threshold nothing happens, NaN included
    assert(hit_strength(0.0f, p) == 0.0f && hit_strength(p.threshold, p) == 0.0f && hit_strength(NAN, p) == 0.0f);
    assert(near(hit_strength(p.threshold + 4.0f, p), 4.0f * p.per_ms));
    assert(hit_strength(1000.0f, p) == 1.0f);  // saturates
    State s;
    const float front[3] = {1, 0, 0}, rear[3] = {-1, 0, 0}, right[3] = {0, 1, 0}, left[3] = {0, -1, 0};
    // front: engine and (half) alignment, nothing on the drive; the head-on hit pulls to neither side's sign -> positive by convention
    add_hit(s, front, 0.4f);
    assert(near(s.engine, 0.4f) && s.drive == 0.0f && near(s.align, 0.2f));
    // rear: the drive only, no alignment
    State r;
    add_hit(r, rear, 0.3f);
    assert(r.engine == 0.0f && near(r.drive, 0.3f) && r.align == 0.0f);
    // sides: alignment only, signed by the side hit
    State a, b;
    add_hit(a, right, 0.5f);
    add_hit(b, left, 0.5f);
    assert(a.engine == 0.0f && a.drive == 0.0f && near(a.align, 0.5f) && near(b.align, -0.5f));
    // a diagonal front-right hit splits
    State d;
    const float diag[3] = {0.6f, 0.8f, 0.0f};
    add_hit(d, diag, 0.5f);
    assert(near(d.engine, 0.3f) && near(d.align, 0.5f * (0.3f + 0.8f)));
    // zero or negative strength: no change
    State z;
    add_hit(z, front, 0.0f);
    add_hit(z, front, -1.0f);
    assert(z.engine == 0.0f && z.align == 0.0f);
    // everything saturates at 1
    State m;
    for (int i = 0; i < 20; ++i) { add_hit(m, front, 0.5f); add_hit(m, rear, 0.5f); add_hit(m, right, 0.5f); }
    assert(m.engine == 1.0f && m.drive == 1.0f && m.align == 1.0f && powertrain(m) == 1.0f);
    // effects: undamaged = identity; full = -40 % throttle, 0.06 rad/s, the lowest cap
    State clean;
    assert(throttle_scale(clean, p) == 1.0f && yaw_bias(clean, p) == 0.0f);
    float cap = 0.0f;
    assert(!speed_cap(clean, p, &cap));
    assert(near(throttle_scale(m, p), 0.6f) && near(yaw_bias(m, p), 0.06f));
    assert(speed_cap(m, p, &cap) && near(cap, p.cap_low));
    State half;
    half.engine = 0.5f;
    assert(near(throttle_scale(half, p), 0.8f) && !speed_cap(half, p, &cap));  // below the severe threshold: no cap
    State sev;
    sev.engine = 0.8f;  // 0.6 + 0.5 * 0.4 -> halfway between the caps
    assert(speed_cap(sev, p, &cap) && near(cap, 35.0f));
    // scale 0 disables every effect (the check's sabotage)
    Params off = p;
    off.scale = 0.0f;
    assert(throttle_scale(m, off) == 1.0f && yaw_bias(m, off) == 0.0f && !speed_cap(m, off, &cap));
    Params wild = p;  // effect_scale above 1 or an engine_loss above 1 never turns the engine into a brake
    wild.scale = 5.0f;
    assert(near(throttle_scale(m, wild), 0.6f) && throttle_scale(clean, wild) == 1.0f);  // the scale is clamped to 0..1
    wild.engine_loss = 5.0f;
    assert(throttle_scale(m, wild) == 0.0f);
    assert(percent(0.426f) == 43 && percent(-0.5f) == 50 && percent(2.0f) == 100);
    repair(m);
    assert(m.engine == 0.0f && m.drive == 0.0f && m.align == 0.0f);
    return 0;
}
