#include "rollover_logic.hpp"

#include <cassert>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <vector>

using namespace rollover;

static Settings parse(const std::map<std::string, std::string> &m, std::vector<std::string> *warns) {
    return parse_settings([&](const char *k) -> const char * { auto it = m.find(k); return it == m.end() ? nullptr : it->second.c_str(); },
                          [&](const std::string &w) { warns->push_back(w); });
}
static bool near(float a, float b) { return std::fabs(a - b) < 1e-4f; }
static Settings base() {  // round numbers: threshold 3 m/s, 2 rad/s per m/s, cap 9 rad/s (the shipped defaults are 3 / 2.5 / 12)
    Settings s;
    s.threshold = 3.0f;
    s.gain = 2.0f;
    s.cap = 9.0f;
    return s;
}

static void settings_defaults_and_invalid() {
    std::vector<std::string> w;
    Settings d = parse({}, &w);
    assert(w.empty() && d.style == Style::Realistic && d.threshold == 3.0f && d.gain == 2.5f && d.cap == 12.0f && d.sabotage == 0);
    Settings s = parse({{"style", "arcade"}, {"threshold", "1.5"}, {"gain", "4"}, {"cap", "12"}}, &w);
    assert(w.empty() && s.style == Style::Arcade && s.threshold == 1.5f && s.gain == 4.0f && s.cap == 12.0f);
    w.clear();
    s = parse({{"style", "drift"}, {"threshold", "abc"}, {"gain", "-1"}, {"cap", "99"}}, &w);
    assert(w.size() == 4 && s.style == Style::Realistic && s.threshold == 3.0f && s.gain == 2.5f && s.cap == 12.0f);
    w.clear();
    s = parse({{"threshold", "0"}}, &w);  // a zero threshold would roll the car in normal driving: invalid
    assert(w.size() == 1 && s.threshold == 3.0f);
    s = parse({{"threshold", "nan"}}, &w);
    assert(w.size() == 2 && s.threshold == 3.0f);
}

static void style_changes_threshold_and_gain() {
    Settings s = base();
    assert(near(effective_threshold(s), 3.0f) && near(effective_gain(s), 2.0f));
    s.style = Style::Arcade;
    assert(near(effective_threshold(s), 1.5f) && near(effective_gain(s), 3.0f));
    assert(effective_threshold(s) < 3.0f);
}

static void roll_rate_threshold_and_cap() {
    Settings s = base();  // threshold 3, gain 2, cap 9
    assert(roll_rate_for(s, 0.0f) == 0.0f);
    assert(roll_rate_for(s, 0.2f) == 0.0f);   // normal cornering: nothing
    assert(roll_rate_for(s, 3.0f) == 0.0f);   // exactly the threshold: nothing
    assert(roll_rate_for(s, -2.9f) == 0.0f);
    assert(near(roll_rate_for(s, 4.0f), 2.0f));   // proportional to the excess
    assert(near(roll_rate_for(s, -5.0f), 4.0f));  // magnitude only: the sign is the tracker's
    assert(near(roll_rate_for(s, 100.0f), 9.0f)); // capped
    assert(roll_rate_for(s, std::numeric_limits<float>::quiet_NaN()) == 0.0f);
    assert(roll_rate_for(s, std::numeric_limits<float>::infinity()) == 0.0f);
    s.style = Style::Arcade;                       // threshold 1.5, gain 3
    assert(roll_rate_for(s, 1.4f) == 0.0f);
    assert(near(roll_rate_for(s, 2.5f), 3.0f));
    assert(roll_rate_for(s, 2.5f) > 0.0f && roll_rate_for(base(), 2.5f) == 0.0f);  // arcade rolls on a hit realistic ignores
    s.cap = 2.0f;
    assert(near(roll_rate_for(s, 100.0f), 2.0f));
    s = base();
    s.sabotage = 3;  // check build: threshold 0
    assert(roll_rate_for(s, 0.2f) > 0.0f);
}

static const float kRight[3] = {1, 0, 0};
static const float kPos[3] = {0, 0, 0};

static void tracker_ignores_normal_driving() {
    Tracker t{base()};
    float v[3] = {0, 20, 0};
    for (int i = 0; i < 300; ++i) {
        v[0] = 0.15f * std::sin(i * 0.1f);  // 0.15 m/s per step of cornering
        assert(t.step(v, kRight, kPos) == 0.0f);
    }
}

static void tracker_hit_signs_and_decay() {
    Tracker t{base()};
    float v[3] = {6, 20, 0};  // sliding right at 6 m/s, hits a wall on its right: velocity change to the left (-x)
    assert(t.step(v, kRight, kPos) == 0.0f);  // first sample: nothing to compare
    v[0] = 0.0f;  // dv_lat = -6: rate = 2 * (6 - 3) = 6, toward the side hit (+right)
    float r = t.step(v, kRight, kPos);
    assert(near(r, 6.0f));
    assert(t.steps_left() == kDriveSteps - 1);
    float prev = r;
    for (int i = 1; i < kDriveSteps; ++i) {
        r = t.step(v, kRight, kPos);
        assert(r > 0.0f && r < prev);  // decays linearly
        prev = r;
    }
    assert(t.step(v, kRight, kPos) == 0.0f && t.steps_left() == 0);  // over
}

static void tracker_other_side_and_stronger_hit() {
    Tracker t{base()};
    float v[3] = {-6, 20, 0};
    t.step(v, kRight, kPos);
    v[0] = 0.0f;  // dv_lat = +6: the car was sliding left and hit the left wall: rolls toward -right
    assert(near(t.step(v, kRight, kPos), -6.0f));
    v[0] = 5.0f;  // a second, bigger hit while rolling replaces the weaker, decayed one: dv +5 -> rate 4 but the decayed target is ~5.85: kept
    float kept = t.step(v, kRight, kPos);
    assert(kept < 0.0f);
    Tracker t2{base()};
    float a[3] = {0, 20, 0};
    t2.step(a, kRight, kPos);
    a[0] = -4.0f;  // dv -4 -> rate 2
    assert(near(t2.step(a, kRight, kPos), 2.0f));
    a[0] = -12.0f;  // dv -8 -> rate 10 capped to 9: stronger, replaces
    assert(near(t2.step(a, kRight, kPos), 9.0f));
}

static void tracker_rotated_axis() {
    const float right[3] = {0, 1, 0};  // the car faces +x: its lateral axis is y
    Tracker u{base()};
    float w[3] = {20, 8, 0};
    u.step(w, right, kPos);
    w[1] = 0.0f;  // dv_lat = -8: rate 2*(8-3) = 10 capped 9
    assert(near(u.step(w, right, kPos), 9.0f));
    // a velocity change along the car's forward axis (braking into a wall head-on) is not a side impact
    Tracker f{base()};
    float x[3] = {20, 0, 0};
    f.step(x, right, kPos);
    x[0] = 0.0f;
    assert(f.step(x, right, kPos) == 0.0f);
}

static void tracker_ignores_teleports_and_junk() {
    Tracker t{base()};
    float v[3] = {10, 20, 0};
    float p[3] = {0, 0, 0};
    t.step(v, kRight, p);
    p[0] = 50.0f;  // moved 50 m in one step: reset-to-track / respawn
    v[0] = 0.0f;
    assert(t.step(v, kRight, p) == 0.0f);
    v[0] = 10.0f;  // the step after: compared against the new state (dv_lat +10 would be an impact, but this one is a plain +10 change)
    assert(t.step(v, kRight, p) < 0.0f);
    Tracker u{base()};
    float a[3] = {0, 0, 0};
    float q[3] = {0, 0, 0};
    u.step(a, kRight, q);
    a[0] = 100.0f;  // a 100 m/s jump in one step is not an impact
    assert(u.step(a, kRight, q) == 0.0f);
    // reset-to-track zeroes the velocity and moves the car: no impulse on a freshly restored upright car
    Tracker r{base()};
    float s[3] = {15, 20, 0};
    float rp[3] = {0, 0, 0};
    r.step(s, kRight, rp);
    s[0] = s[1] = 0.0f;
    rp[1] = -60.0f;
    assert(r.step(s, kRight, rp) == 0.0f && r.steps_left() == 0);
    // NaN input resets, never produces a rate
    Tracker n{base()};
    float nv[3] = {0, 0, 0};
    n.step(nv, kRight, kPos);
    nv[0] = std::numeric_limits<float>::quiet_NaN();
    assert(n.step(nv, kRight, kPos) == 0.0f);
    nv[0] = 0.0f;
    assert(n.step(nv, kRight, kPos) == 0.0f);  // first sample after the reset
}

static void tracker_reset() {
    Tracker t{base()};
    float v[3] = {8, 0, 0};
    t.step(v, kRight, kPos);
    v[0] = 0.0f;
    assert(t.step(v, kRight, kPos) != 0.0f);
    t.reset();
    assert(t.steps_left() == 0 && t.step(v, kRight, kPos) == 0.0f);
}

static void apply_rate_rules() {
    assert(apply_rate(0.0f, 0.0f, 9.0f) == 0.0f);
    assert(near(apply_rate(0.0f, 6.0f, 9.0f), 6.0f));
    assert(near(apply_rate(1.0f, 6.0f, 9.0f), 6.0f));
    assert(near(apply_rate(8.0f, 6.0f, 9.0f), 8.0f));       // already faster the same way: untouched
    assert(near(apply_rate(-3.0f, 6.0f, 9.0f), 6.0f));      // the opposite way: the hit wins
    assert(near(apply_rate(0.0f, -20.0f, 9.0f), -9.0f));    // capped
    assert(near(apply_rate(12.0f, 20.0f, 9.0f), 12.0f));    // beyond the cap on its own: never slowed, never pushed further
    assert(near(apply_rate(-12.0f, 20.0f, 9.0f), 9.0f));    // beyond the cap the other way: the hit still only asks for the cap
    assert(near(apply_rate(5.0f, 0.0f, 9.0f), 5.0f));       // nothing held: untouched
    assert(std::isnan(apply_rate(std::numeric_limits<float>::quiet_NaN(), 5.0f, 9.0f)));  // garbage in the car's state is left alone
}

int main() {
    settings_defaults_and_invalid();
    style_changes_threshold_and_gain();
    roll_rate_threshold_and_cap();
    tracker_ignores_normal_driving();
    tracker_hit_signs_and_decay();
    tracker_other_side_and_stronger_hit();
    tracker_rotated_axis();
    tracker_ignores_teleports_and_junk();
    tracker_reset();
    apply_rate_rules();
    return 0;
}
