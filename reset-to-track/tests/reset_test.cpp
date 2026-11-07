#include "reset_logic.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

using namespace reset;

static CarSample good_sample() { return CarSample{4, 1.0f, 20.0f, false}; }
static Pose pose_at(float x) {
    Pose p;
    p.v[0] = x;
    return p;
}
static Settings steps_settings() {  // 1 step = 1000/60 ms; round numbers in steps
    Settings s;
    s.hold_ms = 10 * kStepMs;
    s.cooldown_ms = 30 * kStepMs;
    s.rewind_ms = 60 * kStepMs;
    return s;
}

// --- settings --------------------------------------------------------------------------------------------------------
static Settings parse(const std::map<std::string, std::string> &m, std::vector<std::string> *warns) {
    return parse_settings([&](const char *k) -> const char * { auto it = m.find(k); return it == m.end() ? nullptr : it->second.c_str(); },
                          [&](const std::string &w) { warns->push_back(w); });
}
static void settings_defaults_and_invalid() {
    std::vector<std::string> w;
    Settings d = parse({}, &w);
    assert(w.empty() && d.hold_ms == 500.0f && d.cooldown_ms == 1000.0f && d.rewind_ms == 2500.0f && d.sabotage == 0);
    Settings s = parse({{"hold_ms", "250"}, {"cooldown_ms", "2000"}, {"rewind_ms", "3000"}}, &w);
    assert(w.empty() && s.hold_ms == 250.0f && s.cooldown_ms == 2000.0f && s.rewind_ms == 3000.0f);
    w.clear();
    s = parse({{"hold_ms", "abc"}, {"cooldown_ms", "-1"}, {"rewind_ms", "99999"}}, &w);
    assert(w.size() == 3 && s.hold_ms == 500.0f && s.cooldown_ms == 1000.0f && s.rewind_ms == 2500.0f);
    w.clear();
    s = parse({{"hold_ms", "0"}}, &w);  // an instant press is allowed
    assert(w.empty() && s.hold_ms == 0.0f);
    s = parse({{"rewind_ms", "100"}}, &w);  // below the ring's spacing: invalid
    assert(w.size() == 1 && s.rewind_ms == 2500.0f);
}
static void ms_to_steps() {
    assert(steps_for_ms(500.0f) == 30 && steps_for_ms(1000.0f) == 60 && steps_for_ms(0.0f) == 0 && steps_for_ms(8.0f) == 0);
    assert(steps_for_ms(2500.0f) == 150);
}

// --- good pose -------------------------------------------------------------------------------------------------------
static void good_pose_predicate() {
    assert(is_good(good_sample()));
    CarSample s = good_sample();
    s.wheels_on_asphalt = 3;
    assert(!is_good(s));
    s = good_sample(); s.up_z = 0.9f; assert(!is_good(s));  // strictly above 0.9
    s = good_sample(); s.up_z = 0.91f; assert(is_good(s));
    s = good_sample(); s.speed = 5.0f; assert(!is_good(s));
    s = good_sample(); s.speed = 5.1f; assert(is_good(s));
    s = good_sample(); s.flag_bit2 = true; assert(!is_good(s));
    s = good_sample(); s.up_z = NAN; assert(!is_good(s));
    s = good_sample(); s.speed = INFINITY; assert(!is_good(s));
}

// --- button ----------------------------------------------------------------------------------------------------------
static void hold_fires_once_after_the_hold() {
    HoldDetector h(5);
    for (int i = 0; i < 4; ++i) assert(!h.update(true));
    assert(h.update(true));                 // the 5th consecutive step
    for (int i = 0; i < 20; ++i) assert(!h.update(true));  // still held: no repeat
    assert(!h.update(false));
    for (int i = 0; i < 4; ++i) assert(!h.update(true));
    assert(h.update(true));                 // released and pressed again
}
static void short_press_does_nothing() {
    HoldDetector h(5);
    for (int round = 0; round < 3; ++round) {
        for (int i = 0; i < 4; ++i) assert(!h.update(true));
        assert(!h.update(false));
    }
}
static void zero_hold_fires_on_the_press() {
    HoldDetector h(0);
    assert(h.update(true));
    assert(!h.update(true));
    assert(!h.update(false));
    assert(h.update(true));
}

// --- ring ------------------------------------------------------------------------------------------------------------
static void ring_keeps_only_the_newest_capacity() {
    PoseRing r(3);
    for (int i = 0; i < 5; ++i) r.push(i * 10, pose_at(float(i)));
    assert(r.size() == 3);
    const Pose *p = r.pick(100, 0);
    assert(p && p->v[0] == 4.0f);  // newest
    r.clear();
    assert(r.size() == 0 && r.pick(100, 0) == nullptr);
}
static void pick_takes_the_newest_old_enough() {
    PoseRing r(8);
    for (int i = 0; i < 6; ++i) r.push(i * 15, pose_at(float(i)));  // steps 0,15,...,75
    const Pose *p = r.pick(90, 60);  // age >= 60: step <= 30 -> the one at 30
    assert(p && p->v[0] == 2.0f);
    p = r.pick(90, 15);              // step <= 75
    assert(p && p->v[0] == 5.0f);
    p = r.pick(90, 0);
    assert(p && p->v[0] == 5.0f);
}
static void pick_falls_back_to_the_oldest_when_all_too_young() {
    PoseRing r(8);
    r.push(70, pose_at(7.0f));
    r.push(85, pose_at(8.0f));
    const Pose *p = r.pick(90, 60);
    assert(p && p->v[0] == 7.0f);
}

// --- resetter --------------------------------------------------------------------------------------------------------
static int drive(Resetter &rs, int steps, float x0, bool held, std::optional<Pose> *out, const CarSample &s = good_sample()) {
    int fired = 0;
    for (int i = 0; i < steps; ++i) {
        auto r = rs.step(s, pose_at(x0 + i), held);
        if (r) { ++fired; *out = r; }
    }
    return fired;
}
static void records_every_interval_only_good_poses() {
    Resetter rs(steps_settings());
    std::optional<Pose> out;
    CarSample bad = good_sample();
    bad.wheels_on_asphalt = 2;
    drive(rs, 100, 0.0f, false, &out, bad);
    assert(rs.recorded() == 0);
    drive(rs, 100, 100.0f, false, &out);
    assert(rs.recorded() >= 100 / kPoseEveryNSteps - 1 && rs.recorded() <= 100 / kPoseEveryNSteps + 1);
}
static void button_with_empty_ring_does_nothing() {
    Resetter rs(steps_settings());
    std::optional<Pose> out;
    CarSample slow = good_sample();
    slow.speed = 1.0f;
    assert(drive(rs, 50, 0.0f, true, &out, slow) == 0);
    assert(!out);
}
static void restore_picks_the_pose_from_the_rewind_time() {
    Resetter rs(steps_settings());  // rewind 60 steps
    std::optional<Pose> out;
    drive(rs, 200, 0.0f, false, &out);  // x == step index
    CarSample off = good_sample();
    off.wheels_on_asphalt = 0;           // off the road: nothing more is recorded
    assert(drive(rs, 5, 200.0f, false, &out, off) == 0);
    int fired = 0;
    for (int i = 0; i < 12 && !fired; ++i)
        if (auto r = rs.step(off, pose_at(205.0f + i), true)) { ++fired; out = r; }
    assert(fired == 1 && out);
    const float x = out->v[0];
    // restored pose is at least `rewind` steps older than the press (press at ~step 214) and the newest such one
    assert(x <= 214.0f - 60.0f + 1.0f && x >= 214.0f - 60.0f - float(kPoseEveryNSteps) - 2.0f);
}
static void cooldown_blocks_a_second_reset_then_allows_it() {
    Resetter rs(steps_settings());  // hold 10, cooldown 30
    std::optional<Pose> out;
    drive(rs, 200, 0.0f, false, &out);
    int fired = 0;
    for (int i = 0; i < 10; ++i) fired += rs.step(good_sample(), pose_at(200.0f), true).has_value();
    assert(fired == 1);
    // release, press again at once: the hold completes inside the cooldown -> nothing
    rs.step(good_sample(), pose_at(200.0f), false);
    for (int i = 0; i < 10; ++i) fired += rs.step(good_sample(), pose_at(200.0f), true).has_value();
    assert(fired == 1);
    for (int i = 0; i < 30; ++i) rs.step(good_sample(), pose_at(200.0f), false);
    for (int i = 0; i < 10; ++i) fired += rs.step(good_sample(), pose_at(200.0f), true).has_value();
    assert(fired == 2);
}
static void a_reset_keeps_the_restored_pose_as_the_only_one() {
    Resetter rs(steps_settings());
    std::optional<Pose> out;
    drive(rs, 200, 0.0f, false, &out);
    for (int i = 0; i < 10; ++i) if (auto r = rs.step(good_sample(), pose_at(500.0f), true)) out = r;
    assert(out);
    const float first = out->v[0];
    assert(rs.recorded() == 1);
    CarSample rest = good_sample();
    rest.speed = 0.0f;  // from rest: not recorded
    for (int i = 0; i < 60; ++i) rs.step(rest, pose_at(first), false);
    assert(rs.recorded() == 1);
    out.reset();
    for (int i = 0; i < 10; ++i) if (auto r = rs.step(rest, pose_at(first), true)) out = r;  // second press after the cooldown
    assert(out && out->v[0] == first);
}
static void losing_the_car_clears_everything() {
    Resetter rs(steps_settings());
    std::optional<Pose> out;
    drive(rs, 100, 0.0f, false, &out);
    assert(rs.recorded() > 0);
    rs.lose_car();
    assert(rs.recorded() == 0);
    assert(drive(rs, 12, 0.0f, true, &out) == 0);  // the ring is empty again: a press does nothing
}
static void sabotage_newest_records_everything_and_restores_the_latest() {
    Settings s = steps_settings();
    s.sabotage = 2;
    Resetter rs(s);
    std::optional<Pose> out;
    CarSample off = good_sample();
    off.wheels_on_asphalt = 0;
    off.speed = 0.0f;
    for (int i = 0; i < 40; ++i) rs.step(off, pose_at(float(i)), false);
    for (int i = 0; i < 10; ++i) if (auto r = rs.step(off, pose_at(100.0f), true)) out = r;
    assert(out && out->v[0] <= 100.0f && out->v[0] >= 90.0f);  // a pose from within a few steps: the off-track one
}
static void fake_parsing() {
    assert(!parse_fake(nullptr) && !parse_fake("") && !parse_fake("abc") && !parse_fake("-3"));
    assert(parse_fake("1900") && *parse_fake("1900") == 1900u);
}
static void fake_held_window() {
    assert(!fake_held(1899, 1900) && fake_held(1900, 1900) && fake_held(1900 + kFakeHeldSteps - 1, 1900) && !fake_held(1900 + kFakeHeldSteps, 1900));
}

int main() {
    settings_defaults_and_invalid();
    ms_to_steps();
    good_pose_predicate();
    hold_fires_once_after_the_hold();
    short_press_does_nothing();
    zero_hold_fires_on_the_press();
    ring_keeps_only_the_newest_capacity();
    pick_takes_the_newest_old_enough();
    pick_falls_back_to_the_oldest_when_all_too_young();
    records_every_interval_only_good_poses();
    button_with_empty_ring_does_nothing();
    restore_picks_the_pose_from_the_rewind_time();
    cooldown_blocks_a_second_reset_then_allows_it();
    a_reset_keeps_the_restored_pose_as_the_only_one();
    losing_the_car_clears_everything();
    sabotage_newest_records_everything_and_restores_the_latest();
    fake_parsing();
    fake_held_window();
    std::puts("reset_test: all passed");
    return 0;
}
