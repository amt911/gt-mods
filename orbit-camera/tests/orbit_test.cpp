#include "orbit_input.hpp"
#include "orbit_math.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>

using namespace orbit;

static bool near(float a, float b, float eps = 1e-4f) { return std::fabs(a - b) <= eps; }

// --- math ---------------------------------------------------------------------------------------------------------
static void sample_matrix(float m[16]) {  // a rotation about Y (0.3 rad) with a translation: a typical model-to-view matrix
    float c = std::cos(0.3f), s = std::sin(0.3f);
    const float v[16] = {c, 0, s, 0, 0, 1, 0, 0, -s, 0, c, 0, 1.5f, -2.0f, 7.0f, 1};
    std::memcpy(m, v, sizeof v);
}
static void row_times(const float x[3], const float m[16], float out[3]) {  // [x 1] * M, row vector
    for (int c = 0; c < 3; ++c) out[c] = x[0] * m[c] + x[1] * m[4 + c] + x[2] * m[8 + c] + m[12 + c];
}

static void zero_angle_is_identity() {
    float m[16], o[16];
    sample_matrix(m);
    const float pv[3] = {0.5f, -1.0f, 6.0f};
    orbit_matrix(m, pv, 0.0f, 0.0f, o);
    for (int i = 0; i < 16; ++i) assert(near(o[i], m[i]));
}
static void pivot_is_a_fixed_point() {
    float m[16], o[16];
    sample_matrix(m);
    const float model[3] = {0.7f, 0.2f, -1.3f};
    float pv[3];
    row_times(model, m, pv);  // the model point sits at pv in view space
    for (float yaw : {37.0f, 90.0f, 180.0f, -90.0f})
        for (float pitch : {0.0f, 10.0f}) {
            orbit_matrix(m, pv, yaw, pitch, o);
            float v[3];
            row_times(model, o, v);
            for (int k = 0; k < 3; ++k) assert(near(v[k], pv[k], 1e-3f));
        }
}
static void other_points_swing_around_the_pivot() {
    float m[16], o[16];
    sample_matrix(m);
    const float pv[3] = {0.0f, 0.0f, 10.0f};
    // a model point whose view position is pv + (1,0,0): after +90 deg yaw about pv it must be at the same distance
    float id[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    const float x[3] = {1.0f, 0.0f, 10.0f};
    orbit_matrix(id, pv, 90.0f, 0.0f, o);
    float v[3];
    row_times(x, o, v);
    assert(near(v[1], 0.0f, 1e-3f));
    assert(near(std::hypot(v[0] - pv[0], v[2] - pv[2]), 1.0f, 1e-3f));
    assert(near(v[0], pv[0], 1e-3f) && near(std::fabs(v[2] - pv[2]), 1.0f, 1e-3f));  // a quarter turn: x offset became z offset
    // the original matrix is not modified by the helper
    float m2[16];
    sample_matrix(m2);
    for (int i = 0; i < 16; ++i) assert(m[i] == m2[i]);
}
static void yaw_sign_puts_camera_on_the_stick_side() {
    // View space: the car is 10 ahead (z = 10, +z away from the camera is the view direction of this game's view space:
    // proven by frames, docs/FACTS.md). Yaw +90 must move the car's view-space position to the LEFT of the screen
    // (camera went to the car's right side), i.e. x < pivot x.
    float id[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}, o[16];
    const float pv[3] = {0, 0, 10};
    const float front_of_car[3] = {0, 0, 9};  // a point 1 unit nearer to the camera than the car
    orbit_matrix(id, pv, 90.0f, 0.0f, o);
    float v[3];
    row_times(front_of_car, o, v);
    assert(v[0] > pv[0]);  // see kYawSignNote in orbit_math.hpp
}
static void pivot_at_eye_is_pure_rotation() {
    float id[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}, o[16];
    const float eye[3] = {0, 0, 0};
    orbit_matrix(id, eye, 90.0f, 0.0f, o);
    assert(near(o[12], 0) && near(o[13], 0) && near(o[14], 0));
}
static void perspective_test() {
    assert(is_perspective(-1.0f));
    assert(!is_perspective(0.0f));  // ortho (HUD): m[11] = 0
    assert(!is_perspective(-0.2f));
}
static void pivot_in_view_space() {
    // camera-to-world: identity basis, eye at (1,2,3); car at (4,2,3) -> view-space (3,0,0)
    float cam[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 2, 3, 1}, pv[3];
    const float car[3] = {4, 2, 3};
    pivot_in_view(cam, car, pv);
    assert(near(pv[0], 3) && near(pv[1], 0) && near(pv[2], 0));
    // rotated camera (yaw 90 about Y): rows = basis vectors
    float cam2[16] = {0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
    const float car2[3] = {0, 0, 5};
    pivot_in_view(cam2, car2, pv);
    assert(near(pv[0], -5) && near(pv[1], 0) && near(pv[2], 0));
}
static void car_position_to_y_up() {
    float p[3];
    car_to_yup(1.0f, 2.0f, 3.0f, p);  // (x, y, z) -> (x, z, -y)
    assert(p[0] == 1.0f && p[1] == 3.0f && p[2] == -2.0f);
}

// --- input --------------------------------------------------------------------------------------------------------
static void settings_defaults_and_invalid() {
    std::map<std::string, std::string> kv;
    std::string warned;
    auto get = [&](const char *k) -> const char * { auto it = kv.find(k); return it == kv.end() ? nullptr : it->second.c_str(); };
    auto warn = [&](const std::string &w) { warned += w + ";"; };
    Settings s = parse_settings(get, warn);
    assert(near(s.deadzone, 0.25f) && near(s.back_threshold, 0.7f) && near(s.smooth_ms, 80.0f) && near(s.pitch_deg, 0.0f));
    assert(warned.empty());
    kv["deadzone"] = "0.4"; kv["smooth_ms"] = "0"; kv["pitch_deg"] = "5";
    s = parse_settings(get, warn);
    assert(near(s.deadzone, 0.4f) && s.smooth_ms == 0.0f && near(s.pitch_deg, 5.0f));
    kv["deadzone"] = "1.5"; kv["back_threshold"] = "abc"; kv["smooth_ms"] = "-3"; kv["pitch_deg"] = "80";
    warned.clear();
    s = parse_settings(get, warn);
    assert(near(s.deadzone, 0.25f) && near(s.back_threshold, 0.7f) && near(s.smooth_ms, 80.0f) && near(s.pitch_deg, 0.0f));
    int n = 0;
    for (char ch : warned) n += ch == ';';
    assert(n == 4);
}

static void invert_outside_chase_setting() {
    std::map<std::string, std::string> kv;
    std::string warned;
    auto get = [&](const char *k) -> const char * { auto it = kv.find(k); return it == kv.end() ? nullptr : it->second.c_str(); };
    auto warn = [&](const std::string &w) { warned += w + ";"; };
    assert(parse_settings(get, warn).invert_outside_chase);  // default: GT7-like, on
    kv["invert_outside_chase"] = "false";
    assert(!parse_settings(get, warn).invert_outside_chase && warned.empty());
    kv["invert_outside_chase"] = "true";
    assert(parse_settings(get, warn).invert_outside_chase && warned.empty());
    kv["invert_outside_chase"] = "0";
    assert(!parse_settings(get, warn).invert_outside_chase);
    kv["invert_outside_chase"] = "1";
    assert(parse_settings(get, warn).invert_outside_chase && warned.empty());
    kv["invert_outside_chase"] = "maybe";  // invalid: the default (true) is kept, not the value of an earlier key
    assert(parse_settings(get, warn).invert_outside_chase && warned.find("invert_outside_chase") != std::string::npos);
}

static void full_chase_view_by_camera_distance() {
    // measured (chase route, docs/FACTS of the host repo): far chase ~(0, -1.60, -9.01), near chase ~(0, -1.69, -3.82),
    // cockpit ~(0.3, -1.02, -0.63), bumper ~(0, -0.94, -0.38), race intro ~(0, -0.2, -4.5)
    const float far_chase[3] = {0.0f, -1.60f, -9.01f}, near_chase[3] = {0.0f, -1.69f, -3.82f}, bumper[3] = {0.0f, -0.94f, -0.38f};
    const float intro[3] = {0.36f, -0.44f, -5.30f};
    for (bool was : {false, true}) {
        assert(is_full_chase_view(far_chase, was));
        assert(!is_full_chase_view(near_chase, was) && !is_full_chase_view(bumper, was) && !is_full_chase_view(intro, was));
    }
    const float d65[3] = {0.0f, 0.0f, -6.5f};  // between the thresholds: keeps the last answer
    assert(is_full_chase_view(d65, true) && !is_full_chase_view(d65, false));
    const float d71[3] = {0.0f, 0.0f, -7.1f}, d59[3] = {0.0f, 0.0f, -5.9f};
    assert(is_full_chase_view(d71, false) && !is_full_chase_view(d59, true));
}

static void view_yaw_inverts_only_outside_chase() {
    Settings s;
    assert(view_yaw(90.0f, true, s) == 90.0f);     // chase view: as the stick says
    assert(view_yaw(90.0f, false, s) == -90.0f);   // bumper/cockpit/hood: mirrored, stick right looks the other way round
    assert(view_yaw(-35.0f, false, s) == 35.0f);
    assert(view_yaw(0.0f, false, s) == 0.0f);      // no look stays the identity (the mod writes nothing)
    assert(std::fabs(view_yaw(180.0f, false, s)) == 180.0f);  // rear view stays the rear view
    s.invert_outside_chase = false;
    assert(view_yaw(90.0f, false, s) == 90.0f);
}
static void stick_dead_zone_gives_no_look() {
    Settings s;
    Raw r;
    r.x = 0.2f; r.y = 0.1f;
    assert(target_yaw(r, s) == 0.0f);
    r.x = 0; r.y = -0.24f;
    assert(target_yaw(r, s) == 0.0f);
}
static void stick_direction_picks_the_angle() {
    Settings s;
    Raw r;
    r.x = 1.0f; assert(near(target_yaw(r, s), 90.0f));
    r.x = -1.0f; assert(near(target_yaw(r, s), -90.0f));
    r.x = 0.0f; r.y = -1.0f; assert(near(target_yaw(r, s), 0.0f));  // stick forward: look ahead
    r.x = 0.8f; r.y = -0.8f; assert(near(target_yaw(r, s), 45.0f, 0.01f));
    r.x = 0.6f; r.y = 0.5f; assert(near(target_yaw(r, s), 129.8f, 0.2f));  // diagonal back, not past the back threshold
}
static void stick_back_or_r3_is_rear_view() {
    Settings s;
    Raw r;
    r.y = 0.9f; assert(target_yaw(r, s) == 180.0f);
    r.x = 0.0f; r.y = 1.0f; assert(target_yaw(r, s) == 180.0f);
    r.x = 0.3f; r.y = 1.0f; assert(near(target_yaw(r, s), 180.0f)); // full back: rear view whatever x
    r = Raw{}; r.r3 = true; assert(target_yaw(r, s) == 180.0f);
    r = Raw{}; r.y = 0.5f; assert(near(std::fabs(target_yaw(r, s)), 180.0f));
}
static void back_blend_is_continuous() {
    Settings s;
    Raw r;
    // just below / above the threshold: no jump (the old snap went from ~160 to 180)
    r.x = 0.3f; r.y = s.back_threshold - 0.001f;
    const float below = target_yaw(r, s);
    r.y = s.back_threshold + 0.001f;
    const float above = target_yaw(r, s);
    assert(std::fabs(above - below) < 0.5f);
    r.y = 0.85f; // halfway: between the angle and 180
    const float mid = target_yaw(r, s);
    assert(mid > below && mid < 180.0f);
    // back-left mirrors back-right (sign preserved, so the camera goes round the side of the stick)
    Raw l; l.x = -0.3f; l.y = 0.85f;
    assert(near(target_yaw(l, s), -mid));
    // monotonic in y
    float prev = -1.0f;
    for (float y = 0.71f; y <= 1.0f; y += 0.03f) { r.y = y; const float v = target_yaw(r, s); assert(v >= prev); prev = v; }
}
static void step_gate_is_game_paced() {
    StepGate g(10);
    assert(!g.live());              // no step yet
    g.on_step(); assert(g.live());
    for (int i = 0; i < 10; ++i) g.on_frame();
    assert(g.live());               // 10 frames without a step: still live (slow machine, a hitch)
    g.on_frame();
    assert(!g.live());              // 11: the steps stopped (menu, pause, replay)
    g.on_step(); assert(g.live());  // steps are back
    for (int i = 0; i < 100000; ++i) g.on_frame();  // no overflow
    assert(!g.live());
}
static void step_time_is_nominal() { assert(near(kStepMs, 16.6667f, 0.001f)); }
static void keys_set_fixed_targets() {
    Settings s;
    Raw r;
    r.left = true; assert(target_yaw(r, s) == -90.0f);
    r = Raw{}; r.right = true; assert(target_yaw(r, s) == 90.0f);
    r.left = true; assert(target_yaw(r, s) == 0.0f);  // both: cancel
    r = Raw{}; r.back = true; assert(target_yaw(r, s) == 180.0f);
    r.left = true; assert(target_yaw(r, s) == 180.0f);  // back wins
}
static void smoother_converges_and_snaps() {
    YawSmoother sm;
    float v = 0;
    for (int i = 0; i < 100; ++i) v = sm.step(90.0f, 16.0f, 80.0f);
    assert(v == 90.0f);  // snapped, exact
    v = sm.step(90.0f, 16.0f, 80.0f);
    assert(v == 90.0f);
}
static void smoother_time_constant() {
    YawSmoother sm;
    float v = sm.step(90.0f, 80.0f, 80.0f);  // one time constant: 1 - 1/e of the way
    assert(near(v, 90.0f * (1.0f - std::exp(-1.0f)), 0.01f));
}
static void smoother_zero_time_is_instant() {
    YawSmoother sm;
    assert(sm.step(37.0f, 16.0f, 0.0f) == 37.0f);
}
static void smoother_takes_the_short_way_round() {
    YawSmoother sm;
    sm.reset(170.0f);
    float v = sm.step(-170.0f, 16.0f, 80.0f);
    assert(v > 170.0f || v < -170.0f);  // went through 180, not back through 0
    for (int i = 0; i < 100; ++i) v = sm.step(-170.0f, 16.0f, 80.0f);
    assert(v == -170.0f);
}
static void smoother_release_returns_to_zero_exactly() {
    YawSmoother sm;
    sm.reset(180.0f);
    float v = 180.0f;
    for (int i = 0; i < 200; ++i) v = sm.step(0.0f, 16.0f, 80.0f);
    assert(v == 0.0f);
}
static void fake_parsing() {
    assert(!parse_fake(nullptr) && !parse_fake("") && !parse_fake("abc") && !parse_fake("400") && !parse_fake("10,99"));
    auto f = parse_fake("90");
    assert(f && f->first == 90.0f && f->second == 0.0f);
    f = parse_fake("-180,5");
    assert(f && f->first == -180.0f && f->second == 5.0f);
    f = parse_fake("0");
    assert(f && f->first == 0.0f);
}
static void needs_edit_only_off_identity() {
    assert(!needs_edit(0.0f, 0.0f));
    assert(needs_edit(0.01f, 0.0f));
    assert(needs_edit(0.0f, 3.0f));
}

int main() {
    zero_angle_is_identity();
    pivot_is_a_fixed_point();
    other_points_swing_around_the_pivot();
    yaw_sign_puts_camera_on_the_stick_side();
    pivot_at_eye_is_pure_rotation();
    perspective_test();
    pivot_in_view_space();
    car_position_to_y_up();
    settings_defaults_and_invalid();
    invert_outside_chase_setting();
    view_yaw_inverts_only_outside_chase();
    full_chase_view_by_camera_distance();
    stick_dead_zone_gives_no_look();
    stick_direction_picks_the_angle();
    stick_back_or_r3_is_rear_view();
    back_blend_is_continuous();
    step_gate_is_game_paced();
    step_time_is_nominal();
    keys_set_fixed_targets();
    smoother_converges_and_snaps();
    smoother_time_constant();
    smoother_zero_time_is_instant();
    smoother_takes_the_short_way_round();
    smoother_release_returns_to_zero_exactly();
    fake_parsing();
    needs_edit_only_off_identity();
    std::puts("orbit_test: all passed");
    return 0;
}
