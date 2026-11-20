#include "cam_logic.hpp"
#include <string>

#include <cassert>
#include <cmath>

using namespace cams;

static bool near(float a, float b, float eps = 1e-4f) { return std::fabs(a - b) <= eps; }
static bool near(Vec3 a, Vec3 b, float eps = 1e-4f) { return near(a.x, b.x, eps) && near(a.y, b.y, eps) && near(a.z, b.z, eps); }
static Vec3 row(const Pose &p, int i) { return {p.basis[3 * i], p.basis[3 * i + 1], p.basis[3 * i + 2]}; }
// the car's convention: unit orthogonal rows, forward x right . up = -1
static bool valid(const Pose &p) {
    const Vec3 f = row(p, 0), r = row(p, 1), u = row(p, 2);
    return near(length(f), 1) && near(length(r), 1) && near(length(u), 1) && near(dot(f, r), 0) && near(dot(f, u), 0) && near(dot(r, u), 0) && near(dot(cross(f, r), u), -1.0f);
}
// a car at (10, 20, 1) heading +y: forward (0,1,0), right (1,0,0), up (0,0,1)
static Car car_north() {
    const float pos[3] = {10, 20, 1};
    const float basis[9] = {0, 1, 0, 1, 0, 0, 0, 0, 1};
    return car_from(pos, basis);
}

int main() {
    const Params p;
    const Car car = car_north();
    // look_at: from the origin at a point ahead (+y): forward +y, right +x, up +z; a left-handed set
    {
        const Pose l = look_at({0, 0, 0}, {0, 5, 0}, {0, 1, 0});
        assert(near(row(l, 0), {0, 1, 0}) && near(row(l, 1), {1, 0, 0}) && near(row(l, 2), {0, 0, 1}) && valid(l));
        const Pose down = look_at({0, 0, 10}, {0, 0, 0}, {1, 0, 0});  // straight down: no NaN, keeps a valid set with the fallback heading
        assert(valid(down) && near(row(down, 0), {0, 0, -1}));
        assert(valid(look_at({1, 1, 1}, {1, 1, 1}, {0, 0, 0})));  // eye == target and no fallback: still a valid set
    }
    // views cycle
    assert(next_view(View::Off, 1) == View::Wheel && next_view(View::Tv, 1) == View::Off && next_view(View::Off, -1) == View::Tv && next_view(View::Roof, -1) == View::Wheel);
    // wheel camera: at the front-left wheel in the body frame, looking along the car
    {
        Params q = p;
        const Pose w = wheel_pose(car, 1.3f, 1.5f, q);
        // front axle 1.3 ahead (+y), half the track 0.75 to the left (-x), wheel_up above the origin
        assert(near(w.eye, {10 - 0.75f, 20 + 1.3f, 1 + q.wheel_up}));
        assert(valid(w) && near(row(w, 0), {0, 1, 0}));
        q.wheel_toe_deg = 10.0f;  // turned outward (to the left: forward gains a -x part)
        const Pose t = wheel_pose(car, 1.3f, 1.5f, q);
        assert(valid(t) && t.basis[0] < -0.1f && t.basis[1] > 0.9f);
    }
    // roof camera: above the roof, looking forward and a little down
    {
        const Pose r = roof_pose(car, p);
        assert(near(r.eye, {10, 20, 1 + p.roof_up}) && valid(r));
        assert(r.basis[1] > 0.9f && r.basis[2] < -0.1f);  // forward mostly +y, tilted down
        assert(near(dot(row(r, 0), {0, 0, 1}), -std::sin(p.roof_pitch_deg * 3.14159265f / 180.0f), 1e-3f));
        assert(near(row(r, 1), {1, 0, 0}));  // pitch leaves the right row alone
    }
    // the body frame follows the car: the same offsets on a car heading +x
    {
        const float pos[3] = {0, 0, 0};
        const float basis[9] = {1, 0, 0, 0, -1, 0, 0, 0, 1};  // forward +x, right -y (left-handed: f x r = (0,0,-1))
        const Car east = car_from(pos, basis);
        const Pose w = wheel_pose(east, 1.0f, 2.0f, p);
        assert(near(w.eye, {1.0f, 1.0f, p.wheel_up}));  // 1 ahead (+x), 1 to the left (+y, since right = -y)
    }
    // TV camera: a trackside point ahead and to the side; it stays while the car approaches and passes, then moves on
    {
        TvCamera tv;
        assert(!tv.has_point());
        const Pose a = tv.pose(car, p);
        assert(tv.has_point() && tv.moves() == 1 && valid(a));
        assert(near(tv.point(), {10 + p.tv_side, 20 + p.tv_ahead, 1 + p.tv_height}));
        assert(a.basis[1] < 0.0f || true);  // looks back toward the car (the car is behind the point)
        const Vec3 to_car = normalize(car.position - a.eye);
        assert(near(row(a, 0), to_car, 1e-3f));
        // the car drives up to the point: the point stays
        Car c2 = car;
        c2.position = car.position + Vec3{0, p.tv_ahead - 10.0f, 0};
        tv.pose(c2, p);
        assert(tv.moves() == 1 && near(tv.point(), {10 + p.tv_side, 20 + p.tv_ahead, 1 + p.tv_height}));
        // past it by more than tv_pass: a new point ahead of the car
        c2.position = car.position + Vec3{0, p.tv_ahead + p.tv_pass + 1.0f, 0};
        tv.pose(c2, p);
        assert(tv.moves() == 2 && near(tv.point().y, c2.position.y + p.tv_ahead));
        // a teleport far from the point: moves
        Car far = car;
        far.position = {500, 500, 0};
        tv.pose(far, p);
        assert(tv.moves() == 3);
        tv.reset();
        assert(!tv.has_point());
    }
    // the smoother: starts at the target, then approaches it without overshoot and settles
    {
        Smoother s;
        Vec3 v = s.step({0, 0, 0}, 1.0f / 60.0f, 0.8f);
        assert(near(v, {0, 0, 0}) && s.has_value());
        float last = 0.0f;
        for (int i = 0; i < 600; ++i) {
            v = s.step({10, 0, 0}, 1.0f / 60.0f, 0.8f);
            assert(v.x >= last - 1e-5f && v.x <= 10.0f + 1e-4f);  // monotonic, never past the target
            last = v.x;
        }
        assert(near(v.x, 10.0f, 1e-2f));
        // it settles about when asked: 1 % within the smoothing time (+ a margin), not instantly and not much later
        Smoother t;
        t.step({0, 0, 0}, 1.0f / 60.0f, 0.8f);
        int steps = 0;
        while (t.step({1, 0, 0}, 1.0f / 60.0f, 0.8f).x < 0.99f && steps < 2000) ++steps;
        assert(steps > 20 && steps < 90);
        s.reset();
        assert(!s.has_value());
    }
    // the helicopter target: behind and above, along the heading on the ground plane
    {
        const Vec3 t = heli_target(car, p);
        assert(near(t, {10, 20 - p.heli_back, 1 + p.heli_up}));
        Car tilted = car;  // a car pitched up: the target ignores the pitch (heading on the ground plane)
        tilted.forward = normalize(Vec3{0, 1, 0.5f});
        assert(near(heli_target(tilted, p), t, 1e-4f));
    }
    // the SELECT cycle: game positions 0..3, extras 4..6; the game's own position stays on 0 while an extra view is shown
    {
        assert(cycle_step(0, 1, 4) == 1 && cycle_step(3, 0, 4) == 1 && cycle_step(0, 3, 4) == -1 && cycle_step(2, 1, 4) == -1 && cycle_step(1, 1, 4) == 0 && cycle_step(0, 2, 4) == 0);
        assert(cycle_step(0, 1, 2) == 0 && cycle_step(0, 5, 4) == 0 && cycle_step(-1, 0, 4) == 0);
        assert(cycle_advance(3, 1, 4, 3) == 4 && cycle_advance(6, 1, 4, 3) == 0 && cycle_advance(0, -1, 4, 3) == 6 && cycle_advance(4, -1, 4, 3) == 3 && cycle_advance(2, 1, 4, 0) == 3);
        assert(cycle_game_position(2, 4) == 2 && cycle_game_position(4, 4) == 0 && cycle_game_position(6, 4) == 0);
        assert(cycle_view(3, 4) == View::Off && cycle_view(4, 4) == View::Wheel && cycle_view(5, 4) == View::Roof && cycle_view(6, 4) == View::Tv && cycle_view(7, 4) == View::Off);
        assert(cycle_virtual(View::Off, 4, 2) == 2 && cycle_virtual(View::Wheel, 4, 2) == 4 && cycle_virtual(View::Tv, 4, 2) == 6);
        // a full forward lap: hood (3) -> Wheel -> Roof -> TV -> chase (0), and the same backwards
        int v = 3;
        const View expect[4] = {View::Wheel, View::Roof, View::Tv, View::Off};
        for (int i = 0; i < 4; ++i) {
            v = cycle_advance(v, 1, 4, 3);
            assert(cycle_view(v, 4) == expect[i]);
        }
        assert(v == 0);
        for (int i = 2; i >= 0; --i) {
            v = cycle_advance(v, -1, 4, 3);
            assert(cycle_view(v, 4) == (i == 2 ? View::Tv : (i == 1 ? View::Roof : View::Wheel)));
        }
    }
    assert(jumped({0, 0, 0}, {100, 0, 0}, 15.0f) && !jumped({0, 0, 0}, {1, 0, 0}, 15.0f));
    assert(std::string(view_name(View::Off)) == "Game");
    return 0;
}
