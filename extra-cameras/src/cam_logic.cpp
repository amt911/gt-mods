#include "cam_logic.hpp"

#include <cmath>

namespace cams {

namespace {
constexpr float kDegToRad = 3.14159265358979f / 180.0f;
}

Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 operator*(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 cross(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
float length(Vec3 a) { return std::sqrt(dot(a, a)); }
Vec3 normalize(Vec3 a) {
    const float n = length(a);
    return n > 1e-9f ? a * (1.0f / n) : Vec3{};
}

static void set_rows(Pose &p, Vec3 f, Vec3 r, Vec3 u) {
    const Vec3 rows[3] = {f, r, u};
    for (int i = 0; i < 3; ++i) {
        p.basis[3 * i] = rows[i].x;
        p.basis[3 * i + 1] = rows[i].y;
        p.basis[3 * i + 2] = rows[i].z;
    }
}

Pose look_at(Vec3 eye, Vec3 target, Vec3 fallback_forward) {
    Pose p;
    p.eye = eye;
    Vec3 f = normalize(target - eye);
    if (length(f) == 0.0f) f = normalize(fallback_forward);
    if (length(f) == 0.0f) f = {0, 1, 0};
    Vec3 r = cross(f, Vec3{0, 0, 1});
    if (length(r) < 1e-4f) {  // straight up or down: keep the fallback's heading for the roll
        const Vec3 h = normalize(Vec3{fallback_forward.x, fallback_forward.y, 0});
        r = cross(length(h) > 0.0f ? h : Vec3{0, 1, 0}, Vec3{0, 0, 1});
    }
    r = normalize(r);
    const Vec3 u = normalize(cross(r, f));
    set_rows(p, f, r, u);
    return p;
}

Car car_from(const float position[3], const float basis[9]) {
    Car c;
    c.position = {position[0], position[1], position[2]};
    c.forward = {basis[0], basis[1], basis[2]};
    c.right = {basis[3], basis[4], basis[5]};
    c.up = {basis[6], basis[7], basis[8]};
    return c;
}

View next_view(View v, int direction) {
    const int n = (static_cast<int>(v) + (direction < 0 ? kViewCount - 1 : 1)) % kViewCount;
    return static_cast<View>(n);
}

int cycle_step(int p0, int p1, int count) {
    if (count < 3 || p0 == p1 || p0 < 0 || p1 < 0 || p0 >= count || p1 >= count) return 0;  // with two views a step is ambiguous
    if (p1 == (p0 + 1) % count) return 1;
    if (p1 == (p0 + count - 1) % count) return -1;
    return 0;
}

int cycle_advance(int v, int dir, int count, int extra) {
    const int n = count + extra;
    if (n <= 0) return 0;
    return ((v + (dir < 0 ? -1 : 1)) % n + n) % n;
}

int cycle_game_position(int v, int count) { return v < count ? v : 0; }

View cycle_view(int v, int count) {
    const int e = v - count + 1;
    return e >= 1 && e < kViewCount ? static_cast<View>(e) : View::Off;
}

int cycle_virtual(View view, int count, int game_position) { return view == View::Off ? game_position : count + static_cast<int>(view) - 1; }

const char *view_name(View v) {
    switch (v) {
    case View::Off: return "Game";
    case View::Wheel: return "Wheel";
    case View::Roof: return "Roof";
    case View::Tv: return "TV";
    }
    return "?";
}

// Turns the view about the body's up axis (positive = to the right) and down about its right axis.
static Pose oriented(const Car &car, Vec3 eye, float yaw_deg, float pitch_down_deg) {
    const float y = yaw_deg * kDegToRad, d = pitch_down_deg * kDegToRad;
    // yaw: forward' = cos y f + sin y r ; right' = cos y r - sin y f
    const Vec3 f1 = car.forward * std::cos(y) + car.right * std::sin(y);
    const Vec3 r1 = car.right * std::cos(y) - car.forward * std::sin(y);
    // pitch down: forward'' = cos d f1 - sin d up ; up'' = cos d up + sin d f1
    const Vec3 f2 = f1 * std::cos(d) - car.up * std::sin(d);
    const Vec3 u2 = car.up * std::cos(d) + f1 * std::sin(d);
    Pose p;
    p.eye = eye;
    set_rows(p, f2, r1, u2);
    return p;
}

Pose wheel_pose(const Car &car, float front_axle, float track, const Params &p) {
    const Vec3 eye = car.position + car.forward * front_axle - car.right * (0.5f * track + p.wheel_out) + car.up * p.wheel_up;
    return oriented(car, eye, -p.wheel_toe_deg, 0.0f);  // the front-left wheel looks outward to the left
}

Pose roof_pose(const Car &car, const Params &p) {
    return oriented(car, car.position + car.up * p.roof_up + car.forward * p.roof_forward, 0.0f, p.roof_pitch_deg);
}

Pose TvCamera::pose(const Car &car, const Params &p) {
    const Vec3 heading = normalize(Vec3{car.forward.x, car.forward.y, 0});
    bool move = !have_;
    if (have_) {
        const float along = dot(car.position - point_, axis_);  // how far the car is beyond the point, along the heading it had when the point was chosen
        if (along > p.tv_pass) move = true;
        if (length(car.position - point_) > 2.5f * p.tv_ahead) move = true;  // a retry or a reset: the point is somewhere else
    }
    if (move) {
        const Vec3 side = normalize(Vec3{car.right.x, car.right.y, 0});
        point_ = car.position + heading * p.tv_ahead + side * p.tv_side + Vec3{0, 0, p.tv_height};
        axis_ = length(heading) > 0.0f ? heading : Vec3{0, 1, 0};
        have_ = true;
        ++moves_;
    }
    return look_at(point_, car.position, heading);
}

Vec3 Smoother::step(Vec3 target, float dt, float smooth_s) {
    if (!have_) {
        value_ = target;
        velocity_ = {};
        have_ = true;
        return value_;
    }
    // critically damped spring, exact solution over dt: omega from the settling time (~ 4.6 / omega for 1 %)
    const float omega = smooth_s > 1e-3f ? 4.6f / smooth_s : 1e3f;
    const float e = std::exp(-omega * dt);
    const Vec3 delta = value_ - target;
    const Vec3 temp = (velocity_ + delta * omega) * dt;
    velocity_ = (velocity_ - temp * omega) * e;
    value_ = target + (delta + temp) * e;
    return value_;
}

Vec3 heli_target(const Car &car, const Params &p) {
    const Vec3 heading = normalize(Vec3{car.forward.x, car.forward.y, 0});
    return car.position - heading * p.heli_back + Vec3{0, 0, p.heli_up};
}

bool jumped(Vec3 previous, Vec3 now, float limit) { return length(now - previous) > limit; }

}  // namespace cams
