// Extra cameras: the pure maths (no game access). Everything is in the physics frame (Z up) with the car's row convention: a basis is nine floats,
// the rows forward, right, up, left-handed (forward x right . up = -1), as GtCarState::basis and the SDK's camera_get / camera_set.
#pragma once

namespace cams {

struct Vec3 {
    float x = 0, y = 0, z = 0;
};
Vec3 operator+(Vec3 a, Vec3 b);
Vec3 operator-(Vec3 a, Vec3 b);
Vec3 operator*(Vec3 a, float s);
float dot(Vec3 a, Vec3 b);
Vec3 cross(Vec3 a, Vec3 b);
float length(Vec3 a);
Vec3 normalize(Vec3 a);  // a zero vector stays zero

struct Pose {
    Vec3 eye;
    float basis[9] = {0, 1, 0, 1, 0, 0, 0, 0, 1};  // forward, right, up
};

// Pose looking from `eye` at `target` with the world's up (Z) as the roll reference. A look straight up or down keeps `fallback_forward`'s heading.
Pose look_at(Vec3 eye, Vec3 target, Vec3 fallback_forward);

// The car, as the cameras read it: position and basis rows (forward, right, up) in the physics frame.
struct Car {
    Vec3 position;
    Vec3 forward, right, up;
};
Car car_from(const float position[3], const float basis[9]);

enum class View { Off = 0, Wheel = 1, Roof = 2, Tv = 3 };
constexpr int kViewCount = 4;
View next_view(View v, int direction);  // cycles Off -> Wheel -> Roof -> TV -> Off (direction +1) or backwards (-1)
const char *view_name(View v);

// ---- The game's SELECT cycle, extended (docs/MODS.md "API v9: view cycle" in gran-turismo-decomp). The game moves the position of its view list by +-1 with wrap (count = 4 in a race: chase, bumper,
// cockpit, hood). The mod keeps a virtual position v in [0, count + extra): below `count` it is the game's position, from `count` on it is an extra view (Wheel, Roof, TV) while the game's own
// position stays on its first view (the chase view, which draws the body). ----
// +1 / -1 when the game moved from p0 to p1 by one step (with wrap), 0 when it did not move or the move is no single step.
int cycle_step(int p0, int p1, int count);
// The virtual position after one step in `dir` (+-1), with wrap over count + extra.
int cycle_advance(int v, int dir, int count, int extra);
// The position the GAME's list must have for the virtual position v.
int cycle_game_position(int v, int count);
// The extra view of v (Off below `count`).
View cycle_view(int v, int count);
// The virtual position of an extra view chosen from the menu (Off: the game's own position `game_position`).
int cycle_virtual(View view, int count, int game_position);

struct Params {
    float wheel_up = 0.15f;       // m above the body origin: low, at the wheel arch
    float wheel_out = 0.0f;       // m outward beyond the wheel's centre line (0 = on the front-left wheel)
    float wheel_toe_deg = 0.0f;   // degrees the view turns outward
    float roof_up = 1.25f;        // m above the body origin (the origin is near the centre of mass: ~0.5 m above the ground)
    float roof_forward = 0.0f;    // m ahead of the origin
    float roof_pitch_deg = 8.0f;  // looking down
    float tv_ahead = 70.0f;       // m ahead of the car where a new trackside point is placed
    float tv_side = 14.0f;        // m to the side
    float tv_height = 3.5f;       // m above the car's origin
    float tv_pass = 6.0f;         // m beyond the point the car may go before the camera moves on
    float heli_back = 14.0f;      // m behind the car (replay helicopter)
    float heli_up = 9.0f;         // m above it
    float heli_smooth_s = 0.8f;   // seconds: the critically damped spring's time to settle
};

// Wheel camera: fixed to the body at the front-left wheel (axle offset and track from the car's dimensions), looking forward.
Pose wheel_pose(const Car &car, float front_axle, float track, const Params &p);
// Roof camera: above the roof looking forward and slightly down.
Pose roof_pose(const Car &car, const Params &p);

// TV camera: a trackside point the camera stays at, looking at the car; when the car has gone past it (or it is far behind), a new point is chosen ahead of the car.
class TvCamera {
public:
    Pose pose(const Car &car, const Params &p);
    void reset() { have_ = false; }
    bool has_point() const { return have_; }
    Vec3 point() const { return point_; }
    int moves() const { return moves_; }

private:
    bool have_ = false;
    Vec3 point_, axis_;  // the point and the car's heading when it was chosen (what "past" is measured along)
    int moves_ = 0;
};

// Critically damped spring on a point (replay helicopter camera): step(target, dt) moves toward the target with no overshoot and settles in about `smooth_s`.
class Smoother {
public:
    Vec3 step(Vec3 target, float dt, float smooth_s);
    void reset() { have_ = false; }
    bool has_value() const { return have_; }

private:
    bool have_ = false;
    Vec3 value_, velocity_;
};
// The helicopter's target point: above and behind the car.
Vec3 heli_target(const Car &car, const Params &p);

// A teleport (the car's position jumped further than `limit` m since the last sample): the smoothers start again.
bool jumped(Vec3 previous, Vec3 now, float limit);

}  // namespace cams
