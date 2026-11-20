// Rigid-body core of the rollover mod's "realistic" style (docs/superpowers/specs/2026-11-18-rollover-physics-design.md of the host repo).
// No game dependency: a box hull, gravity, impulse contacts (restitution + Coulomb friction) against a ground surface, driven by plain values so
// the unit test runs it. World frame: the game's physics frame, Z up. The hull's local frame is right-handed: x forward, y LEFT, z up
// (the game's basis rows are forward, right, up and are left-handed: see frame_from_rows / rows_from_frame).
#pragma once

#include <cmath>

namespace rollphys {

struct V3 {
    double x = 0, y = 0, z = 0;
};
inline V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 operator*(V3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline V3 operator-(V3 a) { return {-a.x, -a.y, -a.z}; }
inline double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline double len(V3 a) { return std::sqrt(dot(a, a)); }

struct Quat {
    double w = 1, x = 0, y = 0, z = 0;
};
Quat normalized(Quat q);
Quat operator*(Quat a, Quat b);
V3 rotate(Quat q, V3 v);          // q v q*
V3 rotate_inverse(Quat q, V3 v);  // q* v q
Quat integrate(Quat q, V3 w_world, double dt);  // q += 0.5 dt (w q), normalised

// Rotation <-> the game's basis rows (forward, right, up: world vectors; the basis is left-handed, FACTS "Rollovers": f x r = -u). The hull's right-handed
// frame is (forward, -right, up). `rows` = 9 floats f, r, u as in GtCarState::basis.
Quat quat_from_rows(const float rows[9]);
void rows_from_quat(Quat q, float rows[9]);

// The car's hull, in the car's own frame relative to the body origin: half length (forward) and half width, the floor and roof heights, and the
// forward offset of the box's centre. The centre of mass is the box's centre.
struct Hull {
    double hl = 2.0, hw = 0.9, zb = -0.03, zt = 0.8, cx = 0.0;
};

struct Params {
    double mass = 1300.0;
    double restitution = 0.15;       // applied above `bounce_speed` of approach; a slower contact is inelastic (no jitter)
    double bounce_speed = 1.0;
    double friction_scale = 1.0;     // multiplies the ground's own coefficient (0 = no friction: a check sabotage)
    double gravity = 9.81;
    int iterations = 8;              // sequential-impulse iterations per substep
    int substeps = 10;               // a step is split in this many: a point cannot sink far into the ground before the response (bounce height stays e^2 h)
    double slop = 0.002;             // m of penetration left alone
    double baumgarte = 0.2;          // fraction of the remaining depth the position correction removes per step
    double max_correct = 0.25;       // m: largest position correction per step
    double lin_damping = 0.03;       // 1/s, air drag
    double ang_damping = 0.12;       // 1/s, rolling resistance and drag
    double sleep_speed = 0.03;       // m/s and rad/s under which a body with >= 3 contacts is put to rest
};

struct GroundSample {
    double height = 0;
    V3 normal{0, 0, 1};
    double friction = 0.7;  // Coulomb coefficient of the surface here
};
class Ground {
  public:
    virtual ~Ground() = default;
    virtual GroundSample at(double x, double y) const = 0;
};
// A plane: height h0 at (x0, y0) with the gradient (gx, gy).
class PlaneGround : public Ground {
  public:
    double x0 = 0, y0 = 0, h0 = 0, gx = 0, gy = 0, mu = 0.7;
    GroundSample at(double x, double y) const override;
};

struct Body {
    V3 com;       // centre of mass, world
    Quat q;
    V3 v, w;      // linear and angular velocity, world
    int rest_steps = 0;
};

struct StepInfo {
    int contacts = 0;          // hull points in contact after the position step
    int bottom_contacts = 0;   // those on the floor face (local z == floor)
    double max_depth = 0;      // deepest penetration before the correction (m)
    double lowest = 0;         // lowest hull point above the ground after the step (negative = inside)
    bool asleep = false;
};

// Hull points relative to the centre of mass, in the hull frame: 8 corners, then 4 midpoints of the long edges. Returns 12.
constexpr int kHullPoints = 12;
void hull_points(const Hull &h, V3 out[kHullPoints], bool bottom[kHullPoints]);
V3 com_offset(const Hull &h);  // origin -> centre of mass, hull frame

// Body from / to the game's origin pose.
Body body_from_origin(const Hull &h, V3 origin, Quat q, V3 v_at_origin, V3 w);
V3 origin_of(const Hull &h, const Body &b);
V3 velocity_at_origin(const Hull &h, const Body &b);

// Inertia of the box about its centre, hull frame (diagonal).
V3 inertia_diag(const Hull &h, double mass);

// An impulse J (N s, world) at the world point p: v += J/m, w += I^-1 (r x J).
void apply_impulse(const Hull &h, const Params &p, Body &b, V3 point, V3 J);

// One fixed step of dt seconds.
StepInfo step(const Hull &h, const Params &p, const Ground &g, Body &b, double dt);

}  // namespace rollphys
