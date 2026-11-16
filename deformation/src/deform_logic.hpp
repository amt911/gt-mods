// Pure core of the deformation mod: when a hit counts, how deep a vertex is dented, and the restore. No SDK, no guest memory.
//
// Every part of a car shares ONE frame: the s16 positions of all the parts are in the car's box, full s16 range = the box's half extents (the GE world
// matrix is the same for every part draw, host FACTS "Car draw matrix"). So a vertex is turned into metres with the half extents of that box, the
// dent is computed there from the hit point, and the displacement goes back to s16: the same point of two overlapping parts, or of a part and its
// VRAM copy, moves exactly the same.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace deform {

struct Vec3 {
    float x = 0, y = 0, z = 0;
};

struct Params {
    float threshold = 4.0f;     // m/s of velocity change in one step
    float depth_per_ms = 0.03f;  // metres of dent per m/s above the threshold
    float radius = 0.9f;         // metres
    float cap = 0.35f;           // metres, total per vertex
    float noise = 0.0f;           // 0..1: the depth of each vertex varies by +-noise (hash of its position), so highlights break up like crumpled metal
    float jitter = 0.0f;          // metres: each dented vertex is also moved by up to +-jitter (times the falloff) in a random direction (hash of its position)
    float vertical_scale = 2.0f;  // distances along y count this many times: the dent reaches half as far up and down (the roof edge is not the flank's neighbour)
    float roof_coupling = 0.0f;  // 0..1: vertices above `roof_start` (fraction of the box's half height) are dented by their distance on the ground plane alone (the vertical part dropped), times this: a flank or front hit at mid height still crumples the roof and the pillars above it. 0 = off
    float roof_start = 0.45f;
    float normal_push = 0.0f;    // 0..1: how much of the push goes along -normal of the vertex's surface (crumples the flank) instead of along the hit direction
};

// The car's box: half extents in metres along the model's x, y, z (full s16 range = one half extent).
struct Frame {
    float half[3] = {1, 1, 1};
};

// True when `dv_magnitude` (m/s) is a hit: strictly above the threshold, finite.
bool hit_detected(float dv_magnitude, float threshold);

// Dent depth of one hit, in metres: (dv - threshold) * depth_per_ms, 0 for no hit.
float hit_strength(float dv_magnitude, const Params &params);

// 1 at the contact point, 0 at `radius` and beyond, smooth (1 - t)^2 (1 + 2t) between: 0.5 at half the radius.
float falloff(float distance, float radius);

// Tells a hit from a discontinuity of the simulation. The game changes a car's velocity without any contact at a race start (the rolling start of a Time Trial sets
// the speed to exactly 50 km/h when the countdown ends), after a retry (the host compares with the previous race's last sample) and when a car is put back on the
// track (reset): a one-step velocity change that reads as a hit and dented the car where nothing touched it.
struct StepFilterParams {
    std::uint32_t gap_vblanks = 60;        // no world step for this many vblanks: a new race (or a pause): start again
    std::uint32_t warmup_steps = 30;       // steps after a start that never count
    float teleport_margin = 1.0f;          // metres: a position jump larger than the car could have travelled (plus this) is a teleport
    float start_speed = 13.8889f;          // m/s: the speed the game sets at a rolling start (50 km/h)
    float start_tolerance = 0.06f;
    std::uint32_t start_window_steps = 1500;  // the rolling start's speed set happens within this many steps of the first step
};

class StepFilter {
public:
    explicit StepFilter(StepFilterParams params = {}) : params_(params) {}
    // Once per world step of the player's car: the game's vblank count, the car's position (metres) and speed (m/s) after the step, the velocity change of the step
    // (|dv|, m/s), the yaw-rate jump and the off-track flag. True when the step may be a real impact.
    bool accept(std::uint64_t vblank, const float position[3], float speed, float dv_magnitude, float yaw_rate_jump, bool off_track);
    void reset();

private:
    StepFilterParams params_;
    bool have_ = false;
    std::uint64_t last_vblank_ = 0;
    float last_position_[3] = {0, 0, 0};
    float last_speed_ = 0;
    std::uint32_t warm_ = 0;
    std::uint32_t since_start_ = 0;
};

// A body mesh as the mod edits it: s16 positions (x, y, z per vertex) and the originals, kept so the dents can be undone.
struct Body {
    std::vector<std::int16_t> positions;
    std::vector<std::int16_t> original;
    std::vector<float> dented;  // total displacement per vertex, in metres
    std::vector<std::int8_t> normals;           // x, y, z per vertex in 1/127, or empty for a part without normals
    std::vector<std::int8_t> original_normals;
};

Body make_body(const std::vector<std::int16_t> &positions, const std::vector<std::int8_t> &normals = {});

// The point where the ray from the box centre along `direction` (model axes) leaves the box: the contact point of a hit that pushed the car along
// -direction. A zero direction gives the centre.
Vec3 box_contact(const Frame &frame, Vec3 direction);

// With `normal_push` b > 0 a vertex whose position faces the hit or runs along it (box outward direction . inward < 0.3) is pushed along normalize((1 - b) * inward + b * -outward): a flank
// or a fender caves in where it is, so the dent reads from any side. The direction depends on the position only (duplicated vertices of a hard edge move together). Pushes the vertices within `radius` of `contact` (metres, model axes) along `inward` (unit vector, into the body) by
// cap * falloff * (1 - exp(-depth / cap)) metres: about depth * falloff for a light hit, saturating smoothly towards `cap` (never beyond it, in total
// per vertex). The displacement is computed in metres from the vertex position, then converted back to s16 per axis. The normals of the dented
// vertices (when the body has them) are tilted like the surface: n' = normalize(n + (-u.n) * grad_t h), h = the displacement field, grad_t its part
// tangent to the surface (normals transform with the inverse of the box's scale). Returns how many vertices moved.
std::size_t apply_dent(Body &body, const Frame &frame, Vec3 contact, Vec3 inward, float depth, const Params &params);

// Puts every vertex back.
void restore(Body &body);

} // namespace deform
