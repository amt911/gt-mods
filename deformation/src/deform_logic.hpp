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
