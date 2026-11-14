// Pure core of the deformation mod: when a hit counts, how deep a vertex is dented, and the restore. No SDK, no guest memory.
#pragma once

#include <cstdint>
#include <vector>

namespace deform {

struct Vec3 {
    float x = 0, y = 0, z = 0;
};

struct Params {
    float threshold = 4.0f;      // m/s of velocity change in one step
    float depth_per_ms = 0.01f;  // body-size fractions per m/s above the threshold
    float radius = 0.25f;        // body-size fractions
    float cap = 0.12f;           // body-size fractions, total per vertex
};

// True when `dv_magnitude` (m/s) is a hit: strictly above the threshold, finite.
bool hit_detected(float dv_magnitude, float threshold);

// Dent depth of one hit, in body-size fractions: (dv - threshold) * depth_per_ms, 0 for no hit.
float hit_strength(float dv_magnitude, const Params &params);

// 1 at the contact point, 0 at `radius` and beyond, smooth (1 - t)^2 (1 + 2t) between: 0.5 at half the radius.
float falloff(float distance, float radius);

// A body mesh as the mod edits it: s16 positions (x, y, z per vertex) and the originals, kept so the dents can be undone.
struct Body {
    std::vector<std::int16_t> positions;
    std::vector<std::int16_t> original;
    std::vector<float> dented;  // total displacement per vertex, in model units
    float size = 0;             // model units of the body's longest side (the unit of the fractions above)
};

// Builds the body from s16 positions (3 per vertex); size = the longest side of the bounding box.
Body make_body(const std::vector<std::int16_t> &positions);

// Pushes the vertices near `contact` (model units) along `inward` (unit vector, into the body) by cap * size * falloff * (1 - exp(-strength / cap)): about strength * size * falloff for a light hit, saturating smoothly towards
// cap * size (never beyond it, in total per vertex). Returns how many vertices moved.
std::size_t apply_dent(Body &body, Vec3 contact, Vec3 inward, float strength, const Params &params);

// Puts every vertex back.
void restore(Body &body);

} // namespace deform
