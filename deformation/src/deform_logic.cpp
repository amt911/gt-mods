#include "deform_logic.hpp"

#include <algorithm>
#include <cmath>

namespace deform {

bool hit_detected(float dv, float threshold) { return std::isfinite(dv) && dv > threshold; }

float hit_strength(float dv, const Params &p) { return hit_detected(dv, p.threshold) ? (dv - p.threshold) * p.depth_per_ms : 0.0f; }

float falloff(float distance, float radius) {
    if (!(radius > 0.0f) || distance >= radius) return 0.0f;
    const float t = std::max(distance, 0.0f) / radius;  // smooth (C1) from 1 at the contact to 0 at the radius: no crease at either end
    return (1.0f - t) * (1.0f - t) * (1.0f + 2.0f * t);
}

Body make_body(const std::vector<std::int16_t> &positions) {
    Body b;
    b.positions = positions;
    b.original = positions;
    b.dented.assign(positions.size() / 3, 0.0f);
    int lo[3] = {32767, 32767, 32767}, hi[3] = {-32768, -32768, -32768};
    for (std::size_t i = 0; i + 2 < positions.size(); i += 3)
        for (int c = 0; c < 3; ++c) {
            lo[c] = std::min<int>(lo[c], positions[i + c]);
            hi[c] = std::max<int>(hi[c], positions[i + c]);
        }
    for (int c = 0; c < 3; ++c) b.size = std::max(b.size, positions.empty() ? 0.0f : static_cast<float>(hi[c] - lo[c]));
    return b;
}

std::size_t apply_dent(Body &b, Vec3 contact, Vec3 inward, float strength, const Params &p) {
    if (!(strength > 0.0f) || b.size <= 0.0f) return 0;
    const float depth = strength * b.size, radius = p.radius * b.size, cap = p.cap * b.size;
    std::size_t moved = 0;
    for (std::size_t v = 0; v < b.dented.size(); ++v) {
        std::int16_t *at = &b.positions[3 * v];
        const float dx = at[0] - contact.x, dy = at[1] - contact.y, dz = at[2] - contact.z;
        const float weight = falloff(std::sqrt(dx * dx + dy * dy + dz * dz), radius);
        if (weight <= 0.0f) continue;
        // the depth saturates smoothly towards the cap and the falloff shapes the dent: a hit far above the cap makes a deeper, rounded dent of the
        // same smooth profile, never a flat plateau with a cliff at its rim
        const float target = cap * weight * (1.0f - std::exp(-depth / cap));
        const float step = std::min(target, cap - b.dented[v]);
        if (!(step > 0.0f)) continue;
        const float moves[3] = {inward.x * step, inward.y * step, inward.z * step};
        bool changed = false;
        for (int c = 0; c < 3; ++c) {
            const int target = std::clamp(static_cast<int>(std::lround(at[c] + moves[c])), -32768, 32767);
            changed |= target != at[c];
            at[c] = static_cast<std::int16_t>(target);
        }
        b.dented[v] += step;
        if (changed) ++moved;
    }
    return moved;
}

void restore(Body &b) {
    b.positions = b.original;
    std::fill(b.dented.begin(), b.dented.end(), 0.0f);
}

} // namespace deform
