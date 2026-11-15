#include "deform_logic.hpp"

#include <algorithm>
#include <cmath>

namespace deform {

namespace {
// Deterministic noise in [0, 1) from a position (the ORIGINAL s16 position: duplicated vertices of a hard edge, a part and its VRAM copy, get the same value).
float hash01(std::int16_t x, std::int16_t y, std::int16_t z, std::uint32_t salt) {
    std::uint32_t h = static_cast<std::uint16_t>(x) * 0x9E3779B1u ^ (static_cast<std::uint16_t>(y) + salt) * 0x85EBCA77u ^ static_cast<std::uint16_t>(z) * 0xC2B2AE3Du;
    h ^= h >> 15;
    h *= 0x2C1B3C6Du;
    h ^= h >> 12;
    h *= 0x297A2D39u;
    h ^= h >> 15;
    return (h & 0xFFFFFFu) / 16777216.0f;
}
} // namespace

bool hit_detected(float dv, float threshold) { return std::isfinite(dv) && dv > threshold; }

float hit_strength(float dv, const Params &p) { return hit_detected(dv, p.threshold) ? (dv - p.threshold) * p.depth_per_ms : 0.0f; }

float falloff(float distance, float radius) {
    if (!(radius > 0.0f) || distance >= radius) return 0.0f;
    const float t = std::max(distance, 0.0f) / radius;  // smooth (C1) from 1 at the contact to 0 at the radius: no crease at either end
    return (1.0f - t) * (1.0f - t) * (1.0f + 2.0f * t);
}

Body make_body(const std::vector<std::int16_t> &positions, const std::vector<std::int8_t> &normals) {
    Body b;
    b.positions = positions;
    b.original = positions;
    b.normals = normals;
    b.original_normals = normals;
    b.dented.assign(positions.size() / 3, 0.0f);
    return b;
}

Vec3 box_contact(const Frame &frame, Vec3 d) {
    const float dir[3] = {d.x, d.y, d.z};
    float t = 1e30f;
    for (int i = 0; i < 3; ++i)
        if (std::fabs(dir[i]) > 1e-6f) t = std::min(t, frame.half[i] / std::fabs(dir[i]));
    if (t > 1e29f) return {};
    return {d.x * t, d.y * t, d.z * t};
}

std::size_t apply_dent(Body &b, const Frame &frame, Vec3 contact, Vec3 inward, float depth, const Params &p) {
    if (!(depth > 0.0f) || !(p.cap > 0.0f)) return 0;
    const float unit = 1.0f / 32767.0f, in[3] = {inward.x, inward.y, inward.z};
    const float centre[3] = {contact.x, contact.y, contact.z};
    const float amount = p.cap * (1.0f - std::exp(-depth / p.cap));
    std::size_t moved = 0;
    for (std::size_t v = 0; v < b.dented.size(); ++v) {
        std::int16_t *at = &b.positions[3 * v];
        float d2 = 0;
        for (int c = 0; c < 3; ++c) {
            float d = at[c] * unit * frame.half[c] - centre[c];
            if (c == 1) d *= p.vertical_scale;  // the dent reaches less far up and down than along the car
            d2 += d * d;
        }
        const float weight = falloff(std::sqrt(d2), p.radius);
        if (weight <= 0.0f) continue;
        // crumpled metal, not a smooth bowl: the depth varies per vertex by +-noise and the vertex is jittered a little (both from the position)
        const std::int16_t *orig = &b.original[3 * v];
        const float crumple = 1.0f + p.noise * (2.0f * hash01(orig[0], orig[1], orig[2], 1u) - 1.0f);
        const float step = std::min(amount * weight * crumple, p.cap - b.dented[v]);
        if (!(step > 0.0f)) continue;
        // The push direction: the hit's, blended with -n where n is the outward direction of the car's box at this POSITION (p / half^2, normalised): a flank
        // caves in sideways, the roof sinks, a nose goes back. It depends on the position only, never on the vertex's own normal, so duplicated vertices of a
        // hard edge (same position, different normals) move together and the surface does not tear.
        float dir[3] = {in[0], in[1], in[2]};
        if (p.normal_push > 0.0f) {
            float n_m[3], len0 = 0;
            for (int c = 0; c < 3; ++c) {
                n_m[c] = at[c] * unit / frame.half[c];  // p / half^2 up to a constant: at is p / half * 32767
                len0 += n_m[c] * n_m[c];
            }
            len0 = std::sqrt(len0);
            if (len0 > 1e-4f) {
                for (float &x : n_m) x /= len0;
                const float un0 = in[0] * n_m[0] + in[1] * n_m[1] + in[2] * n_m[2];
                if (un0 < 0.3f) {  // the surface faces the push or runs along it; the far side keeps the hit's direction
                    float m[3], mlen = 0;
                    for (int c = 0; c < 3; ++c) {
                        m[c] = (1.0f - p.normal_push) * in[c] - p.normal_push * n_m[c];
                        mlen += m[c] * m[c];
                    }
                    mlen = std::sqrt(mlen);
                    if (mlen > 1e-6f)
                        for (int c = 0; c < 3; ++c) dir[c] = m[c] / mlen;
                }
            }
        }
        bool changed = false;
        for (int c = 0; c < 3; ++c) {
            const float jitter_m = p.jitter * weight * (2.0f * hash01(orig[0], orig[1], orig[2], 2u + static_cast<std::uint32_t>(c)) - 1.0f);
            const float delta_s16 = (dir[c] * step + jitter_m) / frame.half[c] * 32767.0f;  // metres back to s16 on this axis
            const int target = std::clamp(static_cast<int>(std::lround(at[c] + delta_s16)), -32768, 32767);
            changed |= target != at[c];
            at[c] = static_cast<std::int16_t>(target);
        }
        if (!b.normals.empty() && d2 > 1e-12f) {  // the surface tilts with the dent
            const float t = std::sqrt(d2) / p.radius, dw_dr = -6.0f * t * (1.0f - t) / p.radius, r = std::sqrt(d2);
            std::int8_t *nrm = &b.normals[3 * v];
            float n[3], g[3], len = 0;
            for (int c = 0; c < 3; ++c) {
                n[c] = nrm[c] / 127.0f / frame.half[c];  // inverse transpose of the box's scale
                len += n[c] * n[c];
            }
            len = std::sqrt(len);
            if (len > 1e-6f) {
                for (float &x : n) x /= len;
                float un = 0, gn = 0;
                for (int c = 0; c < 3; ++c) {
                    const float d = at[c] * unit * frame.half[c] - centre[c];
                    g[c] = amount * dw_dr * d / r;
                    un += dir[c] * n[c];
                    gn += g[c] * n[c];
                }
                if (un < 0.0f) {  // the surface faces the push
                    float m[3], mlen = 0;
                    for (int c = 0; c < 3; ++c) {
                        m[c] = n[c] + (-un) * (g[c] - gn * n[c]);
                        m[c] *= frame.half[c];  // back to model axes
                        mlen += m[c] * m[c];
                    }
                    mlen = std::sqrt(mlen);
                    if (mlen > 1e-6f)
                        for (int c = 0; c < 3; ++c) nrm[c] = static_cast<std::int8_t>(std::clamp(static_cast<int>(std::lround(m[c] / mlen * 127.0f)), -127, 127));
                }
            }
        }
        b.dented[v] += step;
        if (changed) ++moved;
    }
    return moved;
}

void restore(Body &b) {
    b.positions = b.original;
    b.normals = b.original_normals;
    std::fill(b.dented.begin(), b.dented.end(), 0.0f);
}

} // namespace deform
