#include "deform_logic.hpp"

#include <cassert>
#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace deform;

// A 3 x 3 x 3 grid of vertices, 1000 model units apart: the body is 2000 long.
static std::vector<std::int16_t> grid() {
    std::vector<std::int16_t> p;
    for (int x = -1; x <= 1; ++x)
        for (int y = -1; y <= 1; ++y)
            for (int z = -1; z <= 1; ++z) {
                p.push_back(static_cast<std::int16_t>(1000 * x));
                p.push_back(static_cast<std::int16_t>(1000 * y));
                p.push_back(static_cast<std::int16_t>(1000 * z));
            }
    return p;
}

int main() {
    Params p;
    assert(!hit_detected(0.0f, p.threshold) && !hit_detected(p.threshold, p.threshold) && hit_detected(p.threshold + 0.1f, p.threshold));  // hit_below_threshold_is_ignored
    assert(!hit_detected(NAN, p.threshold) && !hit_detected(-5.0f, p.threshold));
    assert(hit_strength(2.0f, p) == 0.0f);
    assert(std::fabs(hit_strength(p.threshold + 10.0f, p) - 10.0f * p.depth_per_ms) < 1e-6f);
    assert(falloff(0, 10) == 1.0f && falloff(10, 10) == 0.0f && std::fabs(falloff(5, 10) - 0.5f) < 1e-6f && falloff(3, 0) == 0.0f);  // dent_falls_off_with_distance
    Body b = make_body(grid());
    assert(b.size == 2000.0f && b.dented.size() == 27);
    const Vec3 contact{1000, 0, 0}, inward{-1, 0, 0};
    p.radius = 0.6f;  // 1200 units: reaches the 1000-unit neighbours
    const std::size_t moved = apply_dent(b, contact, inward, 0.1f, p);  // depth 200 at the contact
    assert(moved > 0 && moved < 27);
    // the vertex under the contact moved the most, a neighbour less, a far one not at all
    auto vertex = [&](int x, int y, int z) { return 9 * (x + 1) + 3 * (y + 1) + (z + 1); };
    const int under = vertex(1, 0, 0), near = vertex(1, 1, 0), far = vertex(-1, 0, 0);
    const int moved_under = b.original[3 * under] - b.positions[3 * under], moved_near = b.original[3 * near] - b.positions[3 * near];
    assert(moved_under > moved_near && moved_near > 0 && b.positions[3 * far] == b.original[3 * far]);
    // a dent with zero strength changes nothing (the sabotage the in-game check relies on)
    Body c = make_body(grid());
    assert(apply_dent(c, contact, inward, 0.0f, p) == 0 && c.positions == c.original);
    // dent_is_capped_when_accumulated: ten big hits never push a vertex past cap * size
    Body d = make_body(grid());
    for (int i = 0; i < 10; ++i) apply_dent(d, contact, inward, 0.3f, p);
    for (std::size_t v = 0; v < d.dented.size(); ++v) assert(d.dented[v] <= p.cap * d.size + 1e-3f);
    assert(d.original[3 * under] - d.positions[3 * under] <= static_cast<int>(p.cap * d.size) + 1);
    // restore puts every vertex back, byte for byte
    restore(d);
    assert(d.positions == d.original && d.dented[under] == 0.0f);
    // an edit at the s16 limit cannot wrap around
    Body e = make_body({32767, 0, 0, -32768, 0, 0});
    apply_dent(e, {32767, 0, 0}, {1, 0, 0}, 0.5f, p);
    assert(e.positions[0] == 32767);
    // smooth: even a hit far above the cap leaves no cliff between neighbours (a row of vertices 100 units apart), and nothing passes the cap
    {
        std::vector<std::int16_t> row;
        for (int i = 0; i < 21; ++i) { row.push_back(static_cast<std::int16_t>(-1000 + 100 * i)); row.push_back(0); row.push_back(0); }
        row.push_back(-1000); row.push_back(1000); row.push_back(0);  // gives the body a size of 2000 in y
        Body f = make_body(row);
        Params q;
        q.radius = 0.5f;
        q.cap = 0.12f;
        apply_dent(f, {1000, 0, 0}, {0, -1, 0}, 3.0f, q);
        float biggest_jump = 0, biggest = 0;
        for (int i = 0; i < 21; ++i) {
            const float moved = static_cast<float>(f.original[3 * i + 1] - f.positions[3 * i + 1]);
            biggest = std::max(biggest, moved);
            if (i > 0) biggest_jump = std::max(biggest_jump, std::fabs(moved - static_cast<float>(f.original[3 * (i - 1) + 1] - f.positions[3 * (i - 1) + 1])));
        }
        assert(biggest <= q.cap * f.size + 1.0f && biggest > 0.8f * q.cap * f.size);  // reaches nearly the cap, never beyond
        assert(biggest_jump < 0.3f * q.cap * f.size);                                   // dent_is_smooth: no spike or cliff between neighbours
    }
    std::puts("deformation tests passed");
    return 0;
}
