#include "deform_logic.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>

using namespace deform;

// A 3 x 3 x 3 grid of vertices in the car frame: -1, 0, +1 of the half extents (a car 2 m wide, 1.4 high, 4 long: half = 1, 0.7, 2).
static std::vector<std::int16_t> grid() {
    std::vector<std::int16_t> p;
    for (int x = -1; x <= 1; ++x)
        for (int y = -1; y <= 1; ++y)
            for (int z = -1; z <= 1; ++z) {
                p.push_back(static_cast<std::int16_t>(32767 * x));
                p.push_back(static_cast<std::int16_t>(32767 * y));
                p.push_back(static_cast<std::int16_t>(32767 * z));
            }
    return p;
}
static float metres(const Body &b, size_t v, int c, const Frame &f, bool original) {
    return (original ? b.original : b.positions)[3 * v + c] / 32767.0f * f.half[c];
}

int main() {
    Params p;
    Frame frame;
    frame.half[0] = 1.0f; frame.half[1] = 0.7f; frame.half[2] = 2.0f;
    assert(!hit_detected(0.0f, p.threshold) && !hit_detected(p.threshold, p.threshold) && hit_detected(p.threshold + 0.1f, p.threshold));  // hit_below_threshold_is_ignored
    assert(!hit_detected(NAN, p.threshold) && !hit_detected(-5.0f, p.threshold));
    assert(hit_strength(2.0f, p) == 0.0f);
    assert(std::fabs(hit_strength(p.threshold + 10.0f, p) - 10.0f * p.depth_per_ms) < 1e-6f);
    assert(falloff(0, 10) == 1.0f && falloff(10, 10) == 0.0f && std::fabs(falloff(5, 10) - 0.5f) < 1e-6f && falloff(3, 0) == 0.0f);  // dent_falls_off_with_distance
    // box_contact: where the ray along a direction leaves the car's box
    {
        const Vec3 right = box_contact(frame, {1, 0, 0}), diag = box_contact(frame, {1, 0, 1}), none = box_contact(frame, {0, 0, 0});
        assert(std::fabs(right.x - 1.0f) < 1e-5f && right.y == 0.0f && right.z == 0.0f);   // out of the right side
        assert(std::fabs(diag.x - 1.0f) < 1e-5f && std::fabs(diag.z - 1.0f) < 1e-5f);     // x binds first (1 < 2): the point (1, 0, 1)
        assert(none.x == 0.0f && none.y == 0.0f && none.z == 0.0f);
    }
    // frame_mapping: the dent is computed in metres from the vertex position
    Body b = make_body(grid());
    assert(b.dented.size() == 27);
    const Vec3 contact{1.0f, 0.0f, 0.0f}, inward{-1, 0, 0};  // the right side, pushed in along -x
    p.radius = 2.0f;
    const size_t moved = apply_dent(b, frame, contact, inward, 0.1f, p);
    assert(moved > 0 && moved < 27);
    auto vertex = [&](int x, int y, int z) { return 9 * (x + 1) + 3 * (y + 1) + (z + 1); };
    const int under = vertex(1, 0, 0), near = vertex(1, 1, 0), far = vertex(-1, 0, 0);
    const float moved_under = metres(b, under, 0, frame, true) - metres(b, under, 0, frame, false);
    const float moved_near = metres(b, near, 0, frame, true) - metres(b, near, 0, frame, false);
    assert(moved_under > moved_near && moved_near > 0 && b.positions[3 * far] == b.original[3 * far]);
    assert(std::fabs(metres(b, under, 1, frame, true) - metres(b, under, 1, frame, false)) < 1e-4f);  // pushed along x only: y and z untouched
    // the displacement is in metres, not in model units: along x the half extent is 1 m, along z 2 m, the same 0.1 m push is twice the s16 on x
    Body bx = make_body({0, 0, 0}), bz = make_body({0, 0, 0});
    apply_dent(bx, frame, {0, 0, 0}, {1, 0, 0}, 0.1f, p);
    apply_dent(bz, frame, {0, 0, 0}, {0, 0, 1}, 0.1f, p);
    assert(bx.positions[0] != 0 && bz.positions[2] != 0 && std::abs(bx.positions[0]) > std::abs(bz.positions[2]) * 19 / 10);  // x moves ~2x the s16 of z for equal metres
    assert(std::fabs(bx.dented[0] - bz.dented[0]) < 1e-6f);                                                                      // and both are the same displacement in metres
    // one point, two parts: the same position in the car frame moves the same s16 in two bodies (overlapping layers, a part and its VRAM copy)
    Body layer_a = make_body({20000, 5000, -3000, 100, 200, 300}), layer_b = make_body({100, 200, 300, 20000, 5000, -3000});
    apply_dent(layer_a, frame, {0.6f, 0.1f, -0.2f}, {-1, 0, 0}, 0.2f, p);
    apply_dent(layer_b, frame, {0.6f, 0.1f, -0.2f}, {-1, 0, 0}, 0.2f, p);
    for (int c = 0; c < 3; ++c) assert(layer_a.positions[c] == layer_b.positions[3 + c] && layer_a.positions[3 + c] == layer_b.positions[c]);
    // a dent with zero depth changes nothing (the sabotage the in-game check relies on)
    Body c = make_body(grid());
    assert(apply_dent(c, frame, contact, inward, 0.0f, p) == 0 && c.positions == c.original);
    // dent_is_capped_when_accumulated: ten big hits never push a vertex past the cap
    Body d = make_body(grid());
    for (int i = 0; i < 10; ++i) apply_dent(d, frame, contact, inward, 1.0f, p);
    for (size_t v = 0; v < d.dented.size(); ++v) assert(d.dented[v] <= p.cap + 1e-4f);
    assert(metres(d, under, 0, frame, true) - metres(d, under, 0, frame, false) <= p.cap + 1e-3f);
    // restore puts every vertex back, byte for byte
    restore(d);
    assert(d.positions == d.original && d.dented[under] == 0.0f);
    // an edit at the s16 limit cannot wrap around
    Body e = make_body({32767, 0, 0, -32768, 0, 0});
    apply_dent(e, frame, {1, 0, 0}, {1, 0, 0}, 0.5f, p);
    assert(e.positions[0] == 32767);
    // smooth: even a hit far above the cap leaves no cliff between neighbours (a row of vertices 0.1 m apart), and nothing passes the cap
    {
        std::vector<std::int16_t> row;
        for (int i = 0; i < 21; ++i) { row.push_back(static_cast<std::int16_t>(-32767 + i * 3276)); row.push_back(0); row.push_back(0); }
        Body f = make_body(row);
        Params q;
        q.radius = 0.5f;
        q.cap = 0.12f;
        apply_dent(f, frame, {1, 0, 0}, {0, 1, 0}, 3.0f, q);
        float biggest_jump = 0, biggest = 0, prev = 0;
        for (int i = 0; i < 21; ++i) {
            const float moved = (f.positions[3 * i + 1] - f.original[3 * i + 1]) / 32767.0f * frame.half[1];
            biggest = std::max(biggest, moved);
            if (i > 0) biggest_jump = std::max(biggest_jump, std::fabs(moved - prev));
            prev = moved;
        }
        assert(biggest <= q.cap + 1e-3f && biggest > 0.8f * q.cap);  // reaches nearly the cap, never beyond
        assert(biggest_jump < 0.4f * q.cap);                         // dent_is_smooth: no spike or cliff (the steepest slope of the falloff is 1.5 / radius: 0.3 cap per 0.1 m here)
    }
    // normals tilt with the surface: a flat face (normal +z) pushed in along -z: a vertex at +x leans towards the centre (-x), one at -x towards +x,
    // the vertex at the centre and one outside the radius keep their normal, and the normal stays a unit vector
    {
        Frame unit_box;
        std::vector<std::int16_t> pos = {16384, 0, 0, -16384, 0, 0, 0, 0, 0, 32767, 0, 0};  // x = +0.5, -0.5, 0, +1.0 (half extent 1)
        std::vector<std::int8_t> nrm = {0, 0, 127, 0, 0, 127, 0, 0, 127, 0, 0, 127};
        Body f = make_body(pos, nrm);
        Params q;
        q.radius = 0.8f;
        q.cap = 0.3f;
        apply_dent(f, unit_box, {0, 0, 0}, {0, 0, -1}, 0.3f, q);
        assert(f.normals[0] < -5 && f.normals[3] > 5 && f.normals[6] == 0 && f.normals[9] == 0 && f.normals[2] > 100);   // dent_tilts_normals
        assert(f.normals[2] > 0 && std::abs(f.normals[0]) == std::abs(f.normals[3]));                                    // symmetric
        restore(f);
        assert(f.normals == f.original_normals);
        Body none = make_body(pos);
        apply_dent(none, unit_box, {0, 0, 0}, {0, 0, -1}, 0.3f, q);
        assert(none.normals.empty());                                                                                     // a part without normals gets none
    }
    // normal_push: a vertex on the right flank (position +x) hit along -z with normal_push 0.5 caves in along -x as well; with 0 it only slides along -z; the same
    // position with a different vertex normal moves identically (no tear at a hard edge)
    {
        Frame unit_box;
        std::vector<std::int16_t> pos = {32767, 0, 0, 32767, 0, 0, -32767, 0, 0};
        std::vector<std::int8_t> nrm = {127, 0, 0, 0, 127, 0, -127, 0, 0};  // two vertices at the same position with different normals, one on the far flank
        Params q;
        q.radius = 3.0f;
        q.cap = 0.5f;
        Body slide = make_body(pos, nrm), cave = make_body(pos, nrm);
        apply_dent(slide, unit_box, {1, 0, 0}, {0, 0, -1}, 0.5f, q);
        q.normal_push = 0.5f;
        apply_dent(cave, unit_box, {1, 0, 0}, {0, 0, -1}, 0.5f, q);
        assert(slide.positions[0] == 32767 && slide.positions[2] < 0);                                         // slides along -z only
        assert(cave.positions[0] < 32767 && cave.positions[2] < 0 && cave.positions[2] > slide.positions[2]);  // caves in along -x and slides a bit less
        assert(cave.positions[0] == cave.positions[3] && cave.positions[1] == cave.positions[4] && cave.positions[2] == cave.positions[5]);  // same position: same move
    }
    // vertical reach: a vertex 0.6 m above the contact is dented less than one 0.6 m along z (vertical_scale 2)
    {
        Frame unit_box;
        Params q;
        q.radius = 1.4f;
        q.cap = 0.5f;
        Body up = make_body({0, 19660, 0}), along = make_body({0, 0, 19660});  // 0.6 m on y / on z of a box with half extent 1
        apply_dent(up, unit_box, {0, 0, 0}, {-1, 0, 0}, 0.5f, q);
        apply_dent(along, unit_box, {0, 0, 0}, {-1, 0, 0}, 0.5f, q);
        assert(up.dented[0] < along.dented[0]);
    }
    // crumple: with noise and jitter the same position moves identically in two bodies (duplicates stay together), neighbours differ, the cap still holds, and a
    // zero-noise run is the smooth one
    {
        Params q;
        q.radius = 2.0f;
        q.cap = 0.3f;
        q.noise = 0.5f;
        q.jitter = 0.05f;
        std::vector<std::int16_t> pts;
        for (int i = 0; i < 40; ++i) { pts.push_back(static_cast<std::int16_t>(2000 * i)); pts.push_back(1000); pts.push_back(-3000); }
        Body a1 = make_body(pts), a2 = make_body(pts);
        apply_dent(a1, frame, {0.3f, 0, 0}, {0, 0, 1}, 1.0f, q);
        apply_dent(a2, frame, {0.3f, 0, 0}, {0, 0, 1}, 1.0f, q);
        assert(a1.positions == a2.positions);                                   // deterministic
        for (size_t v = 0; v < a1.dented.size(); ++v) assert(a1.dented[v] <= q.cap + 1e-4f);
        q.noise = 0.0f;
        q.jitter = 0.0f;
        Body smooth = make_body(pts);
        apply_dent(smooth, frame, {0.3f, 0, 0}, {0, 0, 1}, 1.0f, q);
        int differing = 0;
        for (size_t v = 0; v < a1.dented.size(); ++v) differing += a1.positions[3 * v + 2] != smooth.positions[3 * v + 2];
        assert(differing > 5);                                                   // the crumple changes the surface
    }
    // roof: a side hit at mid height (contact on the right flank, y = 0) used to leave the roof alone (the vertical reach 2x radius stops 1.4 m up at the
    // roof edge and the roof centre is further still); with roof_coupling the roof vertices within the radius (measured on the ground plane) sink and lean in,
    // the cap holds, and 0 is the old behaviour
    {
        Frame car;
        car.half[0] = 0.89f; car.half[1] = 0.72f; car.half[2] = 1.91f;
        Params q;
        q.radius = 2.0f; q.cap = 0.7f; q.normal_push = 0.6f; q.vertical_scale = 2.0f;
        const std::vector<std::int16_t> roof = {18409, 32767, 0,    // 0.5 m right of the centre line, on the roof
                                                0, 32767, 0,        // the roof centre
                                                0, 32767, 30000,    // the roof far forward (outside the radius of a hit at z = 0)
                                                18409, 0, 0};      // a flank vertex at the same place, mid height: dented as before
        Body off = make_body(roof), on = make_body(roof);
        apply_dent(off, car, {0.89f, 0.0f, 0.0f}, {-1, 0, 0}, 1.0f, q);
        q.roof_coupling = 0.6f;
        apply_dent(on, car, {0.89f, 0.0f, 0.0f}, {-1, 0, 0}, 1.0f, q);
        assert(off.dented[0] < 0.1f && off.dented[1] < 0.05f);                 // roof_is_untouched_without_coupling: the bug
        assert(on.dented[0] > 0.25f && on.dented[1] > 0.15f);                     // roof_dents_on_a_side_hit
        assert(on.positions[1] < on.original[1] && on.positions[4] < on.original[4]);  // it sinks
        assert(on.dented[2] < 0.01f);                                            // almost at the radius: hardly moved
        assert(on.positions[9] == off.positions[9] && on.positions[10] == off.positions[10]);  // the flank is not changed by the coupling
        for (int i = 0; i < 20; ++i) apply_dent(on, car, {0.89f, 0.0f, 0.0f}, {-1, 0, 0}, 1.0f, q);
        for (size_t v = 0; v < on.dented.size(); ++v) assert(on.dented[v] <= q.cap + 1e-4f);  // roof_is_capped
    }
    // StepFilter: a velocity change that is not a hit never counts
    {
        const float here[3] = {0, 0, 0};
        StepFilter f;
        // the first steps of a race never count, even with a huge change (the host compares the first step with the previous race's last sample)
        assert(!f.accept(1000, here, 5.0f, 40.0f, 0.0f, false));
        for (int i = 1; i < 30; ++i) assert(!f.accept(1000 + 2 * i, here, 5.0f, 40.0f, 0.0f, false));
        assert(f.accept(1062, here, 5.0f, 12.0f, 0.0f, false));                    // after the warm-up a real hit counts
        // a gap (a retry, a pause) starts again
        assert(!f.accept(2000, here, 5.0f, 30.0f, 0.0f, false));
        for (int i = 1; i < 30; ++i) f.accept(2000 + 2 * i, here, 5.0f, 0.0f, 0.0f, false);
        assert(f.accept(2062, here, 5.0f, 8.0f, 0.0f, false));
        // a teleport (the car put back on the track, 80 m away at 10 m/s) starts again, and its velocity reset does not count
        const float far_away[3] = {80, 0, 0};
        assert(!f.accept(2064, far_away, 0.0f, 10.0f, 0.0f, false));
        for (int i = 1; i < 30; ++i) f.accept(2064 + 2 * i, far_away, 0.0f, 0.0f, 0.0f, false);
        assert(f.accept(2126, far_away, 0.0f, 9.0f, 0.0f, false));                  // and a hit afterwards counts again
        // moving at speed is not a teleport: 14 m in 1 s at 14 m/s
        float p[3] = {0, 0, 0};
        StepFilter g;
        g.accept(0, p, 14.0f, 0.0f, 0.0f, false);
        for (int i = 1; i <= 60; ++i) { p[0] += 14.0f / 30.0f; assert(!g.accept(2 * i, p, 14.0f, 0.0f, 0.0f, false) || i >= 30); }
        p[0] += 0.5f;
        assert(g.accept(122, p, 3.0f, 11.0f, 0.2f, true));                          // hit_at_speed_counts: a wall at 14 m/s
        // the rolling start: speed set to 50 km/h within the first steps of the race, from slower or from faster, is not a hit; the same change to another speed is
        StepFilter r;
        float q[3] = {0, 0, 0};
        r.accept(100, q, 13.9f, 0.0f, 0.0f, false);
        for (int i = 1; i <= 272; ++i) { q[0] += 0.1f; r.accept(100 + 2 * i, q, 7.9f, 0.0f, 0.0f, false); }
        assert(!r.accept(100 + 2 * 273, q, 13.8889f, 5.95f, 0.0f, false));          // rolling_start_is_not_a_hit (slower before: the push is forward)
        StepFilter r2;
        r2.accept(100, q, 25.0f, 0.0f, 0.0f, false);
        for (int i = 1; i <= 100; ++i) r2.accept(100 + 2 * i, q, 25.0f, 0.0f, 0.0f, false);
        assert(!r2.accept(100 + 2 * 101, q, 13.8889f, 11.1f, 0.0f, false));         // faster before: the push is backward, which dented the FRONT
        StepFilter r3;
        r3.accept(100, q, 25.0f, 0.0f, 0.0f, false);
        for (int i = 1; i <= 100; ++i) r3.accept(100 + 2 * i, q, 25.0f, 0.0f, 0.0f, false);
        assert(r3.accept(100 + 2 * 101, q, 16.0f, 9.0f, 0.0f, false));              // another speed after the change: a hit
        // a wall hit that happens to leave the car at the start speed is still a hit when the car is off the track or turned
        StepFilter r4;
        r4.accept(100, q, 25.0f, 0.0f, 0.0f, false);
        for (int i = 1; i <= 100; ++i) r4.accept(100 + 2 * i, q, 25.0f, 0.0f, 0.0f, false);
        assert(r4.accept(100 + 2 * 101, q, 13.8889f, 11.1f, 0.0f, true));
        // the start window ends: later, the same speed after a change counts
        StepFilter r5;
        r5.accept(100, q, 25.0f, 0.0f, 0.0f, false);
        for (int i = 1; i <= 2000; ++i) r5.accept(100 + 2 * i, q, 25.0f, 0.0f, 0.0f, false);
        assert(r5.accept(100 + 2 * 2001, q, 13.8889f, 11.1f, 0.0f, false));
    }
    std::puts("deformation tests passed");
    return 0;
}
