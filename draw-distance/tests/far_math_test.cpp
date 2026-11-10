#include "far_math.hpp"

#include <cassert>
#include <cmath>
#include <map>
#include <string>
#include <vector>

using namespace drawdist;

static Settings parse(const std::map<std::string, std::string> &m, std::vector<std::string> *warns = nullptr) {
    std::vector<std::string> sink;
    return parse_settings(
        [&](const char *k) -> const char * {
            auto it = m.find(k);
            return it == m.end() ? nullptr : it->second.c_str();
        },
        [&](const std::string &w) { (warns ? warns : &sink)->push_back(w); });
}

static void default_is_one_and_a_half() {
    std::vector<std::string> w;
    assert(parse({}, &w).far_scale == 1.5f && w.empty());
}
static void valid_value_is_kept() { assert(parse({{"far_scale", "4"}}).far_scale == 4.0f); }
static void out_of_range_is_clamped_with_a_warning() {
    std::vector<std::string> w;
    assert(parse({{"far_scale", "9"}}, &w).far_scale == 6.0f && w.size() == 1);
    w.clear();
    assert(parse({{"far_scale", "0.2"}}, &w).far_scale == 1.0f && w.size() == 1);
    w.clear();
    assert(parse({{"far_scale", "6"}}, &w).far_scale == 6.0f && w.empty());
}
static void garbage_keeps_the_default() {
    for (const char *bad : {"abc", "2x", "nan", "inf", "-inf"}) {
        std::vector<std::string> w;
        assert(parse({{"far_scale", bad}}, &w).far_scale == 1.5f && w.size() == 1);
    }
}
static void scale_parsing_for_env() {
    float v = 0;
    assert(parse_scale("3", &v) && v == 3.0f);
    assert(parse_scale("100", &v) && v == 6.0f);
    assert(!parse_scale("", &v) && !parse_scale(nullptr, &v) && !parse_scale("x", &v));
}
static void identity_scale_writes_nothing() {
    assert(!needs_edit(1200.0f, 1.3f, 1.0f));
    assert(needs_edit(1200.0f, 1.3f, 2.5f));
}
static void unusable_planes_are_left_alone() {
    assert(!needs_edit(NAN, 1.3f, 2.5f));
    assert(!needs_edit(INFINITY, 1.3f, 2.5f));
    assert(!needs_edit(1.0f, 1.3f, 2.5f));  // far <= near
    assert(!needs_edit(0.0f, 0.0f, 2.5f));
    assert(!needs_edit(-5.0f, 1.3f, 2.5f));
}
static void scale_is_clamped_and_reported() {
    float v = 0;
    bool clamped = false;
    assert(parse_scale("100", &v, &clamped) && v == 6.0f && clamped);
    assert(parse_scale("0.5", &v, &clamped) && v == 1.0f && clamped);
    assert(parse_scale("2.5", &v, &clamped) && v == 2.5f && !clamped);
    assert(parse_scale("6", &v, &clamped) && !clamped);
}

static void depth_terms(float n, float f, float *m10, float *m14) {
    *m10 = static_cast<float>(-(double(f) + n) / (double(f) - n));
    *m14 = static_cast<float>(-2.0 * f * n / (double(f) - n));
}
static void rescale_depth_matches_a_direct_build() {
    float a, b, want10, want14, na, nb;
    depth_terms(1.3f, 1200.0f, &a, &b);
    depth_terms(1.3f, 3000.0f, &want10, &want14);
    assert(rescale_depth(a, b, 2.5f, &na, &nb));
    assert(std::fabs(na - want10) < 1e-6f && std::fabs(nb - want14) < 1e-3f);
}
static void rescale_depth_identity_and_garbage_write_nothing() {
    float a, b, x = 7.0f, y = 8.0f;
    depth_terms(1.3f, 1200.0f, &a, &b);
    assert(!rescale_depth(a, b, 1.0f, &x, &y) && x == 7.0f && y == 8.0f);
    assert(!rescale_depth(NAN, b, 2.5f, &x, &y));
    assert(!rescale_depth(1.0f, b, 2.5f, &x, &y));   // m10 - 1 == 0
    assert(!rescale_depth(-1.0f, b, 2.5f, &x, &y));  // m10 + 1 == 0
    assert(!rescale_depth(0.0f, 0.0f, 2.5f, &x, &y)); // far <= near
}

static void world_depth_is_told_by_its_range() {
    float a, b;
    depth_terms(1.3f, 1200.0f, &a, &b);
    assert(is_world_depth(a, b));
    depth_terms(0.1f, 100.0f, &a, &b);  // car reflection pass
    assert(!is_world_depth(a, b));
    depth_terms(33.0f, 30000.0f, &a, &b);  // sky pass
    assert(!is_world_depth(a, b));
    assert(!is_world_depth(NAN, 1.0f) && !is_world_depth(1.0f, 1.0f) && !is_world_depth(-1.0f, 1.0f));
    assert(!is_world_depth(-1.0001f, 0.0f));  // near == 0
}
static void world_depth_bounds() {
    float a, b;
    // near < 5 and far 300..6000: near and far come back from m10/m14 with float rounding, so the cases sit 1 unit either side
    depth_terms(1.3f, 301.0f, &a, &b);
    assert(is_world_depth(a, b));
    depth_terms(1.3f, 299.0f, &a, &b);
    assert(!is_world_depth(a, b));
    depth_terms(1.3f, 5999.0f, &a, &b);
    assert(is_world_depth(a, b));
    depth_terms(1.3f, 6001.0f, &a, &b);
    assert(!is_world_depth(a, b));
    depth_terms(4.99f, 1200.0f, &a, &b);
    assert(is_world_depth(a, b));
    depth_terms(5.01f, 1200.0f, &a, &b);
    assert(!is_world_depth(a, b));
}

int main() {
    world_depth_is_told_by_its_range();
    world_depth_bounds();
    rescale_depth_matches_a_direct_build();
    rescale_depth_identity_and_garbage_write_nothing();
    default_is_one_and_a_half();
    valid_value_is_kept();
    out_of_range_is_clamped_with_a_warning();
    garbage_keeps_the_default();
    scale_parsing_for_env();
    identity_scale_writes_nothing();
    unusable_planes_are_left_alone();
    scale_is_clamped_and_reported();
    return 0;
}
