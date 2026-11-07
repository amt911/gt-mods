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

static void default_is_two_and_a_half() {
    std::vector<std::string> w;
    assert(parse({}, &w).far_scale == 2.5f && w.empty());
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
        assert(parse({{"far_scale", bad}}, &w).far_scale == 2.5f && w.size() == 1);
    }
}
static void scale_parsing_for_env() {
    float v = 0;
    assert(parse_scale("3", &v) && v == 3.0f);
    assert(parse_scale("100", &v) && v == 6.0f);
    assert(!parse_scale("", &v) && !parse_scale(nullptr, &v) && !parse_scale("x", &v));
}
static void scaled_far_multiplies() { assert(scaled_far(1200.0f, 1.3f, 2.5f) == 3000.0f); }
static void identity_scale_writes_nothing() {
    assert(scaled_far(1200.0f, 1.3f, 1.0f) == 1200.0f);
    assert(!needs_edit(1200.0f, 1.3f, 1.0f));
}
static void unusable_planes_are_left_alone() {
    assert(!needs_edit(NAN, 1.3f, 2.5f));
    assert(!needs_edit(INFINITY, 1.3f, 2.5f));
    assert(!needs_edit(1.0f, 1.3f, 2.5f));  // far <= near
    assert(!needs_edit(0.0f, 0.0f, 2.5f));
    assert(!needs_edit(-5.0f, 1.3f, 2.5f));
}
static void scaled_far_clamps_the_scale() {
    assert(scaled_far(1000.0f, 1.0f, 50.0f) == 6000.0f);
    assert(scaled_far(1000.0f, 1.0f, 0.1f) == 1000.0f);
    assert(scaled_far(1000.0f, 1.0f, NAN) == 2500.0f);
}

int main() {
    default_is_two_and_a_half();
    valid_value_is_kept();
    out_of_range_is_clamped_with_a_warning();
    garbage_keeps_the_default();
    scale_parsing_for_env();
    scaled_far_multiplies();
    identity_scale_writes_nothing();
    unusable_planes_are_left_alone();
    scaled_far_clamps_the_scale();
    return 0;
}
