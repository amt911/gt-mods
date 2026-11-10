#include <filesystem>
#include <fstream>
#include <string>
#include "cheats_logic.hpp"

#include <cassert>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <string>
#include <vector>

using namespace cheats;

static bool near(float a, float b, float eps = 1e-4f) { return std::fabs(a - b) < eps; }

static Kin base() {
    Kin k;
    k.v[kIxRows + 0] = 1;  // identity basis
    k.v[kIxRows + 4] = 1;
    k.v[kIxRows + 8] = 1;
    return k;
}

static void scale_endpoints_are_exact() {
    Kin pre = base(), post = base();
    pre.v[kIxPos] = 10.0f;
    post.v[kIxPos] = 11.7f;
    post.v[kIxVel] = 30.0f;
    post.v[kIxSpeed] = 30.0f;
    const Kin full = scale_step(pre, post, 1.0f), none = scale_step(pre, post, 0.0f);
    assert(std::memcmp(&full, &post, sizeof post) == 0);   // s = 1: the full step, bit for bit
    assert(std::memcmp(&none, &pre, sizeof pre) == 0);     // s = 0: the step never happened
    Kin half = scale_step(pre, post, 0.5f);
    assert(near(half.v[kIxPos], 10.85f) && near(half.v[kIxVel], 15.0f) && near(half.v[kIxSpeed], 15.0f));
    Kin quarter = scale_step(pre, post, 0.25f);
    assert(near(quarter.v[kIxPos], 10.425f) && near(quarter.v[kIxVel], 7.5f));
}

static void scale_time_dilation_matches_half_the_steps() {
    // constant acceleration a, step dt: state after n full steps; the half-time run reaches after 2n steps what the full run reaches after n.
    const float a = 4.0f, dt = 0.01f;
    float pos_full = 0, vel_full = 0, pos_slow = 0, vel_slow = 0;
    for (int n = 0; n < 800; ++n) {
        pos_full += vel_full * dt;  // explicit Euler: position then velocity
        vel_full += a * dt;
    }
    for (int n = 0; n < 1600; ++n) {
        Kin pre = base(), post = base();
        pre.v[kIxPos] = pos_slow;
        pre.v[kIxVel] = vel_slow;
        post.v[kIxPos] = pos_slow + vel_slow * dt;
        post.v[kIxVel] = vel_slow + a * dt;
        const Kin out = scale_step(pre, post, 0.5f);
        pos_slow = out.v[kIxPos];
        vel_slow = out.v[kIxVel];
    }
    assert(near(vel_slow, vel_full, 1e-2f) && std::fabs(pos_slow - pos_full) < 0.01f * pos_full);  // 1600 half-time steps = 800 full steps (to the Euler error, ~1/n)
}

static void euler_angles_wrap_the_short_way() {
    Kin pre = base(), post = base();
    pre.v[kIxEuler + 2] = 3.10f;   // just below +pi
    post.v[kIxEuler + 2] = -3.10f;  // just past -pi: a turn of 0.083 rad, not -6.2
    const Kin half = scale_step(pre, post, 0.5f);
    const float yaw = half.v[kIxEuler + 2];
    assert(std::fabs(std::fabs(yaw) - 3.1416f) < 0.02f);  // near +-pi, not near 0
}

static void rows_stay_orthonormal() {
    Kin pre = base(), post = base();
    const float c = std::cos(0.6f), s = std::sin(0.6f);  // post: rotated 0.6 rad about z
    post.v[kIxRows + 0] = c; post.v[kIxRows + 1] = s;
    post.v[kIxRows + 3] = -s; post.v[kIxRows + 4] = c;
    post.v[kIxPos] = 0.1f;
    const Kin q = scale_step(pre, post, 0.5f);
    const float *r = &q.v[kIxRows];
    auto dot = [](const float *a, const float *b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; };
    assert(near(dot(r, r), 1) && near(dot(r + 3, r + 3), 1) && near(dot(r + 6, r + 6), 1));
    assert(near(dot(r, r + 3), 0) && near(dot(r, r + 6), 0) && near(dot(r + 3, r + 6), 0));
    assert(near(r[1], std::sin(0.3f), 1e-3f));  // half the rotation
}

static void a_jump_or_nan_is_not_scaled() {
    Kin pre = base(), post = base();
    post.v[kIxPos] = 50.0f;  // a teleport (reset-to-track)
    assert(scale_step(pre, post, 0.5f).v[kIxPos] == 50.0f);
    post.v[kIxPos] = 1.0f;
    post.v[kIxVel] = std::numeric_limits<float>::quiet_NaN();
    assert(std::isnan(scale_step(pre, post, 0.5f).v[kIxVel]));  // handed back as is: never invent numbers
    assert(near(scale_step(pre, post, 0.5f).v[kIxPos], 0.5f));  // the other fields are still scaled
    const Kin frozen = scale_step(pre, post, 0.0f);
    assert(frozen.v[kIxPos] == 0.0f);                           // s = 0 restores the whole pre-step state
    Kin pre_nan = base();
    pre_nan.v[kIxRates] = std::numeric_limits<float>::quiet_NaN();  // a NaN in an unused field before the step
    Kin post_ok = base();
    post_ok.v[kIxPos] = 2.0f;
    post_ok.v[kIxRates] = 1.0f;
    const Kin mixed = scale_step(pre_nan, post_ok, 0.5f);
    assert(near(mixed.v[kIxPos], 1.0f) && mixed.v[kIxRates] == 1.0f);
}

static void slowmo_cycle_and_names() {
    assert(cycle(SlowMo::Off, 1) == SlowMo::Half && cycle(SlowMo::Half, 1) == SlowMo::Quarter && cycle(SlowMo::Quarter, 1) == SlowMo::Off);
    assert(cycle(SlowMo::Off, -1) == SlowMo::Quarter && cycle(SlowMo::Quarter, -1) == SlowMo::Half && cycle(SlowMo::Half, -1) == SlowMo::Off);
    assert(cycle(SlowMo::Half, 0) == SlowMo::Quarter);  // Enter goes forward
    assert(factor(SlowMo::Off) == 1.0f && factor(SlowMo::Half) == 0.5f && factor(SlowMo::Quarter) == 0.25f);
    assert(std::string(name(SlowMo::Half)) == "0.5x");
    SlowMo m;
    assert(parse_slowmo("0.25", m) && m == SlowMo::Quarter && parse_slowmo("off", m) && m == SlowMo::Off && !parse_slowmo("0.3", m) && !parse_slowmo(nullptr, m));
}

static void car_factors() {
    assert(car_factor(false, SlowMo::Half, true) == 0.5f);   // the player is never frozen
    assert(car_factor(true, SlowMo::Half, false) == 0.5f);   // slow motion covers AI cars
    assert(car_factor(true, SlowMo::Half, true) == 0.0f);    // freeze wins for AI
    assert(car_factor(true, SlowMo::Off, true) == 0.0f && car_factor(true, SlowMo::Off, false) == 1.0f);
}

static Settings parse(const std::map<std::string, std::string> &m, std::vector<std::string> *warns) {
    return parse_settings([&](const char *k) -> const char * { auto it = m.find(k); return it == m.end() ? nullptr : it->second.c_str(); },
                          [&](const std::string &w) { warns->push_back(w); });
}

static void settings() {
    std::vector<std::string> w;
    Settings d = parse({}, &w);
    assert(w.empty() && d.slow == SlowMo::Off && !d.freeze_ai && d.from_step == 0 && d.sabotage == 0);
    Settings s = parse({{"slow_motion", "0.25"}, {"freeze_ai", "true"}, {"from_step", "700"}}, &w);
    assert(w.empty() && s.slow == SlowMo::Quarter && s.freeze_ai && s.from_step == 700);
    s = parse({{"slow_motion", "fast"}, {"freeze_ai", "maybe"}, {"from_step", "-3"}}, &w);
    assert(w.size() == 3 && s.slow == SlowMo::Off && !s.freeze_ai && s.from_step == 0);
}

void money() {
    using cheats::money_cell_valid;
    assert(money_cell_valid(100000u, 0u, 0x00020100u));
    assert(!money_cell_valid(100000u, 1u, 0x00020100u));          // high word set: not the balance
    assert(!money_cell_valid(100000u, 0u, 0x00020101u));          // another script value type
    assert(money_cell_valid(1100000u, 0u, 0x01030000u));          // after the game saved or loaded
    assert(!money_cell_valid(1000000000u, 0u, 0x00020100u));      // above the game's cap
    assert(cheats::money_after(100000u, +1) == 1100000u);
    assert(cheats::money_after(500000u, -1) == 0u);               // never below zero
    assert(cheats::money_after(999500000u, +1) == 999999999u);    // the game's cap
    assert(cheats::money_after(42u, 0) == 1000042u);               // Enter adds
}

void backup() {
    namespace fs = std::filesystem;
    const fs::path root = fs::temp_directory_path() / "gt_cheats_backup_test";
    fs::remove_all(root);
    fs::create_directories(root / "savedata" / "UCES01245-GAMEDAT");
    std::ofstream(root / "savedata" / "UCES01245-GAMEDAT" / "GAME.DAT") << "save";
    std::string where, error;
    assert(cheats::backup_save(root / "savedata", 1234567890ll, where, error));
    assert(fs::exists(fs::path(where) / "GAME.DAT"));
    assert(fs::path(where).parent_path() == root / "savedata-backups");  // outside the savedata folder the game scans
    assert(cheats::backup_save(root / "missing", 1ll, where, error) && where.empty());  // no save yet: nothing to lose
    fs::remove_all(root);
}

int main() {
    money();
    backup();
    scale_endpoints_are_exact();
    scale_time_dilation_matches_half_the_steps();
    euler_angles_wrap_the_short_way();
    rows_stay_orthonormal();
    a_jump_or_nan_is_not_scaled();
    slowmo_cycle_and_names();
    car_factors();
    settings();
    return 0;
}
