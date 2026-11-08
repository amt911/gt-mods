#include "cheats_logic.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>

namespace cheats {
namespace {

constexpr float kPi = 3.14159265358979f;

float wrap_pi(float a) {
    while (a > kPi) a -= 2.0f * kPi;
    while (a < -kPi) a += 2.0f * kPi;
    return a;
}

void normalise(float *r) {
    const float n = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
    if (n > 1e-9f)
        for (int i = 0; i < 3; ++i) r[i] /= n;
}
float dot(const float *a, const float *b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

// Gram-Schmidt on the three rows (the first keeps its direction).
void orthonormalise(float *rows) {
    float *r0 = rows, *r1 = rows + 3, *r2 = rows + 6;
    normalise(r0);
    const float d1 = dot(r1, r0);
    for (int i = 0; i < 3; ++i) r1[i] -= d1 * r0[i];
    normalise(r1);
    const float d20 = dot(r2, r0), d21 = dot(r2, r1);
    for (int i = 0; i < 3; ++i) r2[i] -= d20 * r0[i] + d21 * r1[i];
    normalise(r2);
}

}  // namespace

Kin scale_step(const Kin &pre, const Kin &post, float s) {
    if (s >= 1.0f || !std::isfinite(s)) return post;
    if (s <= 0.0f) return pre;
    float moved2 = 0.0f;
    for (int i = 0; i < kPos; ++i) {
        const float d = post.v[kIxPos + i] - pre.v[kIxPos + i];
        if (std::isfinite(d)) moved2 += d * d;
    }
    if (moved2 > kJumpMeters * kJumpMeters) return post;
    Kin out;
    bool rows_finite = true;
    for (int i = 0; i < kFloats; ++i) {
        // a field that is not a number before or after the step (some AI cars carry NaN in fields the step never uses) keeps what the game wrote
        out.v[i] = std::isfinite(pre.v[i]) && std::isfinite(post.v[i]) ? pre.v[i] + s * (post.v[i] - pre.v[i]) : post.v[i];
        if (i >= kIxRows && i < kIxRows + kRows && !std::isfinite(out.v[i])) rows_finite = false;
    }
    for (int i = 0; i < kEuler; ++i) {
        const int ix = kIxEuler + i;
        if (std::isfinite(pre.v[ix]) && std::isfinite(post.v[ix])) out.v[ix] = wrap_pi(pre.v[ix] + s * wrap_pi(post.v[ix] - pre.v[ix]));
    }
    if (rows_finite) orthonormalise(&out.v[kIxRows]);
    return out;
}

float factor(SlowMo m) { return m == SlowMo::Half ? 0.5f : m == SlowMo::Quarter ? 0.25f : 1.0f; }
const char *name(SlowMo m) { return m == SlowMo::Half ? "0.5x" : m == SlowMo::Quarter ? "0.25x" : "OFF"; }
SlowMo cycle(SlowMo m, int direction) {
    const int n = static_cast<int>(m) + (direction < 0 ? 2 : 1);
    return static_cast<SlowMo>(n % 3);
}
bool parse_slowmo(const char *text, SlowMo &out) {
    if (text == nullptr) return false;
    if (!std::strcmp(text, "off") || !std::strcmp(text, "OFF") || !std::strcmp(text, "1")) { out = SlowMo::Off; return true; }
    if (!std::strcmp(text, "0.5") || !std::strcmp(text, "0.5x")) { out = SlowMo::Half; return true; }
    if (!std::strcmp(text, "0.25") || !std::strcmp(text, "0.25x")) { out = SlowMo::Quarter; return true; }
    return false;
}

float car_factor(bool is_ai, SlowMo slow, bool freeze_ai) { return is_ai && freeze_ai ? 0.0f : factor(slow); }

Settings parse_settings(const std::function<const char *(const char *)> &get, const std::function<void(const std::string &)> &warn) {
    Settings s;
    if (const char *v = get("slow_motion"); v != nullptr && !parse_slowmo(v, s.slow))
        warn(std::string("slow_motion = \"") + v + "\" is not off, 0.5 or 0.25: using off");
    if (const char *v = get("freeze_ai"); v != nullptr) {
        if (!std::strcmp(v, "true") || !std::strcmp(v, "1")) s.freeze_ai = true;
        else if (std::strcmp(v, "false") && std::strcmp(v, "0")) warn(std::string("freeze_ai = \"") + v + "\" is not true or false: using false");
    }
    if (const char *v = get("from_step"); v != nullptr) {
        char *end = nullptr;
        const long n = std::strtol(v, &end, 10);
        if (end != v && *end == '\0' && n >= 0) s.from_step = n;
        else warn(std::string("from_step = \"") + v + "\" is not a step count >= 0: using 0");
    }
#if defined(CHEATS_SABOTAGE)
    s.sabotage = CHEATS_SABOTAGE;  // the in-game check's sabotage builds (tools/mod_cheats_check.sh: it must fail with them)
#endif
    return s;
}

}  // namespace cheats
