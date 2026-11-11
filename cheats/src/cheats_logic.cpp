#include <algorithm>
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
    if (s == 1.0f || !std::isfinite(s)) return post;
    s = std::min(s, kMaxScale);  // above 1 the step is extrapolated (a faster AI), bounded: the rows are re-orthonormalised below
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

namespace cheats {

std::uint32_t money_after(std::uint32_t value, int direction) {
    constexpr std::uint32_t kStep = 1000000u;
    if (direction < 0) return value > kStep ? value - kStep : 0u;
    return value >= kMoneyCap - kStep ? kMoneyCap : value + kStep;
}

bool backup_save(const std::filesystem::path &savedata, long long stamp, std::string &where, std::string &error) {
    namespace fs = std::filesystem;
    const fs::path from = savedata / "UCES01245-GAMEDAT";
    std::error_code ec;
    if (!fs::exists(from, ec)) {  // no save yet (a new game): nothing to lose, nothing to copy
        where.clear();
        return true;
    }
    const fs::path to = savedata.parent_path() / (savedata.filename().string() + "-backups") /
                        ("UCES01245-GAMEDAT-" + std::to_string(stamp));
    fs::create_directories(to.parent_path(), ec);
    fs::copy(from, to, fs::copy_options::recursive | fs::copy_options::overwrite_existing, ec);
    if (ec) {
        error = "backup failed: " + ec.message();
        return false;
    }
    where = to.string();
    return true;
}

}  // namespace cheats

namespace cheats {

float factor(AiSpeed s) {
    switch (s) {
    case AiSpeed::Half: return 0.5f;
    case AiSpeed::ThreeQuarter: return 0.75f;
    case AiSpeed::Fast: return 1.25f;
    default: return 1.0f;
    }
}
const char *name(AiSpeed s) {
    switch (s) {
    case AiSpeed::Half: return "0.5x";
    case AiSpeed::ThreeQuarter: return "0.75x";
    case AiSpeed::Fast: return "1.25x";
    default: return "1x";
    }
}
AiSpeed cycle(AiSpeed s, int direction) {
    const int n = 4, i = static_cast<int>(s);
    return static_cast<AiSpeed>(((i + (direction < 0 ? -1 : 1)) % n + n) % n);
}
float ai_factor(SlowMo slow, AiSpeed speed, bool freeze_ai) { return freeze_ai ? 0.0f : factor(slow) * factor(speed); }

Kin nitro(const Kin &now, float throttle, float dt) {
    if (throttle < kNitroThrottle) return now;
    Kin out = now;
    const float dv = kNitroAccel * dt;
    for (int i = 0; i < 3; ++i) out.v[kIxVel + i] += now.v[kIxRows + i] * dv;  // along the forward row
    out.v[kIxSpeed] += dv;
    return out;
}

// The car's up row (basis rows 6..8, FACTS "Car body state": forward, right, up) stands for "up": no world axis is assumed.
static float up_dot(const Kin &k, const float *v) {
    return v[0] * k.v[kIxRows + 6] + v[1] * k.v[kIxRows + 7] + v[2] * k.v[kIxRows + 8];
}

Kin moon(const Kin &before, const Kin &now) {
    Kin out = now;
    float change[3];
    for (int i = 0; i < 3; ++i) change[i] = now.v[kIxVel + i] - before.v[kIxVel + i];
    const float down = up_dot(now, change);  // < 0: the step took vertical speed away (falling)
    if (down < 0.0f)
        for (int i = 0; i < 3; ++i) out.v[kIxVel + i] -= (1.0f - kMoonKeep) * down * now.v[kIxRows + 6 + i];
    return out;
}

Kin jump(const Kin &now) {
    Kin out = now;
    for (int i = 0; i < 3; ++i) out.v[kIxVel + i] += kJumpSpeed * now.v[kIxRows + 6 + i];
    return out;
}

std::uint8_t pack(const Controls &c) {
    return static_cast<std::uint8_t>((static_cast<unsigned>(c.slow) & 3u) | (c.freeze ? 4u : 0u) | ((static_cast<unsigned>(c.ai) & 3u) << 3) |
                                     (c.nitro ? 0x20u : 0u) | (c.moon ? 0x40u : 0u) | (c.jump ? 0x80u : 0u));
}

Controls unpack(std::uint8_t b) {
    Controls c;
    c.slow = static_cast<SlowMo>(std::min(b & 3u, 2u));
    c.freeze = (b & 4u) != 0;
    c.ai = static_cast<AiSpeed>((b >> 3) & 3u);
    c.nitro = (b & 0x20u) != 0;
    c.moon = (b & 0x40u) != 0;
    c.jump = (b & 0x80u) != 0;
    return c;
}

void Tape::record(long step, std::uint8_t b) {
    if (step < 0) return;
    const auto at = static_cast<std::size_t>(step);
    if (at < bytes_.size()) {
        bytes_[at] = b;
        return;
    }
    const std::uint8_t fill = bytes_.empty() ? b : bytes_.back();
    bytes_.resize(at, fill);
    bytes_.push_back(b);
}

std::uint8_t Tape::at(long step) const {
    if (bytes_.empty()) return 0;
    if (step < 0) return bytes_.front();
    return static_cast<std::size_t>(step) < bytes_.size() ? bytes_[static_cast<std::size_t>(step)] : 0;
}

}  // namespace cheats
