#include "weather_logic.hpp"

#include <cassert>
#include <cmath>

using namespace weather;

static bool near(float a, float b, float eps = 1e-5f) { return std::fabs(a - b) <= eps; }

int main() {
    // presets: noon is neutral, dusk is dark and blue, the names parse (any case) and round-trip
    assert(neutral(preset_grade(Preset::Noon)) && !neutral(preset_grade(Preset::Dusk)));
    assert(preset_grade(Preset::Dusk).brightness < preset_grade(Preset::Sunset).brightness && preset_grade(Preset::Dusk).b > preset_grade(Preset::Dusk).r);
    assert(preset_grade(Preset::Sunset).r > preset_grade(Preset::Sunset).b);
    for (int i = 0; i < kPresetCount; ++i) {
        Preset p;
        assert(parse_preset(preset_name(static_cast<Preset>(i)), &p) && p == static_cast<Preset>(i));
    }
    Preset p;
    assert(parse_preset("DuSk", &p) && p == Preset::Dusk && !parse_preset("midnight", &p) && !parse_preset(nullptr, &p) && !parse_preset("afternoonafternoon", &p));
    // mix and progressive: the ends are the first and last preset, the thirds are the presets, in between linear; monotonic brightness
    assert(mix(Grade{}, Grade{0, 0, 0, 0, 0}, 0.5f).brightness == 0.5f && mix(Grade{}, Grade{0, 0, 0, 0, 0}, 2.f).brightness == 0.f);
    assert(neutral(progressive_grade(0.f)) && progressive_grade(1.f) == preset_grade(Preset::Dusk));
    assert(near(progressive_grade(1.f / 3.f).brightness, preset_grade(Preset::Afternoon).brightness) && near(progressive_grade(2.f / 3.f).brightness, preset_grade(Preset::Sunset).brightness, 1e-4f));
    assert(near(progressive_grade(0.5f).brightness, 0.5f * (preset_grade(Preset::Afternoon).brightness + preset_grade(Preset::Sunset).brightness), 1e-4f));
    float last = 2.f;
    for (int i = 0; i <= 100; ++i) {
        const float b = progressive_grade(static_cast<float>(i) / 100.f).brightness;
        assert(b <= last + 1e-6f);
        last = b;
    }
    assert(progressive_grade(-1.f) == progressive_grade(0.f) && progressive_grade(9.f) == progressive_grade(1.f));
    assert(near(progress(50, 100), 0.5f) && progress(500, 100) == 1.f && progress(0, 100) == 0.f && progress(5, 0) == 1.f);
    // combine multiplies
    const Grade c = combine(preset_grade(Preset::Dusk), rain_grade(3));
    assert(near(c.brightness, 0.60f * 0.82f) && near(c.saturation, 0.85f * 0.62f) && near(c.r, 0.62f * 0.86f));
    assert(neutral(combine(Grade{}, Grade{})));
    // rain: level 0 neutral and no streaks; stronger levels are darker and more desaturated and have more streaks
    assert(neutral(rain_grade(0)) && neutral(rain_grade(-3)) && rain_grade(1).brightness > rain_grade(2).brightness && rain_grade(2).brightness > rain_grade(3).brightness);
    assert(rain_grade(9) == rain_grade(3));
    Streak s[kMaxStreaks], s2[kMaxStreaks];
    assert(rain_streaks(0, 5, 0.f, s, kMaxStreaks) == 0 && rain_streaks(1, 5, 0.f, nullptr, 10) == 0 && rain_streaks(1, 5, 0.f, s, 0) == 0);
    const int n1 = rain_streaks(1, 5, 0.f, s, kMaxStreaks), n2 = rain_streaks(2, 5, 0.f, s, kMaxStreaks), n3 = rain_streaks(3, 5, 0.f, s, kMaxStreaks);
    assert(n1 > 0 && n1 < n2 && n2 < n3 && n3 <= kMaxStreaks && rain_streaks(3, 5, 0.f, s, 10) == 10);
    // deterministic: the same inputs give the same streaks; another frame moves them down; a shift moves them sideways
    rain_streaks(2, 40, 0.f, s, kMaxStreaks);
    rain_streaks(2, 40, 0.f, s2, kMaxStreaks);
    for (int i = 0; i < n2; ++i) assert(s[i].x0 == s2[i].x0 && s[i].y0 == s2[i].y0 && s[i].rgba == s2[i].rgba);
    rain_streaks(2, 41, 0.f, s2, kMaxStreaks);
    int moved = 0, down = 0;
    for (int i = 0; i < n2; ++i) {
        moved += s[i].y1 != s2[i].y1;
        down += s2[i].y1 > s[i].y1;
    }
    assert(moved > n2 * 9 / 10 && down > n2 * 8 / 10);  // all fall, except the ones that wrapped to the top
    rain_streaks(2, 40, 30.f, s2, kMaxStreaks);
    int sideways = 0;
    for (int i = 0; i < n2; ++i) sideways += near(s2[i].y1, s[i].y1) && s2[i].x1 != s[i].x1;
    assert(sideways > n2 * 9 / 10);
    // every streak is a line on (or just outside) the screen with a translucent colour
    for (int f = 0; f < 50; ++f) {
        const int n = rain_streaks(3, static_cast<std::uint32_t>(f), -75.f, s, kMaxStreaks);
        for (int i = 0; i < n; ++i) {
            assert(s[i].x1 > -41.f && s[i].x1 < 521.f && s[i].y1 > -1.f && s[i].y1 < 300.f && s[i].y0 < s[i].y1);
            assert((s[i].rgba & 0xFF) >= 0x40 && (s[i].rgba & 0xFF) < 0x70);
        }
    }
    // grip: a lateral velocity change is reduced by `loss`; a hit (large change), no change and loss 0 are left alone
    const float right[3] = {1, 0, 0}, prev[3] = {0, 20, 0};
    float out[3];
    const float now[3] = {1.0f, 20.5f, 0};
    assert(grip_adjust(now, prev, right, 0.5f, 1.5f, out) && near(out[0], 0.5f) && near(out[1], 20.5f) && out[2] == 0.f);
    assert(grip_adjust(now, prev, right, 2.f, 1.5f, out) && near(out[0], 0.f));  // loss clamps to 1: no lateral change at all
    assert(!grip_adjust(now, prev, right, 0.f, 1.5f, out));
    const float hit[3] = {6.f, 20.f, 0};
    assert(!grip_adjust(hit, prev, right, 0.5f, 1.5f, out));
    assert(!grip_adjust(prev, prev, right, 0.5f, 1.5f, out));
    const float bad[3] = {NAN, 0, 0};
    assert(!grip_adjust(bad, prev, right, 0.5f, 1.5f, out));
    return 0;
}
