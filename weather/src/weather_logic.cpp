#include "weather_logic.hpp"

#include <cctype>
#include <cmath>
#include <cstring>

namespace weather {

namespace {
float clamp01(float v) { return v < 0.f ? 0.f : (v > 1.f ? 1.f : v); }
float lerp(float a, float b, float t) { return a + (b - a) * t; }
bool near1(float v) { return std::fabs(v - 1.f) < 1e-4f; }
std::uint32_t hash(std::uint32_t x) {  // xorshift-multiply: the streak's fixed random numbers
    x ^= x >> 16;
    x *= 0x7FEB352Du;
    x ^= x >> 15;
    x *= 0x846CA68Bu;
    x ^= x >> 16;
    return x;
}
}  // namespace

bool operator==(const Grade &a, const Grade &b) { return a.r == b.r && a.g == b.g && a.b == b.b && a.brightness == b.brightness && a.saturation == b.saturation; }
bool neutral(const Grade &g) { return near1(g.r) && near1(g.g) && near1(g.b) && near1(g.brightness) && near1(g.saturation); }

const char *preset_name(Preset p) {
    switch (p) {
    case Preset::Noon: return "noon";
    case Preset::Afternoon: return "afternoon";
    case Preset::Sunset: return "sunset";
    case Preset::Dusk: return "dusk";
    }
    return "?";
}

bool parse_preset(const char *text, Preset *out) {
    if (text == nullptr) return false;
    char lower[16];
    std::size_t n = 0;
    for (; text[n] != '\0' && n < sizeof lower - 1; ++n) lower[n] = static_cast<char>(std::tolower(static_cast<unsigned char>(text[n])));
    lower[n] = '\0';
    if (text[n] != '\0') return false;
    for (int i = 0; i < kPresetCount; ++i)
        if (std::strcmp(lower, preset_name(static_cast<Preset>(i))) == 0) {
            *out = static_cast<Preset>(i);
            return true;
        }
    return false;
}

Grade preset_grade(Preset p) {
    switch (p) {
    case Preset::Noon: return {1.00f, 1.00f, 1.00f, 1.00f, 1.00f};
    case Preset::Afternoon: return {1.06f, 0.97f, 0.88f, 0.97f, 1.05f};
    case Preset::Sunset: return {1.20f, 0.82f, 0.62f, 0.88f, 1.10f};
    case Preset::Dusk: return {0.62f, 0.70f, 1.00f, 0.60f, 0.85f};
    }
    return {};
}

Grade mix(const Grade &a, const Grade &b, float t) {
    t = clamp01(t);
    return {lerp(a.r, b.r, t), lerp(a.g, b.g, t), lerp(a.b, b.b, t), lerp(a.brightness, b.brightness, t), lerp(a.saturation, b.saturation, t)};
}

Grade progressive_grade(float t) {
    t = clamp01(t) * (kPresetCount - 1);
    int i = static_cast<int>(t);
    if (i >= kPresetCount - 1) i = kPresetCount - 2;
    return mix(preset_grade(static_cast<Preset>(i)), preset_grade(static_cast<Preset>(i + 1)), t - static_cast<float>(i));
}

float progress(std::uint64_t steps, std::uint64_t total) { return total == 0 ? 1.f : clamp01(static_cast<float>(steps) / static_cast<float>(total)); }

Grade combine(const Grade &a, const Grade &b) { return {a.r * b.r, a.g * b.g, a.b * b.b, a.brightness * b.brightness, a.saturation * b.saturation}; }

Grade rain_grade(int level) {
    if (level <= 0) return {};
    if (level > kRainLevels - 1) level = kRainLevels - 1;
    const float k = static_cast<float>(level) / static_cast<float>(kRainLevels - 1);  // 1/3, 2/3, 1
    return mix(Grade{}, Grade{0.86f, 0.91f, 1.00f, 0.82f, 0.62f}, k);
}

const char *rain_name(int level) {
    static const char *names[kRainLevels] = {"off", "light", "medium", "heavy"};
    return names[level < 0 ? 0 : (level >= kRainLevels ? kRainLevels - 1 : level)];
}

int rain_streaks(int level, std::uint32_t frame, float shift, Streak *out, int capacity) {
    static const int kCount[kRainLevels] = {0, 60, 140, 260};
    if (level <= 0 || out == nullptr || capacity <= 0) return 0;
    const int n = kCount[level >= kRainLevels ? kRainLevels - 1 : level] < capacity ? kCount[level >= kRainLevels ? kRainLevels - 1 : level] : capacity;
    constexpr float kW = 480.f, kH = 272.f, kSlant = 0.22f;
    for (int i = 0; i < n; ++i) {
        const std::uint32_t h = hash(static_cast<std::uint32_t>(i) * 2654435761u + 12345u);
        const float x0 = static_cast<float>(h % 480u);
        const float y0 = static_cast<float>((h >> 9) % 272u);
        const float speed = 7.f + static_cast<float>((h >> 18) % 7u);   // px per picture
        const float len = 8.f + static_cast<float>((h >> 21) % 11u);
        const float span = kH + len;
        float y = std::fmod(y0 + speed * static_cast<float>(frame), span);
        float x = x0 + kSlant * y + shift;
        x = std::fmod(std::fmod(x, kW + 40.f) + kW + 40.f, kW + 40.f) - 20.f;  // wraps over the screen with a margin: a streak leaves one side and re-enters
        const std::uint32_t alpha = 0x40u + (h >> 25) % 0x30u;
        out[i] = {x - kSlant * len, y - len, x, y, 0xC8D2E600u | alpha};
    }
    return n;
}

bool grip_adjust(const float now[3], const float prev[3], const float right[3], float loss, float max_dv, float out[3]) {
    if (!(loss > 0.f)) return false;
    if (loss > 1.f) loss = 1.f;
    float dlat = 0.f;
    for (int i = 0; i < 3; ++i) dlat += right[i] * (now[i] - prev[i]);
    if (!std::isfinite(dlat) || std::fabs(dlat) > max_dv || dlat == 0.f) return false;  // a hit, or nothing to reduce
    for (int i = 0; i < 3; ++i) out[i] = now[i] - right[i] * dlat * loss;
    return true;
}

}  // namespace weather
