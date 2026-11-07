#include "far_math.hpp"

#include <cmath>
#include <cstdlib>

namespace drawdist {

float clamp_scale(float scale) {
    if (!std::isfinite(scale)) return kDefaultScale;
    if (scale < kMinScale) return kMinScale;
    if (scale > kMaxScale) return kMaxScale;
    return scale;
}

bool parse_scale(const char *text, float *out) {
    if (text == nullptr || *text == '\0') return false;
    char *end = nullptr;
    const double v = std::strtod(text, &end);
    if (end == text || *end != '\0' || !std::isfinite(v)) return false;
    *out = clamp_scale(static_cast<float>(v));
    return true;
}

Settings parse_settings(const std::function<const char *(const char *)> &get, const std::function<void(const std::string &)> &warn) {
    Settings s;
    const char *text = get("far_scale");
    if (text == nullptr || *text == '\0') return s;
    char *end = nullptr;
    const double v = std::strtod(text, &end);
    if (end == text || *end != '\0' || !std::isfinite(v)) {
        warn(std::string("far_scale \"") + text + "\" is not a number; using the default");
        return s;
    }
    s.far_scale = clamp_scale(static_cast<float>(v));
    if (static_cast<double>(s.far_scale) != v) warn(std::string("far_scale \"") + text + "\" is outside [1, 6]; clamped");
    return s;
}

float scaled_far(float far_plane, float near_plane, float scale) {
    const float k = clamp_scale(scale);
    if (k == 1.0f) return far_plane;
    if (!std::isfinite(far_plane) || !std::isfinite(near_plane) || far_plane <= near_plane || far_plane <= 0.0f) return far_plane;
    return far_plane * k;
}

bool needs_edit(float far_plane, float near_plane, float scale) {
    if (clamp_scale(scale) == 1.0f) return false;
    return std::isfinite(far_plane) && std::isfinite(near_plane) && far_plane > near_plane && far_plane > 0.0f;
}

} // namespace drawdist
