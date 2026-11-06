#include "orbit_math.hpp"

#include <cmath>
#include <cstring>

namespace orbit {
namespace {
constexpr float kDegToRad = 3.14159265358979f / 180.0f;

void mul(const float a[16], const float b[16], float out[16]) {
    float t[16];
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c) {
            float s = 0.0f;
            for (int k = 0; k < 4; ++k) s += a[4 * r + k] * b[4 * k + c];
            t[4 * r + c] = s;
        }
    std::memcpy(out, t, sizeof t);
}
}  // namespace

void orbit_matrix(const float m[16], const float pivot[3], float yaw_deg, float pitch_deg, float out[16]) {
    const float y = yaw_deg * kDegToRad, q = pitch_deg * kDegToRad;
    const float cs = std::cos(y), sn = std::sin(y), cp = std::cos(q), sp = std::sin(q);
    const float ry[16] = {cs, 0, sn, 0, 0, 1, 0, 0, -sn, 0, cs, 0, 0, 0, 0, 1};
    const float rx[16] = {1, 0, 0, 0, 0, cp, sp, 0, 0, -sp, cp, 0, 0, 0, 0, 1};
    const float tn[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, -pivot[0], -pivot[1], -pivot[2], 1};
    const float tp[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, pivot[0], pivot[1], pivot[2], 1};
    float n[16];
    mul(m, tn, n);
    mul(n, ry, n);
    mul(n, rx, n);
    mul(n, tp, out);
}

bool is_perspective(float proj_m11) { return proj_m11 < -0.5f; }

void car_to_yup(float x, float y, float z, float out[3]) {
    out[0] = x;
    out[1] = z;
    out[2] = -y;
}

void pivot_in_view(const float c[16], const float car[3], float out[3]) {
    const float d[3] = {car[0] - c[12], car[1] - c[13], car[2] - c[14]};
    for (int r = 0; r < 3; ++r) out[r] = c[4 * r] * d[0] + c[4 * r + 1] * d[1] + c[4 * r + 2] * d[2];
}

bool needs_edit(float yaw_deg, float pitch_deg) { return yaw_deg != 0.0f || pitch_deg != 0.0f; }

}  // namespace orbit
