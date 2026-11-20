#include "ghost_logic.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace ghost {

const int kBoxEdges[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};

namespace {
float dot3(const float a[3], const float b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
}  // namespace

bool LapLine::step(const float position[3], const float velocity[3]) {
    for (int i = 0; i < 3; ++i)
        if (!std::isfinite(position[i]) || !std::isfinite(velocity[i])) return false;
    if (!have_) {
        const float hx = velocity[0], hy = velocity[1];
        const float speed = std::sqrt(hx * hx + hy * hy);
        if (speed < 1.5f) return false;  // the line is where the car first moves
        have_ = true;
        std::memcpy(origin_, position, sizeof origin_);
        heading_[0] = hx / speed;
        heading_[1] = hy / speed;
        heading_[2] = 0.f;
        last_s_ = 0.f;
        return false;
    }
    const float d[3] = {position[0] - origin_[0], position[1] - origin_[1], position[2] - origin_[2]};
    const float s = dot3(d, heading_);
    const float side = -d[0] * heading_[1] + d[1] * heading_[0];  // distance across the line
    const float dist = std::sqrt(d[0] * d[0] + d[1] * d[1]);
    bool crossed = false;
    if (armed_ && last_s_ < 0.f && s >= 0.f && std::fabs(side) <= half_width_) {  // armed by an EARLIER step: a jump past the line in one step is no lap
        crossed = true;
        armed_ = false;
    }
    if (dist >= min_distance_) armed_ = true;
    last_s_ = s;
    return crossed;
}

bool Recorder::lap_done(std::uint32_t min_steps) {
    const bool valid = current_.steps() >= min_steps;
    bool better = false;
    if (valid) {
        ++laps_;
        if (!has_best() || current_.steps() < best_.steps()) {
            best_ = current_;
            better = true;
        }
    }
    current_.poses.clear();
    return better;
}

bool ghost_pose(const Lap &best, std::uint32_t step, Pose *out) {
    if (best.poses.empty()) return false;
    *out = best.poses[step < best.poses.size() ? step : best.poses.size() - 1];
    return true;
}

Pose lerp(const Pose &a, const Pose &b, float t) {
    Pose r;
    if (t < 0.f) t = 0.f;
    if (t > 1.f) t = 1.f;
    for (int i = 0; i < 3; ++i) r.position[i] = a.position[i] + (b.position[i] - a.position[i]) * t;
    for (int i = 0; i < 9; ++i) r.basis[i] = a.basis[i] + (b.basis[i] - a.basis[i]) * t;
    for (int row = 0; row < 3; ++row) {
        float n = 0.f;
        for (int k = 0; k < 3; ++k) n += r.basis[3 * row + k] * r.basis[3 * row + k];
        n = std::sqrt(n);
        if (n > 1e-6f)
            for (int k = 0; k < 3; ++k) r.basis[3 * row + k] /= n;
    }
    return r;
}

void box_corners(const Pose &pose, const Box &box, float out[8][3]) {
    const float hl = 0.5f * box.length, hw = 0.5f * box.width, hh = 0.5f * box.height;
    // corners 0..3 the bottom face (front-left, front-right, rear-right, rear-left), 4..7 the top face above them
    const float sx[4] = {-1, 1, 1, -1}, sy[4] = {1, 1, -1, -1};
    for (int i = 0; i < 8; ++i) {
        const float f = sy[i & 3] * hl, r = sx[i & 3] * hw, u = (i < 4 ? -hh : hh) + box.lift;
        for (int k = 0; k < 3; ++k) out[i][k] = pose.position[k] + pose.basis[k] * f + pose.basis[3 + k] * r + pose.basis[6 + k] * u;
    }
}

bool project(const Camera &camera, const float point[3], float *x, float *y, float near_plane) {
    const float d[3] = {point[0] - camera.eye[0], point[1] - camera.eye[1], point[2] - camera.eye[2]};
    const float vz = -dot3(camera.basis, d);        // forward row: depth along the viewing direction is -z in view space
    if (!(vz <= -near_plane)) return false;
    const float vx = dot3(camera.basis + 3, d), vy = dot3(camera.basis + 6, d);
    *x = 240.f + camera.focal_x * vx / -vz;
    *y = 136.f - camera.focal_y * vy / -vz;
    return std::isfinite(*x) && std::isfinite(*y);
}

std::string serialize_lap(const Lap &lap) {
    std::string out("GTGL", 4);
    const std::uint32_t version = 1, n = lap.steps();
    out.append(reinterpret_cast<const char *>(&version), 4);
    out.append(reinterpret_cast<const char *>(&n), 4);
    for (const Pose &p : lap.poses) {
        out.append(reinterpret_cast<const char *>(p.position), sizeof p.position);
        out.append(reinterpret_cast<const char *>(p.basis), sizeof p.basis);
    }
    return out;
}

bool parse_lap(const std::string &bytes, Lap *out) {
    if (bytes.size() < 12 || bytes.compare(0, 4, "GTGL") != 0) return false;
    std::uint32_t version, n;
    std::memcpy(&version, bytes.data() + 4, 4);
    std::memcpy(&n, bytes.data() + 8, 4);
    if (version != 1 || n == 0 || n > 60u * 60u * 30u) return false;  // up to 30 minutes
    if (bytes.size() != 12 + static_cast<std::size_t>(n) * sizeof(Pose)) return false;
    Lap lap;
    lap.poses.resize(n);
    for (std::uint32_t i = 0; i < n; ++i) {
        std::memcpy(&lap.poses[i], bytes.data() + 12 + static_cast<std::size_t>(i) * sizeof(Pose), sizeof(Pose));
        for (float v : lap.poses[i].position)
            if (!std::isfinite(v)) return false;
        for (float v : lap.poses[i].basis)
            if (!std::isfinite(v)) return false;
    }
    *out = std::move(lap);
    return true;
}

std::string lap_file_name(const float origin[3], const float heading[3], float wheelbase) {
    std::uint64_t h = 1469598103934665603ull;
    auto mix = [&](long v) {
        for (int i = 0; i < 8; ++i) {
            h ^= static_cast<std::uint8_t>((static_cast<unsigned long>(v) >> (8 * i)) & 0xFF);
            h *= 1099511628211ull;
        }
    };
    mix(std::lround(origin[0]));
    mix(std::lround(origin[1]));
    mix(std::lround(origin[2]));
    mix(std::lround(std::atan2(heading[1], heading[0]) * 180.0 / 3.14159265358979 / 5.0));  // 5-degree buckets
    mix(std::lround(wheelbase * 100.f));
    char buf[40];
    std::snprintf(buf, sizeof buf, "%016llx.lap", static_cast<unsigned long long>(h));
    return buf;
}

}  // namespace ghost
