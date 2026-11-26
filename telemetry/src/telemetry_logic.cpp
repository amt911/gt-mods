#include "telemetry_logic.hpp"

#include <algorithm>
#include <cmath>

namespace telemetry {

GForce g_from_velocity_change(const float v_prev[3], const float v_now[3], const float forward[3], const float right[3], float dt) {
    float a[3];
    for (int i = 0; i < 3; ++i) a[i] = (v_now[i] - v_prev[i]) / dt;
    GForce g;
    g.longitudinal = (a[0] * forward[0] + a[1] * forward[1] + a[2] * forward[2]) / kGravity;
    g.lateral = (a[0] * right[0] + a[1] * right[1] + a[2] * right[2]) / kGravity;
    return g;
}

float Smoother::push(float value) {
    if (count_ == window_) sum_ -= values_[next_];
    else ++count_;
    values_[next_] = value;
    sum_ += value;
    next_ = (next_ + 1) % window_;
    return sum_ / static_cast<float>(count_);
}

void Smoother::reset() {
    next_ = 0;
    count_ = 0;
    sum_ = 0.f;
}

void Ring::push(const Sample &sample) {
    data_[head_] = sample;
    head_ = (head_ + 1) % data_.size();
    if (size_ < data_.size()) ++size_;
}

const Sample &Ring::at(size_t i) const {
    const size_t oldest = (head_ + data_.size() - size_) % data_.size();
    return data_[(oldest + i) % data_.size()];
}

void Tracker::reset() {
    ring_.clear();
    smooth_long_.reset();
    smooth_lat_.reset();
    have_prev_ = false;
}

Sample Tracker::feed(const CarInput &in) {
    bool teleported = false;
    if (have_prev_) {
        const float dx = in.position[0] - prev_position_[0], dy = in.position[1] - prev_position_[1], dz = in.position[2] - prev_position_[2];
        teleported = std::sqrt(dx * dx + dy * dy + dz * dz) > kJumpMeters;
    }
    if (teleported) {
        smooth_long_.reset();
        smooth_lat_.reset();
    }
    Sample s;
    s.speed = in.speed;
    s.throttle = in.throttle;
    s.brake = in.brake;
    if (have_prev_ && !teleported) {
        const GForce g = g_from_velocity_change(prev_velocity_, in.velocity, in.forward, in.right, kStepSeconds);
        s.g_long = smooth_long_.push(g.longitudinal);
        s.g_lat = smooth_lat_.push(g.lateral);
    }
    for (int i = 0; i < 3; ++i) {
        prev_position_[i] = in.position[i];
        prev_velocity_[i] = in.velocity[i];
    }
    have_prev_ = true;
    ring_.push(s);
    return s;
}

Range speed_range(const Ring &ring, float floor_hi) {
    float peak = 0.f;
    for (size_t i = 0; i < ring.size(); ++i) peak = std::max(peak, ring.at(i).speed);
    const float rounded = std::ceil(peak / 20.f) * 20.f;
    return {0.f, std::max(floor_hi, rounded)};
}

Range g_range(const Ring &ring, float floor_half) {
    float peak = 0.f;
    for (size_t i = 0; i < ring.size(); ++i) peak = std::max({peak, std::fabs(ring.at(i).g_long), std::fabs(ring.at(i).g_lat)});
    const float half = std::min(kShownGLimit, std::max(floor_half, std::ceil(peak)));
    return {-half, half};
}

float graph_y(float value, Range range, float y_top, float height) {
    const float span = range.hi - range.lo;
    if (!(span > 0.f)) return y_top + height;
    float t = (value - range.lo) / span;
    t = std::min(1.f, std::max(0.f, t));
    return y_top + (1.f - t) * height;
}

float graph_x(size_t i, size_t n, size_t capacity, float x0, float width) {
    // the newest sample sits at the right edge; a ring that is not full leaves the left part empty
    if (n == 0 || capacity < 2) return x0 + width;
    const float step = width / static_cast<float>(capacity - 1);
    return x0 + width - static_cast<float>(n - 1 - i) * step;
}

float shown_g(float g) { return std::min(kShownGLimit, std::max(-kShownGLimit, g)); }

}  // namespace telemetry
