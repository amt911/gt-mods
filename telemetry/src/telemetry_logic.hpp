// Pure logic of the telemetry mod (no SDK, no guest memory): G forces from the velocity change, a fixed-size history ring, graph scaling.
#pragma once

#include <cstddef>
#include <vector>

namespace telemetry {

constexpr float kGravity = 9.81f;       // m/s^2 per G
constexpr float kStepSeconds = 1.f / 60.f;  // one physics step (FACTS "Car body state": 60 per second of game time)
constexpr size_t kHistory = 300;        // 300 steps = 5 s
constexpr float kJumpMeters = 15.f;     // a car that moves further in one step was teleported (race start, retry, reset): not driving
constexpr float kShownGLimit = 12.f;    // what the graph and the digits clamp to (a barrier hit is 40+ G in one step)

struct GForce {
    float longitudinal;  // + accelerating, - braking
    float lateral;       // + toward the car's right
};

// Acceleration of the body from the velocity change of one step, projected on the car's forward and right axes (the up axis is left out:
// gravity and suspension live there). `forward` and `right` are unit rows of the body basis.
GForce g_from_velocity_change(const float v_prev[3], const float v_now[3], const float forward[3], const float right[3], float dt);

// Moving average over the last `window` values (window >= 1).
class Smoother {
public:
    explicit Smoother(size_t window = 4) : window_(window < 1 ? 1 : window), values_(window_, 0.f) {}
    float push(float value);
    void reset();

private:
    size_t window_;
    std::vector<float> values_;
    size_t next_ = 0;
    size_t count_ = 0;
    float sum_ = 0.f;
};

struct Sample {
    float speed = 0.f;  // m/s
    float g_long = 0.f;
    float g_lat = 0.f;
    float throttle = 0.f;
    float brake = 0.f;
};

// A ring of the newest `capacity` samples; at(0) is the oldest.
class Ring {
public:
    explicit Ring(size_t capacity = kHistory) : data_(capacity) {}
    void push(const Sample &sample);
    void clear() { size_ = 0; head_ = 0; }
    size_t size() const { return size_; }
    size_t capacity() const { return data_.size(); }
    const Sample &at(size_t i) const;  // oldest first; i < size()

private:
    std::vector<Sample> data_;
    size_t head_ = 0;  // where the next sample goes
    size_t size_ = 0;
};

struct CarInput {
    float position[3];
    float velocity[3];
    float forward[3];
    float right[3];
    float speed;
    float throttle;
    float brake;
};

// Turns the car's state of each world step into samples of the ring: G from the velocity change smoothed over 4 steps, the first step and a
// teleport giving G = 0.
class Tracker {
public:
    Tracker() : smooth_long_(4), smooth_lat_(4) {}
    Sample feed(const CarInput &in);  // returns the sample it appended to the ring
    void reset();
    const Ring &ring() const { return ring_; }

private:
    Ring ring_{kHistory};
    Smoother smooth_long_, smooth_lat_;
    bool have_prev_ = false;
    float prev_position_[3] = {0, 0, 0};
    float prev_velocity_[3] = {0, 0, 0};
};

// Graph scaling. A y range for a value series: [lo, hi] with hi >= lo + min_span.
struct Range {
    float lo, hi;
};
Range speed_range(const Ring &ring, float floor_hi);          // 0 .. max(floor_hi, peak speed rounded up to a multiple of 20 m/s)
Range g_range(const Ring &ring, float floor_half);            // symmetric about 0: +-max(floor_half, peak |G| rounded up to a whole G), clamped to kShownGLimit
// Maps `value` in [lo, hi] to a y in [y_top, y_top + height] (hi at the top), clamping to the box.
float graph_y(float value, Range range, float y_top, float height);
// x of sample i of n across [x0, x0 + width] (the newest at the right edge); n == 1 gives x0 + width.
float graph_x(size_t i, size_t n, size_t capacity, float x0, float width);
// Clamp a G for display.
float shown_g(float g);

}  // namespace telemetry
