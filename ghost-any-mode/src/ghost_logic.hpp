// Best-lap ghost in any mode: the pure part (no game access). Everything is in the physics frame (Z up) with the car's row convention (forward, right, up rows).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace ghost {

struct Pose {
    float position[3] = {0, 0, 0};
    float basis[9] = {0, 1, 0, 1, 0, 0, 0, 0, 1};
};

// The lap line: the point where the car first moved and its heading then (the rolling start's line is what the game's own lap counter uses; the game's lap event was not found: FACTS "Lap line").
// A lap is counted when the car, having been at least `min_distance` m away, crosses the plane through `origin` perpendicular to `heading` forward, within `half_width` m of it sideways.
class LapLine {
public:
    explicit LapLine(float min_distance = 60.f, float half_width = 45.f) : min_distance_(min_distance), half_width_(half_width) {}
    void reset() { *this = LapLine(min_distance_, half_width_); }
    // The car's position and horizontal velocity after a physics step. Returns true when this step crossed the line (a lap completed).
    bool step(const float position[3], const float velocity[3]);
    bool has_line() const { return have_; }
    const float *origin() const { return origin_; }
    const float *heading() const { return heading_; }

private:
    float min_distance_, half_width_;
    bool have_ = false, armed_ = false;
    float origin_[3] = {0, 0, 0}, heading_[3] = {0, 1, 0};
    float last_s_ = 0.f;
};

// The poses of a lap, one per physics step (the lap time is its length / 60 s).
struct Lap {
    std::vector<Pose> poses;
    std::uint32_t steps() const { return static_cast<std::uint32_t>(poses.size()); }
};

// Records the current lap and keeps the best one (fewest steps). `lap_done` is called when the line is crossed.
class Recorder {
public:
    void push(const Pose &pose) { current_.poses.push_back(pose); }
    // Ends the lap: returns true when it is the new best (kept). Laps shorter than `min_steps` (a wrong crossing) are ignored. The next lap starts empty.
    bool lap_done(std::uint32_t min_steps = 300);
    void clear_current() { current_.poses.clear(); }
    void clear_all() { current_.poses.clear(); best_.poses.clear(); laps_ = 0; }
    void set_best(Lap lap) { best_ = std::move(lap); }
    std::uint32_t current_steps() const { return current_.steps(); }
    const Lap &best() const { return best_; }
    bool has_best() const { return best_.steps() > 0; }
    unsigned laps() const { return laps_; }

private:
    Lap current_, best_;
    unsigned laps_ = 0;
};

// The ghost's pose at lap step `step` (0 = the lap's first step): the recorded pose, clamped to the last one after the best lap's end. False when there is no best lap.
bool ghost_pose(const Lap &best, std::uint32_t step, Pose *out);
// Interpolated between two poses (the physics runs at 60 Hz, a picture can fall between): position linear, basis rows renormalised.
Pose lerp(const Pose &a, const Pose &b, float t);

// The 8 corners of the ghost's box (the car's `length` x `width` x `height`, its centre `lift` above the pose position) in the physics frame, and the 12 edges as corner index pairs.
struct Box {
    float length = 4.5f, width = 1.9f, height = 1.3f, lift = 0.25f;
};
void box_corners(const Pose &pose, const Box &box, float out[8][3]);
extern const int kBoxEdges[12][2];

// The camera as camera_get / camera_projection give it, and the screen position (480 x 272) of a point; false when the point is behind the camera (z_view > -near).
struct Camera {
    float eye[3] = {0, 0, 0};
    float basis[9] = {0, 1, 0, 1, 0, 0, 0, 0, 1};  // forward, right, up rows
    float focal_x = 0.f, focal_y = 0.f;
};
bool project(const Camera &camera, const float point[3], float *x, float *y, float near_plane = 0.3f);

// Serialisation of a best lap for the persist setting: a small header (magic, version, step count) and the poses as float32. read_lap returns false for anything that is not a valid file.
std::string serialize_lap(const Lap &lap);
bool parse_lap(const std::string &bytes, Lap *out);
// A file name for a (track, car): from the start line (origin and heading, rounded to a metre / a few degrees) and the car's wheelbase (rounded to a centimetre): "<16 hex digits>.lap".
std::string lap_file_name(const float origin[3], const float heading[3], float wheelbase);

}  // namespace ghost
