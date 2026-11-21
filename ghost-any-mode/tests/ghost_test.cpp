#include "ghost_logic.hpp"
#include <cstring>

#include <cassert>
#include <cmath>

using namespace ghost;

static bool near(float a, float b, float eps = 1e-4f) { return std::fabs(a - b) <= eps; }

static Pose pose_at(float x, float y, float z = 0.f) {
    Pose p;
    p.position[0] = x;
    p.position[1] = y;
    p.position[2] = z;
    return p;
}

int main() {
    // lap line: set where the car first moves (>= 1.5 m/s), crossed forward after being away, once per lap, not by a pass beside the track or a short wiggle
    {
        LapLine line(60.f, 45.f);
        const float still[3] = {0, 0, 0};
        float pos[3] = {10, 20, 0}, vel[3] = {0, 0, 0};
        assert(!line.step(pos, vel) && !line.has_line());  // standing: no line
        vel[0] = -30;                                       // heading -x
        assert(!line.step(pos, vel) && line.has_line() && near(line.heading()[0], -1.f) && near(line.origin()[0], 10.f));
        // drives away (-x) 100 m, loops, comes back from the -x side? the line is crossed going -x again: from s < 0? s = dot(d, heading): ahead is positive
        // so a lap ends when the car, behind the line (s < 0: it came from +x, behind the origin), crosses it moving ahead; start behind:
        pos[0] = 15; pos[1] = 20;                           // 5 m behind the line (s = -5)
        assert(!line.step(pos, vel));
        pos[0] = -200; pos[1] = 20;                         // far ahead: armed (distance >= 60)
        assert(!line.step(pos, vel));
        pos[0] = 40; pos[1] = 20;                           // back behind the line after the lap (s < 0)
        assert(!line.step(pos, vel));
        pos[0] = 9; pos[1] = 20;                            // crossing: s goes from -30 to +1
        assert(line.step(pos, vel));
        pos[0] = 8; pos[1] = 20;
        assert(!line.step(pos, vel));                       // no second count right after
        // not armed again until it has driven away
        pos[0] = 40; pos[1] = 20;
        assert(!line.step(pos, vel));
        pos[0] = 9; pos[1] = 20;
        assert(!line.step(pos, vel));
        (void)still;
    }
    {
        LapLine line(60.f, 45.f);
        float pos[3] = {0, 0, 0}, vel[3] = {0, 20, 0};  // heading +y
        line.step(pos, vel);
        pos[1] = -10; line.step(pos, vel);
        pos[1] = 120; line.step(pos, vel);              // armed
        pos[1] = -5; pos[0] = 200;                      // back behind the line but 200 m to the side
        line.step(pos, vel);
        pos[1] = 5;                                      // crosses far to the side: not the line
        assert(!line.step(pos, vel));
        pos[0] = 3; pos[1] = -5; line.step(pos, vel);
        pos[1] = 4;
        assert(line.step(pos, vel));                     // near the line: counted
        const float bad[3] = {NAN, 0, 0};
        assert(!line.step(bad, vel));
        line.reset();
        assert(!line.has_line());
    }
    // recorder: keeps the best lap by steps, ignores laps that are too short, starts each lap empty
    {
        Recorder rec;
        for (int i = 0; i < 400; ++i) rec.push(pose_at(float(i), 0));
        assert(rec.current_steps() == 400 && !rec.has_best());
        assert(rec.lap_done() && rec.has_best() && rec.best().steps() == 400 && rec.laps() == 1 && rec.current_steps() == 0);
        for (int i = 0; i < 500; ++i) rec.push(pose_at(float(i), 1));
        assert(!rec.lap_done() && rec.best().steps() == 400 && rec.best().poses[10].position[1] == 0.f && rec.laps() == 2);  // slower: not kept
        for (int i = 0; i < 350; ++i) rec.push(pose_at(float(i), 2));
        assert(rec.lap_done() && rec.best().steps() == 350 && rec.best().poses[10].position[1] == 2.f);                      // faster: kept
        for (int i = 0; i < 100; ++i) rec.push(pose_at(float(i), 3));
        assert(!rec.lap_done() && rec.best().steps() == 350 && rec.laps() == 3);                                                // too short: ignored
        rec.clear_all();
        assert(!rec.has_best() && rec.laps() == 0);
    }
    // ghost pose: the recorded one, clamped after the end; none without a best lap
    {
        Lap lap;
        for (int i = 0; i < 10; ++i) lap.poses.push_back(pose_at(float(i), 0));
        Pose p;
        assert(ghost_pose(lap, 3, &p) && near(p.position[0], 3.f) && ghost_pose(lap, 99, &p) && near(p.position[0], 9.f) && !ghost_pose(Lap(), 0, &p));
        const Pose mid = lerp(pose_at(0, 0), pose_at(10, 20), 0.25f);
        assert(near(mid.position[0], 2.5f) && near(mid.position[1], 5.f) && near(lerp(pose_at(0, 0), pose_at(1, 1), 5.f).position[0], 1.f));
        Pose a, b;                                     // basis: halfway between two headings stays a unit row
        b.basis[0] = 1; b.basis[1] = 0; b.basis[3] = 0; b.basis[4] = -1;
        const Pose m = lerp(a, b, 0.5f);
        assert(near(std::sqrt(m.basis[0] * m.basis[0] + m.basis[1] * m.basis[1] + m.basis[2] * m.basis[2]), 1.f));
    }
    // box: 8 corners around the pose, the right size, edges join corners that differ in one axis
    {
        Pose p = pose_at(10, 20, 1);
        Box box;
        box.length = 4.f; box.width = 2.f; box.height = 1.f; box.lift = 0.5f;
        float c[8][3];
        box_corners(p, box, c);
        assert(near(c[0][0], 9.f) && near(c[0][1], 22.f) && near(c[0][2], 1.f));    // front-left (forward +y, right +x): x - 1, y + 2, z: 1 + 0.5 - 0.5
        assert(near(c[1][0], 11.f) && near(c[2][1], 18.f) && near(c[4][2], 2.f));
        for (const auto &e : kBoxEdges) {
            int differing = 0;
            for (int k = 0; k < 3; ++k) differing += !near(c[e[0]][k], c[e[1]][k]);
            assert(differing == 1);
        }
        Pose yawed;  // forward +x, right -y (left-handed: forward x right . up = -1)
        yawed.basis[0] = 1; yawed.basis[1] = 0; yawed.basis[3] = 0; yawed.basis[4] = -1;
        box_corners(yawed, box, c);
        assert(near(c[0][0], 2.f) && near(c[0][1], 1.f));                           // front (+x 2), left (+y 1)
    }
    // projection: a point straight ahead lands in the centre, offsets move it by focal * x / depth, behind the camera is refused
    {
        Camera cam;
        cam.eye[1] = -10.f;      // looking along +y
        cam.focal_x = 300.f; cam.focal_y = 280.f;
        float x, y;
        const float ahead[3] = {0, 10, 0};
        assert(project(cam, ahead, &x, &y) && near(x, 240.f) && near(y, 136.f));
        const float right[3] = {2, 10, 0}, up[3] = {0, 10, 1};
        assert(project(cam, right, &x, &y) && near(x, 240.f + 300.f * 2.f / 20.f) && near(y, 136.f));
        assert(project(cam, up, &x, &y) && near(y, 136.f - 280.f * 1.f / 20.f));
        const float behind[3] = {0, -20, 0}, close[3] = {0, -9.9f, 0};
        assert(!project(cam, behind, &x, &y) && !project(cam, close, &x, &y));
    }
    // persistence: the round trip is exact, anything else is refused
    {
        Lap lap;
        for (int i = 0; i < 5; ++i) lap.poses.push_back(pose_at(float(i), float(i) * 2.f, 1.f));
        const std::string bytes = serialize_lap(lap);
        Lap back;
        assert(parse_lap(bytes, &back) && back.steps() == 5 && near(back.poses[3].position[1], 6.f) && back.poses[4].basis[1] == 1.f);
        assert(!parse_lap("", &back) && !parse_lap(bytes.substr(0, bytes.size() - 1), &back) && !parse_lap("XXXX" + bytes.substr(4), &back));
        std::string wrong = bytes;
        wrong[4] = 2;  // another version
        assert(!parse_lap(wrong, &back));
        std::string nan = bytes;
        const float n = NAN;
        std::memcpy(&nan[12], &n, 4);
        assert(!parse_lap(nan, &back));
        const float o1[3] = {1, 2, 3}, o2[3] = {1.2f, 2, 3}, o3[3] = {50, 2, 3}, h[3] = {0, 1, 0}, h2[3] = {1, 0, 0};
        assert(lap_file_name(o1, h, 2.5f) == lap_file_name(o2, h, 2.5f));  // the same start line (rounded to a metre), the same file
        assert(lap_file_name(o1, h, 2.5f) != lap_file_name(o3, h, 2.5f) && lap_file_name(o1, h, 2.5f) != lap_file_name(o1, h2, 2.5f) && lap_file_name(o1, h, 2.5f) != lap_file_name(o1, h, 2.6f));
        assert(lap_file_name(o1, h, 2.5f).size() == 20);
    }
    return 0;
}
