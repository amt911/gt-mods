// When the rollover mod's rigid body takes a car from the game and when it gives it back (spec 2026-11-18, "Takeover and handback"). Pure: plain values in,
// an event out, so the unit test runs it.
#pragma once

#include "physics_core.hpp"

namespace rollphys {

struct TakeoverParams {
    double hit_dv = 5.0;            // m/s of velocity change in ONE step that counts as a crash (a barrier hit at ~90 km/h gives ~8 and ~15 along; normal driving ~0.2)
    double airborne_w = 2.0;        // rad/s: all four wheels off the ground and spinning faster than this
    int airborne_steps = 5;         // ... for this many steps
    double flipped_up_z = 0.5;      // up.z below this: the car is on its side or roof
    double handback_up_z = 0.85;    // handback: up.z above this,
    int handback_contacts = 3;      // at least this many floor points on the ground,
    double handback_w = 0.5;        // |w| below this (rad/s)
    int handback_steps = 10;        // for this many steps
    double reset_jump = 3.0;        // m: a car that moved this far from where the simulation left it was moved by something else (reset-to-track, a respawn)
    double sim_hit_dv = 1.5;        // m/s: while simulating, the game's own step changing the velocity ALONG THE GROUND by more than this is a barrier hit
    double reset_rows = 0.3;        // the up row differing this much from the written one is a reset
};

enum class Event { None, Takeover, Handback, Reset };
enum class Reason { None, Hit, Airborne, Flipped };

// What the game state says at the start of a step while the game drives the car.
struct GameObs {
    double dv = 0;           // |velocity change since the previous step| (m/s)
    double up_z = 1;
    double w = 0;            // |angular velocity| estimated from the basis change over the last step (rad/s)
    int loaded_wheels = 4;   // wheels with a load (the record's `load` > 0) and ground under them
    bool valid = true;       // false on the first sample of a car / after a teleport: nothing is judged
};

// While the simulation drives.
struct SimObs {
    double up_z = 1;
    double w = 0;
    int bottom_contacts = 0;
    double rows_jump = 0;  // largest difference between the game's up row and the one written last: a car turned by something else (reset-to-track) is not ours any more
    double jump = 0;  // distance between the pose the simulation wrote last and the one the game reports now
};

class Supervisor {
  public:
    explicit Supervisor(const TakeoverParams &p) : p_(p) {}
    // Game drives: Event::Takeover (reason() says why) or None.
    Event update_game(const GameObs &o);
    // Simulation drives: Event::Handback, Event::Reset or None.
    Event update_sim(const SimObs &o);
    bool simulating() const { return sim_; }
    Reason reason() const { return reason_; }
    void reset();  // a new race, no player car: back to the game, nothing counted
    void force_takeover(Reason r) { sim_ = true; reason_ = r; calm_ = 0; air_ = 0; }  // fake hits of the checks, the cheats mod

  private:
    TakeoverParams p_;
    bool sim_ = false;
    Reason reason_ = Reason::None;
    int air_ = 0, calm_ = 0;
};

// The world angular velocity (rad/s) that turns the basis `before` into `after` in `dt` seconds (rows: forward, right, up; as GtCarState::basis).
V3 angular_velocity_from_rows(const float before[9], const float after[9], double dt);

// Where an impact of velocity change `dv` (world, m/s) lands on the hull: the hull surface point in the direction -dv from the centre of mass, at
// `height_fraction` (0 = floor, 1 = roof) of the hull's height when that is a side or end face; for a hit that is mostly vertical, the point under/over the centre.
V3 hit_point(const Hull &h, const Body &b, V3 dv, double height_fraction);

}  // namespace rollphys
