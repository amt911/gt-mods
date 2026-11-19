// Mechanical damage: the pure model (no game access). A hit adds damage to three parts by where it landed on the body; the damage turns into three
// effects: less engine power (a throttle scale), a pull to one side (a yaw-rate bias) and, when badly hurt, a top-speed cap.
#pragma once

namespace damage {

struct Params {
    float threshold = 6.0f;       // velocity change of one physics step (m/s) below which a hit does nothing (ordinary racing contact is ~1-3)
    float per_ms = 0.05f;         // damage (0..1) per m/s of velocity change above the threshold
    float engine_loss = 0.4f;     // throttle lost at full powertrain damage
    float yaw_bias = 0.06f;       // rad/s of pull at full alignment damage
    float severe_start = 0.6f;    // powertrain damage above which the top speed is capped
    float cap_high = 45.0f;       // m/s: the cap when the damage just passes severe_start
    float cap_low = 25.0f;        // m/s: the cap at full damage
    float scale = 1.0f;           // 0 turns every effect off (the sabotage), 1 = as designed
};

// engine and drive in 0..1; align in -1..1 (the sign is the side the car pulls to: toward the side that was hit).
struct State {
    float engine = 0.0f;
    float drive = 0.0f;
    float align = 0.0f;
};

// Damage (0..1) of one hit from its velocity change; 0 below the threshold.
float hit_strength(float dv, const Params &p);

// `contact` is the unit direction in the body frame (forward, right, up) where the hit landed (GtCarImpact::contact_dir). Front: engine and alignment; rear: drive;
// sides: alignment.
void add_hit(State &s, const float contact[3], float strength);

// Engine and drive wear combined, 0..1: what the throttle scale and the cap read.
float powertrain(const State &s);
// Multiplier for the throttle (1 = undamaged).
float throttle_scale(const State &s, const Params &p);
// Yaw rate to add per second of pull (rad/s, signed), 0 when undamaged.
float yaw_bias(const State &s, const Params &p);
// True and the cap (m/s) when the powertrain is past severe_start.
bool speed_cap(const State &s, const Params &p, float *cap);
// Whole percentages for the menu.
int percent(float v);
void repair(State &s);

}  // namespace damage
