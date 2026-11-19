// The "realistic" style of the rollover mod (0.5.0): a rigid-body simulation takes the player's car while it tumbles (spec 2026-11-18 of the host repo).
// Everything the game gives goes through the named SDK (API 7): car_state, car_wheel_contacts, car_dimensions, car_set_pose, tilt_free.
#pragma once

#include "gt_mod/gt_mod_api.h"
#include "rollover_logic.hpp"

namespace realistic {

// Registers the world-step hook (gt_mod_init only). 0 = ok. (A pose write at the car-draw hook changes nothing on screen: host FACTS "Rollover physics research".)
int init(const GtModApi *api, GtMod *self, const rollover::Settings &settings, bool log);

}  // namespace realistic
