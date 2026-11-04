// Where the analog-pedals mod gets its pedal values: GT_MOD_PEDALS_FAKE (tests) or SDL (first game controller's
// triggers, plus keyboard keys as pedals).
#pragma once

#include "pedal_input.hpp"

namespace pedals {

struct PedalInput {
    float throttle = 0.0f;
    float brake = 0.0f;
};

// Comma-separated SDL key names ("Up,W"). Unknown names are skipped.
void configure_keys(const char *throttle_keys, const char *brake_keys);

// One reading per physics step. `elapsed_ms` advances the keyboard ramps. GT_MOD_PEDALS_FAKE="t,b" (read once) wins over SDL.
// SDL is read at hook time: SDL_GameControllerGetAxis takes the joystick lock, SDL_GetKeyboardState is a plain cached
// array - both are meant to be read while another thread pumps events.
PedalInput read_input(const PedalSettings &settings, int elapsed_ms);

} // namespace pedals
