// Where the orbit-camera mod gets its look input: the first SDL game controller's right stick and R3, plus keyboard keys.
#pragma once

#include "orbit_input.hpp"

namespace orbit {

// Comma-separated SDL key names ("J"). Unknown names are skipped.
void configure_keys(const char *left_keys, const char *right_keys, const char *back_keys);

// One reading. SDL is read at hook time (SDL_GameControllerGetAxis takes the joystick lock, SDL_GetKeyboardState is a plain
// cached array: both are meant to be read while another thread pumps events).
Raw read_raw();

}  // namespace orbit
