// Where the reset-to-track mod reads its button: the first SDL game controller's Select (BACK) and keyboard keys.
#pragma once

namespace reset {

// Comma-separated SDL key names ("R"). Unknown names are skipped.
void configure_keys(const char *keys);

// Is the button held right now. SDL is read at hook time (SDL_GameControllerGetButton takes the joystick lock, SDL_GetKeyboardState
// is a plain cached array: both are meant to be read while another thread pumps events).
bool button_held();

}  // namespace reset
