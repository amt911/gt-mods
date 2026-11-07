// Where the reset-to-track mod reads its button: keyboard keys (the pad's Select comes from the game's own pad buffer, see mod.cpp).
#pragma once

namespace reset {

// Comma-separated SDL key names ("R"). Unknown names are skipped.
void configure_keys(const char *keys);

// Is a configured key held right now (SDL_GetKeyboardState is a plain cached array, meant to be read while another thread pumps events).
bool button_held();

}  // namespace reset
