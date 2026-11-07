#include "input_source.hpp"

#include <SDL.h>

#include <sstream>
#include <string>
#include <vector>

namespace reset {
namespace {

struct State {
    std::vector<SDL_Scancode> keys;
    SDL_GameController *pad = nullptr;
    unsigned scan_countdown = 0;
};

State &state() {
    static State s;
    return s;
}

std::vector<SDL_Scancode> parse_keys(const char *list) {
    std::vector<SDL_Scancode> out;
    if (!list) return out;
    std::stringstream ss(list);
    std::string name;
    while (std::getline(ss, name, ',')) {
        while (!name.empty() && name.front() == ' ') name.erase(name.begin());
        while (!name.empty() && name.back() == ' ') name.pop_back();
        SDL_Scancode code = SDL_GetScancodeFromName(name.c_str());
        if (code != SDL_SCANCODE_UNKNOWN) out.push_back(code);
    }
    return out;
}

// The mod's own handle on the first controller (SDL ref-counts it next to the host's); looked for about once a second while
// none is open, dropped when it is unplugged.
void refresh_pad(State &s) {
    if (s.pad && !SDL_GameControllerGetAttached(s.pad)) {
        SDL_GameControllerClose(s.pad);
        s.pad = nullptr;
    }
    if (s.pad) return;
    if (s.scan_countdown > 0) {
        --s.scan_countdown;
        return;
    }
    s.scan_countdown = 60;
    if (!SDL_WasInit(SDL_INIT_GAMECONTROLLER)) return;
    for (int i = 0, n = SDL_NumJoysticks(); i < n; ++i)
        if (SDL_IsGameController(i) && (s.pad = SDL_GameControllerOpen(i))) return;
}

}  // namespace

void configure_keys(const char *keys) { state().keys = parse_keys(keys); }

bool button_held() {
    State &s = state();
    refresh_pad(s);
    if (s.pad && SDL_GameControllerGetButton(s.pad, SDL_CONTROLLER_BUTTON_BACK) != 0) return true;
    const Uint8 *down = SDL_GetKeyboardState(nullptr);
    if (!down) return false;
    for (SDL_Scancode k : s.keys)
        if (down[k]) return true;
    return false;
}

}  // namespace reset
