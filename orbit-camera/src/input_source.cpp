#include "input_source.hpp"

#include <SDL.h>

#include <sstream>
#include <string>
#include <vector>

namespace orbit {
namespace {

struct State {
    std::vector<SDL_Scancode> left, right, back;
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

bool any_held(const std::vector<SDL_Scancode> &keys) {
    if (keys.empty()) return false;
    const Uint8 *down = SDL_GetKeyboardState(nullptr);
    if (!down) return false;
    for (SDL_Scancode k : keys)
        if (down[k]) return true;
    return false;
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

float axis(SDL_GameController *pad, SDL_GameControllerAxis a) {
    const int v = SDL_GameControllerGetAxis(pad, a);
    return v < 0 ? static_cast<float>(v) / 32768.0f : static_cast<float>(v) / 32767.0f;
}

}  // namespace

void configure_keys(const char *left_keys, const char *right_keys, const char *back_keys) {
    State &s = state();
    s.left = parse_keys(left_keys);
    s.right = parse_keys(right_keys);
    s.back = parse_keys(back_keys);
}

Raw read_raw() {
    State &s = state();
    refresh_pad(s);
    Raw r;
    if (s.pad) {
        r.x = axis(s.pad, SDL_CONTROLLER_AXIS_RIGHTX);
        r.y = axis(s.pad, SDL_CONTROLLER_AXIS_RIGHTY);
        r.r3 = SDL_GameControllerGetButton(s.pad, SDL_CONTROLLER_BUTTON_RIGHTSTICK) != 0;
    }
    r.left = any_held(s.left);
    r.right = any_held(s.right);
    r.back = any_held(s.back);
    return r;
}

}  // namespace orbit
