#include "input_source.hpp"

#include <SDL.h>

#include <sstream>
#include <string>
#include <vector>

namespace reset {
namespace {

struct State {
    std::vector<SDL_Scancode> keys;
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

}  // namespace

void configure_keys(const char *keys) { state().keys = parse_keys(keys); }

bool button_held() {
    State &s = state();
    const Uint8 *down = SDL_GetKeyboardState(nullptr);
    if (!down) return false;
    for (SDL_Scancode k : s.keys)
        if (down[k]) return true;
    return false;
}

}  // namespace reset
