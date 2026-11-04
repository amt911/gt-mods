#include "input_source.hpp"

#include <SDL.h>

#include <cstdlib>
#include <optional>
#include <sstream>
#include <vector>

namespace pedals {
namespace {

struct State {
    std::vector<SDL_Scancode> throttle_keys, brake_keys;
    KeyRamp throttle_ramp, brake_ramp;
    SDL_GameController *pad = nullptr;
    unsigned scan_countdown = 0;
    bool fake_read = false;
    std::optional<std::pair<float, float>> fake;
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

// The mod's own handle on the first controller (SDL ref-counts it next to the host's); looked for about once a second
// while none is open, dropped when it is unplugged.
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

} // namespace

void configure_keys(const char *throttle_keys, const char *brake_keys) {
    State &s = state();
    s.throttle_keys = parse_keys(throttle_keys);
    s.brake_keys = parse_keys(brake_keys);
}

PedalInput read_input(const PedalSettings &settings, int elapsed_ms) {
    State &s = state();
    if (!s.fake_read) {
        s.fake_read = true;
        s.fake = parse_fake(std::getenv("GT_MOD_PEDALS_FAKE"));
    }
    if (s.fake) return PedalInput{s.fake->first, s.fake->second};
    refresh_pad(s);
    float pad_throttle = 0.0f, pad_brake = 0.0f;
    if (s.pad) {
        pad_throttle = trigger_to_pedal(SDL_GameControllerGetAxis(s.pad, SDL_CONTROLLER_AXIS_TRIGGERRIGHT), settings);
        pad_brake = trigger_to_pedal(SDL_GameControllerGetAxis(s.pad, SDL_CONTROLLER_AXIS_TRIGGERLEFT), settings);
    }
    float key_throttle = s.throttle_ramp.step(any_held(s.throttle_keys), elapsed_ms, settings.ramp_ms);
    float key_brake = s.brake_ramp.step(any_held(s.brake_keys), elapsed_ms, settings.ramp_ms);
    return PedalInput{combine(pad_throttle, key_throttle), combine(pad_brake, key_brake)};
}

} // namespace pedals
