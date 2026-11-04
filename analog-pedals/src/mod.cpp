// analog-pedals: analog throttle and brake for the player's car (gran-turismo-decomp mod SDK, API 1).
//
// Per physics step (0x0894DD9C) the mod reads its input once; then, after 0x08948B78 has clamped the player's 0x20-byte
// input record, it writes the analog values into that record:
//   accelerate: s16 +8,  analog flag 0x02 in byte 0  (-> pedal float car->constants+0x530, then +0x534 effective throttle)
//   brake:      s16 +0xA, analog flag 0x04 in byte 0 (-> pedal float car->constants+0x538)
// (measured: Cross held gives +8 = 1 and +0x530 = 1.0; docs/FACTS.md of the host repo). 0x08952B68, which consumes the record,
// is not hooked: the default recompiled build reaches it by an in-unit goto, so a hook there never fires. The game turns the
// record into pedals with its own response (accelerator curve, 10 % brake floor), so the pedals are not the raw trigger value,
// and a replay shows only the button presses it recorded, not these injected values (measured; FINDINGS in the host repo).
// Player = driver entry with vtable 0x08CC1F60 and byte +0xA8 == 0 (live, not replay).
#include "gt_mod/gt_mod_api.h"

#include "input_source.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {

// --- guest layout (runtime addresses; evidence in gran-turismo-decomp docs/FACTS.md) ---
constexpr std::uint32_t kStep = 0x0894DD9C;         // a0 = world
constexpr std::uint32_t kClampRecord = 0x08948B78;  // a0 = input record
constexpr std::uint32_t kPlayerVtable = 0x08CC1F60;
constexpr std::uint32_t kEntryReplay = 0xA8;
constexpr std::uint32_t kSlotSize = 0x2100, kCarInSlot = 0x2010;
constexpr std::uint32_t kPedalThrottle = 0x530, kPedalBrake = 0x538, kEffectiveThrottle = 0x534;  // floats at word(car+8)+off
constexpr std::uint8_t kFlagThrottle = 0x02, kFlagBrake = 0x04;       // record byte 0
#ifdef ANALOG_PEDALS_SABOTAGE
constexpr std::uint32_t kRecordThrottle = 0xC;  // wrong field on purpose: the in-game check must fail
#else
constexpr std::uint32_t kRecordThrottle = 0x8;
#endif
constexpr std::uint32_t kRecordBrake = 0xA;

const GtModApi *g_api = nullptr;
GtMod *g_self = nullptr;
pedals::PedalSettings g_settings;
bool g_log_steps = false;

std::uint32_t g_player_car = 0;  // car address of this step's human car, 0 if none/ambiguous
pedals::PedalInput g_input;
unsigned g_steps = 0, g_replay_steps = 0;

// Finds the live, non-replay human car(s) of `world`. Exactly one -> its car address, else 0.
std::uint32_t find_player_car(GtModCtx *c, std::uint32_t world, std::uint32_t *first_ai_car, std::uint32_t *replay_car) {
    const GtModApi &a = *g_api;
    const std::uint32_t slots = a.read32(c, world + 4);
    const std::uint32_t controller = a.read32(c, world + 0x10);
    if (slots == 0 || controller == 0) return 0;
    const std::uint32_t list = a.read32(c, controller + 0xC);
    if (list == 0) return 0;
    const int count = static_cast<int>(a.read32(c, world + 8));
    std::uint32_t found = 0;
    int humans = 0;
    for (int i = 0; i < count && i < 32; ++i) {
        const int ix = static_cast<std::int8_t>(a.read8(c, list + 0x24 + i));
        if (ix < 0) continue;
        const std::uint32_t entry = a.read32(c, list + 4 + (2 * i + ix) * 4);
        if (entry == 0) continue;
        const std::uint32_t car = slots + i * kSlotSize + kCarInSlot;
        if (a.read32(c, entry) == kPlayerVtable && a.read8(c, entry + kEntryReplay) == 0) {
            ++humans;
            found = car;
        } else if (replay_car && a.read32(c, entry) == kPlayerVtable) {
            *replay_car = car;  // the human car while a replay plays (log only)
        } else if (first_ai_car && *first_ai_car == 0 && a.read32(c, entry) != kPlayerVtable) {
            *first_ai_car = car;
        }
    }
    return humans == 1 ? found : 0;
}

std::uint32_t g_ai_car = 0, g_replay_car = 0;

void on_step(GtModCtx *c) {
    g_input = pedals::read_input(g_settings, 16);  // one physics step ~ 1/60 s
    g_ai_car = g_replay_car = 0;
    g_player_car = find_player_car(c, g_api->gpr(c, 4), &g_ai_car, &g_replay_car);
    g_api->call_original(c);
    if (g_log_steps && g_replay_car != 0 && (++g_replay_steps % 60) == 0) {
        const std::uint32_t rd = g_api->read32(c, g_replay_car + 8);
        char line[120];
        std::snprintf(line, sizeof line, "replay step=%u thr=%.4f brk=%.4f eff=%.4f", g_replay_steps,
                      g_api->readf(c, rd + kPedalThrottle), g_api->readf(c, rd + kPedalBrake),
                      g_api->readf(c, rd + kEffectiveThrottle));
        g_api->log(g_self, line);
    }
    if (g_log_steps && g_player_car != 0 && (++g_steps % 60) == 0) {
        const std::uint32_t pd = g_api->read32(c, g_player_car + 8);
        char line[200];
        float ai_t = -1, ai_b = -1;
        if (g_ai_car) {
            const std::uint32_t ad = g_api->read32(c, g_ai_car + 8);
            ai_t = g_api->readf(c, ad + kPedalThrottle);
            ai_b = g_api->readf(c, ad + kPedalBrake);
        }
        std::snprintf(line, sizeof line, "step=%u player thr=%.4f brk=%.4f eff=%.4f ai thr=%.4f brk=%.4f", g_steps,
                      g_api->readf(c, pd + kPedalThrottle), g_api->readf(c, pd + kPedalBrake),
                      g_api->readf(c, pd + kEffectiveThrottle), ai_t, ai_b);
        g_api->log(g_self, line);
    }
}

// Writes `value` (0..1) into record field `field` as an analog press, unless the game already holds a bigger one there
// (a digital press is "full": non-zero without the analog flag).
bool inject(GtModCtx *c, std::uint32_t record, std::uint32_t field, std::uint8_t flag, float value) {
    if (value <= 0.0f) return false;
    const int target = std::clamp(static_cast<int>(std::lround(value * 4096.0f)), 0, 4096);
    const std::uint8_t flags = g_api->read8(c, record);
    const int raw = static_cast<std::int16_t>(g_api->read16(c, record + field));
    const int existing = (flags & flag) != 0 ? raw : (raw != 0 ? 4096 : 0);
    if (target <= existing) return false;
    g_api->write8(c, record, flags | flag);
    g_api->write16(c, record + field, static_cast<std::uint16_t>(target));
    return true;
}

void on_clamp_record(GtModCtx *c) {
    g_api->call_original(c);
    if (g_player_car == 0) return;
    const std::uint32_t record = g_api->gpr(c, 4);
    inject(c, record, kRecordThrottle, kFlagThrottle, g_input.throttle);
    inject(c, record, kRecordBrake, kFlagBrake, g_input.brake);
}

}  // namespace

extern "C" __attribute__((visibility("default"))) int gt_mod_init(const GtModApi *api, GtMod *self) {
    if (api == nullptr || api->version != GT_MOD_API_VERSION) return 1;
    g_api = api;
    g_self = self;
    g_settings = pedals::parse_settings([&](const char *k) { return api->config(self, k); },
                                        [&](const std::string &w) { api->log(self, w.c_str()); });
    pedals::configure_keys(api->config(self, "throttle_keys"), api->config(self, "brake_keys"));
    const char *log = std::getenv("GT_MOD_PEDALS_LOG");
    g_log_steps = log != nullptr && *log != '\0' && std::string(log) != "0";
    if (api->hook(self, kStep, on_step) != 0) return 1;
    if (api->hook(self, kClampRecord, on_clamp_record) != 0) return 1;
    return 0;
}
