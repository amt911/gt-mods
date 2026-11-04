// analog-pedals: analog throttle and brake for the player's car (gran-turismo-decomp mod SDK, API 1).
//
// Per physics step (0x0894DD9C) the mod reads its input once; then, after 0x08948B78 has clamped the player's 0x20-byte
// input record, it writes the analog values into that record:
//   accelerate: s16 +8,  analog flag 0x02 in byte 0  (-> pedal float car->constants+0x530, then +0x534 effective throttle)
//   brake:      s16 +0xA, analog flag 0x04 in byte 0 (-> pedal float car->constants+0x538)
// (measured: Cross held gives +8 = 1 and +0x530 = 1.0; docs/FACTS.md of the host repo). 0x08952B68, which consumes the record,
// is not hooked: the default recompiled build reaches it by an in-unit goto, so a hook there never fires. The game answers the
// record with its own response (accelerator x^8, then a 10 % floor for both pedals; docs/FACTS.md), so the mod writes the record
// value whose response is the requested pedal (pedal_input.hpp). A pedal below 0.05 is not injected; 0.05-0.1 gives the game's
// smallest press (0.1). A replay shows only the button presses it recorded, not these injected values (FINDINGS).
// The record is a stack buffer of 0x0894DD9C at sp_entry - 0x50 (frame 0x70, record at +0x20: asm 0x0894DD9C/0x0894DE44); the
// clamp hook injects only into that exact address and only on the first call of a step.
// Player = driver entry with vtable 0x08CC1F60 and byte +0xA8 == 0 (live, not replay).
#include "gt_mod/gt_mod_api.h"

#include "input_source.hpp"

#include <algorithm>
#include <chrono>
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
constexpr std::uint32_t kRecordFromEntrySp = 0x50;  // record = sp at entry of 0x0894DD9C minus this
// Sabotage builds (-DANALOG_PEDALS_SABOTAGE=1 throttle / =2 brake) write the field to +0xC: the in-game check must fail.
#if defined(ANALOG_PEDALS_SABOTAGE) && ANALOG_PEDALS_SABOTAGE == 1
constexpr std::uint32_t kRecordThrottle = 0xC, kRecordBrake = 0xA;
#elif defined(ANALOG_PEDALS_SABOTAGE) && ANALOG_PEDALS_SABOTAGE == 2
constexpr std::uint32_t kRecordThrottle = 0x8, kRecordBrake = 0xC;
#else
constexpr std::uint32_t kRecordThrottle = 0x8, kRecordBrake = 0xA;
#endif

const GtModApi *g_api = nullptr;
GtMod *g_self = nullptr;
pedals::PedalSettings g_settings;
bool g_log_steps = false;

std::uint32_t g_player_car = 0;  // car address of this step's human car, 0 if none/ambiguous
pedals::PedalInput g_input;
unsigned g_steps = 0, g_replay_steps = 0;
std::uint32_t g_expected_record = 0;  // this step's record address (stack of 0x0894DD9C)
unsigned g_clamp_calls = 0;           // calls of 0x08948B78 in this step
bool g_warned_record = false;
std::chrono::steady_clock::time_point g_last_step;
bool g_have_last_step = false;

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
    const auto now = std::chrono::steady_clock::now();
    int elapsed_ms = 16;  // the first step
    if (g_have_last_step)
        elapsed_ms = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(now - g_last_step).count());
    g_last_step = now;
    g_have_last_step = true;
    g_input = pedals::read_input(g_settings, std::min(elapsed_ms, 50));
    g_expected_record = g_api->gpr(c, 29) - kRecordFromEntrySp;
    g_clamp_calls = 0;
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

// Writes `record_value` (0..4096) into record field `field` as an analog press, unless the game already holds a bigger one
// there (a digital press is "full": non-zero without the analog flag).
void inject(GtModCtx *c, std::uint32_t record, std::uint32_t field, std::uint8_t flag, int record_value) {
    if (record_value <= 0) return;
    const std::uint8_t flags = g_api->read8(c, record);
    const int raw = static_cast<std::int16_t>(g_api->read16(c, record + field));
    const int existing = (flags & flag) != 0 ? raw : (raw != 0 ? 4096 : 0);
    if (record_value <= existing) return;
    g_api->write8(c, record, flags | flag);
    g_api->write16(c, record + field, static_cast<std::uint16_t>(record_value));
}

void on_clamp_record(GtModCtx *c) {
    g_api->call_original(c);
    if (g_player_car == 0) return;
    const std::uint32_t record = g_api->gpr(c, 4);
    // Provable player record: the human car's step calls 0x08948B78 once, with the buffer 0x0894DD9C keeps for it.
    if (++g_clamp_calls != 1 || record != g_expected_record) {
        if (!g_warned_record) {
            g_warned_record = true;
            char line[160];
            std::snprintf(line, sizeof line, "unexpected 0x08948B78 call (#%u, record %08X, expected %08X): not injecting",
                          g_clamp_calls, record, g_expected_record);
            g_api->log(g_self, line);
        }
        return;
    }
    inject(c, record, kRecordThrottle, kFlagThrottle, pedals::throttle_record_for_pedal(g_input.throttle));
    inject(c, record, kRecordBrake, kFlagBrake, pedals::brake_record_for_pedal(g_input.brake));
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
