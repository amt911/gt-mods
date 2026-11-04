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
// Which car a 0x08948B78 call belongs to is proven by call order, not by assumption. The record is a stack buffer of 0x0894DD9C
// shared by all cars (sp_entry - 0x50: frame 0x70, record at +0x20, asm 0x0894DE44), so its address says nothing about the car.
// Per car, mode 0, the step calls controller slot 0xC = 0x08899094, which fetches the driver entry of that car index
// (func_088F8D58, asm 0x088990B8) and calls entry vtable slot 0x64 = 0x08896780(entry, buffer, index) (asm 0x088990D4-0x088990D8,
// a2 = the car index), copies the buffer (0x0892B5F4) and returns; 0x0894DD9C then calls 0x08948B78(record) (asm 0x0894DEC8).
// The 0x08896780 hook notes whether it was called for exactly the human entry/index found in the step hook; the clamp hook injects
// only right after such a fill and consumes it. (Reading the caller's s3 does not work: the decompiled 0x0894DD9C of --decomp on
// does not keep the index in a register.) The record-address check is kept as a second guard.
// Player = driver entry with vtable 0x08CC1F60, byte +0xA8 == 0 (live, not replay) and +0x6C != 0 (reads the pad; see kEntryInput).
#include "gt_mod/gt_mod_api.h"

#include "input_source.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sys/syscall.h>
#include <unistd.h>
#include <string>

namespace {

// --- guest layout (runtime addresses; evidence in gran-turismo-decomp docs/FACTS.md) ---
constexpr std::uint32_t kStep = 0x0894DD9C;         // a0 = world
constexpr std::uint32_t kClampRecord = 0x08948B78;  // a0 = input record
constexpr std::uint32_t kFillRecord = 0x08896780;   // a0 = driver entry, a1 = temp record, a2 = car index (entry vtable slot 0x64)
constexpr std::uint32_t kPlayerVtable = 0x08CC1F60;
constexpr std::uint32_t kEntryReplay = 0xA8;
// The human entry that reads the pad: +0x6C points to the pad input object (the same object, 0x094168D8, in a race and in
// Time Trial). Time Trial also builds a second human-class entry (vtable 0x08CC1F60, +0xA8 == 0) for the ghost/lap recorder,
// with +0x6C == 0, +0xF8 == 0 and +0x08 == 0x00010100 (dumps 2026-11-04, perf_tt_newyork route; the race route's single
// human has +0x6C = 0x094168D8, +0xF8 = 1). So "human" = vtable + not replay + +0x6C != 0.
constexpr std::uint32_t kEntryInput = 0x6C;
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
std::uint32_t g_player_entry = 0, g_player_index = 0;
bool g_fill_was_player = false;  // 0x08896780 just ran for the human entry/index and its record has not been clamped yet
std::uint32_t g_expected_record = 0;  // this step's record address (stack of 0x0894DD9C)
unsigned g_proven_clamps = 0;           // proven clamps (right after a human fill) injected in this step
bool g_warned_record = false, g_logged_thread = false;
std::chrono::steady_clock::time_point g_last_step;
bool g_have_last_step = false;

// Finds the live, non-replay human car(s) of `world`. Exactly one -> its car address, else 0.
std::uint32_t find_player_car(GtModCtx *c, std::uint32_t world, std::uint32_t *first_ai_car, std::uint32_t *replay_car) {
    g_player_entry = 0;
    g_player_index = 0;
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
        if (a.read32(c, entry) == kPlayerVtable && a.read8(c, entry + kEntryReplay) == 0 && a.read32(c, entry + kEntryInput) != 0) {
            ++humans;
            found = car;
            g_player_entry = entry;
            g_player_index = static_cast<std::uint32_t>(i);
        } else if (replay_car && a.read32(c, entry) == kPlayerVtable && a.read8(c, entry + kEntryReplay) != 0) {
            *replay_car = car;  // the human car while a replay plays (log only)
        } else if (first_ai_car && *first_ai_car == 0 && a.read32(c, entry) != kPlayerVtable) {
            *first_ai_car = car;
        }
    }
    if (humans != 1) g_player_entry = 0;
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
    g_proven_clamps = 0;
    g_fill_was_player = false;
    g_ai_car = g_replay_car = 0;
    g_player_car = find_player_car(c, g_api->gpr(c, 4), &g_ai_car, &g_replay_car);
    g_api->call_original(c);
    if (g_log_steps && !g_logged_thread) {
        g_logged_thread = true;
        char line[80];
        std::snprintf(line, sizeof line, "step hook runs on tid %ld", static_cast<long>(::syscall(SYS_gettid)));
        g_api->log(g_self, line);
    }
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

// Writes `record_value` (0..4096) into record field `field` as an analog press. While the controller trigger is pressed its
// value replaces whatever the game put there (the host also turns a trigger past ~25 % into a digital button, which would
// otherwise make every press past that point full); otherwise the larger press wins (a digital press is "full": non-zero
// without the analog flag).
void inject(GtModCtx *c, std::uint32_t record, std::uint32_t field, std::uint8_t flag, int record_value, bool trigger_pressed) {
    const std::uint8_t flags = g_api->read8(c, record);
    const int raw = static_cast<std::int16_t>(g_api->read16(c, record + field));
    const int existing = (flags & flag) != 0 ? raw : (raw != 0 ? 4096 : 0);
    const int value = pedals::resolve_record(record_value, existing, trigger_pressed);
    // The game's own press stays as it is (no trigger, or a trigger below the pedal floor: resolve_record kept `existing`).
    if (value == existing && (!trigger_pressed || record_value <= 0)) return;
    g_api->write8(c, record, value > 0 ? (flags | flag) : flags);
    g_api->write16(c, record + field, static_cast<std::uint16_t>(value));
}

void on_fill_record(GtModCtx *c) {
    if (g_player_car != 0 && g_api->gpr(c, 4) == g_player_entry && g_api->gpr(c, 6) == g_player_index) g_fill_was_player = true;
    g_api->call_original(c);
}

void on_clamp_record(GtModCtx *c) {
    g_api->call_original(c);
    if (g_player_car == 0) return;
    const std::uint32_t record = g_api->gpr(c, 4);
    // Provable player record: the human car's step calls 0x08948B78 once, with the buffer 0x0894DD9C keeps for it.
    const bool proven = g_fill_was_player;
    g_fill_was_player = false;  // one clamp per fill
    if (!proven || ++g_proven_clamps != 1 || record != g_expected_record) {
        if (!g_warned_record) {
            g_warned_record = true;
            char line[160];
            std::snprintf(line, sizeof line, "unexpected 0x08948B78 call (human fill %d, proven clamp #%u, record %08X, expected %08X): not injecting",
                          proven ? 1 : 0, g_proven_clamps, record, g_expected_record);
            g_api->log(g_self, line);
        }
        return;
    }
    inject(c, record, kRecordThrottle, kFlagThrottle, pedals::throttle_record_for_pedal(g_input.throttle), g_input.throttle_trigger);
    inject(c, record, kRecordBrake, kFlagBrake, pedals::brake_record_for_pedal(g_input.brake), g_input.brake_trigger);
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
    if (api->hook(self, kFillRecord, on_fill_record) != 0) return 1;
    if (api->hook(self, kClampRecord, on_clamp_record) != 0) return 1;
    return 0;
}
