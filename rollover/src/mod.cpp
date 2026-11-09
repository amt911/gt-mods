// rollover: lets the player's car roll over (gran-turismo-decomp mod SDK, API 1).
//
// Why a stock GT PSP car cannot roll (docs/FACTS.md of the host repo, "rollover"): 0x0893E0FC clamps the car's two tilt angles (K+0x6FC,
// K+0x700) to +-0.611 rad (35 deg) and zeroes their rates unless the word at 0x08CF2F08 is non-zero; 0x0894CCB8 reads that word (its
// setter has no caller in the binary: a dormant switch). The suspension springs also pull a tilted car back, so a car needs (1) the flag
// and (2) a roll impulse to go over.
//
// (1) The flag is not written: the mod hooks the reader 0x0894CCB8 (exactly six callers, all `jal`: a text scan of the whole .text found
// no other call, no address built by a lui/addiu pair and no pointer to it in the data; a computed jump would not show) and returns 1 only
// to the readers inside the body-motion step 0x08956DC8 that the roll needs: the tilt clamp and the Euler angle wrap. Every other reader
// gets the game's real word, 0, including 0x08945F6C although it runs inside that step (its flag branch is inert in the asm unless the flip
// state byte's bit 2 is set, but with `--decomp on` lifting it changed the frames before any hit: hook on_contact_step takes it out of the
// extent), and: the contact-normal reset 0x08956EF4 and the step's ground clamp (they take a different path with the flag set, which changed
// the pixels of a car settling on the grid in the first version, that lifted the flag for everyone) and above all 0x08956930, which with
// the flag set starts a dormant "flipped car" state machine (byte +0x74B bits 1|2, countdown +0x74C = 120). That machine skips the
// contact/force pipeline while the car is on its side: the car free-falls through the road and is pushed back up ~6 m (docs/FACTS.md).
// A mod switched off in the F9 menu never reaches its hooks, so the game reads the real word, 0, again: nothing to restore.
// Residual effect: the clamp is lifted for EVERY car (AI cars included): one that tilts past 35 deg is no longer held. AI cars are not
// given impulses, and a stock AI car never got past the clamp in the runs seen, so no difference was measured; it is not proven for
// every track. With two human players (find_player_car wants exactly one) nobody gets an impulse while the clamp stays lifted.
// (2) A pre-step hook on 0x0894DD9C (a0 = world; same player-car pick as reset-to-track and orbit-camera) watches the player car's
// world velocity between physics steps. K = word(car+8) is Z-up: velocity +0x6CC/6D0/6D4, forward axis = first basis row +0x1C/20/24,
// lateral axis = second row +0x28/2C/30, position +0x10/14/18, body roll rate +0x6F4 (positive tips the car toward the lateral axis). A lateral velocity change in ONE step above `threshold` is a hard side impact: the car
// is made to roll toward the side it hit with a rate `gain` * excess (at most `cap`), held for 40 steps with a linear decay (a one-step
// kick is undone by the springs: a 6 rad/s kick peaked at 0.34 rad). Normal driving changes the lateral velocity by ~0.1-0.2 m/s per
// step: nothing is written. A jump of the car (> 3 m in a step: reset-to-track, a respawn) or of its velocity (> 60 m/s) is not an
// impact and drops what was being held.
//
// GT_MOD_ROLLOVER_FAKE=<step>:<dv> adds <dv> m/s of lateral velocity change in the tracker's view at that player-car step (used by
// mods_together_check); GT_MOD_ROLLOVER_LOG=1 logs the car's state (see log_state).
#include "gt_mod/gt_mod_api.h"

#include "bodybox.hpp"
#include "rollover_logic.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

namespace {

constexpr std::uint32_t kStep = 0x0894DD9C;       // a0 = world
constexpr std::uint32_t kFlagReader = 0x0894CCB8;  // returns the word at 0x08CF2F08
constexpr std::uint32_t kPlayerVtable = 0x08CC1F60;
constexpr std::uint32_t kEntryReplay = 0xA8, kEntryInput = 0x6C;
constexpr std::uint32_t kSlotSize = 0x2100, kCarInSlot = 0x2010;
constexpr std::uint32_t kBody = 8;
constexpr std::uint32_t kPos = 0x10, kForward = 0x1C, kLateral = 0x28, kUp = 0x34, kVel = 0x6CC, kRollRate = 0x6F4, kTiltA = 0x6FC, kTiltB = 0x700;
// Measured (docs/FACTS.md, rollover): a positive +0x6F4 tips the up vector toward the +0x28 row (the lateral axis); +0x1C is the forward axis.
constexpr float kRollSign = 1.0f;
// The flag is lifted only for the readers in the body-motion step 0x08956DC8 (called once per car per physics step; a hook on the clamp
// function 0x0893E0FC itself never fires in `--decomp off`, its caller is in the same recompiled unit). Two mechanisms, see kClampRa. Its readers are the tilt clamp (0x0893E0FC, the one that must see 1), the Euler angle
// wrap (0x08956B20: keeps the angles in +-pi through a full roll) and 0x08945F6C, whose flag branch only does something when the flip
// state byte's bit 2 is set (never, without the flip machine). Every reader outside it gets the game's real word (0): the contact-normal
// reset 0x08956EF4, the step's ground clamp 0x0894E200 and above all the dormant flip state machine of 0x08956930 (it sinks the car
// through the road). ROLLOVER_SABOTAGE=4 lifts the flag inside 0x08956930 too.
constexpr std::uint32_t kBodyStep = 0x08956DC8, kFlipMachine = 0x08956930, kContactStep = 0x08945F6C;
// `--decomp off`: the extent of a hook on 0x08956DC8 does not cover its callees (measured: no reader ever saw it set), but `ra` is the real
// return address there: the clamp's and the wrap's call sites (+ 8), and for sabotage 4 the flip machine's. `--decomp on`: `ra` is a small
// site number shared by several callers; the extent works there.
constexpr std::uint32_t kClampRa = 0x0893E5AC, kWrapRa = 0x08956BF0, kFlipRa = 0x08956958;

const GtModApi *g_api = nullptr;
GtMod *g_self = nullptr;
rollover::Settings g_settings;
std::unique_ptr<rollover::Tracker> g_tracker;
bool g_log = false;
std::uint32_t g_car = 0, g_car_steps = 0, g_impulses = 0, g_since_impulse = 0;
long g_fake_step = -1;
float g_fake_dv = 0.0f;
float g_fake_offset[3] = {0, 0, 0};
std::uint32_t g_ras[32], g_ra_count = 0;
bool g_in_body = false, g_in_flip = false, g_only_step = false;
std::chrono::steady_clock::time_point g_last_step_wall;
constexpr double kWallGapSeconds = 1.0;  // no player-car step for this long (switched off in F9, a pause): the tracker's last sample is stale
unsigned g_body_hits = 0;
float g_prev_vel[3] = {0, 0, 0};
bool g_have_prev_vel = false;

std::uint32_t find_player_car(GtModCtx *c, std::uint32_t world) {
    const GtModApi &a = *g_api;
    const std::uint32_t slots = a.read32(c, world + 4), controller = a.read32(c, world + 0x10);
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
        if (a.read32(c, entry) == kPlayerVtable && a.read8(c, entry + kEntryReplay) == 0 && a.read32(c, entry + kEntryInput) != 0) {
            ++humans;
            found = slots + i * kSlotSize + kCarInSlot;
        }
    }
    return humans == 1 ? found : 0;
}

// `[mod:rollover] st=<player-car step> imp=<impulses so far> since=<steps since the last, -1 none> pos vel up (the body's up vector) tilt=<+0x6FC,+0x700>
// rate=<+0x6F4> flag=<+0x74B>/<+0x74C> held=<steps left of the held roll rate>`
void log_state(GtModCtx *c, std::uint32_t body, float held) {
    const GtModApi &a = *g_api;
    char line[420];
    std::snprintf(line, sizeof line,
                  "st=%u imp=%u since=%d pos=%.2f,%.2f,%.2f vel=%.2f,%.2f,%.2f fwd=%.2f,%.2f,%.2f lat=%.2f,%.2f,%.2f up=%.3f,%.3f,%.3f tilt=%.3f,%.3f rate=%.2f flag=%02X/%d held=%d(%.2f) body_calls=%u",
                  g_car_steps, g_impulses, g_impulses == 0 ? -1 : static_cast<int>(g_since_impulse), a.readf(c, body + kPos), a.readf(c, body + kPos + 4),
                  a.readf(c, body + kPos + 8), a.readf(c, body + kVel), a.readf(c, body + kVel + 4), a.readf(c, body + kVel + 8), a.readf(c, body + kForward),
                  a.readf(c, body + kForward + 4), a.readf(c, body + kForward + 8), a.readf(c, body + kLateral), a.readf(c, body + kLateral + 4), a.readf(c, body + kLateral + 8), a.readf(c, body + kUp), a.readf(c, body + kUp + 4), a.readf(c, body + kUp + 8),
                  a.readf(c, body + kTiltA), a.readf(c, body + kTiltB), a.readf(c, body + kRollRate), a.read8(c, body + 0x74B),
                  static_cast<int>(static_cast<std::int16_t>(a.read16(c, body + 0x74C))), g_tracker->steps_left(), held, g_body_hits);
    a.log(g_self, line);
}

// Dynamic-extent markers (the host bypasses these hooks when the mod is switched off).
void on_body_step(GtModCtx *c) {
    ++g_body_hits;
    g_in_body = true;
    g_api->call_original(c);
    g_in_body = false;
}
void on_contact_step(GtModCtx *c) {  // 0x08945F6C is inside the body step but must read the game's real word (its flag-1 path differs under --decomp on)
    const bool was = g_in_body;
    g_in_body = false;
    g_api->call_original(c);
    g_in_body = was;
}
void on_flip_machine(GtModCtx *c) {  // installed by the sabotage build only
    g_in_flip = true;
    g_api->call_original(c);
    g_in_flip = false;
}

// The flag the game reads: 1 inside the clamp (and the wrap), the game's own word (0) everywhere else.
void on_flag_read(GtModCtx *c) {
    const std::uint32_t ra = g_api->gpr(c, 31);
    g_api->call_original(c);
    const bool lift = ra == kClampRa || ra == kWrapRa || g_in_body || (g_settings.sabotage == 4 && (ra == kFlipRa || g_in_flip));
    if (g_log) {  // one line per distinct caller (ra, `--decomp off` only): which readers there are and which of them got 1
        bool seen = false;
        const std::uint32_t key = ra ^ ((lift ? 1u : 0u));  // the same caller seen lifted and not is logged twice
        for (std::uint32_t i = 0; i < g_ra_count; ++i) seen = seen || g_ras[i] == key;
        if (!seen && g_ra_count < 32) {
            g_ras[g_ra_count++] = key;
            char line[100];
            std::snprintf(line, sizeof line, "FLAGREAD ra=%08X lifted=%d", ra, lift ? 1 : 0);
            g_api->log(g_self, line);
        }
    }
    if (lift) g_api->set_gpr(c, 2, 1);
}

// ---- body box (0.2.0): the car's body against the ground while it tumbles --------------------------------------------------------------
// The game has no body collision: a rolling car's roof sinks 0.5-1.3 m into the road (the wheels are its only contact). A box around the
// body (bodybox.hpp; 4 roof corners always, the 4 floor corners only past ~40 deg of tilt, so an upright car is untouched) is pushed out of
// the ground. The ground comes from the game's own road query results, the four wheel contact records word(car+0x10) + 0x38*i: float +0x00 x,
// +0x08 = -y, +0x0C = ground height there, byte +0x2C != 0 = no road under that wheel (the height is then a fallback and is skipped; off-track terrain, word 0x0300, has byte +0x2C = 0 and a real height). The
// heights stay valid while the car is on its roof. Nothing is written unless a corner penetrates, so the car is byte-identical to the stock
// game until it touches the ground with its body. The box size comes from the same records (wheelbase, track) and the car record
// word(car+0xC) (+0x40/+0x48 = the axles' offsets from the body origin); the game has no body dimensions, so the length, width and the
// floor/roof heights are a heuristic (docs/FACTS.md of the host repo, "rollover body box"). Walls are not handled.
bodybox::Box g_box;
bodybox::Params g_box_params;
bool g_box_sized = false, g_have_rows = false;
bodybox::V3 g_prev_rows[3];
float g_gx = 0.0f, g_gy = 0.0f;    // ground slope fitted while the car is upright, kept while it tumbles (the wheel points are not spread then)
unsigned g_box_steps = 0, g_box_touch_steps = 0, g_box_touch_upright = 0, g_box_tilted_steps = 0;
float g_box_min_clear = 1e9f, g_box_min_roof = 1e9f;

bodybox::V3 rd3(GtModCtx *c, std::uint32_t at) { const GtModApi &a = *g_api; return {a.readf(c, at), a.readf(c, at + 4), a.readf(c, at + 8)}; }

void reset_box_state() {
    g_box_sized = false;
    g_have_rows = false;
    g_gx = g_gy = 0.0f;
    g_box_steps = g_box_touch_steps = g_box_touch_upright = g_box_tilted_steps = 0;
    g_box_min_clear = g_box_min_roof = 1e9f;
}

// The ground plane under the car; false when no wheel has a road under it.
bool read_ground(GtModCtx *c, std::uint32_t car, float up_z, bodybox::Ground *g) {
    const GtModApi &a = *g_api;
    const std::uint32_t cr = a.read32(c, car + 0x10);
    if (cr == 0) return false;
    float xs[4], ys[4], hs[4];
    int n = 0;
    for (int i = 0; i < 4; ++i) {
        const std::uint32_t r = cr + 0x38 * i;
        if (a.read8(c, r + 0x2C) != 0) continue;  // byte +0x2C of the surface word: 1 = no road under the wheel (the low byte is 0 on asphalt and on 0x0300 off-track terrain, which has a ground height too)
        xs[n] = a.readf(c, r);
        ys[n] = -a.readf(c, r + 8);
        hs[n] = a.readf(c, r + 0xC);
        if (std::isfinite(xs[n] + ys[n] + hs[n])) ++n;
    }
    if (n == 0) return false;
    float mx = 0, my = 0, mh = 0;
    for (int i = 0; i < n; ++i) { mx += xs[i]; my += ys[i]; mh += hs[i]; }
    mx /= n; my /= n; mh /= n;
    if (n == 4 && up_z > 0.9f) {  // upright: the four wheel points are spread out: fit the slope
        float sxx = 0, sxy = 0, syy = 0, sxh = 0, syh = 0;
        for (int i = 0; i < 4; ++i) {
            const float dx = xs[i] - mx, dy = ys[i] - my, dh = hs[i] - mh;
            sxx += dx * dx; sxy += dx * dy; syy += dy * dy; sxh += dx * dh; syh += dy * dh;
        }
        const float det = sxx * syy - sxy * sxy;
        if (det > 1e-3f) {
            g_gx = (sxh * syy - syh * sxy) / det;
            g_gy = (syh * sxx - sxh * sxy) / det;
        }
        if (!g_box_sized && up_z > 0.95f) {  // the box size, once, while the car is level
            const bodybox::V3 w0 = rd3(c, cr), w1 = rd3(c, cr + 0x38), w2 = rd3(c, cr + 0x70), w3 = rd3(c, cr + 0xA8);
            const float fx = (w0.x + w1.x) / 2 - (w2.x + w3.x) / 2, fy = (w0.z + w1.z) / 2 - (w2.z + w3.z) / 2;  // records are Y-up: x and z are the ground plane
            const float wheelbase = std::hypot(fx, fy), track = std::hypot(w0.x - w1.x, w0.z - w1.z);
            g_box.hl = std::min(2.6f, std::max(1.7f, 0.8f * wheelbase));
            g_box.hw = std::min(1.1f, std::max(0.75f, track / 2 + 0.12f));
            g_box.cx = 0.0f;
            if (const std::uint32_t spec = a.read32(c, car + 0xC)) {
                const float cx = (a.readf(c, spec + 0x40) + a.readf(c, spec + 0x48)) / 2;  // axle midpoint (0.879 and -1.563 on the logged car)
                if (std::isfinite(cx) && std::fabs(cx) < 1.5f) g_box.cx = cx;
            }
            g_box_sized = true;
            if (g_log) {
                char line[160];
                std::snprintf(line, sizeof line, "BOXSIZE wheelbase=%.2f track=%.2f hl=%.2f hw=%.2f zb=%.2f zt=%.2f cx=%.3f", wheelbase, track, g_box.hl, g_box.hw, g_box.zb, g_box.zt, g_box.cx);
                a.log(g_self, line);
            }
        }
    }
    g->x0 = mx; g->y0 = my; g->h0 = mh; g->gx = g_gx; g->gy = g_gy;
    return true;
}

// One call per physics step, after the roll tracker's write. `apply` false only measures (the log's BOX lines show what the stock game does).
void body_box_step(GtModCtx *c, std::uint32_t car, std::uint32_t body, bool apply) {
    const GtModApi &a = *g_api;
    bodybox::State st;
    st.p = rd3(c, body + kPos);
    st.v = rd3(c, body + kVel);
    st.f = rd3(c, body + kForward); st.l = rd3(c, body + kLateral); st.u = rd3(c, body + kUp);
    // body rates (docs/FACTS.md): +0x6F4 about the forward axis (roll; measured), +0x6F0 / +0x6F8 taken as the lateral / up axes with sign + (assumed, small in a roll tumble: < 1.6 rad/s)
    const float roll = a.readf(c, body + kRollRate), pitch = a.readf(c, body + 0x6F0), yaw = a.readf(c, body + 0x6F8);
    st.w = st.f * roll + st.l * pitch + st.u * yaw;
    bodybox::Ground g;
    if (!read_ground(c, car, st.u.z, &g)) return;
    if (g_settings.sabotage == 7) g.h0 += 0.6f;  // check build: the ground 0.6 m too high, floor corners always active: pushes an upright car
    ++g_box_steps;
    const float upn = bodybox::dot(st.u, g.normal());
    // measurement of the body-rate axes/signs (docs/FACTS.md "rollover body box"): the world angular velocity from the change of the basis rows over
    // the last step, projected on the car's own axes, against the three rates the game keeps
    if (g_log && g_have_rows && upn < 0.9f) {
        const bodybox::V3 cur[3] = {st.f, st.l, st.u};
        bodybox::V3 wm{0, 0, 0};
        for (int i = 0; i < 3; ++i) wm = wm + bodybox::cross(g_prev_rows[i], cur[i] - g_prev_rows[i]) * 0.5f;
        wm = wm * 60.0f;
        char line[220];
        std::snprintf(line, sizeof line, "WMEAS st=%u meas(f,l,u)=%.2f,%.2f,%.2f rates(roll,pitch,yaw)=%.2f,%.2f,%.2f", g_car_steps, bodybox::dot(wm, st.f), bodybox::dot(wm, st.l),
                      bodybox::dot(wm, st.u), roll, pitch, yaw);
        a.log(g_self, line);
    }
    g_prev_rows[0] = st.f; g_prev_rows[1] = st.l; g_prev_rows[2] = st.u; g_have_rows = true;
    const bodybox::State before = st;
    bodybox::Params params = g_box_params;
    if (g_settings.sabotage == 7) params.tilt_active_cos = 2.0f;
    const bodybox::Result res = bodybox::resolve(st, g_box, g, params, apply);
    g_box_min_clear = std::min(g_box_min_clear, res.min_clear);
    if (upn < g_box_params.tilt_active_cos) { ++g_box_tilted_steps; g_box_min_roof = std::min(g_box_min_roof, res.min_clear_roof); }
    if (res.contacts) {
        ++g_box_touch_steps;
        if (upn > 0.9f) ++g_box_touch_upright;
    }
    if (apply && (res.push > 0.0f || res.dv > 0.0f)) {
        a.writef(c, body + kPos, st.p.x); a.writef(c, body + kPos + 4, st.p.y); a.writef(c, body + kPos + 8, st.p.z);
        a.writef(c, body + kVel, st.v.x); a.writef(c, body + kVel + 4, st.v.y); a.writef(c, body + kVel + 8, st.v.z);
        a.writef(c, body + kRollRate, bodybox::dot(st.w, st.f));
        // pitch (+0x6F0) and yaw (+0x6F8) are NOT written back: their mapping to the body axes was not established (docs/FACTS.md, "Rollover body box": the WMEAS fit of
        // a roll-dominated tumble gives R2 0.73 / 0.37 and no clean axis or sign), so a contact's torque about the lateral and up axes is dropped rather than written with a guessed sign
    }
    if (g_log && (upn < 0.9f || res.contacts)) {  // clear = the lowest of the 8 corners above the ground at the start of the step (negative = inside it)
        char line[300];
        std::snprintf(line, sizeof line, "BOX st=%u box=%d upn=%.3f z-g=%.3f clear=%.3f roof=%.3f contacts=%d depth=%.3f push=%.3f dv=%.2f vz=%.2f",
                      g_car_steps, apply ? 1 : 0, upn, before.p.z - g.at(before.p.x, before.p.y), res.min_clear, res.min_clear_roof, res.contacts, res.max_depth, res.push, res.dv, before.v.z);
        a.log(g_self, line);
    }
    if (g_log && g_car_steps % 600 == 0) {
        char line[240];
        std::snprintf(line, sizeof line, "BOXSTAT st=%u steps=%u touch=%u touch_upright=%u tilted=%u min_clear=%.3f min_roof_tilted=%.3f", g_car_steps, g_box_steps, g_box_touch_steps,
                      g_box_touch_upright, g_box_tilted_steps, g_box_min_clear, g_box_min_roof);
        a.log(g_self, line);
    }
}

// Everything that reads the player car. A read outside PSP RAM throws (the host's psprecomp::Error); the world/slot pointers can hold stale
// values once a race is over, and an exception out of a hook would abort the game, so on_step treats it as "no player car" (defensive: the
// `--decomp on` abort seen ~12000 steps after the route is in the game's own func_08BD4270, not here; docs/FINDINGS.md).
void step_body(GtModCtx *c) {
    const GtModApi &a = *g_api;
    const std::uint32_t car = find_player_car(c, a.gpr(c, 4));
    const std::uint32_t body = car != 0 ? a.read32(c, car + kBody) : 0;
    if (car == 0 || body == 0) {
        if (g_car != 0) g_tracker->reset();
        reset_box_state();
        g_car = 0;
        g_car_steps = 0;
        g_have_prev_vel = false;
        return;
    }
    if (car != g_car) {  // a new race: the car's address changed
        g_tracker->reset();
        reset_box_state();
        g_car = car;
        g_car_steps = 0;
        g_impulses = 0;
        g_have_prev_vel = false;
    }
    const auto now = std::chrono::steady_clock::now();
    if (g_last_step_wall.time_since_epoch().count() != 0 && std::chrono::duration<double>(now - g_last_step_wall).count() > kWallGapSeconds) {
        g_tracker->reset();  // toggled back on after an off period (or a long pause): no comparison against the old velocity
        g_have_prev_vel = false;
    }
    g_last_step_wall = now;
    ++g_car_steps;
    ++g_since_impulse;
    float vel[3], right[3], fwd[3], pos[3];
    for (int i = 0; i < 3; ++i) {
        vel[i] = a.readf(c, body + kVel + 4 * i);
        right[i] = a.readf(c, body + kLateral + 4 * i);
        fwd[i] = a.readf(c, body + kForward + 4 * i);
        pos[i] = a.readf(c, body + kPos + 4 * i);
    }
    if (g_fake_step >= 0 && static_cast<long>(g_car_steps) == g_fake_step)  // the tracker sees a velocity jump that stays (a constant offset: no second jump)
        for (int i = 0; i < 3; ++i) g_fake_offset[i] = right[i] * g_fake_dv;
    for (int i = 0; i < 3; ++i) vel[i] += g_fake_offset[i];
    if (g_log && g_have_prev_vel) {  // diagnostics for tuning: every step whose lateral or forward velocity change is noticeable
        float dl = 0.0f, df = 0.0f;
        for (int i = 0; i < 3; ++i) {
            dl += (vel[i] - g_prev_vel[i]) * right[i];
            df += (vel[i] - g_prev_vel[i]) * fwd[i];
        }
        if (std::fabs(dl) > 0.8f || std::fabs(df) > 2.0f) {
            char line[140];
            std::snprintf(line, sizeof line, "DV st=%u lat=%.2f fwd=%.2f", g_car_steps, dl, df);
            a.log(g_self, line);
        }
    }
    for (int i = 0; i < 3; ++i) g_prev_vel[i] = vel[i];
    g_have_prev_vel = true;
    const bool was_idle = g_tracker->steps_left() == 0;
    float held = g_tracker->step(vel, right, pos);
    if (held != 0.0f && was_idle) {  // a hit started a drive
        ++g_impulses;
        g_since_impulse = 0;
        if (g_log) {
            char line[200];  // lat = the lateral axis the car had before this step's physics: the side it then tips toward is read against it
            std::snprintf(line, sizeof line, "IMPULSE st=%u held=%.2f lat=%.3f,%.3f,%.3f", g_car_steps, held, right[0], right[1], right[2]);
            a.log(g_self, line);
        }
    }
    if (g_settings.sabotage == 2 || g_settings.sabotage == 5) held = 0.0f;  // sabotage (the check must fail): the flag without any impulse / stock physics
    if (held != 0.0f) {
        const float rate = a.readf(c, body + kRollRate);
        const float want = rollover::apply_rate(rate, held * kRollSign, g_settings.cap);
        if (want != rate) a.writef(c, body + kRollRate, want);
    }
    if (g_settings.body_box || g_log) body_box_step(c, car, body, g_settings.body_box);  // log-only runs measure what the stock game does
    if (g_log && (g_car_steps % 30 == 0 || (g_impulses != 0 && g_since_impulse < 150 && g_car_steps % 3 == 0))) log_state(c, body, held);
}

void on_step(GtModCtx *c) {
    try {
        step_body(c);
    } catch (...) {  // an unreadable car: treat it as no player car
        g_tracker->reset();
        g_car = 0;
        g_car_steps = 0;
        g_have_prev_vel = false;
    }
    g_api->call_original(c);
}

}  // namespace

extern "C" __attribute__((visibility("default"))) int gt_mod_init(const GtModApi *api, GtMod *self) {
    if (api == nullptr || api->version != GT_MOD_API_VERSION) return 1;
    g_api = api;
    g_self = self;
    g_settings = rollover::parse_settings([&](const char *k) { return api->config(self, k); }, [&](const std::string &w) { api->log(self, w.c_str()); });
    g_tracker = std::make_unique<rollover::Tracker>(g_settings);
    g_box.zt = g_settings.box_height;
    g_box_params.max_up_speed = g_settings.box_kick_cap;
    if (const char *tune = std::getenv("GT_MOD_ROLLOVER_BOX_TUNE")) {  // measurement aid: "soft=0.25,correct=1,e=0.15,mu=0.55,cap=3,skin=0.01,zt=1.15,zb=-0.03"
        std::string t = tune;
        for (std::size_t at = 0; at < t.size();) {
            const std::size_t end = std::min(t.find(',', at), t.size()), eq = t.find('=', at);
            if (eq != std::string::npos && eq < end) {
                const std::string k = t.substr(at, eq - at);
                const float v = std::strtof(t.substr(eq + 1, end - eq - 1).c_str(), nullptr);
                if (k == "soft") g_box_params.soft = v; else if (k == "correct") g_box_params.correct = v; else if (k == "e") g_box_params.restitution = v;
                else if (k == "mu") g_box_params.friction = v; else if (k == "cap") g_box_params.max_up_speed = v; else if (k == "skin") g_box_params.skin = v;
                else if (k == "zt") g_box.zt = v; else if (k == "zb") g_box.zb = v;
            }
            at = end + 1;
        }
    }
    if (g_settings.sabotage == 8) g_box_params.max_up_speed = 20.0f;  // check build: no effective cap
    if (g_settings.sabotage == 6) g_settings.body_box = false;  // check build: the box off, the log still measures
    if (const char *log = std::getenv("GT_MOD_ROLLOVER_LOG")) g_log = *log != '\0' && std::string(log) != "0";
    if (const char *fake = std::getenv("GT_MOD_ROLLOVER_FAKE")) {
        float dv = 0.0f;
        long step = 0;
        if (std::sscanf(fake, "%ld:%f", &step, &dv) == 2 && step >= 0 && std::isfinite(dv)) {
            g_fake_step = step;
            g_fake_dv = dv;
        }
    }
    g_only_step = std::getenv("GT_MOD_ROLLOVER_ONLY_STEP") != nullptr;  // measurement aid (docs/FINDINGS.md): the step hook alone
    if (g_settings.sabotage != 1 && g_settings.sabotage != 5 && !g_only_step) {  // 1: the impulse without the flag; 5: stock physics, log only
        if (api->hook(self, kBodyStep, on_body_step) != 0 || api->hook(self, kContactStep, on_contact_step) != 0 ||
            api->hook(self, kFlagReader, on_flag_read) != 0)
            return 1;
    }
    if (g_settings.sabotage == 4 && api->hook(self, kFlipMachine, on_flip_machine) != 0) return 1;
    return api->hook(self, kStep, on_step) != 0 ? 1 : 0;
}
