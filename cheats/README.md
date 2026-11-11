# cheats

Race-time cheats for Gran Turismo PSP, as rows of the F9 mod menu (keyboard or pad): **slow motion** (0.5x / 0.25x, the player's car and the AI cars)
and **freeze AI** (AI cars skip their physics step). Opt-in twice: the mod ships `enabled = false`, and with it on both cheats start off.
Needs mod API v2 (`api = 2`: `add_menu_row`); the host repo's `docs/MODS.md` has the API and the pad menu (chord SELECT+START).

## Use

F9 (or the pad chord SELECT+START) opens the MODS page: after the mods' own rows come `Slow motion  OFF|0.5x|0.25x` and `Freeze AI  OFF|ON`.
`AI speed  1x|0.5x|0.75x|1.25x` (AI cars' step at that fraction of the clock; above 1 it is extrapolated, up to 1.5x),
`Nitro  ON|OFF` (with the throttle past 90 %, +6 m/s² along the car's heading), `Moon gravity  ON|OFF` (only 30 % of the speed lost
falling along the car's up axis is applied), `Jump  ENTER` (+6 m/s along the car's up axis, once).
Measured on the `cheats` route, steps 860-1400 (host repo `--decomp off`): nitro 503 m vs 204 m and top speed 92 vs 34 m/s; AI
0.5x 118.6 m vs 256.3 m; AI 1.25x 329.8 m; jump: the car rises 1.3 m more than without it; jump with moon gravity: 4.5 m.

`Money  <balance> Cr +-1M`: Enter or Right adds 1,000,000 Cr, Left takes 1,000,000 away (between 0 and the game's cap,
999,999,999). The first change of a run copies the save folder to `savedata-backups/UCES01245-GAMEDAT-<time>` next to the
savedata folder (with no save yet there is nothing to copy). The new balance is kept once the game saves (Options, or any of its
own saves): measured in game, a +1M then the Options save then a restart showed the new balance (host repo FACTS
"Money and cars"). The row shows `--` when no game is loaded and refuses to change anything if the backup fails.
Right / Enter step forward, Left back (pad: D-pad right / cross, D-pad left). Both act only in a race (exactly one human car);
outside one nothing is touched. In the race's replay the menu is not read: see Replays below. Turning the mod off in F9 stops everything at once (the host passes the hook straight to the game).

## How

`0x0894DD9C(world)` steps every car once per physics step. With `--decomp off` the recompiled unit runs the body of that function **after** the hook
returns, so a hook cannot read "the state after the step" there. The mod works on the next call instead: at its entry each car's state is the previous
step's full result, and the mod writes back `base + f * (state - base)` over the kinematic fields (position, basis rows, speed, velocity, angular rates,
Euler angles), where `base` is what it left after the previous call. Explicit Euler is linear in dt, so this is the same physics at a fraction of the clock:
velocities change `f` times as slowly as well (a plain velocity multiplier would brake the car instead). `f` = 0.5 / 0.25 for slow motion, 0 for an AI
car while AI is frozen (it keeps its speed and goes on from it when you turn the cheat off). With `f = 1` nothing is written: the mod on and idle gives
frames byte-identical to no mod (the in-game check proves it). The Euler angles go the short way round their +-pi wrap, the basis rows are
re-orthonormalised, a car that moved more than 8 m between two calls (reset-to-track, a respawn) is left alone, and the bases restart after 8 vblanks
without a call (the game's own count, `vblank_index`).

Why not skip whole steps (every other world step does not run): the game steps at ~30 Hz, so 0.25x would draw 7 steps per second. Why not scale a
time step: the world step takes no dt (its caller passes one, `0x0894DD9C` ignores it: fixed step).

## Replays (0.4.0)

The game's replay re-simulates the race from the recorded pad stream, so a cheat that changes physics must act there exactly as it did live, or
the replayed cars leave the recorded line. The live race records the controls in force at each physics step (slow motion, freeze AI, AI speed, nitro,
moon gravity, a jump on that step: one byte a step), counted from the step at which the player's car first moved; the replay plays them back from the
same moment and ignores the menu. (The replay counts its steps from another point of the start countdown, -14 steps measured, so the first movement is
the shared reference.) Needs a host whose `race_cars` lists the replayed player car (role 3); with an older host the replay runs without cheats, as
before. Checked in game by `tools/mod_cheats_check.sh --replay` in the host repo.

## Limits

- **Not slowed:** the game's clocks (race timer, lap counter, countdown), the AI drivers' own state (a frozen AI car's driver keeps running against a car
  that does not move), tyre and engine state outside the body (wheel spin, rpm), sound and camera. Collisions during slow motion resolve over more steps.
- The picture drawn between two calls shows each car one full step ahead of the mod's base: a constant lead, so the motion on screen is `f` times as fast.
- A Time Trial ghost may be treated like an AI car (slowed or frozen); untested, as are
   two human players (the mod then does nothing). A cheat changed during the start countdown (before the player's car moves) is replayed with
   the controls of the first moving step.

## Settings (`mod.toml`)

| key | default | meaning |
| --- | --- | --- |
| `slow_motion` | `"off"` | initial state: `off`, `0.5` or `0.25` |
| `freeze_ai` | `false` | initial state |

Debugging aids used by the in-game check (`tools/mod_cheats_check.sh` in the host repo): `GT_MOD_CHEATS_SLOWMO=<off|0.5|0.25>`, `GT_MOD_CHEATS_FREEZE_AI=1`,
`GT_MOD_CHEATS_FROM=<player-car step>`, `GT_MOD_CHEATS_LOG=1` (+ `GT_MOD_CHEATS_LOG_EVERY=<steps>`): `[mod:cheats] st=<step> slow=.. freeze=.. P=<player pos> spd acc A<i>=<AI pos>`.
