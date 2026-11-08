# rollover

Lets the player's car roll over in Gran Turismo PSP. The stock game cannot: it clamps the car's tilt angles to +-35 degrees and zeroes their
rates. This mod lifts the clamp and, on a **hard side impact**, gives the car a roll. Disabled by default (`enabled = false`); it toggles
live from the F9 menu. Pairs with `reset-to-track` (one button puts a car that ended up on its roof back on the road).

## What it does

1. **The clamp.** `0x0893E0FC` limits the tilt angles (`K+0x6FC`, `K+0x700`) unless the word at `0x08CF2F08` is non-zero; `0x0894CCB8` reads
   that word (its setter has no caller in the binary). The mod hooks that reader; nothing is written to the game, and when the mod is
   switched off the host stops calling the hook, so the game reads its real word (0) again.
2. **The flag is lifted only where the roll needs it.** The reader has six callers, all `jal`. The mod returns 1 only to the readers inside
   the body-motion step `0x08956DC8` that the roll needs (the tilt clamp and the Euler angle wrap) and the real word (0) to the others: the
   contact-normal reset `0x08956EF4`, `0x08945F6C` (inside that step, but lifting it changes frames under `--decomp on`), the step's ground clamp
   and, above all, `0x08956930`. With the flag set that
   one wakes a dormant "overturned car" state machine that skips the car's force pipeline while it lies on its side: the car sinks through
   the road and is pushed back up ~6 m (measured, host repo `docs/FACTS.md`, "Rollovers"). A first version lifted the flag for every reader and
   changed a few pixels of a car settling on the grid; with the narrowed flag a run with the mod on and no hit is frame-identical to a run
   without it (the in-game check proves it, `--decomp off` and `on`).
3. **The impulse.** A pre-step hook on `0x0894DD9C` (the player car only, never a replay) compares the car's world velocity with the previous step. A change along
   the car's lateral axis (`K+0x28` row) above `threshold` in ONE step is a hard side impact: the car is made to roll toward the side it hit
   (`K+0x6F4`, the roll rate) at `gain` x (change - threshold) rad/s, at most `cap`, held for 40 steps with a linear decay (a one-step kick
   is undone by the springs). Normal driving changes the lateral velocity by ~0.2 m/s per step: nothing happens. A jump of the car (> 3 m in
   a step: reset-to-track, a respawn) or of its velocity (> 60 m/s) is not an impact and drops what was being held.

## Settings (`mod.toml`)

| key | default | meaning |
| --- | --- | --- |
| `style` | `"realistic"` | `realistic`: only big impacts roll the car. `arcade`: the threshold is halved and the gain x1.5, so smaller hits roll it |
| `threshold` | `3.0` | lateral velocity change in one step (m/s) above which a hit tips the car (0.1 - 50) |
| `gain` | `2.5` | roll rate (rad/s) per m/s above the threshold (0 - 50) |
| `cap` | `12.0` | largest roll rate (rad/s) the mod asks for (0 - 30) |

An invalid value is logged and the default is used. Reference points (measured on the `rollover` pad route): the barrier at the end of the
straight at ~38 m/s gives ~8.3 m/s of lateral change (rate 12, a tumble past upside down); the car lands, the springs bring it back and
it drives on.

## Known limits

- There is no roof collision in the game: a rolling car keeps its wheels' contact model and always recovers; it does not stay on its roof.
- The clamp is lifted for every car: an AI car that tilts past 35 degrees is no longer held. AI cars get no impulses and none was seen to tilt
  that far in the runs, but this is not proven for every track.
- Only the player gets impulses, and the player is found as exactly one human (driver vtable `0x08CC1F60`, not a replay, with input). With two humans
  (ad hoc / split screen) nobody gets an impulse while the clamp stays lifted.
- The roll is a held roll rate, not a physical simulation of the impact; the 40-step drive, the gain and the cap were tuned on one barrier hit.
- `GT_MOD_ROLLOVER_FAKE=<step>:<dv>` adds `<dv>` m/s of lateral change in the tracker's view at that player-car step (used by `mods_together_check`);
  `GT_MOD_ROLLOVER_LOG=1` logs the car's state (used by `tools/mod_rollover_check.sh`).

## Tests

`ctest --test-dir build` runs `rollover_test` (settings, threshold, cap, style, signs, decay, teleport guards). The in-game check lives in the
decomp repo: `tools/mod_rollover_check.sh` (see `docs/MODS.md` there).
