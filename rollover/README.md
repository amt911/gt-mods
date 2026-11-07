# rollover

Lets the player's car roll over in Gran Turismo PSP. The stock game cannot: it clamps the car's tilt angles to +-35 degrees and zeroes their
rates. This mod lifts the clamp and, on a **hard side impact**, gives the car a roll. Disabled by default (`enabled = false`); it toggles
live from the F9 menu. Pairs with `reset-to-track` (one button puts a car that ended up on its roof back on the road).

## What it does

1. **The clamp.** `0x0893E0FC` limits the tilt angles (`K+0x6FC`, `K+0x700`) unless the word at `0x08CF2F08` is non-zero; `0x0894CCB8` reads
   that word (its setter has no caller in the binary). The mod hooks that reader and returns 1 while it is active; nothing is written to the
   game, and when the mod is switched off the host stops calling the hook, so the game reads its real word (0) again.
2. **The flip state machine stays asleep.** With the flag set, `0x08956930` also wakes a dormant "overturned car" mode that skips the car's
   force pipeline while it lies on its side: the car then sinks through the road and is pushed back up ~6 m (measured, see the host repo's
   `docs/FACTS.md`, "Rollovers"). The reader hook answers 0 to that one caller (by `ra` in `--decomp off`, and by a hook on `0x08956930` itself,
   which works in `--decomp off` and `on`), so the normal pipeline keeps the car on the road through the roll.
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
- The flag is global: AI cars also lose the clamp (they get no impulses, so nothing changes for them in practice).
- The roll is a held roll rate, not a physical simulation of the impact; the 40-step drive, the gain and the cap were tuned on one barrier hit.
- `GT_MOD_ROLLOVER_FAKE=<step>:<dv>` adds `<dv>` m/s of lateral change in the tracker's view at that player-car step (used by `mods_together_check`);
  `GT_MOD_ROLLOVER_LOG=1` logs the car's state (used by `tools/mod_rollover_check.sh`).

## Tests

`ctest --test-dir build` runs `rollover_test` (settings, threshold, cap, style, signs, decay, teleport guards). The in-game check lives in the
decomp repo: `tools/mod_rollover_check.sh` (see `docs/MODS.md` there).
