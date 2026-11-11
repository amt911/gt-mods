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
3. **The impulse.** A pre-step hook on `0x0894DD9C` (the player car only, live and in its replay) compares the car's world velocity with the previous step. A change along
   the car's lateral axis (`K+0x28` row) above `threshold` in ONE step is a hard side impact: the car is made to roll toward the side it hit
   (`K+0x6F4`, the roll rate) at `gain` x (change - threshold) rad/s, at most `cap`, held for 40 steps with a linear decay (a one-step kick
   is undone by the springs). Normal driving changes the lateral velocity by ~0.2 m/s per step: nothing happens. A jump of the car (> 3 m in
   a step: reset-to-track, a respawn) or of its velocity (> 60 m/s) is not an impact and drops what was being held.
   The roll is about the car's forward axis, so it only tips the car while that axis is near horizontal (0.3.0): the held rate fades out as
   the nose points up or down (full up to |forward.z| 0.5, none from 0.8) and the drive ends there. Before, a car standing on its nose got the
   roll rate about a vertical axis and spun like a top.
   **Replays (0.3.0).** The game's replay re-simulates the race from the recorded pad stream, so the mod acts in the replay too, from a clean
   state when the replay starts (the same car; entry byte `+0xA8` turns 1): the hit falls on the same step and the car rolls as it did live.
   Before, the mod skipped the replay and the replayed car drove on upright, off the recorded line.

4. **The body box (0.2.0).** The game has no body collision: only the wheels touch the road, so a car on its roof sank 0.5-1.3 m into it
   (measured: the deepest of the box's 8 corners 1.02 m below the ground). The mod puts a box around the player's car (length 0.8 x wheelbase
   clamped to 1.7-2.6 m, width track/2 + 0.12 clamped, floor at -0.03 m and roof at `box_height` above the body origin, centred on the axle midpoint;
   the game has no body dimensions, so these are a heuristic) and pushes it out of the ground: the 4 roof corners always, the 4 floor corners only
   once the car is tilted past ~40 degrees (upright, the wheels own the contact). The ground is the game's own road query result, read from the four
   wheel contact records (`word(car+0x10) + 0x38*i`: `+0x00` x, `+0x08` -y, `+0x0C` ground height; byte `+0x2C` = 1 means no road under that wheel and the
   record is skipped; off-track terrain has a height and is used), a plane fitted while the car is upright and kept while it tumbles. The response is a
   soft sequential impulse per penetrating corner (friction 0.55, restitution only above 2 m/s of approach) plus a position push; **in a step where the
   body touches the ground** the car's speed away from it is capped (`box_kick_cap`), otherwise the roof corners of a spinning car hop it 2.5 m up.
   **Nothing is written unless a corner penetrates**: before the first impact the in-game check compares every state line with a run with
   `body_box = false` (byte-identical). Limits: the origin rises to 1.43 m in the tumble (stock 0.64 m, where the roof simply sinks into the road; a box rolling
   over its roof edge puts the origin at the half-diagonal of its cross-section, 1.17 m); the car then **stays on its roof** until reset-to-track (without the box
   the stock tumble rights itself and the car drives on; with the cap removed the car also rights itself but hops 2.5 m and sinks 0.75 m); the pitch (`+0x6F0`)
   and yaw (`+0x6F8`) rates are read and written with assumed axes and signs (not established, see `docs/FACTS.md` "Rollover body box" in the host repo: removing those two
   writes made the hop 2.57 m). Walls and barriers are not handled (no geometry found for them), nor other cars.

## Settings (`mod.toml`)

| key | default | meaning |
| --- | --- | --- |
| `style` | `"realistic"` | `realistic`: only big impacts roll the car. `arcade`: the threshold is halved and the gain x1.5, so smaller hits roll it |
| `threshold` | `3.0` | lateral velocity change in one step (m/s) above which a hit tips the car (0.1 - 50) |
| `gain` | `2.5` | roll rate (rad/s) per m/s above the threshold (0 - 50) |
| `cap` | `12.0` | largest roll rate (rad/s) the mod asks for (0 - 30) |
| `body_box` | `true` | the body box on or off (`false` = the stock tumble, the roof sinks into the road) |
| `box_height` | `0.8` | the box's roof above the car's origin (m, 0.5 - 3) |
| `box_kick_cap` | `0.5` | largest speed (m/s) away from the ground the car keeps in a step where its body touches it (0.1 - 20) |

An invalid value is logged and the default is used. Reference points (measured on the `rollover` pad route): the barrier at the end of the
straight at ~38 m/s gives ~8.3 m/s of lateral change (rate 12, a tumble past upside down); the car lands, the springs bring it back and
it drives on.

## Known limits

- There is no roof collision in the game: a rolling car keeps its wheels' contact model and always recovers; it does not stay on its roof.
- The clamp is lifted for every car: an AI car that tilts past 35 degrees is no longer held. AI cars get no impulses and none was seen to tilt
  that far in the runs, but this is not proven for every track.
- Only the player gets impulses, and the player is found as exactly one human (driver vtable `0x08CC1F60`, with input live or `+0xA8` set in its replay). With two humans
  (ad hoc / split screen) nobody gets an impulse while the clamp stays lifted.
- The roll is a held roll rate, not a physical simulation of the impact; the 40-step drive, the gain and the cap were tuned on one barrier hit.
- `GT_MOD_ROLLOVER_FAKE=<step>:<dv>` adds `<dv>` m/s of lateral change in the tracker's view at that player-car step (used by `mods_together_check`);
  `GT_MOD_ROLLOVER_LOG=1` logs the car's state (used by `tools/mod_rollover_check.sh`).

## Tests

`ctest --test-dir build` runs `rollover_test` (settings, threshold, cap, style, signs, decay, teleport guards, the forward-axis weight). The in-game check lives in the
decomp repo: `tools/mod_rollover_check.sh` (see `docs/MODS.md` there).
