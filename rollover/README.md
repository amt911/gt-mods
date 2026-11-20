# rollover

Lets the player's car roll over in Gran Turismo PSP, and bounce and slide like a body when it does. The stock game cannot roll: it clamps the car's tilt angles to +-35 degrees,
zeroes their rates, and keeps applying its wheel and suspension model whatever the car's attitude. Disabled by default (`enabled = false`); it toggles live from the F9 menu. Pairs with
`reset-to-track` (one button puts a car that ended up on its roof back on the road). Needs host API 7 (`api = 7`).

## Two styles (`style` in `mod.toml`)

- **`realistic` (default, 0.5.0).** A rigid-body simulation (`src/physics_core.*`, no game dependency, unit-tested) takes the player's car when it is hit hard, flipped, or
  airborne and spinning, and hands it back once it is on its wheels again. While it drives the car it has mass, inertia, gravity, impulse contacts of the hull's corners and
  edges with the road (restitution 0.15, Coulomb friction 0.7 on asphalt, 5/7 of it off the track), damping. A car dropped on its roof loses most of its vertical speed,
  slides, and may roll over a corner; it does not hop or spin like a top. If it ends on its roof or side it stays there until reset-to-track.
- **`arcade`.** The 0.4.0 scripted roll described below (a roll rate held after a hard side impact, with the old arcade numbers: threshold halved, gain x1.5), the body box, and no
  simulation.

## Realistic: what it does

1. **Takeover** (`src/takeover.*`, pure). The simulation takes the car when, in one physics step, its velocity changes by more than `takeover_dv` (a barrier hit at ~90 km/h gives
   8-15 m/s; normal driving 0.2), when its up vector drops under 0.5 (on its side), or when all four wheels are off the ground and it spins faster than 2 rad/s for 5 steps. It
   starts from the game's state (position, basis as a quaternion, velocity, angular velocity from the basis change of the last step); after a hit it starts from the car
   *before* the hit and applies the hit as an impulse `m * dv` at a contact point on the hull side (`hit_height` of the way up from the floor: barriers and kerbs strike the lower
   body, which tips the car toward them).
2. **Each physics step while it drives** (the world-step hook, before the game's step): one fixed step (1/60 s, 10 substeps) of the simulation, then the pose is written with
   the named call `car_set_pose` (position, rows, the Euler angles derived from them, velocity; the rate fields zero) and the 35-degree clamp is lifted with `tilt_free`. The
   game's step then runs on that pose (the renderer and the camera read the car after that step, so what is on screen is the simulation's pose plus one step of the game's own
   motion; a write at the car-draw hook was measured to change nothing on screen). The game's own collision (a barrier) changes the velocity it finds; the next step reads the part of
   that change along the ground (`sim_hit_dv`) and applies it as an impulse. A car whose up row differs from the written one by more than 0.3, or that moved more than 3 m, was moved by
   something else (reset-to-track) and goes back to the game.
3. **Ground.** A plane through the four wheel records' ground heights (`car_wheel_contacts`), the slope fitted while the car is level, frozen from the takeover to the handback or reset.
   The hull is a box from `car_dimensions` (0.78 x wheelbase long, track/2 + 0.17 wide, 0.59 x wheelbase tall, its floor at the height the origin has over the wheels' ground at takeover (0.35 m when the records cannot say)).
4. **Handback.** up.z above 0.85, at least 3 floor points on the road and |w| under 0.5 rad/s for 10 steps: the final pose is written with zero rates, the clamp is restored and
   the game's suspension settles it. A position jump above 3 m (reset-to-track) ends the simulation.
5. **Before the first hit nothing is written**: the car is the stock game's (the in-game check shows it).

Limits: no barrier or wall geometry (the game keeps none that has been found: `docs/FACTS.md` "Collision code ... NOT FOUND"), so a wall only acts through the velocity change the
game's own collision gives; the road is a plane under the four wheels, not a per-point height (`ground_height` is not offered: the game's road query is not frame-neutral from a
tick, `docs/FINDINGS.md`); the hull proportions come from the Peugeot 206's model box and are a heuristic; player car only (AI cars are untouched).

## Arcade: what it does (0.4.0)

1. **The clamp.** `0x0893E0FC` limits the tilt angles (`K+0x6FC`, `K+0x700`) unless the word at `0x08CF2F08` is non-zero; `0x0894CCB8` reads
   that word (its setter has no caller in the binary). Since 0.5.0 the host hooks that reader on the mod's behalf (`tilt_free`, API 7); nothing is written to the game, and when the mod is
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
   **The pitch limit (0.4.0).** The game builds the body's axes from three Euler angles (yaw, pitch, roll about the forward axis; the forward axis' height is
   `sin(pitch)`: asm of `0x0893C3E4`). Its tilt clamp `0x0893E0FC` holds pitch (`K+0x6FC`) and roll (`K+0x700`) at 35 degrees with the same flag the mod lifts, so
   lifting it for the roll also freed the pitch: a hard hit saturates the pitch rate at 1.57 rad/s for ~25 steps (measured on the barrier route), the nose can
   go to vertical, and at pitch +-90 degrees yaw and roll turn about the same axis (a gimbal lock): the car stands on its tail and spins like a top. The fade of the
   roll drive above does not help there. The mod now keeps the game's own pitch limit itself: every step before the car's step, a pitch past 35 degrees is put back
   at 35 and its Euler rate zeroed, as the stock clamp does; the roll stays free. Only the player's car (AI cars still lose the whole clamp, see Known limits).
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
| `style` | `"realistic"` | `realistic`: the rigid-body simulation. `arcade`: the 0.4.0 scripted roll (threshold halved and gain x1.5 against the numbers below, so a barrier hit flips the car like 0.3.0 did) |
| `mass` | `1300.0` | realistic: the simulated car's mass (kg, 300 - 5000) |
| `restitution` | `0.15` | realistic: bounce of the hull on the road (0 - 0.9; never below 1 m/s of approach) |
| `friction` | `0.7` | realistic: Coulomb coefficient on asphalt (0 - 2) |
| `hit_height` | `0.2` | realistic: where a barrier hit lands on the hull side, from the floor (0) to the roof (1) |
| `sim_hit_dv` | `1.5` | realistic: while simulating, the velocity change along the ground (m/s) in one step that counts as a barrier hit (0.2 - 20) |
| `takeover_dv` | `5.0` | realistic: velocity change in one step (m/s) that hands the car to the simulation (1 - 50) |
| `threshold` | `6.0` | lateral velocity change in one step (m/s) above which a hit tips the car (0.1 - 50) |
| `gain` | `0.9` | roll rate (rad/s) per m/s above the threshold (0 - 50) |
| `cap` | `8.0` | largest roll rate (rad/s) the mod asks for (0 - 30) |
| `body_box` | `true` | the body box on or off (`false` = the stock tumble, the roof sinks into the road) |
| `box_height` | `0.8` | the box's roof above the car's origin (m, 0.5 - 3) |
| `box_kick_cap` | `0.5` | largest speed (m/s) away from the ground the car keeps in a step where its body touches it (0.1 - 20) |

An invalid value is logged and the default is used. Reference points (measured on the `rollover` pad route): the barrier at the end of the
straight at ~38 m/s gives ~8.3 m/s of lateral change: 0.3.0 (threshold 3, gain 2.5, cap 12) asked 12 rad/s for it and the car always ended on its
roof; 0.4.0 (6 / 0.9 / 8) asks ~2 rad/s, a lean, and the car drives on. A drive of R rad/s held 40 steps turns the car about R/3 rad and the car goes over
only past ~1 rad (its centre of mass over the wheel edge), so the flip needs ~5 rad/s: a lateral change of ~12 m/s in one step (`arcade`: ~7 m/s).

## Known limits

- There is no roof collision in the game: a rolling car keeps its wheels' contact model and always recovers; it does not stay on its roof.
- The clamp is lifted for every car: an AI car that tilts past 35 degrees is no longer held. AI cars get no impulses and none was seen to tilt
  that far in the runs, but this is not proven for every track.
- Only the player gets impulses, and the player is found as exactly one human (driver vtable `0x08CC1F60`, with input live or `+0xA8` set in its replay). With two humans
  (ad hoc / split screen) nobody gets an impulse while the clamp stays lifted.
- `arcade`: the roll is a held roll rate, not a physical simulation of the impact; the 40-step drive, the gain and the cap were tuned on one barrier hit.
- `GT_MOD_ROLLOVER_FAKE=<step>:<dv>` adds `<dv>` m/s of lateral change in the tracker's view at that player-car step (used by `mods_together_check`; `m<step>:<dv>` counts from the car's first movement instead, as `--replay` does);
  `GT_MOD_ROLLOVER_LOG=1` logs the car's state (used by `tools/mod_rollover_check.sh`).

## Tests

`ctest --test-dir build` runs `rollover_test` (settings, threshold, cap, style, signs, decay, teleport guards, the forward-axis weight), `physics_core_test` (free fall, rest with no creep, a corner drop that tips over, a slide stopped by friction, a bounce within 0.8-1.1 x e^2 h, a spin that decays, tumbling about the intermediate axis gains no energy, the quaternion <-> basis round trip) and `takeover_test` (each trigger, the handback rule, reset detection, the angular velocity from a basis change, the hit point). The in-game check lives in the
decomp repo: `tools/mod_rollover_check.sh` (see `docs/MODS.md` there).
