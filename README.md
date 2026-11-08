# gt-mods

Native mods for the PC build of the Gran Turismo (PSP) decompilation,
[gran-turismo-decomp](https://github.com/amt911/gran-turismo-decomp). Each mod is a shared library plus a
`mod.toml` manifest, loaded by `gt_play` through the C-ABI SDK (`include/gt_mod/gt_mod_api.h`).

Nothing game-derived lives here: only hand-written mod code.

## Mods

- `analog-pedals` - analog triggers / keyboard keys mapped to throttle and brake (0..1). Disabled by default.
  Settings in its `mod.toml` (`deadzone` in [0,1), `curve` > 0, `ramp_ms` >= 0; an invalid value is logged and the default
  used). Controller: the first SDL game controller's right trigger = accelerate, left trigger = brake. Keyboard: `throttle_keys`
  / `brake_keys` (SDL key names, default `Up,W` / `Down,S`), ramped over `ramp_ms`. The pad's own buttons keep working: the
  larger press wins, except while a trigger is pressed: then the trigger decides (gt_play also maps a trigger past ~25 % to
  Cross/Square, which would otherwise turn a half press into a full one). Works in races and Time Trial. The pedal follows the trigger exactly (the mod writes the record value whose game response is that pedal),
  except below 0.05 (not injected) and 0.05-0.1 (the game's smallest press is 0.1). Only the player's live car is touched (not AI cars, not during a replay).
  `GT_MOD_PEDALS_FAKE=0.5,0.3` replaces the input with a fixed throttle/brake (used by the in-game check);
  `GT_MOD_PEDALS_LOG=1` logs the player's and an AI car's pedals once a second.
- `draw-distance` - pushes the race world's far clip plane further away, so far scenery (buildings, hills, grandstands) shows
  instead of popping in. The game does not cull by that plane (it submits the same geometry and the GPU clips), so a larger far
  reveals what was already submitted; the cost is GPU pixels, not game CPU. Post-hook on the depth builder `0x08AF4B4C` (called from `0x0890E41C`
  with near = `self+4`, far = `self+0xC` = 1200 in the races seen): the matrix terms m10/m14 are rewritten for `far * far_scale`
  (scene data is never written; a hook on `0x0890E41C` that scales and restores `self+0xC` does nothing, because `call_original`
  returns before the rest of that body runs). `far_scale` in `mod.toml`, default `2.5`
  (1200 -> 3000), range 1.0-6.0 (clamped, a warning is logged; non-numbers keep the default); 1.0 = frames identical to no
  mod (the mod writes nothing). Larger values thin the depth buffer (a few speckles on far geometry at 8000). The sky pass and
  the car reflection pass do not use this function and are untouched. Disabled by default; toggles live from the F9 menu.
  `GT_MOD_DRAWDIST_SCALE=<n>` overrides the setting (used by the in-game check, `tools/mod_drawdist_check.sh`);
  `GT_MOD_DRAWDIST_LOG=1` logs near/far every 120 edits.
- `orbit-camera` - Gran Turismo 7 style orbit camera on the right stick. The stick's direction picks the look direction around the
  player's car (right = +90 deg, left = -90 deg, forward = 0); pulled back (past `back_threshold`) or clicked (R3) = rear view
  (180 deg). The camera swings around the car in chase views and looks around from the seat in bumper/cockpit views (the pivot is
  the car, which there is ~at the eye). The stick turns the other way in every view: `invert_outside_chase` (bumper, cockpit, near
  chase; default true, like GT7) and `invert_in_chase` (the far, full third-person chase view; default true; told by the car's
  distance from the eye, > 6.5); no pitch from the stick (`pitch_deg`, default 0, adds a constant tilt). Smoothed with a
  `smooth_ms` time constant (80) and back to 0 on release; dead zone `deadzone` (0.25). Keyboard: `look_left_keys` /
  `look_right_keys` / `look_back_keys` (SDL key names, defaults `J` / `L` / `K`, free in gt_play's default bindings). Only while
  driving (live player car, not a replay, not a menu); with no input it writes nothing. Disabled by default; toggles live from the
  F9 menu. `GT_MOD_CAMERA_FAKE=<yaw>[,<pitch>]` fixes the angle (used by the in-game check, `tools/mod_camera_check.sh`);
  `GT_MOD_CAMERA_LOG=1` logs yaw, scene and pivot once a second. Pulling the stick back blends the angle smoothly into the rear
  view (full 180 only at full back). The orbit is live while physics steps keep arriving (a gap of 10 frames, counted by the HUD's
  uploads, ends it). The orbit is applied to the camera's view when the scene renderer loads it for each layer (0x08AF1524 ->
  0x08B568DC), so the sky dome and far layers are placed and culled for the orbited heading (rotating only the objects' world
  matrices left the sky undrawn behind the car, and the game never clears the colour buffer in a race: earlier frames showed
  through as a hall-of-mirrors ghost); the rear-view mirror's camera is left alone. Known limits: in bumper/cockpit views the pivot is the car, which is not exactly at the eye (the camera
  swings slightly instead of turning in place); the mod opens its own SDL controller handle next to the host's; the look keys
  (J/K/L) also act while the F9 mod menu is open (the SDK cannot tell whether the menu is open).
- `reset-to-track` - one button puts the player's car back on the road, upright and facing the way it was going, like "recover car" in
  modern racing games. Button: pad Select held 0.5 s (`hold_ms`), or the keyboard key `reset_keys` (default `R`). While driving it keeps a
  ring of "good" poses (all four wheels on asphalt, up vector z > 0.9, faster than 5 m/s, yaw rate <= 0.6 rad/s, velocity within ~25 deg of
  the heading; one every 15 steps); the reset restores the newest pose at least `rewind_ms` (2500) old, zeroes velocity, speed and all
  rotation rates (the car restarts from rest) and ignores further presses for `cooldown_ms` (1000). Nothing happens when no good pose
  exists yet. The ring is emptied when the car jumps more than 15 m in a step (a respawn, a new attempt), after 2 s without steps (the mod
  was switched off in F9, a long pause) and on a new car; poses older than 8 s are never restored. Allowed everywhere (Time Trial
  included); only the live player car, never a replay. Disabled by default; toggles live from the F9 menu.
  **Select and the camera:** the game uses Select to change the view, so while racing the mod hides a held Select from the game; a press
  shorter than `hold_ms` is replayed as a tap on release (the camera view still changes), a longer one resets and the view does not change.
  Outside a race (menus, pause) Select is untouched. **Known limits:** gear, rpm and the previous-position fields are not restored (not
  located; the automatic gearbox relaunches fine from rest, the in-game check requires > 15 m/s with the throttle held after a reset);
  lap counter and timer are not rewound (measured on a mid-lap reset: HUD 0'24.000 -> 0'27.000 over 3 s of play, lap 1/2 unchanged), and a reset that crosses the start/finish line was not tested (no route completes a lap) and may
  count the lap twice; a Time Trial ghost / replay recording will show the jump (untested); no track geometry is used, the car goes back
  where it was, not to the nearest asphalt. `GT_MOD_RESET_FAKE=<step>` holds the button from that player-car step (a debugging aid); `GT_MOD_RESET_LOG=1` logs the car's state (used by `tools/mod_reset_check.sh`).
- `cheats` - race-time cheats as rows of the F9 menu (keyboard or pad): slow motion (0.5x / 0.25x, player and AI cars) and freeze AI. Needs mod API v2
  (`api = 2`). Disabled by default and both cheats start off. See `cheats/README.md`; check: `tools/mod_cheats_check.sh` in the host repo.
- `rollover` - lets the player's car roll over, which the stock game cannot (it clamps the tilt to 35 degrees). Hooks the flag reader
  `0x0894CCB8` (a dormant switch) to lift the clamp and adds a roll rate on a hard side impact (a lateral velocity change above `threshold`
  in one physics step; `gain`, `cap`, `style = realistic|arcade` in `mod.toml`); the dormant flip state machine the flag would also wake
  (the car sinks through the road and pops back up) is kept asleep. Opt-in: `enabled = false` by default; toggles live from the F9 menu.
  See `rollover/README.md`. In-game check: `tools/mod_rollover_check.sh`.

## Build

Needs a `gran-turismo-decomp` checkout for the SDK header (default `../gran-turismo-decomp/include`):

```bash
cmake -S . -B build -DCMAKE_CXX_COMPILER=clang++ [-DGT_SDK_DIR=/path/to/gran-turismo-decomp/include]
cmake --build build -j4
ctest --test-dir build
```

## Install

Copy the mod directory (`analog-pedals/` with its `mod.toml` and the built `libanalog_pedals.so` from
`build/analog-pedals/`) into the game's mods directory (`<build>/mods/` next to `gt_play`, or `GT_MODS_DIR`) and set
`enabled = true` in `mod.toml`. `gt_play` prints `[mods] loaded: ...` at start. Run the in-game check from the
gran-turismo-decomp checkout: `tools/mod_pedals_check.sh` / `tools/mod_camera_check.sh` / `tools/mod_reset_check.sh` / `tools/mod_rollover_check.sh`.

## Releases

A release is a git tag; CI (`.github/workflows/release.yml`) builds every mod against the pinned `gran-turismo-decomp`
`include/` (commit SHA in the workflow's `SDK_REF`), runs the tool tests, packs the archives and uploads them with `index.json`:

```bash
git tag vYYYY.MM.DD        # a second release the same day: vYYYY.MM.DD.1
git push origin vYYYY.MM.DD
```

Run the workflow by hand (Actions -> release -> Run workflow) for a dry run: it builds and packs but uploads nothing
(the files stay as the `release-dry-run` workflow artifact). Bump `version` in a mod's `mod.toml` (`x.y.z`) when the mod changes;
the index lists whatever the tagged commit says.

Locally, the same files (needs a build in `build/`):

```bash
python3 tools/make_index.py --build-dir build --platform linux-x86_64 \
    --base-url https://github.com/amt911/gt-mods/releases/download/vYYYY.MM.DD --out-dir dist
python3 -m pytest tests --cov=tools --cov-fail-under=80
```

The output is deterministic (sorted tar entries, fixed mtime/uid/gid, gzip mtime 0): the same inputs give byte-identical files.
Each mod becomes `<id>-<version>-<platform>.tar.gz` holding a folder `<id>/` with `mod.toml` and the library; unpack it into the
game's mods directory. One-line mod descriptions live in `tools/descriptions.toml` (the host's `mod.toml` reader rejects unknown keys).

### `index.json`

```json
{
  "schema": 1,
  "mods": [
    {
      "id": "analog-pedals",
      "name": "Analog pedals",
      "version": "0.1.0",
      "api": 2,
      "description": "Analog triggers and keyboard keys mapped to throttle and brake (0..1).",
      "platforms": {
        "linux-x86_64": {
          "url": "https://github.com/amt911/gt-mods/releases/download/v2026.11.08/analog-pedals-0.1.0-linux-x86_64.tar.gz",
          "sha256": "<hex sha256 of the archive>",
          "size": 29508
        }
      }
    }
  ]
}
```

## Licence

MIT, see `LICENSE`.
