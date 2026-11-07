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
  the car, which there is ~at the eye). Like GT7, in every view except the far (full third-person) chase view the stick turns the
  other way (`invert_outside_chase`, default true; the far chase view is told by the car's distance from the eye, > 6.5); no pitch from the stick (`pitch_deg`, default 0, adds a constant tilt). Smoothed with a
  `smooth_ms` time constant (80) and back to 0 on release; dead zone `deadzone` (0.25). Keyboard: `look_left_keys` /
  `look_right_keys` / `look_back_keys` (SDL key names, defaults `J` / `L` / `K`, free in gt_play's default bindings). Only while
  driving (live player car, not a replay, not a menu); with no input it writes nothing. Disabled by default; toggles live from the
  F9 menu. `GT_MOD_CAMERA_FAKE=<yaw>[,<pitch>]` fixes the angle (used by the in-game check, `tools/mod_camera_check.sh`);
  `GT_MOD_CAMERA_LOG=1` logs yaw, scene and pivot once a second. Pulling the stick back blends the angle smoothly into the rear
  view (full 180 only at full back). The orbit is live while physics steps keep arriving (a gap of 10 frames, counted by the HUD's
  uploads, ends it). Known limits: in bumper/cockpit views the pivot is the car, which is not exactly at the eye (the camera
  swings slightly instead of turning in place); the mod opens its own SDL controller handle next to the host's; the look keys
  (J/K/L) also act while the F9 mod menu is open (the SDK cannot tell whether the menu is open).

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
gran-turismo-decomp checkout: `tools/mod_pedals_check.sh` / `tools/mod_camera_check.sh`.

## Licence

MIT, see `LICENSE`.
