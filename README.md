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
  larger press wins. The pedal follows the trigger exactly (the mod writes the record value whose game response is that pedal),
  except below 0.05 (not injected) and 0.05-0.1 (the game's smallest press is 0.1). Only the player's live car is touched (not AI cars, not during a replay).
  `GT_MOD_PEDALS_FAKE=0.5,0.3` replaces the input with a fixed throttle/brake (used by the in-game check);
  `GT_MOD_PEDALS_LOG=1` logs the player's and an AI car's pedals once a second.

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
gran-turismo-decomp checkout: `tools/mod_pedals_check.sh`.

## Licence

MIT, see `LICENSE`.
