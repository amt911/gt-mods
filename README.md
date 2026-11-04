# gt-mods

Native mods for the PC build of the Gran Turismo (PSP) decompilation,
[gran-turismo-decomp](https://github.com/amt911/gran-turismo-decomp). Each mod is a shared library plus a
`mod.toml` manifest, loaded by `gt_play` through the C-ABI SDK (`include/gt_mod/gt_mod_api.h`).

Nothing game-derived lives here: only hand-written mod code.

## Mods

- `analog-pedals` - analog triggers / keyboard keys mapped to throttle and brake (0..1). Disabled by default.

## Build

Needs a `gran-turismo-decomp` checkout for the SDK header (default `../gran-turismo-decomp/include`):

```bash
cmake -S . -B build -DCMAKE_CXX_COMPILER=clang++ [-DGT_SDK_DIR=/path/to/gran-turismo-decomp/include]
cmake --build build -j4
ctest --test-dir build
```

## Install

Copy the mod directory (its `mod.toml` and the built library) into the game's mods directory and set
`enabled = true` in `mod.toml`.

## Licence

MIT, see `LICENSE`.
