# Best-lap ghost in any mode

Spec: `docs/superpowers/specs/2026-11-18-ghost-any-mode-design.md` (gran-turismo-decomp). **Needs mod API v9** (world step, camera function with `camera_get` / `camera_projection`, the draw API's overlay). No guest address.

- Every physics step the human's car (live, or in its replay) is recorded. A **lap** ends when the car crosses its start line: the point where it first moved, crossed forward after having been at least 60 m away (and within 45 m of the line sideways).
  The game's own lap event was not found (host FACTS "Lap line"), so this is the fallback the spec allowed; it works for circuits, not for point-to-point stages.
- The **best lap** of the session (fewest steps; laps under 5 s are ignored) is kept and, on the following laps, shown as a translucent **ghost** at the pose the car had at the same lap time. The ghost is visual only (no collision, AI cars have none).
- `persist = true` saves the best lap per start line and car under `<mods dir>/ghost-any-mode/<hash>.lap` and loads it when the same one is raced again (a small binary file: magic, version, step count, poses as float32).
- Cheat device, Race page: the row `Ghost: ON/OFF (best lap 1'23.45)` toggles it, the action `ghost-any-mode.clear` forgets the best lap.

**Deviation from the spec:** the ghost is a **box with the car's size** drawn with overlay lines and projected with the player camera, not the car's own model. The spec's `draw_car_pose` (the extra car pass) is not in the merged API 8; the host research shows
a car-draw replay works (host FACTS "Car draw replay: WORKS") but it was never made an SDK call. The box needs the camera: the mod's camera function reads `camera_get` and the new `camera_projection` (host API 9) and never changes the view. When `draw_car_pose` exists
the box can be replaced by the model without touching the recording.

Limits: no ghost in a menu, the line detection needs a circuit, the box is a fixed size (`box_*` settings; the wheelbase is used only to name the persisted file).

```bash
cmake -S . -B build -DCMAKE_CXX_COMPILER=clang++ -DGT_SDK_DIR=../gran-turismo-decomp/include && cmake --build build --target ghost_any_mode ghost_test && ctest --test-dir build -R ghost
```
In-game check (host repo): `tools/mod_ghost_check.sh`. The routes cannot drive a lap, so the checks make a lap boundary every N steps after the car first moves (`GT_MOD_GHOST_FAKE_LAP=m400`); the real line detection is unit-tested.
Environment (checks only): `GT_MOD_GHOST_LOG=1`, `GT_MOD_GHOST_FAKE_LAP`, `GT_MOD_GHOST_HIDE=1`.
