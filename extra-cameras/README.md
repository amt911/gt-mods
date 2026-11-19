# Extra cameras

Spec: `docs/superpowers/specs/2026-11-18-extra-cameras-design.md` (gran-turismo-decomp). **Needs mod API v9** (host PR "mod API v9: camera", which must merge after #667, the draw API v8). No guest address.

Views, chosen in the cheat device (category **Camera**: the row `Extra view` cycles Game / Wheel / Roof / TV with Enter, Left, Right; the action `extra-cameras.next_view` can be bound to a key or button):

- **Wheel:** fixed to the body at the front-left wheel (the axle offset and the track come from `car_dimensions`), low, looking along the car.
- **Roof:** above the roof, looking forward and slightly down.
- **TV:** a trackside point ahead of the car and to its side; the camera stays there and looks at the car, and moves on to a new point ahead when the car is `tv_pass` metres past it.
- **Replay helicopter** (row `Replay camera`): a point above and behind the car, followed by a critically damped spring (`heli_smooth_s`), looking at the car. Works while `replay_active()`.

How: each physics step (`GT_HOOK_WORLD_STEP`) the human's car (live, or in its replay) is read; the camera function (`GT_HOOK_CAMERA`) replaces the pose with `camera_set`. Frames with no extra view selected are
byte-identical to a run without the mod. The offsets are in the body frame; see `mod.toml`. Best with the game's chase view (in the bumper / cockpit / hood views the game hides the body).

**Not done in 0.1.0:** extending the game's own SELECT cycle (and the replay's camera button) with the extra views, which needs the game's view state (not researched); the near-plane adjustment (the SDK has no accessor);
migrating orbit-camera to `camera_get` / `camera_set`. Settings `start_view`, offsets and smoothing: `mod.toml`. Environment (checks only): `GT_MOD_CAMS_LOG=1`, `GT_MOD_CAMS_VIEW=wheel|roof|tv|heli`.

```bash
cmake -S . -B build -DCMAKE_CXX_COMPILER=clang++ -DGT_SDK_DIR=../gran-turismo-decomp/include && cmake --build build --target extra_cameras cam_test
ctest --test-dir build -R cam
```

In-game check (host repo): `tools/mod_cameras_check.sh`.
