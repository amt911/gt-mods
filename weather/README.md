# Weather and time of day

Spec: `docs/superpowers/specs/2026-11-18-weather-mod-design.md` (gran-turismo-decomp). **Needs mod API v8** (the draw API: `scene_grade`, `on_overlay`, `draw_line`). No guest address.

- **Time of day:** a colour grade of the whole picture (`scene_grade`: tint, brightness, saturation) from a preset - `noon` (no change), `afternoon`, `sunset`, `dusk` - or `progressive`, which walks through them in equal
  thirds over the race (`progressive_steps` physics steps of the human's car, 18000 = 5 minutes; the replay re-simulates the race, so it shows the same light).
- **Rain** (visual only), `rain = 0..3` (off, light, medium, heavy): screen-space streaks drawn with the overlay (`draw_line`, 60 / 140 / 260 streaks, seeded and deterministic from the vblank, shifted sideways as the car yaws for parallax)
  and a darker, desaturated grade multiplied into the time of day's.
- **Grip reduction** (`grip_loss`, 0 = off, the default): the share of each step's *sideways* velocity change that is taken away, for driving changes only (a hit, more than 1.5 m/s in one step, is left alone). Written after the physics
  step like the damage mod's speed cap. **Not checked in game yet** (the pure part is unit-tested).
- The cheat device's Race page has the rows `Time of day` and `Rain`; `mod.toml` has the start values. In menus (no race on screen) the grade is neutral. Mod off: byte-identical frames (the host drops the grade).

Limits: the grade also tints the HUD (FACTS "Scene grade" in the host repo: the backend split that excludes it is not done); the game's own per-track fog / light tables (spec phase 2) were not researched, so the sky and fog do not shift, only the picture is
graded; the far plane is not lowered in rain (draw-distance's approach needs its own hook).

```bash
cmake -S . -B build -DCMAKE_CXX_COMPILER=clang++ -DGT_SDK_DIR=../gran-turismo-decomp/include && cmake --build build --target weather weather_test && ctest --test-dir build -R weather
```
In-game check (host repo): `tools/mod_weather_check.sh`: a graded dumped frame is *predicted* from the baseline frame of the same present with the host's grade formula and compared pixel by pixel.
Environment (checks only): `GT_MOD_WEATHER_LOG=1`, `GT_MOD_WEATHER_PRESET`, `GT_MOD_WEATHER_RAIN`, `GT_MOD_WEATHER_STEPS`.
