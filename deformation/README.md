# Crash deformation (0.1.0, work in progress)

Spec: `docs/superpowers/specs/2026-11-07-crash-deformation-design.md` (gran-turismo-decomp). Needs mod API v5 (gran-turismo-decomp PR "mod SDK v5").

**State (2026-11-14):** detects the player car's hits (`car_impact`, logs `HIT ...`), finds the player's model through the car-draw hook (car -> view -> driver
entry -> model container "3LDM"; every car has its own container, host FACTS "Container per car..."), reads every vertex run of the container once
(`model_runs`: parts with several vertex arrays included), dents the vertices near the contact IN PLACE (`src/deform_logic.*`: smooth falloff, saturating depth,
per-vertex cap; unit-tested) and writes the original vertices back when the race ends or the mod is switched off (`on_switch_off`). A container shared by two
cars is never dented. The in-game check (`tools/mod_deformation_check.sh`, host repo) passes: frames before the hit identical, a zero-strength dent is
detected as "no change after the hit". **Known limit:** on screen the dent shows for about a second after the hit and is small (a few hundred pixels at
480x272; the memory stays dented): the draw path seems to reuse something it drew before. See FINDINGS in the host repo. Off by default.

**0.2.1 (2026-11-16):** (a) the roof now dents: `roof_coupling` / `roof_start` (vertices above the beltline are dented by their ground-plane distance to the
contact point, so a hit at mid height still crumples the roof and pillars; 0.2.0 left the roof within 3-9 cm because the vertical reach stopped below it);
(b) a velocity change that is no hit never dents: `StepFilter` ignores the first steps of a race, a gap in the steps (retry), a teleport (car put back on the track)
and the rolling start's speed set (Time Trial sets the speed to exactly 13.889 m/s = 50 km/h when the countdown ends, a one-step change of +-6..11 m/s that read as a
hit and dented the rear or the front where nothing touched the car). The log says `SKIP step=... (start, retry or reset: not a hit)`; `GT_DEFORM_TRACE=1` logs
every step's velocity change (debugging).

```bash
cmake -S . -B build -DCMAKE_CXX_COMPILER=clang++ -DGT_SDK_DIR=../gran-turismo-decomp/include && cmake --build build --target deformation deformation_test
ctest --test-dir build -R deformation
```
