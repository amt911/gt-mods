# Crash deformation (0.1.0, work in progress)

Spec: `docs/superpowers/specs/2026-11-07-crash-deformation-design.md` (gran-turismo-decomp). Needs mod API v5 (gran-turismo-decomp PR "mod SDK v5").

**State (2026-11-14):** detects the player car's hits (`car_impact`, logs `HIT ...`), finds the player's model through the car-draw hook (car -> view -> driver
entry -> model container "3LDM"; every car has its own container, host FACTS "Container per car..."), reads every vertex run of the container once
(`model_runs`: parts with several vertex arrays included), dents the vertices near the contact IN PLACE (`src/deform_logic.*`: smooth falloff, saturating depth,
per-vertex cap; unit-tested) and writes the original vertices back when the race ends or the mod is switched off (`on_switch_off`). A container shared by two
cars is never dented. The in-game check (`tools/mod_deformation_check.sh`, host repo) passes: frames before the hit identical, a zero-strength dent is
detected as "no change after the hit". **Known limit:** on screen the dent shows for about a second after the hit and is small (a few hundred pixels at
480x272; the memory stays dented): the draw path seems to reuse something it drew before. See FINDINGS in the host repo. Off by default.

```bash
cmake -S . -B build -DCMAKE_CXX_COMPILER=clang++ -DGT_SDK_DIR=../gran-turismo-decomp/include && cmake --build build --target deformation deformation_test
ctest --test-dir build -R deformation
```
