# Crash deformation (0.1.0, work in progress)

Spec: `docs/superpowers/specs/2026-11-07-crash-deformation-design.md` (gran-turismo-decomp). Needs mod API v5 (gran-turismo-decomp PR "mod SDK v5").

**State:** detects the player car's hits from the velocity change of the physics step (`car_impact`) and logs `HIT ...` lines; the dent logic
(`src/deform_logic.*`: threshold, falloff, per-vertex cap, restore) is unit-tested. It does **not** change any mesh yet because the link between the
player car and its body blobs has not been found (FINDINGS in the host repo, 2026-11-14). Off by default.

```bash
cmake -S . -B build -DCMAKE_CXX_COMPILER=clang++ -DGT_SDK_DIR=../gran-turismo-decomp/include && cmake --build build --target deformation deformation_test
ctest --test-dir build -R deformation
```
