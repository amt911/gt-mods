# Crash deformation (0.1.0, work in progress)

Spec: `docs/superpowers/specs/2026-11-07-crash-deformation-design.md` (gran-turismo-decomp). Needs mod API v5 (gran-turismo-decomp PR "mod SDK v5").

**State (2026-11-14):** detects the player car's hits (`car_impact`, logs `HIT ...`), finds the player's model through the car-draw hook (car -> view -> driver
entry -> model container "3LDM", host FACTS "Car to model link"), copies the container's parts once, dents the copies around the contact
(`src/deform_logic.*`, unit-tested) and redirects the mesh draw of each original to its copy. **Not proven visible:** in the in-game check the redirect
fires (`REDIRECT` line) but frames after the hit are identical to a zero-strength run: the two big body parts (446 and 398 vertices) are never drawn
through the entry blobs the hook sees, and the blobs that are drawn near them (e.g. 0x090EB4D0, 0x090F3490, inside the container's memory span) are
not entries of its part table. Which blob draws the visible body is open. Off by default.

```bash
cmake -S . -B build -DCMAKE_CXX_COMPILER=clang++ -DGT_SDK_DIR=../gran-turismo-decomp/include && cmake --build build --target deformation deformation_test
ctest --test-dir build -R deformation
```
