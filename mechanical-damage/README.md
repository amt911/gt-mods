# Mechanical damage

Spec: `docs/superpowers/specs/2026-11-18-mechanical-damage-design.md` (gran-turismo-decomp). Mod API v5 (the cheat-device entries need v4, which every v5 host has). No guest address.

Hard impacts (the same `car_impact` events and the same `StepFilter` the deformation mod uses; a higher bar, `threshold`, so ordinary racing contact does nothing)
add damage to three parts, by where the hit landed on the body:

| hit | engine | drive | steering alignment |
| --- | --- | --- | --- |
| front | yes | | half |
| rear | | yes | |
| sides | | | yes (signed: the car pulls toward the side that was hit) |

Effects, applied to the human's car every physics step, live and in its replay:

- **Engine / drive damage (engine + drive, capped at 1):** the engine's contribution is scaled by `1 - engine_loss * damage` (up to -40 %): the net speed gain per step is
  that fraction of the undamaged one. The pedal itself cannot be used (measured: a scaled `+0x530` written before the step changes nothing, the drive chain derives its own
  effective throttle), so before each step the speed loses `(1 - scale) x` the gain the engine gave in the last step, measured from where that step started (no compounding).
  The top speed stays what the car's drag and gearing give; the cap below is what lowers it.
- **Misaligned steering:** a yaw-rate bias, `yaw_bias` rad/s at full alignment damage.
- **Severe damage** (engine + drive above `severe_start`): a top-speed cap from `cap_high` down to `cap_low` m/s.

Damage lasts for the race (a new race, or the replay starting, clears it; the replay re-simulates the impacts and rebuilds it). Cheat device, category **Car**: a row `Damage: engine
X%  drive Y%  steering Z%` (Enter repairs) and the action `mechanical-damage.repair`. Off by default. Not saved with the car, no AI damage, no visual damage (deformation does that).

Settings: see `mod.toml`. Environment (checks only): `GT_MOD_DAMAGE_LOG=1` (one state line per step), `GT_MOD_DAMAGE_FAKE=m30:25:f` (a fake hit of 25 m/s, from the front, 30 steps after the
accelerator is first pressed; `b` rear, `r` right, `l` left), `GT_MOD_DAMAGE_REPAIR=m200`.

```bash
cmake -S . -B build -DCMAKE_CXX_COMPILER=clang++ -DGT_SDK_DIR=../gran-turismo-decomp/include && cmake --build build --target mechanical_damage damage_test
ctest --test-dir build -R damage
```

In-game check (host repo): `tools/mod_damage_check.sh`.
