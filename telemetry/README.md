# telemetry

A panel over the race (mod API 8: `on_overlay`, `draw_text`, `draw_rect`, `draw_line`). Read-only: the mod writes nothing to the game, and with the
panel hidden it draws nothing (frames identical to a run without it).

Shows, top-left (480 x 272 overlay space, `x` / `y` in `mod.toml`): speed (km/h or mph), longitudinal and lateral G, throttle and brake bars and
two 5-second graphs (speed; G longitudinal in amber, lateral in green).

- **G** = the car's velocity change per physics step (1/60 s) projected on its forward and right axes, over 4 steps (moving average), divided by 9.81.
  The up axis is left out. A teleport (race start, retry, reset: > 15 m in one step) gives 0 G. A wall hit shows as a huge, clamped (+-12 G) spike.
- **Toggle:** the cheat device row *Telemetry panel* (Race) or the hotkey `T` (rebind it in Controls; no default pad button). `visible = false` in
  `mod.toml` starts it hidden.
- The panel shows only while the player's live car steps (not in menus, the pause, a replay).

## Not done

Per-wheel slip and wheel speed need a named SDK call that does not exist yet: `car_wheels(car, out[4])` (spec step 0: the 0x38-byte wheel record).

## Check

`tools/telemetry_check.sh` (needs a host build with API 8): see its header. Sabotages that must fail: `--sabotage-g`, `--sabotage-nodraw`,
`--sabotage-speed`, `--sabotage-bars`.
