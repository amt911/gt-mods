#!/usr/bin/env bash
# In-game check of the telemetry mod: headless gt_play (a host build with mod API 8) on the host's `rollover` pad route (the race: accelerate to the
# barrier at the end of the straight, hit it, throttle, brake), the mod's STEP log (GT_MOD_TELEMETRY_LOG=1) and frame dumps; tools/telemetry_check.py judges.
#   tools/telemetry_check.sh [--decomp off|on] [--seconds N (default 300: the CPU is on powersave)] [--every N] [--toggle] [--others]
#                            [--sabotage-g|--sabotage-nodraw|--sabotage-speed|--sabotage-bars]
#   (default)    runs: baseline (the mod installed, switched off), hidden (on, panel hidden), shown (on, panel shown). PASS only if
#                - hidden: every frame on the same game frame as the baseline's is byte-identical (the mod writes nothing, draws nothing);
#                - shown: outside the panel rectangle (and the lap timer) the paired frames equal the baseline's, the panel darkens its rectangle,
#                  the speed digits differ between frames of different speeds, the THR and BRK bars fill with the logged pedals;
#                - the log: G follows the car (braking < 0, throttle > 0, the barrier hit a large negative spike, and the smoothed G matches the speed
#                  change over 4 steps), and the cheat device's row exists (`[mods]` line).
#   --toggle     the pad chord L+TRIANGLE (bound to telemetry.toggle in the run's bindings.toml) is pressed twice in the race: the panel hides,
#                then shows again; PASS = `TOGGLE hidden` then `TOGGLE visible`, frames inside the hidden stretch identical to the baseline's, the panel back after
#   --others     baseline and shown runs with every other mod of the repo enabled too (the mods must coexist): the same shown judges
#   Each of these must FAIL: --sabotage-g (-DTELEMETRY_SABOTAGE=1: a constant G), --sabotage-nodraw (=2: the overlay draws nothing), --sabotage-speed (=3: the speed digits
#   frozen), --sabotage-bars (=4: the throttle and brake bars swapped).
# Env: KEEP_BASELINE=1 (reuse the baseline run), HOST (a gran-turismo-decomp checkout with the API 8 build in $HOST/build, default ../gt-drawapi), MOD_CHECK_NO_LOCK=1 (the caller holds the CPU lock),
# OUT (default <this repo>/check-out, gitignored; never commit it: it holds game frames). Exit: 0 pass, 1 check failed, 2 setup/run problem.
# Each run is a real-time job inside the memory cgroup and its own hold of the shared CPU lock ($HOST/tools/cpu_lock.sh).
set -u
here=$(cd "$(dirname "$0")/.." && pwd)
host=$(cd "${HOST:-$here/../gt-drawapi}" && pwd) || exit 2
build=${BUILD:-$host/build}
out=${OUT:-$here/check-out}
decomp=off seconds=300 every=30 sabotage=0 toggle=0 others=0
while [ $# -gt 0 ]; do
  case $1 in
    --sabotage-g) sabotage=1;; --sabotage-nodraw) sabotage=2;; --sabotage-speed) sabotage=3;; --sabotage-bars) sabotage=4;;
    --toggle) toggle=1;; --others) others=1;;
    --decomp) decomp=$2; shift;; --seconds) seconds=$2; shift;; --every) every=$2; shift;;
    *) echo "telemetry_check: unknown argument $1" >&2; exit 2;;
  esac
  shift
done
[ -x "$build/gt_play" ] || { echo "telemetry_check: no $build/gt_play" >&2; exit 2; }
route_src="$host/config/pad_routes/rollover.pad"
[ -f "$route_src" ] || { echo "telemetry_check: no pad route $route_src" >&2; exit 2; }
py="$host/.venv/bin/python"
mkdir -p "$out" || exit 2
cd "$host" || exit 2  # gt_play reads extracted/ and recomp/ relative to the host checkout

mbuild="$out/mod-build-s$sabotage"
{ cmake -S "$here" -B "$mbuild" -DCMAKE_CXX_COMPILER=clang++ -DGT_SDK_DIR="$host/include" "-DCMAKE_CXX_FLAGS=-DTELEMETRY_SABOTAGE=$sabotage" \
    && cmake --build "$mbuild" -j2; } > "$out/mod-build-s$sabotage.log" 2>&1 \
  || { echo "telemetry_check: mod build failed (log $out/mod-build-s$sabotage.log)" >&2; exit 2; }

# the route: the rollover route, plus (for --toggle) two L+TRIANGLE presses while driving (the race page's time base: +1800 hides, +2700 shows)
grep -v '^#' "$route_src" | tr -d '\n' > "$out/route_plain.pad"
cp "$out/route_plain.pad" "$out/route_toggle.pad"
printf ',1800:L:20,1800:TRIANGLE:20,2700:L:20,2700:TRIANGLE:20' >> "$out/route_toggle.pad"

# make_mods LABEL TELEMETRY_ENABLED(true|false) [others]: <out>/<label>/mods/<id>/ with the library and mod.toml of each
make_mods() {
  local label=$1 enabled=$2 withothers=$3 d="$out/$1/mods"
  rm -rf "$out/$label" && mkdir -p "$d/telemetry" "$out/$label/savedata" "$out/$label/frames" || return 2
  cp "$mbuild/telemetry/libtelemetry.so" "$d/telemetry/" && sed "s/^enabled = false/enabled = $enabled/" "$here/telemetry/mod.toml" > "$d/telemetry/mod.toml" || return 2
  printf 'telemetry.toggle.pad = "L+TRIANGLE"\n' > "$out/$label/bindings.toml"
  if [ "$withothers" = others ]; then
    local m id lib
    for m in "$here"/*/mod.toml; do
      id=$(basename "$(dirname "$m")"); [ "$id" = telemetry ] && continue
      lib=$(sed -n 's/^library = "\(.*\)"/\1/p' "$m")
      mkdir -p "$d/$id" && cp "$mbuild/$id/$lib" "$d/$id/" && sed 's/^enabled = false/enabled = true/' "$m" > "$d/$id/mod.toml" || return 2
    done
  fi
}

# run_one LABEL VISIBLE(0|1) ROUTE_FILE
run_one() {
  local label=$1 visible=$2 rfile=$3
  local -a vars=(GT_MODS_DIR="$out/$label/mods" GT_SAVEDATA_DIR="$out/$label/savedata" SDL_VIDEODRIVER=dummy LCS_UNCAPPED=1 GT_MOD_TELEMETRY_LOG=1
                 GT_MOD_TELEMETRY_VISIBLE="$visible" GT_PAD="$(cat "$rfile")" GT_PAD_LOG=1 GT_FRAME_EVERY=$every GT_FRAME_DUMP="$out/$label/frames")
  local -a lock=("$host/tools/cpu_lock.sh" "telemetry_check:$label"); [ -n "${MOD_CHECK_NO_LOCK:-}" ] && lock=()
  local try rc=0
  for try in 1 2 3 4; do  # a hold is killed when another session's wait passes 30 min; the run is then repeated
    "${lock[@]}" env "${vars[@]}" \
      systemd-run --user --scope --quiet --slice=gt-heavy.slice -p CPUWeight=10000 -- \
      timeout $((seconds + 120)) "$build/gt_play" --max-seconds "$seconds" --decomp "$decomp" > "$out/$label/run.log" 2>&1
    rc=$?
    [ $rc -ne 143 ] && break
    echo "telemetry_check: $label: the lock hold was stopped (rc 143), run $try of 4" >&2
  done
  if [ $rc -ne 0 ]; then echo "telemetry_check: gt_play exited $rc (log $out/$label/run.log)" >&2; return 2; fi
  grep -aq 'script done' "$out/$label/run.log" || { echo "telemetry_check: $label: the pad route did not finish (log $out/$label/run.log)" >&2; return 2; }
  grep -ah '^\[mods\]' "$out/$label/run.log"
}

sfx=""; [ $others -eq 1 ] && sfx="_others"
oth=""; [ $others -eq 1 ] && oth=others
if [ $toggle -eq 1 ]; then
  make_mods baseline_toggle false "" && make_mods toggle true "" || exit 2
  run_one baseline_toggle 1 "$out/route_toggle.pad" && run_one toggle 1 "$out/route_toggle.pad" || exit 2
  "$py" -I "$here/tools/telemetry_check.py" "$host" "$out" toggle
  exit $?
fi
labels="hidden shown"; [ $sabotage -ne 0 ] && labels="shown"
[ $others -eq 1 ] && labels="shown"
# KEEP_BASELINE=1 reuses a baseline of $out made with the same --decomp, --every and --seconds (the sabotage runs share it: the mod is off in it)
cfg="decomp=$decomp every=$every seconds=$seconds"
if [ -n "${KEEP_BASELINE:-}" ] && [ -d "$out/baseline$sfx/frames" ] && [ "$(cat "$out/baseline$sfx/.config" 2>/dev/null)" = "$cfg" ]; then
  echo "telemetry_check: reusing $out/baseline$sfx"
else
  make_mods "baseline$sfx" false "$oth" || exit 2
  run_one "baseline$sfx" 1 "$out/route_plain.pad" || exit 2
  echo "$cfg" > "$out/baseline$sfx/.config"
fi
for l in $labels; do
  v=1; [ "$l" = hidden ] && v=0
  make_mods "$l$sfx" true "$oth" || exit 2
  run_one "$l$sfx" $v "$out/route_plain.pad" || exit 2
done
"$py" -I "$here/tools/telemetry_check.py" "$host" "$out" "baseline$sfx" $(for l in $labels; do echo "$l$sfx"; done)
