"""Verdict of tools/telemetry_check.sh: gt_play runs of the telemetry mod on the host's `rollover` pad route against a baseline run with the mod installed
but switched off (same route, so the same game state on the same game frame: frames are paired by the lap timer, as the host's mod checks do).

  python -I tools/telemetry_check.py HOST OUT baseline_label label...   (labels starting `hidden` or `shown`)
  python -I tools/telemetry_check.py HOST OUT toggle                    (runs `baseline` and `toggle`)

hidden  the panel hidden: every frame on the same game frame as the baseline's is byte-identical.
shown   outside the panel rectangle the paired frames equal the baseline's, the panel darkens its rectangle, the speed digits differ between frames of
        different speeds, the throttle and brake bars fill with the logged pedals, and the log's G follows the car.
toggle  the bound chord hides the panel (frames identical to the baseline's while it is hidden) and shows it again.
"""
import os
import re
import sys

host = sys.argv[1]
sys.path.insert(0, os.path.join(host, "tools"))  # python -I does not put the script directory on the path
import mod_camera_check as cam  # noqa: E402

PANEL = (8, 52, 158, 180)         # x0, y0, x1, y1 (exclusive): mod.toml's x, y and the panel's 150 x 128
SPEED_BOX = (12, 54, 100, 72)     # the speed readout
THR_BAR = (38, 87, 152, 92)
BRK_BAR = (38, 97, 152, 102)
GREEN, RED = (0x43, 0xA0, 0x47), (0xE5, 0x39, 0x35)
STEP = re.compile(r"STEP n=(\d+) vblank=(\d+) speed=([-\d.]+) glong=([-\d.]+) glat=([-\d.]+) thr=([-\d.]+) brk=([-\d.]+)")
LIVE_GAP = 30       # vblanks: a frame within this of a STEP line shows the panel
MIN_DARK = 0.6      # share of the panel rectangle the panel must change
MIN_FRAMES = 12


def read_log(path):
    return open(path, errors="replace").read() if os.path.exists(path) else ""


def parse_steps(log):
    return [(int(n), int(v), float(sp), float(gl), float(gt), float(th), float(br)) for n, v, sp, gl, gt, th, br in STEP.findall(log)]


def last_step_before(steps_by_vblank, vblanks, k):
    """The newest STEP at a vblank <= k within LIVE_GAP, else None. `vblanks` is the sorted list of the STEP vblanks."""
    lo, hi = 0, len(vblanks)
    while lo < hi:
        mid = (lo + hi) // 2
        if vblanks[mid] <= k:
            lo = mid + 1
        else:
            hi = mid
    if lo == 0 or k - vblanks[lo - 1] > LIVE_GAP:
        return None
    return steps_by_vblank[vblanks[lo - 1]]


def region(frame, box):
    w, _h, px = frame
    x0, y0, x1, y1 = box
    return [(px[(y * w + x) * 3], px[(y * w + x) * 3 + 1], px[(y * w + x) * 3 + 2]) for y in range(y0, y1) for x in range(x0, x1)]


def diff_pixels(a, b, box=None, outside=False):
    """Pixels that differ (any channel); inside `box`, or outside it and outside the lap timer's clock box."""
    w, h, pa = a
    pb = b[2]
    if pa == pb:
        return 0
    n = 0
    cx0, cx1, cy0, cy1 = cam.CLOCK_BOX
    for y in range(h):
        row = y * w * 3
        if pa[row:row + w * 3] == pb[row:row + w * 3]:
            continue
        for x in range(w):
            i = row + x * 3
            if pa[i:i + 3] == pb[i:i + 3]:
                continue
            inside = box[0] <= x < box[2] and box[1] <= y < box[3]
            if outside:
                if not inside and not (cx0 <= x < cx1 and cy0 <= y < cy1):
                    n += 1
            elif inside:
                n += 1
    return n


def near(p, color, tol=14):
    return all(abs(p[i] - color[i]) <= tol for i in range(3))


def share(frame, box, color):
    px = region(frame, box)
    return sum(1 for p in px if near(p, color)) / len(px)


def white_mask(frame):
    return frozenset(i for i, p in enumerate(region(frame, SPEED_BOX)) if p[0] >= 235 and p[1] >= 235 and p[2] >= 235)


def paired_frames(base, mod, mod_keys):
    """[(key, base_path, mod_path)] of the frames whose lap timer equals a baseline frame's (the same game frame)."""
    bt = cam.run_timers(base)
    mt = cam.run_timers({k: mod[k] for k in mod_keys})
    out = []
    for k in sorted(mod_keys):
        partner = cam.pair_by_timer(bt, k, mt.get(k))
        if partner is not None:
            out.append((k, base[partner], mod[k]))
    return out


def judge_hidden(base, mod):
    return cam.judge_identity(base, mod)


def judge_log(steps):
    """The G the mod reports follows the car."""
    if len(steps) < 1500:
        return False, f"only {len(steps)} STEP lines (the race never ran?)"
    by_n = {s[0]: s for s in steps}
    top = max(s[2] for s in steps)
    if top < 20:
        return False, f"the car never got past {top:.1f} m/s"
    brake = sorted(s[3] for s in steps if s[6] > 0.5 and s[5] < 0.1 and s[2] > 10)
    if len(brake) < 10:
        return False, f"only {len(brake)} braking samples (brake > 0.5, throttle < 0.1, speed > 10 m/s)"
    med_brake = brake[len(brake) // 2]
    if med_brake > -0.25:
        return False, f"median longitudinal G while braking is {med_brake:+.2f} (expected < -0.25)"
    accel = sorted(s[3] for s in steps if s[5] > 0.9 and s[6] < 0.1 and 3 < s[2] < 25)
    if len(accel) < 10:
        return False, f"only {len(accel)} accelerating samples"
    med_accel = accel[len(accel) // 2]
    if med_accel < 0.05:
        return False, f"median longitudinal G while accelerating is {med_accel:+.2f} (expected > 0.05)"
    spike = min(s[3] for s in steps)
    if spike > -3.0:
        return False, f"the hardest deceleration is {spike:+.2f} G (the barrier hit should be below -3)"
    ok = total = 0
    for n, s in by_n.items():
        p = by_n.get(n - 4)
        if p is None or min(s[2], p[2]) < 5 or abs(s[4]) > 0.5 or abs(s[3]) > 3:
            continue
        total += 1
        expected = (s[2] - p[2]) / (4 / 60.0) / 9.81
        if abs(expected - s[3]) <= 0.3:
            ok += 1
    if total < 200 or ok / total < 0.85:
        return False, f"G matches the speed change over 4 steps in {ok}/{total} windows (need >= 85% of >= 200)"
    return True, (f"G follows the car: braking median {med_brake:+.2f}, accelerating median {med_accel:+.2f}, barrier {spike:+.2f}, "
                  f"matches the speed change in {ok}/{total} windows")


def judge_shown(base, mod, steps):
    by_vblank = {s[1]: s for s in steps}
    vblanks = sorted(by_vblank)
    pairs = paired_frames(base, mod, mod.keys())
    live = [(k, bp, mp, last_step_before(by_vblank, vblanks, k)) for k, bp, mp in pairs]
    live = [t for t in live if t[3] is not None]
    if len(live) < MIN_FRAMES:
        return False, f"only {len(live)} race frames with the panel live paired with a baseline frame"
    area = (PANEL[2] - PANEL[0]) * (PANEL[3] - PANEL[1])
    frames = {}
    for k, bp, mp, st in live:
        frames[k] = (cam.read_ppm(bp), cam.read_ppm(mp), st)
    for k, (b, m, _st) in frames.items():
        out = diff_pixels(b, m, PANEL, outside=True)
        if out:
            return False, f"frame {k}: {out} pixels differ from the baseline outside the panel (the mod must draw only in its rectangle)"
        inside = diff_pixels(b, m, PANEL)
        if inside < MIN_DARK * area:
            return False, f"frame {k}: the panel changed only {inside}/{area} pixels of its rectangle (no panel drawn?)"
    msgs = [f"{len(frames)} live frames: identical to the baseline outside the panel, the panel covers its rectangle"]
    # the speed digits differ between frames of different speeds
    keys = sorted(frames)
    masks = {k: white_mask(frames[k][1]) for k in keys}
    if min(len(m) for m in masks.values()) < 30:
        return False, "a live frame has (almost) no white text in the speed readout"
    pairs_checked = differing = 0
    for i, a in enumerate(keys):
        for b in keys[i + 1::max(1, len(keys) // 6)]:
            if abs(frames[a][2][2] - frames[b][2][2]) > 5.0:
                pairs_checked += 1
                differing += masks[a] != masks[b]
    if pairs_checked < 10 or differing < 0.98 * pairs_checked:
        return False, f"speed digits: {differing}/{pairs_checked} frame pairs 5+ m/s apart show different digits (need 98% of >= 10 pairs)"
    msgs.append(f"speed digits differ in {differing}/{pairs_checked} frame pairs 5+ m/s apart")
    # throttle and brake bars
    thr_frames = brk_frames = thr_ok = brk_ok = 0
    for k, (_b, m, st) in frames.items():
        _n, _v, _sp, _gl, _gt, th, br = st
        if th >= 0.95 and br <= 0.01:
            thr_frames += 1
            thr_ok += share(m, THR_BAR, GREEN) >= 0.8 and share(m, BRK_BAR, RED) <= 0.05
        elif br >= 0.95 and th <= 0.01:
            brk_frames += 1
            brk_ok += share(m, BRK_BAR, RED) >= 0.8 and share(m, THR_BAR, GREEN) <= 0.05
    if thr_frames < 5 or brk_frames < 3:
        return False, f"bars: only {thr_frames} full-throttle and {brk_frames} full-brake live frames"
    if thr_ok < 0.85 * thr_frames or brk_ok < 0.85 * brk_frames:
        return False, f"bars: throttle bar right in {thr_ok}/{thr_frames} full-throttle frames, brake bar in {brk_ok}/{brk_frames} full-brake frames (need 85%)"
    msgs.append(f"bars follow the pedals ({thr_ok}/{thr_frames} throttle, {brk_ok}/{brk_frames} brake)")
    return True, "; ".join(msgs)


def judge_toggle(base, mod, log):
    steps = parse_steps(log)
    by_vblank = {s[1]: s for s in steps}
    vblanks = sorted(by_vblank)
    hide_v = show_v = None
    last_v = None
    for line in log.splitlines():
        m = STEP.search(line)
        if m:
            last_v = int(m.group(2))
        elif "TOGGLE hidden" in line and hide_v is None:
            hide_v = last_v
        elif "TOGGLE visible" in line and hide_v is not None and show_v is None:
            show_v = last_v
    if hide_v is None or show_v is None:
        return False, f"the log lacks `TOGGLE hidden` then `TOGGLE visible` (hidden at {hide_v}, shown at {show_v})"
    margin = 150
    pairs = paired_frames(base, mod, mod.keys())
    area = (PANEL[2] - PANEL[0]) * (PANEL[3] - PANEL[1])
    during = before = after = 0
    for k, bp, mp in pairs:
        if last_step_before(by_vblank, vblanks, k) is None:
            continue
        b, m = cam.read_ppm(bp), cam.read_ppm(mp)
        if hide_v + margin <= k <= show_v - margin:
            if b[2] != m[2] and diff_pixels(b, m, (0, 0, 0, 0), outside=True) + diff_pixels(b, m, PANEL) > 0:
                return False, f"frame {k} (panel hidden since vblank {hide_v}) differs from the baseline"
            during += 1
        elif k <= hide_v - margin or k >= show_v + margin:
            if diff_pixels(b, m, PANEL) < MIN_DARK * area:
                return False, f"frame {k}: the panel should be showing (hidden {hide_v}..{show_v}) but its rectangle is unchanged"
            if k <= hide_v - margin:
                before += 1
            else:
                after += 1
    if during < 5 or before < 5 or after < 5:
        return False, f"frames judged: {before} before the hide, {during} while hidden, {after} after the show (need 5 each)"
    return True, f"hidden at vblank {hide_v}, shown again at {show_v}: {before} frames with the panel, {during} identical to the baseline while hidden, {after} with it back"


def main(argv):
    out = argv[2]
    failed = False

    def report(label, result):
        nonlocal failed
        ok, message = result
        print(("PASS " if ok else "FAIL ") + f"{label}: {message}")
        failed = failed or not ok

    if argv[3] == "toggle":
        base_log = read_log(os.path.join(out, "baseline", "run.log"))
        base = cam.keyed_frames(os.path.join(out, "baseline", "frames"), base_log)
        log = read_log(os.path.join(out, "toggle", "run.log"))
        mod = cam.keyed_frames(os.path.join(out, "toggle", "frames"), log)
        report("toggle", judge_toggle(base, mod, log))
        return 1 if failed else 0
    base_label = argv[3]
    base_log = read_log(os.path.join(out, base_label, "run.log"))
    base = cam.keyed_frames(os.path.join(out, base_label, "frames"), base_log)
    for label in argv[4:]:
        log = read_log(os.path.join(out, label, "run.log"))
        mod = cam.keyed_frames(os.path.join(out, label, "frames"), log)
        if not re.search(r"\[mods\] loaded:.*telemetry", log):
            report(label, (False, "the log has no `[mods] loaded: ... telemetry` line"))
            continue
        if label.startswith("hidden"):
            report(label, judge_hidden(base, mod))
        else:
            steps = parse_steps(log)
            report(label + " log", judge_log(steps))
            report(label + " frames", judge_shown(base, mod, steps))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
