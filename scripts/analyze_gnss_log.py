#!/usr/bin/env python3
"""
Analyze raw NMEA recordings (from lc29h_gps_node record_nmea:=true, or any NMEA log).

Accepts lines "<unix_time> $GNGGA,..." or plain "$GNGGA,...". Multiple files are
compared side by side (one column per run).

Usage:
  python3 scripts/analyze_gnss_log.py runA.log runB.log runC.log
  python3 scripts/analyze_gnss_log.py runA.log runB.log --plot            # show figures
  python3 scripts/analyze_gnss_log.py runA.log runB.log --save out_prefix # PNGs, no GUI
  python3 scripts/analyze_gnss_log.py runA.log --skip 30                  # ignore first 30 s
"""

import argparse
import os
import sys
from collections import defaultdict

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'cone_robot_control'))
from cone_robot_control.gnss_diagnostics import (  # noqa: E402
    ELEV_LABELS, TALKER_ORDER, band_info, elev_bucket, parse_gsv)


class Run:
    def __init__(self, path: str, skip_s: float) -> None:
        self.name = os.path.basename(path)
        self.bands = defaultdict(list)       # (const, band) -> [snr]
        self.elev = defaultdict(list)        # (group, bucket) -> [snr]
        self.group30 = defaultdict(list)     # group -> [snr] (elev >= 30)
        self.sky = []                        # (az, elev, snr, group)
        self.epochs = defaultdict(int)       # fix quality -> count
        self.first_fix = None
        self.losses = 0
        self.gst = []
        self.samples = []                    # (t_rel, group, elev, snr)
        self.fixes = []                      # (t_rel, quality)
        self.t0 = None
        self.t1 = None
        prev = None
        n_nots = 0
        with open(path, errors='ignore') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                ts = None
                if not line.startswith('$') and ' ' in line:
                    first, _, rest = line.partition(' ')
                    try:
                        ts = float(first)
                        line = rest
                    except ValueError:
                        pass
                if not line.startswith('$'):
                    continue
                if ts is not None:
                    if self.t0 is None:
                        self.t0 = ts
                    if ts - self.t0 < skip_s:
                        continue
                    self.t1 = ts
                else:
                    n_nots += 1
                kind = line.split(',', 1)[0][-3:]
                if kind == 'GSV':
                    self._gsv(line, (ts - self.t0) if (ts is not None and self.t0 is not None) else None)
                elif kind == 'GGA':
                    p = line.split(',')
                    if len(p) > 6 and p[6].isdigit():
                        q = int(p[6])
                        if ts is not None and self.t0 is not None:
                            self.fixes.append((ts - self.t0, q))
                        self.epochs[q] += 1
                        if q == 4 and self.first_fix is None and ts is not None and self.t0 is not None:
                            self.first_fix = ts - self.t0
                        if prev == 4 and q != 4:
                            self.losses += 1
                        prev = q
                elif kind == 'GST':
                    p = line.split('*')[0].split(',')
                    try:
                        self.gst.append((float(p[6]), float(p[7]), float(p[8])))
                    except (ValueError, IndexError):
                        pass

    def _gsv(self, line, t_rel=None):
        parsed = parse_gsv(line)
        if not parsed:
            return
        talker, sig, sats = parsed
        const, band, group = band_info(talker, sig)
        for _, elev, az, snr in sats:
            if snr is None:
                continue
            self.bands[(const, band)].append(snr)
            if t_rel is not None:
                self.samples.append((t_rel, group, elev, snr))
            if elev is not None and az is not None:
                self.sky.append((az, elev, snr, group))
            eb = elev_bucket(elev)
            if group in ('L1', 'L5'):
                if eb is not None:
                    self.elev[(group, eb)].append(snr)
                if elev is not None and elev >= 30:
                    self.group30[group].append(snr)

    def fix_pct(self, q):
        n = sum(self.epochs.values())
        return 100.0 * self.epochs.get(q, 0) / n if n else 0.0


def avg(v):
    return sum(v) / len(v) if v else None


def fmt(v, n=None):
    return "   -   " if v is None else (f"{v:5.1f}({n})" if n is not None else f"{v:5.1f}")


def table(title, rows, runs):
    print(f"\n== {title}")
    w = max(len(r[0]) for r in rows) + 2 if rows else 10
    print(" " * w + "".join(f"{r.name[:18]:>20}" for r in runs))
    for label, vals in rows:
        print(f"{label:<{w}}" + "".join(f"{v:>20}" for v in vals))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('files', nargs='+')
    ap.add_argument('--skip', type=float, default=0.0, help='skip first N seconds (needs timestamps)')
    ap.add_argument('--plot', action='store_true')
    ap.add_argument('--save', metavar='PREFIX')
    ap.add_argument('--timeline', type=float, metavar='SEC', help='print L1/L5 C/N0 (el>=30) and fix per SEC-second bin')
    a = ap.parse_args()

    runs = [Run(p, a.skip) for p in a.files]

    # Per band
    keys = sorted({k for r in runs for k in r.bands},
                  key=lambda k: (TALKER_ORDER.index(k[0]) if k[0] in TALKER_ORDER else 99, k[1]))
    rows = [(f"{c} {b}", [f"{avg(r.bands[(c, b)]):.1f} (n={len(r.bands[(c, b)])})" if (c, b) in r.bands else "-" for r in runs])
            for c, b in keys]
    table("C/N0 [dB-Hz] avg per constellation/band (n = samples)", rows, runs)

    # L1 vs L5
    rows = []
    for g in ('L1', 'L5'):
        rows.append((f"{g} el>=30", [fmt(avg(r.group30[g])) if r.group30.get(g) else "-" for r in runs]))
    rows.append(("L5-L1 (dB)", [f"{avg(r.group30['L5']) - avg(r.group30['L1']):+.1f}" if r.group30.get('L1') and r.group30.get('L5') else "-" for r in runs]))
    table("L1 vs L5 at elevation >= 30 deg", rows, runs)

    # Elevation
    rows = []
    for g in ('L1', 'L5'):
        for i, lab in enumerate(ELEV_LABELS):
            rows.append((f"{g} {lab} deg", [f"{avg(r.elev[(g, i)]):.1f} (n={len(r.elev[(g, i)])})" if r.elev.get((g, i)) else "-" for r in runs]))
    table("C/N0 vs elevation", rows, runs)

    # Fix stats
    rows = [
        ("FIX %", [f"{r.fix_pct(4):.1f}" for r in runs]),
        ("FLOAT %", [f"{r.fix_pct(5):.1f}" for r in runs]),
        ("SINGLE/other %", [f"{100 - r.fix_pct(4) - r.fix_pct(5):.1f}" for r in runs]),
        ("first FIX (s)", [f"{r.first_fix:.0f}" if r.first_fix is not None else "n/a" for r in runs]),
        ("FIX losses", [str(r.losses) for r in runs]),
        ("duration (s)", [f"{r.t1 - r.t0:.0f}" if r.t0 and r.t1 else "n/a" for r in runs]),
        ("GST std lat/lon (cm)", [f"{avg([g[0] for g in r.gst]) * 100:.1f}/{avg([g[1] for g in r.gst]) * 100:.1f}" if r.gst else "n/a" for r in runs]),
    ]
    table("Fix quality", rows, runs)

    if a.timeline:
        for r in runs:
            print(f"\n== Timeline {r.name} (bin {a.timeline:.0f} s, t=0 at first line; L1/L5 at el>=30)")
            print(f"{'t[s]':>6} {'L1':>6} {'L5':>6} {'L5-L1':>7} {'FIX%':>5} {'FLOAT%':>6}")
            if not r.samples:
                print("  (no timestamped samples)")
                continue
            nb = int(max(s[0] for s in r.samples) // a.timeline) + 1
            for b in range(nb):
                lo, hi = b * a.timeline, (b + 1) * a.timeline
                l1 = [s[3] for s in r.samples if lo <= s[0] < hi and s[1] == 'L1' and s[2] is not None and s[2] >= 30]
                l5 = [s[3] for s in r.samples if lo <= s[0] < hi and s[1] == 'L5' and s[2] is not None and s[2] >= 30]
                fx = [q for t, q in r.fixes if lo <= t < hi]
                d = f"{avg(l5) - avg(l1):+7.1f}" if l1 and l5 else "      -"
                pf = f"{100.0 * fx.count(4) / len(fx):5.0f}" if fx else "    -"
                pl = f"{100.0 * fx.count(5) / len(fx):6.0f}" if fx else "     -"
                print(f"{lo:6.0f} {fmt(avg(l1)):>6} {fmt(avg(l5)):>6} {d} {pf} {pl}")

    if not (a.plot or a.save):
        return
    try:
        import matplotlib
        if a.save:
            matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print("\n(matplotlib/numpy not installed: pip install matplotlib numpy)")
        return

    # C/N0 vs elevation (5 deg bins), L1 solid / L5 dashed
    fig, ax = plt.subplots(figsize=(7, 4.5))
    for r in runs:
        for g, ls in (('L1', '-'), ('L5', '--')):
            pts = [(e, s) for az, e, s, gg in r.sky if gg == g]
            if not pts:
                continue
            e = np.array([p[0] for p in pts])
            s = np.array([p[1] for p in pts])
            bins = np.arange(0, 95, 5)
            xs, ys = [], []
            for lo in bins[:-1]:
                m = (e >= lo) & (e < lo + 5)
                if m.sum() >= 3:
                    xs.append(lo + 2.5)
                    ys.append(s[m].mean())
            ax.plot(xs, ys, ls, marker='o', ms=3, label=f"{r.name} {g}")
    ax.set_xlabel('Elevation [deg]')
    ax.set_ylabel('C/N0 [dB-Hz]')
    ax.grid(True, alpha=.3)
    ax.legend(fontsize=7)
    ax.set_title('C/N0 vs elevation (solid L1, dashed L5)')
    if a.save:
        fig.savefig(f"{a.save}_cn0_elev.png", dpi=130)

    # Skyplots (L1 + L5), colour = C/N0
    fig2, axes = plt.subplots(1, len(runs), subplot_kw={'projection': 'polar'}, figsize=(5 * len(runs), 5), squeeze=False)
    for ax, r in zip(axes[0], runs):
        if r.sky:
            az = np.radians([p[0] for p in r.sky])
            el = [90 - p[1] for p in r.sky]
            sc = ax.scatter(az, el, c=[p[2] for p in r.sky], cmap='RdYlGn', vmin=25, vmax=48, s=6)
            fig2.colorbar(sc, ax=ax, shrink=.7, label='C/N0 [dB-Hz]')
        ax.set_theta_zero_location('N')
        ax.set_theta_direction(-1)
        ax.set_rlim(0, 90)
        ax.set_title(r.name, fontsize=9)
    if a.save:
        fig2.savefig(f"{a.save}_skyplot.png", dpi=130)
        print(f"\nSaved {a.save}_cn0_elev.png and {a.save}_skyplot.png")
    else:
        plt.show()


if __name__ == '__main__':
    main()
