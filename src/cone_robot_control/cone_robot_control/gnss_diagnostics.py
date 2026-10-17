#!/usr/bin/env python3
"""
==============================================================================
GNSS signal-quality diagnostics (ROS-free, reused by lc29h_gps_node and
scripts/analyze_gnss_log.py)
==============================================================================
Aggregates NMEA data into windowed statistics:
  - C/N0 per constellation and per band (from GSV signal ID)
  - C/N0 vs elevation (per L1 / L5 group) and vs azimuth sector (L1, el>=15)
  - Fix-quality statistics (FIX / FLOAT / SINGLE %, TTFF, losses)
  - $GxGST position standard deviations
  - Raspberry Pi health (throttling, temperature, 5 V rail)
==============================================================================
"""

import math
import re
import shutil
import subprocess
import threading
from typing import Dict, List, Optional, Set, Tuple

TALKER_NAMES = {'GP': 'GPS', 'GL': 'GLO', 'GA': 'GAL', 'GB': 'BDS', 'BD': 'BDS', 'GQ': 'QZS', 'GN': 'GNSS'}
TALKER_ORDER = ['GPS', 'GLO', 'GAL', 'BDS', 'QZS', 'GNSS']

# NMEA 4.11 GSV signal IDs -> (band label, group). Group 'L1'/'L5' are the
# comparable E1/B1/L1 and E5a/B2a/L5 classes. Unknown IDs fall back to S<id>.
# NOTE: GPS/QZSS L5 is ID 7 (L5-I) or 8 (L5-Q) depending on firmware -> both mapped.
SIGNAL_MAP = {
    'GP': {'1': ('L1C/A', 'L1'), '2': ('L1P', 'L1'), '3': ('L1M', 'L1'),
           '4': ('L2P', 'L2'), '5': ('L2C-M', 'L2'), '6': ('L2C-L', 'L2'),
           '7': ('L5', 'L5'), '8': ('L5', 'L5')},
    'GL': {'1': ('G1', 'L1'), '3': ('G2', 'L2')},
    'GA': {'1': ('E5a', 'L5'), '2': ('E5b', 'L5b'), '3': ('E5', 'L5'),
           '6': ('E1', 'L1'), '7': ('E1', 'L1')},
    'GB': {'1': ('B1I', 'L1'), '2': ('B1Q', 'L1'), '3': ('B1C', 'L1'), '4': ('B1A', 'L1'),
           '5': ('B2a', 'L5'), '6': ('B2b', 'L5b'), '7': ('B2b', 'L5b')},
    'GQ': {'1': ('L1C/A', 'L1'), '4': ('L1S', 'L1'), '5': ('L2C-M', 'L2'), '6': ('L2C-L', 'L2'),
           '7': ('L5', 'L5'), '8': ('L5', 'L5')},
}
SIGNAL_MAP['BD'] = SIGNAL_MAP['GB']

ELEV_BUCKETS = [(0, 15), (15, 30), (30, 60), (60, 91)]
ELEV_LABELS = ['0-15', '15-30', '30-60', '60-90']
AZ_LABELS = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']


def nmea_checksum(body: str) -> str:
    """XOR checksum (2 hex chars) of the text between '$' and '*'."""
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return f"{cs:02X}"


def nmea_command(body: str) -> bytes:
    """Build a complete NMEA/PAIR/PQTM command, e.g. nmea_command('PAIR062,8,1')."""
    return f"${body}*{nmea_checksum(body)}\r\n".encode('ascii')


def band_info(talker: str, signal_id: str) -> Tuple[str, str, str]:
    """Return (constellation name, band label, group) for a GSV talker/signal ID."""
    const = TALKER_NAMES.get(talker, talker)
    entry = SIGNAL_MAP.get(talker, {}).get(signal_id)
    if entry:
        return const, entry[0], entry[1]
    return const, (f"S{signal_id}" if signal_id else "S?"), 'OTH'


def elev_bucket(elev: Optional[float]) -> Optional[int]:
    if elev is None:
        return None
    for i, (lo, hi) in enumerate(ELEV_BUCKETS):
        if lo <= elev < hi:
            return i
    return None


def az_sector(az: Optional[float]) -> Optional[int]:
    if az is None:
        return None
    return int(((az + 22.5) % 360.0) // 45.0)


def parse_gsv(line: str) -> Optional[Tuple[str, str, List[Tuple[str, Optional[float], Optional[float], Optional[float]]]]]:
    """
    Parse a $xxGSV sentence -> (talker, signal_id, [(prn, elev, az, snr), ...]).
    Satellites that are in view but not tracked have snr None.
    """
    parts = line.split('*')[0].split(',')
    if len(parts) < 4:
        return None
    talker = parts[0][1:3]
    n_fields = len(parts) - 4
    signal_id = parts[-1] if n_fields % 4 == 1 else ''

    def _f(s: str) -> Optional[float]:
        try:
            return float(s)
        except ValueError:
            return None

    sats = []
    for i in range(4, 4 + (n_fields // 4) * 4, 4):
        if parts[i]:
            sats.append((parts[i], _f(parts[i + 1]), _f(parts[i + 2]), _f(parts[i + 3])))
    return talker, signal_id, sats


class _Acc:
    """Running average / minimum / distinct-signal counter."""
    __slots__ = ('n', 'total', 'min', 'keys')

    def __init__(self) -> None:
        self.n = 0
        self.total = 0.0
        self.min = 99.0
        self.keys: Set[str] = set()

    def add(self, v: float, key: str = '') -> None:
        self.n += 1
        self.total += v
        self.min = min(self.min, v)
        if key:
            self.keys.add(key)

    @property
    def avg(self) -> float:
        return self.total / self.n if self.n else 0.0


class SignalDiagnostics:
    """Thread-safe NMEA aggregator. feed() every sentence, report() every N seconds."""

    def __init__(self, start_time: float = 0.0) -> None:
        self._lock = threading.Lock()
        self.start_time = start_time
        self.fix_quality = 0
        self._reset_window()

        # Cumulative fix statistics
        self.total_epochs = 0
        self.total_fix = 0
        self.total_float = 0
        self.total_single = 0
        self.first_fix_time: Optional[float] = None
        self.fix_losses = 0
        self.last_loss_time: Optional[float] = None
        self._prev_quality: Optional[int] = None

    def _reset_window(self) -> None:
        self.w_bands: Dict[Tuple[str, str], _Acc] = {}
        self.w_group30: Dict[str, _Acc] = {}
        self.w_elev: Dict[Tuple[str, int], _Acc] = {}
        self.w_az: Dict[int, _Acc] = {}
        self.w_untracked: Set[str] = set()
        self.w_epochs = 0
        self.w_fix = 0
        self.w_float = 0
        self.w_single = 0
        self.w_gst: List[Tuple[float, float, float]] = []

    # ------------------------------------------------------------------ feed
    def feed(self, sentence: str, now: float) -> None:
        header = sentence.split(',', 1)[0]
        kind = header[-3:]
        try:
            with self._lock:
                if kind == 'GSV':
                    self._on_gsv(sentence)
                elif kind == 'GGA':
                    self._on_gga(sentence, now)
                elif kind == 'GST':
                    self._on_gst(sentence)
        except Exception:
            pass

    def _on_gsv(self, line: str) -> None:
        parsed = parse_gsv(line)
        if not parsed:
            return
        talker, sig, sats = parsed
        const, band, group = band_info(talker, sig)
        for prn, elev, az, snr in sats:
            key = f"{talker}{sig}-{prn}"
            if snr is None:
                self.w_untracked.add(key)
                continue
            self.w_bands.setdefault((const, band), _Acc()).add(snr, key)
            eb = elev_bucket(elev)
            if group in ('L1', 'L5'):
                if eb is not None:
                    self.w_elev.setdefault((group, eb), _Acc()).add(snr, key)
                if elev is not None and elev >= 30:
                    self.w_group30.setdefault(group, _Acc()).add(snr, key)
            if group == 'L1' and elev is not None and elev >= 15:
                s = az_sector(az)
                if s is not None:
                    self.w_az.setdefault(s, _Acc()).add(snr, key)

    def _on_gga(self, line: str, now: float) -> None:
        parts = line.split(',')
        if len(parts) < 7 or not parts[6].isdigit():
            return
        q = int(parts[6])
        self.fix_quality = q
        self.total_epochs += 1
        self.w_epochs += 1
        if q == 4:
            self.total_fix += 1
            self.w_fix += 1
            if self.first_fix_time is None:
                self.first_fix_time = now
        elif q == 5:
            self.total_float += 1
            self.w_float += 1
        else:
            self.total_single += 1
            self.w_single += 1
        if self._prev_quality == 4 and q != 4:
            self.fix_losses += 1
            self.last_loss_time = now
        self._prev_quality = q

    def _on_gst(self, line: str) -> None:
        parts = line.split('*')[0].split(',')
        if len(parts) < 9:
            return
        try:
            self.w_gst.append((float(parts[6]), float(parts[7]), float(parts[8])))
        except ValueError:
            pass

    # ---------------------------------------------------------------- report
    def report(self, now: float) -> List[str]:
        """Return human-readable lines for the current window and reset the window."""
        with self._lock:
            lines = []

            # --- per constellation / band
            order = sorted(self.w_bands, key=lambda k: (TALKER_ORDER.index(k[0]) if k[0] in TALKER_ORDER else 99, k[1]))
            if order:
                items = []
                for k in order:
                    a = self.w_bands[k]
                    items.append(f"{k[0]} {k[1]} {a.avg:.1f}/{a.min:.0f}/{len(a.keys)}")
                lines.append("[SIGDIAG bands avg/min/#sig] " + " | ".join(items))
                g = self.w_group30
                if 'L1' in g and 'L5' in g:
                    lines.append(
                        f"[SIGDIAG L1 vs L5, el>=30] L1 {g['L1'].avg:.1f} | L5 {g['L5'].avg:.1f} | "
                        f"L5-L1 {g['L5'].avg - g['L1'].avg:+.1f} dB"
                    )
            else:
                lines.append("[SIGDIAG bands] no tracked signals in window")

            # --- elevation / azimuth
            for grp in ('L1', 'L5'):
                cells = []
                for i, lab in enumerate(ELEV_LABELS):
                    a = self.w_elev.get((grp, i))
                    cells.append(f"{lab}deg {a.avg:.1f}({len(a.keys)})" if a else f"{lab}deg -")
                lines.append(f"[SIGDIAG elev {grp}] " + " | ".join(cells))
            cells = []
            for i, lab in enumerate(AZ_LABELS):
                a = self.w_az.get(i)
                cells.append(f"{lab} {a.avg:.1f}({len(a.keys)})" if a else f"{lab} -")
            lines.append("[SIGDIAG azimuth L1, el>=15] " + " | ".join(cells))

            # --- fix statistics
            def pct(x: int, n: int) -> float:
                return 100.0 * x / n if n else 0.0

            ttff = (f"{self.first_fix_time - self.start_time:.0f}s" if self.first_fix_time else "not yet")
            since_loss = f"{now - self.last_loss_time:.0f}s" if self.last_loss_time else "none"
            lines.append(
                f"[SIGDIAG fix] window FIX {pct(self.w_fix, self.w_epochs):.0f}% FLOAT {pct(self.w_float, self.w_epochs):.0f}% "
                f"SINGLE/other {pct(self.w_single, self.w_epochs):.0f}% | total FIX {pct(self.total_fix, self.total_epochs):.1f}% "
                f"FLOAT {pct(self.total_float, self.total_epochs):.1f}% SINGLE/other {pct(self.total_single, self.total_epochs):.1f}% "
                f"| first FIX after {ttff} | FIX losses {self.fix_losses} | since last loss {since_loss}"
            )

            # --- GST
            if self.w_gst:
                n = len(self.w_gst)
                mlat = sum(g[0] for g in self.w_gst) / n
                mlon = sum(g[1] for g in self.w_gst) / n
                malt = sum(g[2] for g in self.w_gst) / n
                horiz = math.hypot(mlat, mlon)
                note = ""
                if self.fix_quality == 4 and horiz < 0.02:
                    note = " | horizontal std < 2 cm in FIX (healthy)"
                lines.append(
                    f"[SIGDIAG gst] std lat {mlat * 100:.1f} cm lon {mlon * 100:.1f} cm alt {malt * 100:.1f} cm "
                    f"(max lat {max(g[0] for g in self.w_gst) * 100:.1f} lon {max(g[1] for g in self.w_gst) * 100:.1f}){note}"
                )
            else:
                lines.append("[SIGDIAG gst] no $GxGST received (enable with $PAIR062,8,1 - done automatically)")

            self._reset_window()
            return lines


# ---------------------------------------------------------------- Pi health
_THROTTLE_BITS = {
    0: 'UNDERVOLTAGE now', 1: 'ARM freq capped now', 2: 'THROTTLED now', 3: 'soft temp limit now',
    16: 'undervoltage occurred', 17: 'freq cap occurred', 18: 'throttling occurred', 19: 'soft temp limit occurred',
}


def _vcgencmd(*args: str) -> Optional[str]:
    exe = shutil.which('vcgencmd')
    if not exe:
        return None
    try:
        return subprocess.check_output([exe, *args], timeout=1.5, stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return None


def pi_health() -> str:
    """One-line Pi health summary: throttling flags, SoC temp, 5 V input rail (Pi 5 PMIC)."""
    parts = []
    thr = _vcgencmd('get_throttled')
    if thr and '=' in thr:
        try:
            val = int(thr.split('=')[1], 16)
            flags = [name for bit, name in _THROTTLE_BITS.items() if val & (1 << bit)]
            parts.append(f"throttled=0x{val:x} ({', '.join(flags) if flags else 'OK'})")
        except ValueError:
            pass
    temp = _vcgencmd('measure_temp')
    if temp:
        parts.append(temp.replace('temp=', 'SoC '))
    else:
        try:
            with open('/sys/class/thermal/thermal_zone0/temp') as f:
                parts.append(f"SoC {int(f.read().strip()) / 1000.0:.1f}'C")
        except Exception:
            pass
    adc = _vcgencmd('pmic_read_adc', 'EXT5V_V')
    if adc:
        m = re.search(r'=([0-9.]+)V', adc)
        if m:
            parts.append(f"EXT5V {float(m.group(1)):.2f} V")
    return ", ".join(parts) if parts else "n/a (vcgencmd not available)"
