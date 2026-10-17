# GNSS Signal-Quality Debugging (LC29H on Pi 5)

Goal: find what limits C/N0 (now ~37–38 dB-Hz, expected 40–45 in open sky) and keep RTK FIX stable while driving.

## 1. Enable the diagnostics

```bash
ros2 run cone_robot_control lc29h_gps_node --ros-args \
  --params-file src/cone_robot_control/config/robot_config.yaml \
  -p debug_signal_log:=true -p record_nmea:=true -p run_label:=A_motors_off
```

| Parameter | Default | Meaning |
|---|---|---|
| `debug_signal_log` | false | Adds `[SIGDIAG ...]` lines every `debug_signal_log_period_s` (5 s); also enables `$GxGST` via `$PAIR062,8,1` |
| `record_nmea` | false | Writes `<unix_time> <NMEA sentence>` to `record_dir/nmea_<date>_<label>.log` |
| `record_dir` | `~/gnss_logs` | Output folder |
| `run_label` | `''` | Tag in the file name (A/B/C runs) |

The normal `[RTK ACTIVE] ...` line is unchanged. Detailed output:

```
[SIGDIAG bands avg/min/#sig] GPS L1C/A 41.2/35/9 | GPS L5 38.0/30/8 | GAL E1 ... | BDS B2a ...
[SIGDIAG L1 vs L5, el>=30]   L1 42.0 | L5 39.5 | L5-L1 -2.5 dB
[SIGDIAG elev L1] 0-15deg 36.0(6) | 15-30deg 39.5(8) | 30-60deg 42.1(9) | 60-90deg 44.0(4)
[SIGDIAG elev L5] ...
[SIGDIAG azimuth L1, el>=15] N 41.0(3) | NE 40.2(2) | ...
[SIGDIAG fix] window FIX 100% ... | total FIX 87% | first FIX after 142s | FIX losses 2 | since last loss 63s
[SIGDIAG gst] std lat 0.8 cm lon 0.9 cm alt 2.0 cm ... | horizontal std < 2 cm in FIX (healthy)
[SIGDIAG pi]  throttled=0x0 (OK), SoC 55.1'C, EXT5V 5.08 V     (every 30 s)
```

Notes:
- Band labels come from the GSV signal ID. Mapping is in `SIGNAL_MAP` in `gnss_diagnostics.py` (NMEA 4.11). If an ID shows as `S<n>`, send me a raw GSV line and I'll fix the map.
- `$PAIR062,8,1` is not saved to flash; the node re-sends it on every start.

### Raw checks on the Pi (node stopped, or in a second terminal on a different tool)
```bash
sudo stty -F /dev/ttyAMA0 115200 raw -echo
sudo timeout 5 cat /dev/ttyAMA0 | grep -E 'GSV|GST'
vcgencmd get_throttled            # want 0x0
vcgencmd measure_temp
vcgencmd pmic_read_adc EXT5V_V    # Pi 5: want >= 4.9 V
ros2 param list /lc29h_gps_node
```

## 2. Analyze

```bash
python3 scripts/analyze_gnss_log.py ~/gnss_logs/nmea_*_A_*.log ~/gnss_logs/nmea_*_B_*.log ~/gnss_logs/nmea_*_C_*.log --skip 60
python3 scripts/analyze_gnss_log.py A.log B.log C.log --save compare   # PNG: C/N0 vs elevation + skyplots
```
Prints per-band C/N0, L1 vs L5, C/N0 per elevation, FIX/FLOAT %, losses, GST. `--plot` needs `pip install matplotlib numpy`.

## 3. A/B test protocol (same spot, same time of day, >= 5 min each, discard first 60 s with `--skip 60`)

| Run | Setup | `run_label` |
|---|---|---|
| A | Current setup, motors/camera/Wi-Fi-heavy tasks off | `A_base` |
| B | Everything on, motors running (or driving) | `B_all_on` |
| C | Antenna on a large metal sheet on the ground, >= 3 m from robot (baseline) | `C_ground_sheet` |
| D (opt.) | Like A but: USB3 devices unplugged / HDMI off / ferrites added | `D_<change>` |

Change only one thing per run. Satellite geometry changes slowly, so compare elevation-binned values, not only the overall average.

## 4. Interpretation

| Observation | Likely cause |
|---|---|
| L5 much weaker than L1 (L5-L1 below about -4 dB at el>=30) | Antenna/LNA/bias issue (L1-only or poor L5 antenna, antenna power) |
| Uniformly weak across bands and elevations | Noise (RFI) or small/uncentered ground plane |
| Weak only below 15° | Normal |
| Specific azimuths weak | Obstruction/reflection by robot body or mast |
| B worse than A | RFI from motors/drivers/Wi-Fi/camera |
| C is 42+ but A/B are 37–38 | The robot is still the problem (RFI or shadowing) |
| C also ~37–38 | Antenna/cable/LNA, not the robot |
| `throttled` flags UNDERVOLTAGE or EXT5V < 4.9 V | Power quality; fix supply before blaming RF |
| FIX losses coincide with motor start | Motor/driver noise or brownout |

Healthy open sky reference: L1 el>=30 ~ 42–46, L5 ~ 40–44, FIX std < 2 cm.

## 5. $PQTM / noise indicators

The code already uses `$PQTMCFGRCVRMODE,W,1` (rover mode). I could not verify from the repo or any datasheet that the LC29H(DA) exposes AGC / jamming / noise-floor messages, so none are logged. Check what your firmware supports:

```bash
# send, then read the reply on the same port (stop the node first)
printf '$PQTMVERNO*58\r\n' | sudo tee /dev/ttyAMA0
```
and look in Quectel's *LC29H Series GNSS Protocol Specification* for `PQTM` messages. If you find an AGC/jamming one, paste it here and I'll add it. Until then, the L1/L5 delta and C/N0 vs elevation are the best noise indicators.

## 6. Hardware fixes (cost/impact order)

1. **Ferrites** (clamp-on, 31/43 mix) on the antenna coax near the HAT and on the 5 V/motor leads, 2–3 turns — a few €, often 1–3 dB.
2. **Move/shield the noise source**: route antenna coax away from motor wires, cross at 90°; keep camera/USB3 cables away. Free.
3. **Clean power**: dedicated 5 V buck (>= 5 A, official-spec) for the Pi, separate from motor supply; add LC filter/bulk caps on motor drivers.
4. **Mast +30 cm** above the robot with the antenna centered on the plate: reduces body shadowing/multipath. Cheap.
5. **Larger, centered ground plane** (>= 20–25 cm, antenna in the middle). Cheap.
6. **Verify antenna**: must be multi-band L1+L5 active (LNA 28–40 dB). Check model; try a known-good antenna. HAT must supply bias power to active antennas (Waveshare HAT: check antenna/jumper settings). 20–60 €.
7. **Shield the Pi** (metal case / foil-lined enclosure), HDMI off (`tvservice`/headless), USB3 devices to USB2 or moved away.
