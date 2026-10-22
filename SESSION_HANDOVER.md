# DD-BOAT 11 (NAVYSEALS) — SESSION HANDOVER & STATE RESUMPTION
> **FOR AGENT / USER RESUMING IN A NEW CHAT:**  
> Read this file first! It provides the complete state of the project, verified hardware milestones, discovered constraints, and the exact next action so you do NOT restart from the beginning.

---

## 1. Project Context & Hardware Identity

* **Event:** BEST 2026 — *ROBOTson Crusoe: Let's explore the lost city*  
* **Location:** Guerlédan Lake, Brittany (ENSTA Bretagne · Lab-STICC · ROBEX)  
* **Instructors:** Prof. Luc Jaulin, Prof. Benoît Zerr, Antoine Morvan  
* **Team Name:** `NavySeals`  
* **Target Vessel:** DD-Boat 11 (Differential-Drive Boat with dual brushless thrusters)  

### Network & Connection Details
* **Wi-Fi SSID:** `DARTAP` | **Password:** `ABACA`
* **Ground Station Network Architecture:** Dual routing (Laptop Wi-Fi connected to `DARTAP` for boat SSH; phone USB-C tethering providing internet connectivity).
* **Boat Static IP:** `172.20.25.211`
* **SSH Credentials:** `ssh ue32@172.20.25.211` (or `best@172.20.25.211`), Password: `ue32` / `best`
* **Remote Code Directory:** `/home/ue32/NavySeals` (with symlink to `/home/best/drivers-ddboat-v2`)
* **Local Code Directory:** `d:\Personal_Projects\BEST_Breast\NavySeals\`

---

## 2. Hard Technical Constraints (DO NOT RE-DEBUG)

Every script running on the boat must comply with these proven constraints:

1. **Python Version is 3.5.3 (Raspbian Stretch):**
   * **STRICT RULE: NO F-STRINGS!** Writing `f"..."` crashes with a fatal `SyntaxError`. Always use `.format()` or `%` string interpolation.
2. **Missing `scipy`:**
   * `scipy` is NOT installed on Boat 11. All matrix operations, inversions, and norm calculations must use pure `numpy` and `numpy.linalg`.
3. **Windows OpenSSH Terminal Freeze:**
   * Printing carriage returns (`\r`) in tight streaming loops causes the Windows OpenSSH client terminal to hang/freeze.
   * Always print scrolling lines with indexes (e.g. `[{:4.1f}s] Heading: ...`) instead of `\r` overwriting.
4. **No Physical RC Transmitter Needed:**
   * When no radio transmitter is transmitting, the Arduino MEGA 2560 on Boat 11 **defaults to Autonomous Mode** automatically.
   * Software timeouts and `stop.py` provide full safety cutoff.
5. **Differential Steering Sign (Prof. Luc Jaulin's Formula):**
   * With Marine Clockwise Compass ($0^\circ=\text{N}, 90^\circ=\text{E}, 180^\circ=\text{S}, 270^\circ=\text{W}$):
   * $$e = \text{sawtooth}(\theta_{\text{target}} - \theta)$$
   * $$u_{\text{left}} = v_0 + K_p \cdot e$$
   * $$u_{\text{right}} = v_0 - K_p \cdot e$$
   * *(A positive error $e > 0$ means the boat must turn Right $\implies$ Left motor increases, Right motor decreases).*

---

## 3. Current Hardware & Syllabus Status (100% Passed)

All 27 pages of the introductory curriculum ([`best_ddboat.pdf`](file:///d:/Personal_Projects/BEST_Breast/best_ddboat.pdf)) are **fully verified on the physical boat**:

| Component / Task | Script Used | Result on Boat 11 | Status |
| :--- | :--- | :--- | :---: |
| **Wi-Fi & SSH Connection** | `ping`, `ssh` | Ping $<20\text{ms}$, SSH stable at `172.20.25.211` | **PASSED** |
| **IMU Sensors (Pololu 9-DOF)** | `imu9_driver_v2.py` | LIS3MDL (mag) and LSM6DS33 (accel/gyro) active | **PASSED** |
| **Compass Calibration** | `calib.py` | Hard/soft iron parameters saved in `calib_boat11.json`. Heading verified across N, E, S, W | **PASSED** |
| **Propeller Motor Drive** | `motor_test.py` | Left=100, Right=-100 in-place turn ran for 2s, stopped `(0, 0)` | **PASSED** |
| **GPS Georeferencing** | `gps.py` | Locked fix at $48.198795^\circ\text{ N}, -3.014180^\circ\text{ W}$ ($65.9\text{m}$ from Guerlédan reference). Saved `gps_data.kml` | **PASSED** |
| **Bench Heading Control** | `heading_hold.py` | Proportional controller tested; dynamic motor steering verified | **PASSED** |
| **Digital Twin Simulator** | `simu_page13.py` / `digital_twin_exercises.py` | Dubins model, discrete Euler integration, and slides 13–14 verified | **PASSED** |
| **Autonomous Waypoint Mission (Lake Trial)** | `waypoint_mission.py` | 15m $\times$ 15m box navigated on Guerlédan Lake; 4/4 waypoints validated; returned to launch home (loop closure to 2.4m); track saved in `mission_track.kml` | **PASSED (In-Water Verified)** |
| **Regatta Competition Loop** | `regatta_loop.py` | Exercise 13 CCW dual-gate crossing, 5-min race timer, real gate-crossing scoring, auto-KML | **BUILT (Ready for Friday)** |

---

## 4. Complete Script Inventory in `NavySeals/`

All scripts are written, tested, Python 3.5.3 compatible, and present in both local `NavySeals/` and boat `~/NavySeals/`:

1. **`stop.py`** — Emergency all-stop cutoff: immediately forces motors to `(0, 0)`.
2. **`motor_test.py`** — Official Page 27 propeller verification test (2s in-place turn).
3. **`calib.py`** — Compass calibration (4-pose / 360° circle fit) & live heading verification (`T` key).
4. **`gps.py`** — Official Pages 11–22 GPS telemetry, UTM Zone 30N projection, Guerlédan distance/bearing calculator, and KML logger (`gps_data.kml`).
5. **`heading_hold.py`** — Autonomous straight-line compass heading hold with Prof. Luc Jaulin's sawtooth steering law.
6. **`control.py`** — **The "No-Kayaking" Return-to-Home Controller** based on the professor's template. Supports `boomerang` (out and back) and `rth_gps` (automatic return to launch GPS coordinates).
7. **`rc_teleop.py`** — Wireless manual remote control via laptop keyboard Arrow Keys (`▲`, `▼`, `◄`, `►`) / `WASD` over SSH with an automatic 0.5s deadman watchdog.
8. **`waypoint_mission.py`** — Autonomous GPS waypoint navigation for the lake course and Friday's Regatta.
9. **`battery_probe.py`** — Diagnostic probe that checks Arduino status (`S;`, `E;`), Pi PMIC throttling (`vcgencmd`), and I2C ADC devices for battery voltage.
10. **`battery_monitor.py`** — Software Coulomb-counter & State-of-Charge (SoC) monitor tracking mAh consumed and remaining run time.

---

## 5. Immediate Next Steps When Reconnecting to Boat 11

When starting a new session or putting the boat on the lake, follow this sequence:

### Step 1: Sync Any Updates from Laptop to Boat (PowerShell)
```powershell
scp d:\Personal_Projects\BEST_Breast\NavySeals\battery.py ue32@172.20.25.211:~/NavySeals/
scp d:\Personal_Projects\BEST_Breast\NavySeals\battery_monitor.py ue32@172.20.25.211:~/NavySeals/
scp d:\Personal_Projects\BEST_Breast\NavySeals\battery_probe.py ue32@172.20.25.211:~/NavySeals/
scp d:\Personal_Projects\BEST_Breast\NavySeals\heading_hold.py ue32@172.20.25.211:~/NavySeals/
scp d:\Personal_Projects\BEST_Breast\NavySeals\control.py ue32@172.20.25.211:~/NavySeals/
scp d:\Personal_Projects\BEST_Breast\NavySeals\rc_teleop.py ue32@172.20.25.211:~/NavySeals/
```

### Step 2: First Water Trial — Safe Boomerang (No Kayaking!)
Place the boat in the water pointing away from the shore into open water (e.g. heading $120^\circ$):
```bash
cd ~/NavySeals
python3 control.py boomerang 120 40 30
```
* Cruises out for 15s, turns $180^\circ$ automatically, and returns right to the shore!

### Step 3: Keyboard RC Manual Driving Test
```bash
python3 rc_teleop.py
```
* Drive with Arrow Keys from your laptop keyboard. Release keys to stop.

### Step 4: Google Earth Trajectory Visualization
On laptop PowerShell, pull the KML file:
```powershell
scp ue32@172.20.25.211:~/NavySeals/gps_data.kml .
```
Import `gps_data.kml` into **[earth.google.com](https://earth.google.com/web/)** to see the satellite map of the boat's trajectory on Guerlédan Lake.

### Step 5: Thursday Digital Twin & Luenberger Observer Session (Antoine Morvan)
Follow the 5-step curriculum documented in [`DIGITAL_TWIN.md`](file:///d:/Personal_Projects/BEST_Breast/DIGITAL_TWIN.md):
1. **Model Baseline ($L=0$):** Run open-loop forward integration to gauge drift rate against lake wind/current.
2. **Sensor Noise Baseline:** Analyze raw measurement jitter from GPS and compass.
3. **Observer Tuning ($L$):** Tune gain matrix $L = \text{diag}([l_1, l_1, l_2])$ (baseline $[2.0, 2.0, 2.0]$) to balance model filtering vs sensor tracking.
4. **Packet Loss Resilience:** Test UDP timeout fallback ($\text{err} = \mathbf{0}$) simulating dropped packets.
5. **Hydrodynamic Gain Identification ($k_1, k_2$):** Measure steady-state speed $v$ and yaw rate $\omega$ for known inputs $(u_1, u_2)$ to refine boat model gains.
