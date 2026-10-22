# DD-BOAT 11 : COMPLETE PROJECT RECORD & RUNBOOK
**BEST 2026 — ROBOTson Crusoe: Let's explore the lost city**  
*Guerlédan Lake, Brittany | ENSTA Bretagne · Lab-STICC · ROBEX*  
**Team:** NavySeals · **Target Platform:** DD-Boat 11 (`172.20.25.211`)  
**Lead Professors:** Prof. Luc Jaulin, Prof. Benoît Zerr, Antoine Morvan  

---

## 1. Executive Summary & Architecture

The **DD-Boat (Differential-Drive Boat)** is an autonomous surface vessel (ASV) driven by two submerged brushless thrusters. Steering is achieved entirely via differential thrust (varying speed between left and right motors) rather than a mechanical rudder.

### Hardware Specifications (DD-Boat 11)
* **Single Board Computer (SBC):** Raspberry Pi 3 Model B (Raspbian Stretch, Linux Kernel 4.9/4.14, Python 3.5.3).
* **Microcontroller:** Arduino MEGA 2560 (connected via USB serial `/dev/ttyACM0` at 115200 baud). Translates motor commands $[-255, 255]$ into PWM pulses for the ESCs.
* **Propulsion:** Dual brushless motors paired with Electronic Speed Controllers (ESCs).
* **IMU (Inertial Measurement Unit):** Pololu 9-DOF IMU v5 connected via I2C Bus 1:
  * Magnetometer: ST LIS3MDL (`0x1e`)
  * Accelerometer + Gyroscope: ST LSM6DS33 (`0x6b`)
* **GPS Receiver:** NMEA 0183 GPS receiver streaming GPGLL sentences at 1 Hz via `/dev/ttyAMA0` / USB serial.
* **Batteries:** Dual 2S LiPo batteries ($7.4\text{ V}$ nominal). Operational cutoff $> 7.5\text{ V}$; long missions require $> 8.0\text{ V}$.
* **Remote Controller (RC):**
  * Base-3 Boat ID Code: Boat $11 \implies \text{Code } 10_{10} = 101_3 \implies \mathbf{A=1,\ B=0,\ D=1}$.
  * Switch F: **UP** = Manual Radio Control, **DOWN** = Autonomous Python Control.

### Network Configuration
* **Wi-Fi SSID:** `DARTAP`
* **Wi-Fi Password:** `ABACA`
* **Boat IP Address:** `172.20.25.211` (Static: `172.20.25.2xx` where `xx = 11`)
* **SSH Credentials:** `ue32@172.20.25.211` / `best@172.20.25.211` (Password: `ue32` / `best`)
* **Boat Working Directory:** `/home/ue32/NavySeals` (with symlink to `/home/best/drivers-ddboat-v2`)

---

## 2. Critical Technical Constraints & Lessons Learned

To ensure rapid execution and zero runtime bugs during field tests, keep these hard constraints in mind:

| Constraint | Problem Identified | Permanent Resolution |
| :--- | :--- | :--- |
| **Python 3.5.3** | Python on Raspbian Stretch is version 3.5.3. Using f-strings (`f"..."`) causes immediate `SyntaxError`. | **Strict rule:** Always use `.format()` or `%` string formatting. Never write f-strings. |
| **Missing `scipy`** | `scipy` is NOT installed on the boat (`ImportError: No module named 'scipy'`). | Replaced all `scipy.linalg` (inv, sqrtm, norm) with pure `numpy.linalg` and algebraic circle/ellipsoid fits. |
| **Windows SSH Buffer Freeze** | Carriage return `\r` cursor resets in live loops cause the Windows OpenSSH client to hang/freeze. | Replaced in-place overwriting with indexed scrolling lines (`[#0001] Heading: ...`). |
| **Marine Heading vs Trig Angle** | Standard `atan2(y, x)` outputs mathematical trigonometric radians ($0^\circ = \text{East}$, counter-clockwise). Marine compasses require $0^\circ = \text{North}$, clockwise. | Formula applied: $\psi_{\text{geo}} = (90^\circ - \text{atan2}(m_y, m_x) \cdot \frac{180}{\pi}) \pmod{360^\circ}$. |
| **ESC Startup Delay** | ESCs will ignore motor commands sent immediately after boot. | Always wait for the ESC melody to play 3 times before sending commands to `arduino_driver_v2`. |
| **No Radio Remote Needed** | Pages 6-7 describe 3 shared transmitters for safety override. | When no transmitter is active, Arduino **defaults to Autonomous Mode** automatically. Software timeouts and `stop.py` provide safety cutoff. |

---

## 3. Chronological Activity & Milestone Log

### Phase 1: Setup & Connection
- **Action:** Established dual-routing network connection on ground station laptop (Wi-Fi connected to `DARTAP` at `172.20.26.64`; cellular USB tethering providing internet).
- **Action:** Verified ping and opened SSH session into `ue32@172.20.25.211`.
- **Action:** Created team directory `/home/ue32/NavySeals` and symlinked `/home/best/drivers-ddboat-v2`.

### Phase 2: Smoke Test Verification
- **Action:** Executed [`boat11_smoke_test.py`](file:///D:/Personal_Projects/BEST_Breast/NavySeals/boat11_smoke_test.py) on Boat 11.
- **Outcome:**
  - IMU communication verified (Accel, Gyro, Mag raw readings streaming).
  - Arduino connected on `/dev/ttyACM0`.
  - GPS receiver confirmed functional.

### Phase 3: Compass Calibration (`calib.py`)
- **Initial Issue:** Running original `calib.py` failed with `ImportError: No module named 'scipy'`.
- **Fix:** Refactored calibration code to use pure `numpy.linalg` eigenvalue decomposition and least-squares circle fitting for hard-iron offset vector $\mathbf{b}$ and soft-iron scaling matrix $\mathbf{A}^{-1}$:
  $$\mathbf{m}_{\text{calib}} = \mathbf{A}^{-1} (\mathbf{m}_{\text{raw}} - \mathbf{b})$$
- **Orientation Issue:** Initial heading stream reported East when pointing West.
- **Fix:** Corrected coordinate sign convention to marine geographic heading. Fixed terminal freeze by switching to indexed scrolling log.
- **Outcome:** Full 360° compass calibration completed. Parameters saved to `calib_boat11.json`. Heading verified accurate across North, East, South, and West.

### Phase 4: Hardware Validation Completed (`best_ddboat.pdf` 100% Passed)
- **Motor Test (Page 27):** Executed `motor_test.py`. Left=100, Right=-100 in-place turn ran for 2.0s and stopped cleanly.
- **GPS Acquisition & Georeferencing (Pages 11–22):** Executed `gps.py`. Live coordinates locked (`lat=48.198795, lon=-3.014180`), UTM distance $65.9\text{ m}$ from Guerlédan reference point calculated, and `gps_data.kml` generated.
- **Outcome:** **All 27 pages of `best_ddboat.pdf` are officially completed and verified!**

### Phase 5: Autonomous Lake Navigation (Current Phase)
- **Heading Hold Dry Run (20s):** Executed `heading_hold.py` on Boat 11. Initial heading was $88.4^\circ - 89.6^\circ$ tracking target $90^\circ$ at base speed 45.
- **Steering Sign Correction:** Log analysis of large heading deviations ($180^\circ$) revealed a sign inversion in the differential steering law. Corrected both `heading_hold.py` and `waypoint_mission.py` to match Prof. Luc Jaulin's control law ($u_{\text{left}} = v_0 + K_p \cdot e$, $u_{\text{right}} = v_0 - K_p \cdot e$).
- **Next Action:** Push updated `heading_hold.py` to Boat 11 and execute on-water trial.

### Phase 6: Professor's `control.py` & The "No-Kayaking" Return-to-Home (RTH)
- **Problem Statement:** If DD-Boat 11 sails far out into Guerlédan Lake and stops, the team must kayak out to recover it.
- **Solution:** Integrated Prof. Luc Jaulin's `control.py` skeleton with two autonomous retrieval strategies:
  1. **Boomerang Mode (Out-and-Back):** Cruises outward on heading $\theta_{\text{out}}$ (e.g. $120^\circ$) for $T/2$ seconds, then flips heading by $180^\circ$ ($\theta_{\text{home}} = (\theta_{\text{out}} + 180^\circ) \pmod{360^\circ}$) to cruise directly back to the dock/launch point for $T/2$ seconds.
  2. **GPS Return-to-Home Mode:** Automatically locks launch coordinates $(x_0, y_0)$, sails out to a geofenced distance, then navigates closed-loop back to within $2.0\text{ m}$ of launch point and stops.

### Phase 7: Wireless Keyboard RC Teleoperation (`rc_teleop.py`)
- **Purpose:** Full manual remote control of DD-Boat 11 from the laptop keyboard over SSH without requiring a physical radio transmitter.
- **Controls:** Arrow Keys (`▲`, `▼`, `◄`, `►`) and `WASD` for forward, reverse, and pivot turning; `Space` for brake stop; `+`/`-` for power adjustment.
- **Safety Deadman Watchdog:** If no key is detected for $>0.5\text{s}$, thrusters automatically disarm to `(0, 0)` preventing boat runaway if Wi-Fi drops.

### Phase 8: Digital Twin & Non-Linear Luenberger Observer (`digital_twin_presentation.pdf`)
- **Instructor:** Antoine Morvan (ENSTA, Thursday Oct 22, 2026). Reference: [`digital_twin_presentation.pdf`](file:///d:/Personal_Projects/BEST_Breast/digital_twin_presentation.pdf) and [`DIGITAL_TWIN.md`](file:///d:/Personal_Projects/BEST_Breast/DIGITAL_TWIN.md).
- **Core Concept:** Real-time bi-directional synchronization between physical boat (`172.20.25.211`) and virtual Python simulation (`roblib`) over a lightweight UDP Wi-Fi socket link.
- **Kinematic Model (Dubins DD-Boat):**
  $$\begin{cases} \dot{x} = v \cos\theta \\ \dot{y} = v \sin\theta \\ \dot{\theta} = \omega \end{cases}\quad\text{where}\quad v = k_2 \frac{u_1 + u_2}{2},\quad \omega = k_1 (u_2 - u_1)$$
- **Sensor-Model Fusion Dilemma:** The physical model alone drifts due to lake wind, water currents, and thruster asymmetry ($k_1, k_2$ parameter mismatch). Raw sensors (GPS & compass) are noisy, jittery, and suffer packet loss.
- **Luenberger Observer Law:**
  $$\dot{\hat{X}} = f(\hat{X}, u) + L \cdot (y - C\hat{X})$$
  With state measurement $y = [x_{\text{meas}}, y_{\text{meas}}, \theta_{\text{meas}}]^\top$, $C = I$, observer gain matrix $L = \text{diag}(l_1, l_1, l_2)$, angular error normalized via $\text{sawtooth}$, and graceful fallback to pure model dead-reckoning on UDP packet timeout.
- **In-Water Verification Sequence:**
  1. $L = 0$: Quantify model drift rate on lake water.
  2. $y$ alone: Benchmark GPS/compass noise amplitude.
  3. Progressive $L$ tuning: Balance responsiveness vs noise rejection.
  4. Dropout test: Cut telemetry for $5\text{s}$ to verify dead reckoning.
  5. Gain identification: Empirically tune propulsive gains $k_1, k_2$.

### 📌 Pending Actions When Reconnected to Boat 11:
1. **Google Earth Trajectory Check:** Pull `gps_data.kml` via SCP to laptop and open in [earth.google.com](https://earth.google.com/web/) to verify logged shore coordinates.
2. **In-Water Lake Test:** Deploy DD-Boat 11 for short-burst autonomous heading hold / Boomerang return (`control.py`).
3. **RC Teleoperation:** Test arrow key control on water using `rc_teleop.py`.

---

## 4. Master Project Roadmap: Component Matrix (What is Built vs. What to Build)

| Module / Layer | File / Script | Role & Functionality | Status |
| :--- | :--- | :--- | :---: |
| **Emergency Safety** | `stop.py` | Immediate thruster cutoff `(0, 0)` | **BUILT & TESTED** |
| **Initial Diagnostics** | `boat11_smoke_test.py` | Validates IMU, GPS, and Arduino connectivity | **BUILT & TESTED** |
| **Motor Drive Test** | `motor_test.py` | Page 27 2-second in-place dual motor test | **BUILT & TESTED** |
| **Compass Calibration** | `calib.py` + `calib_boat11.json` | 360° circle fit for hard/soft-iron matrices ($A^{-1}, b$) | **BUILT & TESTED** |
| **GPS Georeferencing** | `gps.py` | NMEA parsing, UTM Zone 30N projection & KML logging | **BUILT & TESTED** |
| **Heading Controller** | `heading_hold.py` | Sawtooth proportional heading hold on water | **BUILT (Water Ready)** |
| **Safe Retrieval (RTH)** | `control.py` | Boomerang out-and-back and GPS Return-to-Home | **BUILT (Water Ready)** |
| **Keyboard RC Remote** | `rc_teleop.py` | Manual Arrow Key / WASD driving with 0.5s watchdog | **BUILT (Water Ready)** |
| **GPS Waypoints** | `waypoint_mission.py` | Sequenced multi-waypoint lake mission navigator | **BUILT (Water Ready)** |
| **Regatta Competition** | `regatta_loop.py` | 5-minute cyclic gate-crossing loop ($A \leftrightarrow B$) with scoring | **BUILT (Competition Ready)** |
| **Digital Twin Simulator** | `simu.py` / `roblib.py` | Dubins car & DD-boat forward kinematic integration ($dt=0.05$) | **BUILT & TESTED** |
| **Page 13 Boat Simulator** | `simu_page13.py` | Official Page 13 Euler integration & `roblib.draw_tank` visualizer | **BUILT & TESTED** |
| **Practice 1 Exercises** | `digital_twin_exercises.py` | Page 14 exercises: straight, differential turn, in-place spin, square | **BUILT & TESTED** |
| **Luenberger Observer** | `digital_twin_observer.py` | Predictor-corrector state filter fusing model and UDP telemetry | **BUILT & READY** |
| **Line-Following GNC** | `line_follower.py` | Cross-track error determinant control between buoys | **TO BE BUILT** |
| **Live Web HMI / Monitor**| `hmi_monitor.py` | Real-time web/terminal telemetry HUD (voltage, GPS, gate dist) | **OPTIONAL ADD-ON** |

---

## 5. Quick-Action Command Cheat Sheet

Run these commands directly in your SSH terminal on Boat 11 (`ue32@ddboat11:~/NavySeals$`):

### 🚨 Emergency Stop (Always keep a terminal ready)
```bash
python3 stop.py
```

### ⚡ Step 1: Propeller Motor Test (Page 27)
*Safety check: Ensure propellers are clear of hands/cables; Switch F must be DOWN.*
```bash
python3 motor_test.py
```

### 📍 Step 2: GPS Telemetry & Guerlédan Reference Log (Pages 11–22)
*Streams live coordinates, UTM distance/heading to Guerlédan reference, and exports `gps_data.kml`.*
```bash
python3 gps.py
```

### 🧭 Compass Verification (Live Check)
*Quick test to verify current heading against an analog or phone compass.*
```bash
python3 calib.py
# Press 'T' when prompted
```

### 🌊 Stage 1: Closed-Loop Heading Hold
Maintains a straight compass line using Prof. Luc Jaulin's sawtooth proportional controller:
```bash
# Syntax: python3 heading_hold.py [target_heading_deg] [speed] [duration_sec]

# Bench Dry Run (Elevated boat, test dynamic motor reaction):
python3 heading_hold.py 90 40 10
# (Rotate boat by hand; observe thrusters compensating to restore 90° heading)

# Lake Trial (In water, 15 seconds towards open water):
python3 heading_hold.py 120 40 15
```

### 🗺️ Stage 2: Autonomous GPS Waypoint Navigation
Navigates across sequential lake waypoints with automatic arrival detection:
```bash
python3 waypoint_mission.py
```

### 🪃 Stage 3: "No-Kayaking" Return-to-Home Controller (`control.py`)
Prevents stranding in the lake by automatically returning to the launch point:
```bash
# 1. Boomerang Mode (Recommended for testing):
# Cruises out on 120° for 15s, turns 180° around and returns for 15s, stops at your feet!
python3 control.py boomerang 120 40 30

# 2. GPS Return-to-Home Mode:
# Automatically logs launch GPS point, sails out, then guides itself back to launch point:
python3 control.py rth_gps 120 40 30
```

### 🎮 Stage 4: Real-Time Keyboard RC Teleoperation (`rc_teleop.py`)
Direct wireless manual remote control from your laptop keyboard over SSH:
```bash
python3 rc_teleop.py
```
* **Keys:** `▲` / `W` (Forward), `▼` / `S` (Reverse), `◄` / `A` (Turn Left), `►` / `D` (Turn Right)
* **Speed:** `+` / `-` (adjusts power), `Space` (instant all-stop), `Q` (quit)
* **Safety:** Automatic 0.5s deadman watchdog halts motors if keys are released or Wi-Fi drops.

---

## 6. Mathematical Formulas & Coordinate Systems Reference

### 1. Compass Calibration
Raw magnetic vector $\mathbf{m} = [m_x, m_y, m_z]^T$:
$$\mathbf{m}_{\text{calib}} = \mathbf{A}^{-1} (\mathbf{m} - \mathbf{b})$$
Where:
* $\mathbf{b}$: Hard-iron bias offset vector (center of sphere/ellipsoid).
* $\mathbf{A}^{-1}$: Soft-iron distortion correction matrix.

### 2. Marine Geographic Heading
$$\psi_{\text{trig}} = \text{atan2}(m_{y,\text{calib}}, m_{x,\text{calib}}) \cdot \frac{180}{\pi}$$
$$\psi_{\text{geo}} = (90^\circ - \psi_{\text{trig}}) \pmod{360^\circ}$$
* $0^\circ = \text{North}$
* $90^\circ = \text{East}$
* $180^\circ = \text{South}$
* $270^\circ = \text{West}$

### 3. Georeferencing (Guerlédan Lake)
Official Course Reference Point:
$$\text{Lat}_{\text{ref}} = 48.199111^\circ \text{ N},\quad \text{Lon}_{\text{ref}} = -3.014930^\circ \text{ W}$$
Projected to UTM Zone 30N ($x$ Easting, $y$ Northing in meters):
$$dx = x - x_{\text{ref}},\quad dy = y - y_{\text{ref}},\quad d = \sqrt{dx^2 + dy^2}$$
$$\theta_{\text{target}} = \left(90^\circ - \text{atan2}(dy, dx) \cdot \frac{180}{\pi}\right) \pmod{360^\circ}$$

### 4. Differential Steering Control (Prof. Luc Jaulin's Law - Page 14)
For commanded forward speed $v \in [0, 255]$ and heading error $e = \text{sawtooth}(\psi_{\text{target}} - \psi)$:
$$u_{\text{left}} = \text{clamp}(v + K_p \cdot e,\ -255,\ 255)$$
$$u_{\text{right}} = \text{clamp}(v - K_p \cdot e,\ -255,\ 255)$$
Where:
* $e > 0 \implies \psi < \psi_{\text{target}}$: Boat must yaw **RIGHT** (clockwise) $\implies u_{\text{left}} \uparrow,\ u_{\text{right}} \downarrow$.
* $e < 0 \implies \psi > \psi_{\text{target}}$: Boat must yaw **LEFT** (counter-clockwise) $\implies u_{\text{left}} \downarrow,\ u_{\text{right}} \uparrow$.
* $\text{sawtooth}(\alpha) = (\alpha + 180^\circ \pmod{360^\circ}) - 180^\circ \in [-180^\circ, 180^\circ]$.

### 5. Dubins-like DD-Boat Kinematics & Euler Discretization (Antoine Morvan)
State $X = [x, y, \theta]^\top$ actuated by thrusters $u_1$ (left) and $u_2$ (right):
$$\begin{cases} \dot{x} = v \cos\theta \\ \dot{y} = v \sin\theta \\ \dot{\theta} = \omega \end{cases}\quad\text{with}\quad \begin{cases} v = k_2 \dfrac{u_1 + u_2}{2} \\ \omega = k_1 (u_2 - u_1) \end{cases}$$
Euler numerical integration over time step $dt$:
$$x_{k+1} = x_k + dt \cdot v \cos\theta_k, \quad y_{k+1} = y_k + dt \cdot v \sin\theta_k, \quad \theta_{k+1} = \theta_k + dt \cdot \omega$$

### 6. Non-Linear Luenberger Observer State Estimation
Predicts state via physical model $f(\hat{X}, u)$ and corrects via sensor innovation $y - C\hat{X}$:
$$\dot{\hat{X}} = f(\hat{X}, u) + L \cdot (y - C\hat{X})$$
Where:
* State measurement: $y = [x_{\text{GPS}}, y_{\text{GPS}}, \theta_{\text{compass}}]^\top \implies C = I_{3 \times 3}$.
* Observer gain matrix: $L = \text{diag}(l_1, l_1, l_2)$ (trust dial between pure model $L=0$ and pure sensor $L \gg 1$).
* Angle innovation wrapping: $\text{err}[2] = \text{sawtooth}(y[2] - \hat{X}[2]) \in [-\pi, \pi]$.
* Packet loss resilience: When UDP timeout occurs, set $\text{err} = \mathbf{0}$, continuing dead reckoning seamlessly via $f(\hat{X}, u)$.

---

## 7. Google Earth Integration & Waypoint Planning (Pages 19–22)

Google Earth is the primary geographic tool used in the BEST robotics campaign for two purposes:

### 1. Visualizing Boat Trajectories (`.kml`)
Whenever you run `gps.py` or a navigation mission, it records GPS fixes and generates a **`gps_data.kml`** file (KML is Google Earth's native file format).
* **Transfer KML from Boat 11 to Laptop:**
  ```powershell
  scp ue32@172.20.25.211:~/NavySeals/gps_data.kml d:\Personal_Projects\BEST_Breast\NavySeals\
  ```
* **Open in Google Earth:**
  1. Open Chrome and go to **[earth.google.com](https://earth.google.com/web/)** (or open Google Earth Pro desktop).
  2. Click **Projects** (left bookmark icon) $\to$ **New Project** $\to$ **Import KML file from computer**.
  3. Select `gps_data.kml`. Google Earth will zoom directly to Guerlédan Lake and display the exact track points where Boat 11 operated!

### 2. Planning Mission Waypoints
To create an autonomous mission route on the lake:
1. Zoom into your location on Guerlédan Lake in Google Earth.
2. Click **Add placemark / point** on desired waypoints (e.g., 20m offshore, around a buoy, returning to dock).
3. Read the **Latitude and Longitude** from each placemark.
4. Paste these coordinates into the waypoint list in `waypoint_mission.py`.
