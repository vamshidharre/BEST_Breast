# DD-BOAT 11 : REGATTA MISSION PLAN & COMPETITION STRATEGY
**Competition:** Friday Regatta (BEST 2026 — Guerlédan Lake)  
**Reference:** Exercise 13 (*DD Regatta*), Chapter 4 of `drones.pdf`  
**Supervising Jury:** Prof. Luc Jaulin, Prof. Benoît Zerr, Antoine Morvan  
**Team:** NavySeals · **Vessel:** DD-Boat 11 (`172.20.25.211`)  

---

## 1. Regatta Rules & Objective

* **Duration:** 5 minutes (300 seconds).
* **Course:** Two floating gates positioned on the water:
  * **Gate A (Green Gate):** Delimited by coordinates $\mathbf{a} = (x_a, y_a)$
  * **Gate B (Red Gate):** Delimited by coordinates $\mathbf{b} = (x_b, y_b)$
* **Scoring Rules:**
  * Crossing a gate in the **correct direction:** **$+1$ point**
  * Crossing a gate in the **wrong direction:** **$-1$ point**
  * Objective: Score the maximum points within the 5-minute time limit.
* **Deliverables to the Jury:**
  1. Video recording of the autonomous run.
  2. GPS & IMU telemetry log file (`mission_log.csv` / `gps_data.kml`).

---

## 2. Autonomous Navigation Architecture

The competition controller implements a **Cyclic State Machine** with **Sawtooth Proportional Steering**:

```
 ┌────────────────┐         Crossed Gate A         ┌────────────────┐
 │ State 1:       │ ─────────────────────────────> │ State 2:       │
 │ Navigate to A  │                                │ Navigate to B  │
 └────────────────┘ <───────────────────────────── └────────────────┘
                            Crossed Gate B
```

### 1. Vector Guidance to Active Gate
For current position $\mathbf{p} = (x, y)$ and active target gate $\mathbf{p}_{\text{target}} = (x_t, y_t)$:
$$dx = x_t - x,\quad dy = y_t - y,\quad \text{dist} = \sqrt{dx^2 + dy^2}$$
$$\theta_{\text{desired}} = \left(90^\circ - \text{atan2}(dy, dx) \cdot \frac{180}{\pi}\right) \pmod{360^\circ}$$

### 2. Differential Thrust Steering Law (Prof. Luc Jaulin)
$$e = \text{sawtooth}(\theta_{\text{desired}} - \theta_{\text{current}})$$
$$u_{\text{left}} = \text{clamp}(v_0 + K_p \cdot e,\ -255,\ 255)$$
$$u_{\text{right}} = \text{clamp}(v_0 - K_p \cdot e,\ -255,\ 255)$$
* Base speed $v_0 = 55$ (provides steady speed without battery overheating).
* Proportional gain $K_p = 35.0$.

### 3. Gate Crossing Detection
A gate is considered crossed when:
$$\text{dist}(\mathbf{p}, \mathbf{p}_{\text{target}}) \le R_{\text{arrival}} \quad (R_{\text{arrival}} = 2.5\text{ m})$$
Upon arrival, the active target instantly toggles to the next gate ($A \to B$ or $B \to A$), and the lap counter increments.

---

## 3. Pre-Race Checklist (Friday Morning)

| Check | Action | Pass Criterion |
| :---: | :--- | :--- |
| **1** | Check LiPo battery voltages | $>8.0\text{V}$ per battery pack |
| **2** | Test motors | `python3 motor_test.py` spins cleanly |
| **3** | Check compass calibration | `python3 calib.py` (verify with phone compass) |
| **4** | Acquire GPS satellite lock | `python3 gps.py` streaming valid GPGLL fixes |
| **5** | Input Gate Coordinates | Update $A$ and $B$ coordinates in `waypoint_mission.py` |
| **6** | Arm Telemetry Logger | Ensure `mission_log.csv` is being recorded |

---

## 4. Execution Commands

In your SSH terminal on Boat 11:
```bash
# Launch the 5-minute autonomous regatta mission:
python3 waypoint_mission.py

# In case of obstacle or emergency:
python3 stop.py
# Or press Ctrl+C to disarm motors instantly
```
