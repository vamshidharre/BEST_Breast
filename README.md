# BEST 2026: ROBOTson Crusoe — Autonomous DD-Boat Marine Robotics
**Team:** NavySeals · **Vessel:** DD-Boat 11 (`172.20.25.211`)  
*Guerlédan Lake, Brittany | ENSTA Bretagne · Lab-STICC · ROBEX*  
**Supervisors:** Prof. Luc Jaulin, Prof. Benoît Zerr, Antoine Morvan  

> 🚀 **STARTING A NEW CHAT SESSION?**  
> Read [`SESSION_HANDOVER.md`](file:///d:/Personal_Projects/BEST_Breast/SESSION_HANDOVER.md) first! It contains the complete status, technical constraints (Python 3.5.3, no scipy, steering laws), verified milestones, and immediate commands to resume without repeating work.

---

## 🚤 Project Overview

The **DD-Boat (Differential-Drive Boat)** is an autonomous surface vessel (ASV) powered by dual electric thrusters and steered entirely via differential thrust.

This repository contains:
1. **Physical Boat Software (`NavySeals/`):** Deployed on DD-Boat 11 (Raspberry Pi 3, Raspbian Stretch, Python 3.5.3, Arduino Mega 2560).
2. **Simulation Environments:** Non-linear feedback linearization controllers, multi-boat platooning algorithms, and a Unity 3D / Virtual Regatta UDP network client.
3. **Comprehensive Documentation Suite:** Step-by-step guides, project records, calibration reports, and regatta mission plans.

---

## 📚 Master Documentation Suite

| Document | Purpose |
| :--- | :--- |
| **[`SESSION_HANDOVER.md`](file:///d:/Personal_Projects/BEST_Breast/SESSION_HANDOVER.md)** | **Instant context loader for new AI chats & team members.** Contains current state, solved gotchas, and immediate next commands. |
| **[`PROJECT_RECORD.md`](file:///d:/Personal_Projects/BEST_Breast/PROJECT_RECORD.md)** | **Full technical runbook.** Hardware specs, math formulas, complete milestone log, and command cheat sheet. |
| **[`DIGITAL_TWIN.md`](file:///d:/Personal_Projects/BEST_Breast/DIGITAL_TWIN.md)** | **Digital twin, UDP & observer guide.** Dubins model, UDP socket protocol, Luenberger observer, and lake experiment sequence. |
| **[`COMPASS_CALIBRATION_REPORT.md`](file:///d:/Personal_Projects/BEST_Breast/COMPASS_CALIBRATION_REPORT.md)** | **Sensor calibration report.** Details ellipsoid fitting theory, soft/hard iron correction, and verification. |
| **[`REGATTA_MISSION_PLAN.md`](file:///d:/Personal_Projects/BEST_Breast/REGATTA_MISSION_PLAN.md)** | **Friday Regatta competition strategy.** Gate crossing algorithm, scoring optimization, and rules. |

---

## 📁 Repository Structure

```text
BEST_Breast/
├── SESSION_HANDOVER.md             # ⭐ Quick-resume guide for new sessions
├── PROJECT_RECORD.md               # ⭐ Full technical runbook & command cheat sheet
├── DIGITAL_TWIN.md                 # ⭐ Digital twin, UDP & Luenberger observer manual
├── COMPASS_CALIBRATION_REPORT.md   # Magnetometer calibration report & matrices
├── REGATTA_MISSION_PLAN.md         # Friday Regatta race strategy & rules
│
├── NavySeals/                      # 🚢 PRODUCTION CODE RUNNING ON BOAT 11
│   ├── stop.py                     # Emergency all-stop cutoff (0, 0)
│   ├── motor_test.py               # Page 27 motor spin test
│   ├── calib.py                    # Compass calibration & live heading stream
│   ├── calib_boat11.json           # Calibrated hard/soft iron parameters (A^-1, xbar)
│   ├── gps.py                      # GPS acquisition, UTM 30N projection & KML logging
│   ├── heading_hold.py             # Closed-loop sawtooth proportional heading hold
│   ├── control.py                  # "No-Kayaking" Return-to-Home (Boomerang & GPS RTH)
│   ├── rc_teleop.py                # Wireless keyboard RC arrow-key teleoperation
│   └── waypoint_mission.py         # Multi-waypoint GPS lake navigation
│
├── simulation/                     # 💻 DESKTOP SIMULATION & PLANNING
│   ├── simu.py                     # Waypoint navigation & friction simulator
│   ├── single_boat_simulator.py    # Feedback linearization trajectory tracking
│   ├── train_simulator.py          # 3-boat train / platooning simulator
│   ├── ddboatlib.py                # Plotting and boat rendering library
│   └── client_pour_virtual_regatta.py # UDP client for Unity simulator
│
└── docs/                           # 📄 COURSE MATERIALS & TEXTBOOKS
    ├── digital_twin_presentation.pdf # Antoine Morvan's Digital Twin & Observer presentation
    ├── best_ddboat.pdf             # Introductory DD-Boat guide (Pages 1–27) [100% Passed]
    ├── drones.pdf                  # Prof. Luc Jaulin's textbook (Chapter 4: DD-Boats)
    ├── speech_best26.pdf           # Heading control & simulation presentation
    ├── best_godard.pdf             # AI & vision-based obstacle avoidance
    └── guerledan_planning.pdf      # Field operations timetable
```

---

## ⚡ Quick-Action Cheat Sheet (Boat 11)

In your SSH terminal on Boat 11 (`ue32@ddboat11:~/NavySeals$`):

```bash
# 1. Emergency All-Stop
python3 stop.py

# 2. Check Calibrated Compass
python3 calib.py  # (Press 'T' for live check)

# 3. Safe "No-Kayaking" Lake Test (15s out on 120 deg, 15s back to shore)
python3 control.py boomerang 120 40 30

# 4. Keyboard RC Remote Control (Arrow keys / WASD)
python3 rc_teleop.py

# 5. Autonomous Waypoint Navigation
python3 waypoint_mission.py
```
