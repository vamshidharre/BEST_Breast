# BEST 2026: ROBOTson Crusoe — Autonomous DD-Boat Marine Robotics

Autonomous Surface Vehicle (ASV) guidance, navigation, and control algorithms for **DD-Boat (Differential-Drive Boat)** developed for the **BEST 2026 Guerlédan robotics campaign** (*ROBOTson Crusoe: Let's explore the lost city*, ENSTA Bretagne / Lab-STICC / ROBEX).

---

## 🚤 Project Overview

The **DD-Boat** is a differential-drive autonomous surface vessel powered by dual electric thrusters. This repository contains simulation environments, non-linear controllers, multi-boat platooning algorithms, and a hardware-in-the-loop / Unity UDP network client.

### Key Capabilities & Modules
1. **Kinematics & Dynamics Model**: Non-linear equations of motion with differential thrust and hydrodynamic damping.
2. **Trajectory Tracking via Feedback Linearization**:
   - Decoupled MIMO control using dynamic inversion:
     $$\ddot{\mathbf{p}} = \mathbf{A}(\theta, v)\mathbf{u}$$
   - Exact tracking of parametric curves (e.g., ellipses and complex trajectories).
3. **Platooning / Train Formation**:
   - Inter-vehicle distance regulation using chained feedback linearization.
   - Leader-follower kinematic train ($A \to B \to C$) tracking dynamic curves.
4. **Waypoint Navigation with Heading Error Control**:
   - Continuous angle unwrapping with $\text{sawtooth}(\theta_d - \theta)$ for smooth steering.
5. **Unity "Virtual Regatta" UDP Client**:
   - UDP socket communication (`client_pour_virtual_regatta.py`) connecting Python controllers to the Unity 3D simulator.
   - Outgoing motor commands on port `5005`, incoming telemetry pose stream on port `5006`.

---

## 📁 Repository Structure

```text
BEST_Breast/
├── stop.py                         # Emergency motor stop (0, 0)
├── boat11_smoke_test.py            # Hardware smoke test (IMU, GPS, Thrusters) for DD-Boat 11
├── calib.py                        # 4-pose magnetometer calibration & live heading test
├── gps_logger.py                   # GPS telemetry monitor and KML/CSV track logger
├── heading_hold.py                 # Closed-loop sawtooth heading hold controller
├── waypoint_mission.py             # Full autonomous GPS waypoint navigation mission
├── client_pour_virtual_regatta.py  # UDP client for Unity simulator (Robot ID 3)
├── ddboatlib.py                    # Plotting and boat rendering library
├── simu.py                         # Waypoint navigation & friction model simulator
├── single_boat_simulator.py        # Feedback linearization trajectory tracking
├── train_simulator.py              # 3-boat train / platooning simulator
├── single_boat_trajectory.png      # Trajectory visualization for single boat
├── train_trajectory.png            # Trajectory visualization for 3-boat train
├── best26.html                     # Event details & timetable
├── drones_extracted/               # ENSTA course reference exercises & scripts
└── docs/
    ├── best_ddboat.pdf             # DD-Boat architecture, telemetry, & RC specs
    ├── best_godard.pdf             # AI & vision-based obstacle avoidance
    ├── drones.pdf                  # Autonomous drones & mobile robotics textbook
    └── guerledan_planning.pdf      # Field operations and planning
```

---

## 🚀 Quickstart

### 1. Requirements
Ensure Python 3.9+ and dependencies are installed:
```bash
pip install numpy matplotlib
```

### 2. Run Local Simulators
* **Single Boat Ellipse Tracking:**
  ```bash
  python single_boat_simulator.py
  ```
* **Platoon / Train of 3 Boats:**
  ```bash
  python train_simulator.py
  ```
* **Waypoint Following Mission:**
  ```bash
  python simu.py
  ```

### 3. Field Operations on Physical DD-Boat
* **Emergency Stop:**
  ```bash
  python3 stop.py
  ```
* **Hardware Diagnostic Smoke Test:**
  ```bash
  python3 boat11_smoke_test.py
  ```
* **Compass Calibration (4 Poses):**
  ```bash
  python3 calib.py
  ```
* **GPS Telemetry & Track Logger:**
  ```bash
  python3 gps_logger.py
  ```
* **Closed-Loop Heading Hold:**
  ```bash
  python3 heading_hold.py 90 45 20   # Target 90° East, Speed 45, for 20s
  ```
* **Autonomous GPS Waypoint Mission:**
  ```bash
  python3 waypoint_mission.py
  ```

### 4. Connect to Unity "Virtual Regatta"
Set the server IP in `client_pour_virtual_regatta.py` and run:
```bash
python client_pour_virtual_regatta.py
```

---

## 👥 Authors & Acknowledgments
* **Author:** Vamshidhar Reddy ([@vamshidharre](https://github.com/vamshidharre))
* **Organization:** BEST 2026, ENSTA Bretagne, Lab-STICC, ROBEX
* **Instructors & Organizers:** Prof. Luc Jaulin, Benoît Zerr, Antoine Morvan
