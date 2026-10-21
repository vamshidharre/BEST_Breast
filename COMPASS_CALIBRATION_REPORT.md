# DD-BOAT 11 : COMPASS CALIBRATION REPORT
**Platform:** DD-Boat 11 (`NavySeals`) · **Sensor:** Pololu 9-DOF IMU v5 (ST LIS3MDL Magnetometer)  
**Location:** Guerlédan Lake, Brittany | ENSTA Bretagne · Lab-STICC  
**Supervising Faculty:** Prof. Luc Jaulin, Prof. Benoît Zerr, Antoine Morvan  

---

## 1. Objective & Physical Background

Magnetometers mounted on autonomous surface vessels suffer from two significant sources of distortion:
1. **Hard-Iron Distortion:** Static magnetic fields produced by onboard ferromagnetic components (screws, battery terminals, motor casings, electronic circuits). This shifts the center of the magnetic sphere away from the origin by an offset vector $\mathbf{b} = [b_x, b_y, b_z]^T$.
2. **Soft-Iron Distortion:** Distortion caused by materials that temporarily warp the Earth's magnetic field (chassis metal, cables carrying high current). This deforms the ideal sphere into an ellipsoid described by matrix $\mathbf{A}$.

### Calibration Formula (Page 25 of `best_ddboat.pdf`):
$$\mathbf{m}_{\text{calib}} = \mathbf{A}^{-1} (\mathbf{m}_{\text{raw}} - \mathbf{b})$$
When calibrated, rotating the boat in a flat horizontal plane produces a perfect origin-centered circle:
$$m_{x,\text{calib}}^2 + m_{y,\text{calib}}^2 = 1$$

---

## 2. Issues Encountered & Engineering Solutions

| Problem | Root Cause | Solution Implemented |
| :--- | :--- | :--- |
| **`ImportError: No module named 'scipy'`** | Boat 11's Raspberry Pi runs Python 3.5.3 with only standard libraries and `numpy`. | Rewrote `calib.py` using pure `numpy.linalg` least-squares circle fitting and eigenvalue decomposition. Removed all `scipy` imports. |
| **Windows OpenSSH Terminal Freeze** | Overwriting lines with `\r` in a tight 20Hz loop causes the Windows OpenSSH buffer to hang. | Switched live verification to scrolling numbered lines (`[#0001] Heading: ...`) to ensure continuous streaming. |
| **East / West Axis Inversion** | Marine heading convention ($0^\circ=\text{North}$, clockwise) differs from trigonometric math angles ($0^\circ=\text{East}$, counter-clockwise). | Corrected heading math: $\theta_{\text{rad}} = \text{atan2}(m_y, m_x)$, $\theta_{\text{deg}} = (\text{degrees}(\theta_{\text{rad}}) + 360^\circ) \pmod{360^\circ}$. |

---

## 3. Calibration Procedure & Parameters

The calibration was executed using the **360° Horizontal Circle Fit** (Option 1 in `calib.py`):
1. DD-Boat 11 was slowly rotated through two complete 360° horizontal loops.
2. Raw data points were fitted to an algebraic circle $(x - b_x)^2 + (y - b_y)^2 = R^2$.
3. Offsets $\mathbf{b}$ and normalization scaling matrix $\mathbf{A}^{-1}$ were computed.
4. Output files generated:
   * **`calib_boat11.json`:** JSON structure containing `xbar` and `invA` matrices.
   * **`calib_xbar.txt`** and **`calib_invA.txt`:** Plaintext arrays for quick loading.

---

## 4. Live Heading Verification Results

Verification was performed by rotating Boat 11 to the four cardinal directions and checking against a reference magnetic compass:

| Reference Direction | Expected Heading | Calibrated Heading Output | Error | Result |
| :---: | :---: | :---: | :---: | :---: |
| **North** | $0.0^\circ$ | $0.8^\circ$ [N] | $+0.8^\circ$ | **PASS** |
| **East** | $90.0^\circ$ | $89.2^\circ$ [E] | $-0.8^\circ$ | **PASS** |
| **South** | $180.0^\circ$ | $181.5^\circ$ [S] | $+1.5^\circ$ | **PASS** |
| **West** | $270.0^\circ$ | $269.4^\circ$ [W] | $-0.6^\circ$ | **PASS** |

### Conclusion:
DD-Boat 11's magnetometer is fully calibrated, verified accurate within $\pm 1.5^\circ$, and ready for autonomous closed-loop navigation on Guerlédan Lake.
