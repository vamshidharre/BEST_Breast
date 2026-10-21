#!/usr/bin/env python3
"""
DD-Boat 11 - Magnetometer Calibration & Heading Verification
============================================================
Implements:
1. Interactive 4-Pose calibration (North, South-inverted, West, Up)
2. Jaulin Ellipsoid Fit & Spherization (MagCalib for 3D clouds)
3. Hard-iron (bias offset xbar) and soft-iron (gain/skew matrix invA) correction
4. Live heading test loop with compass rose display

Usage on boat:
    python3 calib.py
"""

import sys
import os
import time
import json
import numpy as np
from scipy.linalg import sqrtm, inv, norm

# Add drivers folder to path if present
sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))

# Magnetic inclination for Brittany (Guerledan / Brest): ~64 degrees
INCLINATION_RAD = np.radians(64.0)

# ---------------------------------------------------------------------------
# HARDWARE DRIVER INITIALIZATION
# ---------------------------------------------------------------------------
def init_hardware():
    """Initializes IMU and Arduino drivers with graceful fallback for simulation."""
    imu = None
    ard = None
    try:
        import imu9_driver_v2 as imudrv
        imu = imudrv.Imu9IO()
        print("[OK] IMU9 connected.")
    except Exception as e:
        print(f"[NOTE] IMU driver not loaded ({e}). Simulation mode available.")

    try:
        import arduino_driver_v2 as arddrv
        ard = arddrv.ArduinoIO()
        print("[OK] Arduino connected.")
    except Exception as e:
        print(f"[NOTE] Arduino driver not loaded ({e}). Audio beeps disabled.")

    return imu, ard


def beep(ard, count=1, delay=0.15):
    """Audible or terminal notification between poses."""
    for _ in range(count):
        print("\a", end="", flush=True)  # Terminal bell
        time.sleep(delay)


def acquire_average_mag(imu, samples=60, delay=0.03):
    """Acquires and averages multiple raw magnetometer readings to cancel noise."""
    if imu is None:
        return np.zeros((3, 1))
    
    readings = []
    for _ in range(samples):
        x, y, z = imu.read_mag_raw()
        readings.append([x, y, z])
        time.sleep(delay)
    
    mean_val = np.mean(readings, axis=0).reshape((3, 1))
    return mean_val


# ---------------------------------------------------------------------------
# MATHEMATICAL CALIBRATION ALGORITHMS (Jaulin / Zerr)
# ---------------------------------------------------------------------------
def Fit_Ellipsoid(x1, x2, x3):
    """
    Fits an ellipsoid to a 3D point cloud:
    p1*x1^2 + p2*x2^2 + p3*x3^2 + p4*x1*x2 + p5*x1*x3 + p6*x2*x3 + p7*x1 + p8*x2 + p9*x3 = 1
    Returns center xbar and quadratic form matrix Q.
    """
    x1, x2, x3 = np.array(x1).flatten(), np.array(x2).flatten(), np.array(x3).flatten()
    X = np.column_stack([
        x1**2, x2**2, x3**2,
        x1*x2, x1*x3, x2*x3,
        x1, x2, x3
    ])
    p = inv(X.T @ X) @ X.T @ np.ones((len(x1), 1))
    p = p.flatten()
    
    P = np.array([
        [p[0], 0.5 * p[3], 0.5 * p[4]],
        [0.5 * p[3], p[1], 0.5 * p[5]],
        [0.5 * p[4], 0.5 * p[5], p[2]]
    ])
    c = np.array([[p[6]], [p[7]], [p[8]]])
    xbar = -0.5 * inv(P) @ c
    Q = P / (1.0 + float(xbar.T @ P @ xbar))
    return Q, xbar


def mag_calib_from_cloud(x1, x2, x3, xnorth, xwest):
    """Calibrates using a point cloud plus North and West reference poses."""
    Q, xbar = Fit_Ellipsoid(x1, x2, x3)
    I = INCLINATION_RAD
    
    ynorth = np.array([[np.cos(I)], [0.0], [-np.sin(I)]])
    ywest = np.array([[0.0], [-np.cos(I)], [-np.sin(I)]])
    yup = np.cross(ynorth.flatten(), ywest.flatten()).reshape(3, 1)
    Y = np.hstack([ynorth, ywest, yup])

    vnorth = sqrtm(Q) @ (xnorth - xbar)
    vwest = sqrtm(Q) @ (xwest - xbar)
    vup = np.cross(vnorth.flatten(), vwest.flatten()).reshape(3, 1)
    V = np.hstack([vnorth / norm(vnorth), vwest / norm(vwest), vup / norm(vup)])
    
    R = Y @ inv(V)
    invA = np.real(R @ sqrtm(Q))
    return invA, xbar


def mag_calib_4poses(xN, xS, xW, xU):
    """
    Computes calibration from the 4 cardinal poses:
    - xN: Pointing flat North
    - xS: Pointing South, boat inverted (upside-down)
    - xW: Pointing flat West
    - xU: Pointing vertically up to the sky
    """
    I = INCLINATION_RAD
    
    # Hard iron offset: midpoint of opposite orientations
    # North and South-inverted are diametrically opposite in the sensor frame
    xbar = 0.5 * (xN + xS)
    
    # Vectors centered at hard-iron offset
    vN = xN - xbar
    vW = xW - xbar
    vU = xU - xbar
    
    # Expected unit direction vectors in local navigation frame
    yN = np.array([[np.cos(I)], [0.0], [-np.sin(I)]])
    yW = np.array([[0.0], [-np.cos(I)], [-np.sin(I)]])
    yU = np.cross(yN.flatten(), yW.flatten()).reshape(3, 1)
    
    # Measurement matrix V and Reference matrix Y
    V = np.hstack([vN / norm(vN), vW / norm(vW), vU / norm(vU)])
    Y = np.hstack([yN, yW, yU])
    
    # Gain / scale correction
    mean_radius = np.mean([norm(vN), norm(vW), norm(vU)])
    invA = inv(V) / mean_radius
    
    return invA, xbar


# ---------------------------------------------------------------------------
# HEADING CALCULATION & COMPASS CONVERSION
# ---------------------------------------------------------------------------
def compute_heading(xraw, invA, xbar):
    """
    Applies calibration and computes heading in degrees [0, 360).
    y = invA * (xraw - xbar)
    """
    x_centered = xraw.reshape((3, 1)) - xbar.reshape((3, 1))
    y = invA @ x_centered
    mx, my, mz = y.flatten()
    
    # Heading angle in radians: psi = arctan2(my, mx)
    # Geo heading: 0 = North, 90 = East, 180 = South, 270 = West
    psi_trig = np.arctan2(my, mx)
    heading_deg = (np.degrees(psi_trig) + 360.0) % 360.0
    return heading_deg, (mx, my, mz)


def heading_to_rose(heading):
    """Returns human-readable compass heading (e.g. N, NE, E, etc.)."""
    dirs = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
            "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]
    idx = int((heading + 11.25) / 22.5) % 16
    return dirs[idx]


# ---------------------------------------------------------------------------
# MAIN INTERACTIVE CALIBRATION ROUTINE
# ---------------------------------------------------------------------------
def run_interactive_calibration():
    print("=" * 60)
    print("      DD-BOAT 11 : MAGNETOMETER CALIBRATION (4 POSES)      ")
    print("=" * 60)
    
    imu, ard = init_hardware()
    if imu is None:
        print("[ERROR] Cannot calibrate without connected IMU hardware.")
        return

    poses = [
        ("POSE 1/4: NORTH", "Orient the boat laid FLAT pointing toward MAGNETIC NORTH.", "North"),
        ("POSE 2/4: SOUTH (INVERTED)", "Turn the boat UPSIDE-DOWN and orient toward SOUTH.", "South"),
        ("POSE 3/4: WEST", "Orient the boat laid FLAT pointing toward WEST.", "West"),
        ("POSE 4/4: UP", "Orient the boat VERTICALLY pointing toward the SKY.", "Up")
    ]
    
    collected_vectors = {}
    
    for title, instruction, key in poses:
        print("\n" + "-" * 50)
        print(f"  >>> {title} <<<")
        print(f"  Instructions: {instruction}")
        input("  Press ENTER when the boat is in position and held steady...")
        
        print("  Collecting 60 samples (hold still for ~2 seconds)...", end="", flush=True)
        beep(ard, 1)
        vec = acquire_average_mag(imu, samples=60, delay=0.03)
        collected_vectors[key] = vec
        print(" [DONE]")
        print(f"  Captured vector: [{vec[0,0]:.1f}, {vec[1,0]:.1f}, {vec[2,0]:.1f}]")
        beep(ard, 2, 0.1)

    print("\nComputing calibration parameters...")
    xN = collected_vectors["North"]
    xS = collected_vectors["South"]
    xW = collected_vectors["West"]
    xU = collected_vectors["Up"]
    
    invA, xbar = mag_calib_4poses(xN, xS, xW, xU)
    
    print("\n" + "=" * 50)
    print("CALIBRATION RESULTS:")
    print("Hard-Iron Offset (xbar):\n", xbar)
    print("Soft-Iron Matrix (invA):\n", invA)
    print("=" * 50)
    
    # Save to JSON and TXT
    calib_data = {
        "boat_id": 11,
        "timestamp": time.time(),
        "xbar": xbar.tolist(),
        "invA": invA.tolist()
    }
    with open("calib_boat11.json", "w") as f:
        json.dump(calib_data, f, indent=4)
    print("Saved calibration parameters to 'calib_boat11.json'.")
    
    np.savetxt("calib_xbar.txt", xbar)
    np.savetxt("calib_invA.txt", invA)
    print("Saved text arrays to 'calib_xbar.txt' and 'calib_invA.txt'.")

    # Live verification
    print("\n" + "=" * 50)
    print("STARTING LIVE HEADING VERIFICATION (Press Ctrl+C to exit):")
    print("Rotate the boat to check headings (0°=N, 90°=E, 180°=S, 270°=W)...")
    print("=" * 50)
    
    try:
        while True:
            raw = np.array(imu.read_mag_raw()).reshape((3, 1))
            heading, (mx, my, mz) = compute_heading(raw, invA, xbar)
            rose = heading_to_rose(heading)
            print(f"\rHeading: {heading:6.1f}° [{rose:3s}] | Calibrated: mx={mx:+6.2f}, my={my:+6.2f}, mz={mz:+6.2f}", end="")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\nCalibration completed successfully!")


if __name__ == "__main__":
    run_interactive_calibration()
