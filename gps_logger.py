#!/usr/bin/env python3
"""
DD-Boat 11 - GPS Telemetry Monitor & Track Logger
=================================================
Reads GPS positions (NMEA GPGLL), converts DDMM.MMMM to Decimal Degrees,
projects to local metric coordinates (UTM 30N / Guerledan Lake),
and writes live logs to CSV and Google Earth KML format.

Usage:
    python3 gps_logger.py
"""

import sys
import os
import time
import math

sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))

# Guerledan Lake Reference Point (Benoit Zerr & Luc Jaulin)
REF_LAT = 48.199111
REF_LON = -3.014930

# Equirectangular metric projection constants around Guerledan (Zone 30N)
R_EARTH = 6371000.0  # Earth radius in meters
LAT_RAD = math.radians(REF_LAT)

def latlon_to_meters(lat, lon, ref_lat=REF_LAT, ref_lon=REF_LON):
    """Converts lat/lon (deg) to local Cartesian (x=East, y=North) in meters."""
    dlat = math.radians(lat - ref_lat)
    dlon = math.radians(lon - ref_lon)
    x = dlon * R_EARTH * math.cos(LAT_RAD)
    y = dlat * R_EARTH
    return x, y

def cvt_gll_ddmm_2_dd(st):
    """Converts GLL strings [ilat, 'N', ilon, 'W', time] to decimal degrees."""
    ilat = float(st[0])
    ilon = float(st[2])
    olat = int(ilat / 100) + (ilat % 100) / 60.0
    olon = int(ilon / 100) + (ilon % 100) / 60.0
    if len(st) > 1 and st[1] == 'S':
        olat = -olat
    if len(st) > 3 and st[3] == 'W':
        olon = -olon
    return olat, olon

def main():
    print("=" * 60)
    print("      DD-BOAT 11 : GPS TELEMETRY & LOGGER                  ")
    print(f"      Reference Point: Lat={REF_LAT:.6f}, Lon={REF_LON:.6f}")
    print("=" * 60)

    try:
        import gps_driver_v2 as gpddrv
        gps = gpddrv.GpsIO()
        gps.set_filter_speed("0")
        print("[OK] GPS connected. Waiting for satellite lock...")
    except Exception as e:
        print(f"[ERROR] Could not connect to GPS driver: {e}")
        return

    csv_file = "gps_log.csv"
    kml_file = "gps_track.kml"

    with open(csv_file, "w") as f_csv:
        f_csv.write("timestamp,lat,lon,x_east,y_north,distance_from_ref,heading_from_ref\n")

    track_points = []
    print("\nLogging active. Press Ctrl+C to stop.\n")
    print(f"{'Time':8s} | {'Latitude':11s} | {'Longitude':11s} | {'X (m)':8s} | {'Y (m)':8s} | {'Dist (m)':8s}")
    print("-" * 65)

    try:
        while True:
            gll_ok, gll_data = gps.read_gll_non_blocking()
            if gll_ok:
                lat, lon = cvt_gll_ddmm_2_dd(gll_data)
                x, y = latlon_to_meters(lat, lon)
                dist = math.hypot(x, y)
                heading = (math.degrees(math.atan2(x, y)) + 360.0) % 360.0
                timestamp = time.strftime("%H:%M:%S")

                print(f"{timestamp:8s} | {lat:11.6f} | {lon:11.6f} | {x:8.2f} | {y:8.2f} | {dist:8.2f}")

                with open(csv_file, "a") as f_csv:
                    f_csv.write(f"{timestamp},{lat:.7f},{lon:.7f},{x:.2f},{y:.2f},{dist:.2f},{heading:.1f}\n")
                track_points.append((lon, lat))

            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n\nStopping GPS logger...")

    # Save Google Earth KML
    if track_points:
        coords_str = " ".join([f"{p[0]},{p[1]},0" for p in track_points])
        kml_content = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>DD-Boat 11 Track</name>
    <Style id="yellowLine">
      <LineStyle>
        <color>7f00ffff</color>
        <width>4</width>
      </LineStyle>
    </Style>
    <Placemark>
      <name>Boat 11 Trajectory</name>
      <styleUrl>#yellowLine</styleUrl>
      <LineString>
        <coordinates>{coords_str}</coordinates>
      </LineString>
    </Placemark>
  </Document>
</kml>"""
        with open(kml_file, "w") as f_kml:
            f_kml.write(kml_content)
        print(f"Track saved to '{kml_file}' (Google Earth compatible) and '{csv_file}'.")

if __name__ == "__main__":
    main()
