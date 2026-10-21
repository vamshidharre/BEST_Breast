#!/usr/bin/env python3
"""
DD-Boat 11 - GPS Telemetry Monitor & Track Logger
=================================================
Compatible with Python 3.5+
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

REF_LAT = 48.199111
REF_LON = -3.014930
R_EARTH = 6371000.0
LAT_RAD = math.radians(REF_LAT)

def latlon_to_meters(lat, lon, ref_lat=REF_LAT, ref_lon=REF_LON):
    dlat = math.radians(lat - ref_lat)
    dlon = math.radians(lon - ref_lon)
    x = dlon * R_EARTH * math.cos(LAT_RAD)
    y = dlat * R_EARTH
    return x, y

def cvt_gll_ddmm_2_dd(st):
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
    print("      Reference Point: Lat={:.6f}, Lon={:.6f}".format(REF_LAT, REF_LON))
    print("=" * 60)

    try:
        import gps_driver_v2 as gpddrv
        gps = gpddrv.GpsIO()
        gps.set_filter_speed("0")
        print("[OK] GPS connected. Waiting for satellite lock...")
    except Exception as e:
        print("[ERROR] Could not connect to GPS driver: {}".format(e))
        return

    csv_file = "gps_log.csv"
    kml_file = "gps_track.kml"

    with open(csv_file, "w") as f_csv:
        f_csv.write("timestamp,lat,lon,x_east,y_north,distance_from_ref,heading_from_ref\n")

    track_points = []
    print("\nLogging active. Press Ctrl+C to stop.\n")
    print("{:8s} | {:11s} | {:11s} | {:8s} | {:8s} | {:8s}".format(
        'Time', 'Latitude', 'Longitude', 'X (m)', 'Y (m)', 'Dist (m)'))
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

                print("{:8s} | {:11.6f} | {:11.6f} | {:8.2f} | {:8.2f} | {:8.2f}".format(
                    timestamp, lat, lon, x, y, dist))

                with open(csv_file, "a") as f_csv:
                    f_csv.write("{},{:.7f},{:.7f},{:.2f},{:.2f},{:.2f},{:.1f}\n".format(
                        timestamp, lat, lon, x, y, dist, heading))
                track_points.append((lon, lat))

            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n\nStopping GPS logger...")

    if track_points:
        coords_str = " ".join(["{},{},0".format(p[0], p[1]) for p in track_points])
        kml_content = """<?xml version="1.0" encoding="UTF-8"?>
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
        <coordinates>{}</coordinates>
      </LineString>
    </Placemark>
  </Document>
</kml>""".format(coords_str)
        with open(kml_file, "w") as f_kml:
            f_kml.write(kml_content)
        print("Track saved to '{}' (Google Earth compatible) and '{}'.".format(kml_file, csv_file))

if __name__ == "__main__":
    main()
