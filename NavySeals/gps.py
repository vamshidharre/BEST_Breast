#!/usr/bin/env python3
"""
DD-Boat 11 - GPS Acquisition & Georeferencing (Pages 11-22 of best_ddboat.pdf)
=============================================================================
1. Reads GPS positions (NMEA GLL sentences via gps_driver_v2)
2. Converts DDMM.MMMM to decimal degrees
3. Converts coordinates to UTM Zone 30N metric coordinates
4. Computes distance and heading to Guerledan Lake reference point:
   Reference: Lat = 48.199111, Lon = -3.014930
5. Exports GPS positions to 'gps_data.kml' (viewable in Google Earth / Geoportail)
"""

import sys
import os
import time
import math
signal = None
try:
    import signal
except ImportError:
    pass

# Access to drivers
sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))
import gps_driver_v2 as gpddrv

# Reference point "Guerledan's lake" (Page 15 & 16)
REF_LAT = 48.199111
REF_LON = -3.014930

# Try pyproj (UTM Zone 30N) if installed, otherwise exact WGS84 UTM 30N calculation
try:
    from pyproj import Proj
    projDegree2Meter = Proj("+proj=utm +zone=30, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
    def to_utm(lat, lon):
        x, y = projDegree2Meter(lon, lat)
        return x, y
except Exception:
    # Standalone UTM Zone 30N projection (WGS84) fallback
    def to_utm(lat, lon):
        a = 6378137.0
        f = 1.0 / 298.257223563
        e = math.sqrt(2 * f - f * f)
        e_sq = e * e
        e_prime_sq = e_sq / (1.0 - e_sq)
        k0 = 0.9996
        lon0 = -3.0  # Central meridian for UTM Zone 30
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        lon0_rad = math.radians(lon0)
        n = a / math.sqrt(1 - e_sq * math.sin(lat_rad) ** 2)
        t = math.tan(lat_rad) ** 2
        c = e_prime_sq * math.cos(lat_rad) ** 2
        A = (lon_rad - lon0_rad) * math.cos(lat_rad)
        M = a * ((1 - e_sq / 4 - 3 * e_sq**2 / 64 - 5 * e_sq**3 / 256) * lat_rad
                 - (3 * e_sq / 8 + 3 * e_sq**2 / 32 + 45 * e_sq**3 / 1024) * math.sin(2 * lat_rad)
                 + (15 * e_sq**2 / 256 + 45 * e_sq**3 / 1024) * math.sin(4 * lat_rad)
                 - (35 * e_sq**3 / 3072) * math.sin(6 * lat_rad))
        x = 500000.0 + k0 * n * (A + (1 - t + c) * A**3 / 6 + (5 - 18 * t + t**2 + 72 * c - 58 * e_prime_sq) * A**5 / 120)
        y = k0 * (M + n * math.tan(lat_rad) * (A**2 / 2 + (5 - t + 9 * c + 4 * c**2) * A**4 / 24 + (61 - 58 * t + t**2 + 600 * c - 330 * e_prime_sq) * A**6 / 720))
        return x, y

REF_X, REF_Y = to_utm(REF_LAT, REF_LON)

def cvt_gll_ddmm_2_dd(st):
    """Convert lat,lon from DDMM.MMMM to DD.DDDDD (Pages 13 & 17)."""
    ilat = float(st[0])
    ilon = float(st[2])
    olat = float(int(ilat / 100))
    olon = float(int(ilon / 100))
    olat_mm = (ilat % 100) / 60.0
    olon_mm = (ilon % 100) / 60.0
    olat += olat_mm
    olon += olon_mm
    if len(st) > 1 and st[1] == 'S':
        olat = -olat
    if len(st) > 3 and st[3] == "W":
        olon = -olon
    return olat, olon

def write_kml(points, filename="gps_data.kml"):
    """Export points to KML without needing simplekml (Pages 19-20)."""
    kml = ['<?xml version="1.0" encoding="UTF-8"?>',
           '<kml xmlns="http://www.opengis.net/kml/2.2">',
           '<Document>']
    for p in points:
        kml.append('  <Placemark>')
        kml.append('    <name>GPS</name>')
        kml.append('    <Point>')
        kml.append('      <coordinates>{:.8f},{:.8f},0.0</coordinates>'.format(p[1], p[0]))
        kml.append('    </Point>')
        kml.append('  </Placemark>')
    kml.append('</Document>')
    kml.append('</kml>')
    with open(filename, 'w') as f:
        f.write('\n'.join(kml))
    print("[OK] Saved KML track to '{}'.".format(filename))

def main():
    print("=" * 65)
    print("      DD-BOAT 11 : GPS ACQUISITION & GEOREFERENCING         ")
    print("      (best_ddboat.pdf - Pages 11 to 22)                   ")
    print("=" * 65)
    print("Reference point (Guerledan): Lat={:.6f}, Lon={:.6f}".format(REF_LAT, REF_LON))
    print("Reference UTM: x={:.2f} m, y={:.2f} m\n".format(REF_X, REF_Y))

    gps = gpddrv.GpsIO()
    gps.set_filter_speed("0")
    print("GPS initialized. Waiting for satellite lock (Press Ctrl+C to stop)...")
    
    collected_points = []
    sample_count = 0

    try:
        while True:
            gll_ok, gll_data = gps.read_gll_non_blocking()
            if gll_ok:
                lat, lon = cvt_gll_ddmm_2_dd(gll_data)
                x, y = to_utm(lat, lon)
                dx = x - REF_X
                dy = y - REF_Y
                distance = math.sqrt(dx * dx + dy * dy)
                heading_trigo = math.degrees(math.atan2(dy, dx))
                heading_geo = (90.0 - heading_trigo) % 360.0
                
                sample_count += 1
                collected_points.append((lat, lon))
                
                print("[#{:04d}] lat={:.6f} lon={:.6f} | UTM: x={:.1f} y={:.1f} | dx={:+.1f} dy={:+.1f} | Dist={:.1f} m, Heading={:.1f} deg".format(
                    sample_count, lat, lon, x, y, dx, dy, distance, heading_geo))

            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping GPS acquisition...")

    if collected_points:
        write_kml(collected_points, "gps_data.kml")

if __name__ == "__main__":
    main()
