import math
import csv
import sys
import os

REF_LAT = 48.199111
REF_LON = -3.014930
R_EARTH = 6371000.0
LAT_RAD = math.radians(REF_LAT)

def convert(csv_path, kml_path):
    coords = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if 'x' in row and 'y' in row:
                x = float(row['x'])
                y = float(row['y'])
                dlat = y / R_EARTH
                dlon = x / (R_EARTH * math.cos(LAT_RAD))
                lat = REF_LAT + math.degrees(dlat)
                lon = REF_LON + math.degrees(dlon)
                coords.append((lon, lat))
            elif 'lat' in row and 'lon' in row:
                lat = float(row['lat'])
                lon = float(row['lon'])
                coords.append((lon, lat))

    if not coords:
        print("[WARN] No coordinates found in {}".format(csv_path))
        return

    # Filter consecutive duplicate points
    dedup = [coords[0]]
    for pt in coords[1:]:
        if abs(pt[0] - dedup[-1][0]) > 1e-7 or abs(pt[1] - dedup[-1][1]) > 1e-7:
            dedup.append(pt)

    kml = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<kml xmlns="http://www.opengis.net/kml/2.2">',
        '<Document>',
        '  <name>DD-Boat 11 Mission Track</name>',
        '  <Style id="trackLine">',
        '    <LineStyle><color>ff00ffff</color><width>4</width></LineStyle>',
        '  </Style>',
        '  <Placemark>',
        '    <name>Boat 11 Trajectory ({} points)</name>'.format(len(dedup)),
        '    <styleUrl>#trackLine</styleUrl>',
        '    <LineString>',
        '      <tessellate>1</tessellate>',
        '      <coordinates>',
        ' ' + ' '.join('{:.7f},{:.7f},0.0'.format(lon, lat) for lon, lat in dedup),
        '      </coordinates>',
        '    </LineString>',
        '  </Placemark>',
        '</Document>',
        '</kml>'
    ]

    with open(kml_path, 'w') as f:
        f.write('\n'.join(kml))
    print("[OK] Successfully generated {} ({} points).".format(kml_path, len(dedup)))

if __name__ == '__main__':
    csv_file = sys.argv[1] if len(sys.argv) > 1 else 'mission_log.csv'
    kml_file = sys.argv[2] if len(sys.argv) > 2 else 'mission_track.kml'
    convert(csv_file, kml_file)
