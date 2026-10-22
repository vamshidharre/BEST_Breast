import math

REF_LAT = 48.199111
REF_LON = -3.014930
R_EARTH = 6371000.0
LAT_RAD = math.radians(REF_LAT)

def meters_to_latlon(x, y):
    dlat = y / R_EARTH
    dlon = x / (R_EARTH * math.cos(LAT_RAD))
    lat = REF_LAT + math.degrees(dlat)
    lon = REF_LON + math.degrees(dlon)
    return lat, lon

points = []
waypoints = {}

with open("mission_log.csv", "r") as f:
    header = f.readline()
    for line in f:
        parts = line.strip().split(",")
        if len(parts) >= 6:
            try:
                x, y = float(parts[1]), float(parts[2])
                tx, ty = float(parts[3]), float(parts[4])
                wp_idx = int(parts[5])
                lat, lon = meters_to_latlon(x, y)
                points.append((lon, lat))
                if wp_idx not in waypoints:
                    wplat, wplon = meters_to_latlon(tx, ty)
                    waypoints[wp_idx] = (wplon, wplat)
            except Exception:
                pass

print("Total trajectory points parsed: {}".format(len(points)))

coord_str = " ".join(["{:.6f},{:.6f},0".format(p[0], p[1]) for p in points])

kml = []
kml.append('<?xml version="1.0" encoding="UTF-8"?>')
kml.append('<kml xmlns="http://www.opengis.net/kml/2.2">')
kml.append('  <Document>')
kml.append('    <name>DD-Boat 11 Actual Mission Track</name>')
kml.append('    <Style id="trackStyle">')
kml.append('      <LineStyle><color>ff0000ff</color><width>4</width></LineStyle>')
kml.append('    </Style>')
kml.append('    <Placemark>')
kml.append('      <name>Boat GPS Path (Red Line)</name>')
kml.append('      <styleUrl>#trackStyle</styleUrl>')
kml.append('      <LineString>')
kml.append('        <tessellate>1</tessellate>')
kml.append('        <coordinates>{}</coordinates>'.format(coord_str))
kml.append('      </LineString>')
kml.append('    </Placemark>')

for idx in sorted(waypoints.keys()):
    wplon, wplat = waypoints[idx]
    kml.append('    <Placemark>')
    kml.append('      <name>Waypoint {}</name>'.format(idx + 1))
    kml.append('      <Point><coordinates>{:.6f},{:.6f},0</coordinates></Point>'.format(wplon, wplat))
    kml.append('    </Placemark>')

kml.append('  </Document>')
kml.append('</kml>')

out_path = "C:/Users/vamsh/Desktop/ddboat_mission_track.kml"
with open(out_path, "w") as f:
    f.write("\n".join(kml))

print("[OK] KML file created at: " + out_path)
