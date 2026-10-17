#!/usr/bin/env python3
"""Log ArduPilot SITL GPS position from MAVLink UDP 14550 to gps_position.csv."""
import csv, math, sys, time
from pymavlink import mavutil

OUT = 'gps_position.csv'
R = 6378137.0
print('Connecting to MAVLink UDP 14550. Ensure sim_vehicle/MAVProxy is sending to 127.0.0.1:14550.')
conn = mavutil.mavlink_connection('udpin:0.0.0.0:14550', source_system=255)
conn.wait_heartbeat(timeout=20)
print('Heartbeat received; logging GPS. Press Ctrl+C to stop.')
origin = None
n = 0
with open(OUT, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['timestamp_sec','east_m','north_m','rel_alt_m','latitude_deg','longitude_deg'])
    try:
        while True:
            m = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
            if m is None:
                print('No GLOBAL_POSITION_INT yet; waiting...')
                continue
            lat = m.lat / 1e7
            lon = m.lon / 1e7
            alt = m.relative_alt / 1000.0
            t = m.time_boot_ms / 1000.0
            if origin is None:
                origin = (lat, lon)
            lat0, lon0 = origin
            east = math.radians(lon-lon0) * R * math.cos(math.radians(lat0))
            north = math.radians(lat-lat0) * R
            w.writerow([t, east, north, alt, lat, lon])
            n += 1
            if n % 10 == 0:
                f.flush()
                print(f'GPS samples={n} EN=({east:.2f},{north:.2f}) m alt={alt:.1f}m')
    except KeyboardInterrupt:
        pass
print(f'Saved {n} GPS samples to {OUT}')
