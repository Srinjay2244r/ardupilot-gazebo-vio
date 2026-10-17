#!/usr/bin/env python3
"""Plot relative GPS and camera/gyro trajectory using only the standard csv module + matplotlib/numpy."""
import csv, os, sys
import numpy as np
import matplotlib.pyplot as plt

def read_rows(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))

if not os.path.exists('gps_position.csv') or not os.path.exists('vio_estimate.csv'):
    print('Missing gps_position.csv or vio_estimate.csv. Run both logging processes during the same flight first.')
    sys.exit(1)
gps = read_rows('gps_position.csv')
vio = read_rows('vio_estimate.csv')
if len(gps) < 3 or len(vio) < 3:
    print('Need at least 3 samples from each source. Repeat the flight and record longer.')
    sys.exit(2)
gps_e = np.array([float(r['east_m']) for r in gps]); gps_n = np.array([float(r['north_m']) for r in gps])
vio_x = np.array([float(r['x_m']) for r in vio]); vio_y = np.array([float(r['y_m']) for r in vio])
gps_e -= gps_e[0]; gps_n -= gps_n[0]
vio_x -= vio_x[0]; vio_y -= vio_y[0]
# Normalize each track by cumulative path length. This compares path shape, not metric accuracy.
def normalize_path(x, y):
    d = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    s = np.r_[0.0, np.cumsum(d)]
    if s[-1] > 1e-9:
        return x/s[-1], y/s[-1]
    return x, y
gxe, gyn = normalize_path(gps_e.copy(), gps_n.copy())
vxn, vyn = normalize_path(vio_x.copy(), vio_y.copy())
plt.figure(figsize=(8,6))
plt.plot(gxe, gyn, label='GPS trajectory (normalized path length)')
plt.plot(vxn, vyn, label='Camera optical flow + IMU gyro (normalized path length)')
plt.xlabel('Relative East / normalized distance'); plt.ylabel('Relative North / normalized distance')
plt.title('ArduPilot SITL GPS vs lightweight visual-inertial trajectory')
plt.axis('equal'); plt.grid(True); plt.legend(); plt.tight_layout()
plt.savefig('gps_vs_vio.png', dpi=180)
print('Saved gps_vs_vio.png')
