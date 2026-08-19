import csv
import math

csv_file = '/home/femre/ros-ws/blind_follower_log_20260819_073550.csv'

data = []
with open(csv_file, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        for key in row:
            if key not in ('state', 'wp_idx', 'zed_diag'):
                try:
                    row[key] = float(row[key])
                except ValueError:
                    pass
        data.append(row)

data.sort(key=lambda x: x['timestamp'])
base_time = data[0]['timestamp']
for row in data:
    row['time_rel'] = row['timestamp'] - base_time

# Calibration: calculate initial bias from the first 50 samples (assuming stationary)
calib_samples = 50
zed_ax_bias = sum(r['zed_ax'] for r in data[:calib_samples]) / calib_samples
zed_ay_bias = sum(r['zed_ay'] for r in data[:calib_samples]) / calib_samples
zed_az_bias = sum(r['zed_az'] for r in data[:calib_samples]) / calib_samples

cube_ax_bias = sum(r['cube_ax'] for r in data[:calib_samples]) / calib_samples
cube_ay_bias = sum(r['cube_ay'] for r in data[:calib_samples]) / calib_samples
cube_az_bias = sum(r['cube_az'] for r in data[:calib_samples]) / calib_samples

# Integration Variables
zed_vx, zed_vy, zed_vz = 0.0, 0.0, 0.0
zed_px, zed_py, zed_pz = 0.0, 0.0, 0.0

cube_vx, cube_vy, cube_vz = 0.0, 0.0, 0.0
cube_px, cube_py, cube_pz = 0.0, 0.0, 0.0

for i in range(1, len(data)):
    dt = data[i]['timestamp'] - data[i-1]['timestamp']
    if dt <= 0:
        continue
    
    # Remove bias
    z_ax = data[i]['zed_ax'] - zed_ax_bias
    z_ay = data[i]['zed_ay'] - zed_ay_bias
    z_az = data[i]['zed_az'] - zed_az_bias
    
    c_ax = data[i]['cube_ax'] - cube_ax_bias
    c_ay = data[i]['cube_ay'] - cube_ay_bias
    c_az = data[i]['cube_az'] - cube_az_bias
    
    # Integrate to velocity
    zed_vx += z_ax * dt
    zed_vy += z_ay * dt
    zed_vz += z_az * dt
    
    cube_vx += c_ax * dt
    cube_vy += c_ay * dt
    cube_vz += c_az * dt
    
    # Integrate to position
    zed_px += zed_vx * dt
    zed_py += zed_vy * dt
    zed_pz += zed_vz * dt
    
    cube_px += cube_vx * dt
    cube_py += cube_vy * dt
    cube_pz += cube_vz * dt

zed_total_dist = math.sqrt(zed_px**2 + zed_py**2 + zed_pz**2)
cube_total_dist = math.sqrt(cube_px**2 + cube_py**2 + cube_pz**2)

print("===== RAW IMU DOUBLE INTEGRATION ANALYSIS =====")
print(f"Total time integrated: {data[-1]['time_rel']:.2f}s")
print(f"\n--- ZED IMU Derived Position ---")
print(f"Final Pos (X, Y, Z): ({zed_px:.2f}, {zed_py:.2f}, {zed_pz:.2f}) meters")
print(f"Total Distance magnitude: {zed_total_dist:.2f} meters")

print(f"\n--- Orange Cube IMU Derived Position ---")
print(f"Final Pos (X, Y, Z): ({cube_px:.2f}, {cube_py:.2f}, {cube_pz:.2f}) meters")
print(f"Total Distance magnitude: {cube_total_dist:.2f} meters")

# Compare with the VIO measured ZED position
dx_vio = data[-1]['zed_x'] - data[0]['zed_x']
dy_vio = data[-1]['zed_y'] - data[0]['zed_y']
# We only have XY from the VIO logged prominently, assuming Z is minimal or controlled
dist_vio = math.sqrt(dx_vio**2 + dy_vio**2)
print(f"\n--- ZED VIO (Camera Odometry) Actual Logged Position ---")
print(f"Final Pos Offset (X, Y): ({dx_vio:.2f}, {dy_vio:.2f}) meters")
print(f"Total XY Distance: {dist_vio:.2f} meters")

print("\n--- SUMMARY ---")
print(f"ZED IMU is off by {abs(zed_total_dist - dist_vio):.2f}m from VIO.")
print(f"Cube IMU is off by {abs(cube_total_dist - dist_vio):.2f}m from VIO.")
