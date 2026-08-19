import csv
import math

csv_file = '/home/femre/ros-ws/blind_follower_log_20260819_073550.csv'

data = []
with open(csv_file, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        # Convert numeric values
        for key in row:
            if key not in ('state', 'wp_idx', 'zed_diag'):
                try:
                    row[key] = float(row[key])
                except ValueError:
                    pass
        data.append(row)

# Sort by timestamp
data.sort(key=lambda x: x['timestamp'])

base_time = data[0]['timestamp']
for row in data:
    row['time_rel'] = row['timestamp'] - base_time

# Calculate dt and velocities
for i in range(1, len(data)):
    dt = data[i]['time_rel'] - data[i-1]['time_rel']
    data[i]['dt'] = dt
    if dt > 0:
        data[i]['zed_vx'] = (data[i]['zed_x'] - data[i-1]['zed_x']) / dt
        data[i]['zed_vy'] = (data[i]['zed_y'] - data[i-1]['zed_y']) / dt
        data[i]['ekf_vx'] = (data[i]['ekf_x'] - data[i-1]['ekf_x']) / dt
        data[i]['ekf_vy'] = (data[i]['ekf_y'] - data[i-1]['ekf_y']) / dt
    else:
        data[i]['zed_vx'] = 0.0
        data[i]['zed_vy'] = 0.0
        data[i]['ekf_vx'] = 0.0
        data[i]['ekf_vy'] = 0.0

data[0]['dt'] = 0.0
data[0]['zed_vx'] = 0.0
data[0]['zed_vy'] = 0.0
data[0]['ekf_vx'] = 0.0
data[0]['ekf_vy'] = 0.0

print("===== BLIND TEST DATA ANALYSIS =====")
print(f"Total duration: {data[-1]['time_rel']:.2f} seconds")
print(f"Total records: {len(data)}")

states = set([row['state'] for row in data])
print(f"\nStates logged: {states}")

for state in sorted(list(states)):
    state_data = [row for row in data if row['state'] == state]
    if not state_data:
        continue
    duration = state_data[-1]['time_rel'] - state_data[0]['time_rel']
    fwd_pwms = [r['fwd_pwm'] for r in state_data]
    yaw_pwms = [r['yaw_pwm'] for r in state_data]
    
    print(f"\n--- STATE: {state} ---")
    print(f"Duration: {duration:.2f}s")
    if fwd_pwms:
        print(f"FWD PWM Range: {min(fwd_pwms)} - {max(fwd_pwms)}")
        print(f"YAW PWM Range: {min(yaw_pwms)} - {max(yaw_pwms)}")
    
    # Calculate travel
    dx_zed = state_data[-1]['zed_x'] - state_data[0]['zed_x']
    dy_zed = state_data[-1]['zed_y'] - state_data[0]['zed_y']
    dx_ekf = state_data[-1]['ekf_x'] - state_data[0]['ekf_x']
    dy_ekf = state_data[-1]['ekf_y'] - state_data[0]['ekf_y']
    
    zed_travel = math.sqrt(dx_zed**2 + dy_zed**2)
    ekf_travel = math.sqrt(dx_ekf**2 + dy_ekf**2)
    print(f"ZED overall travel in state: {zed_travel:.2f} m")
    print(f"EKF overall travel in state: {ekf_travel:.2f} m")
    
    yaw_z = abs(state_data[-1]['zed_yaw_deg'] - state_data[0]['zed_yaw_deg'])
    yaw_e = abs(state_data[-1]['ekf_yaw_deg'] - state_data[0]['ekf_yaw_deg'])
    print(f"ZED yaw change: {yaw_z:.2f} deg")
    print(f"EKF yaw change: {yaw_e:.2f} deg")

print("\n--- OVERALL PWM vs ZED POSE CORRELATION ---")
fwd_active = [r for r in data if r['fwd_pwm'] > 1550]
if fwd_active:
    fwd_mean_pwm = sum([r['fwd_pwm'] for r in fwd_active]) / len(fwd_active)
    fwd_zed_vx = sum([r['zed_vx'] for r in fwd_active]) / len(fwd_active)
    fwd_zed_vy = sum([r['zed_vy'] for r in fwd_active]) / len(fwd_active)
    fwd_ekf_vx = sum([r['ekf_vx'] for r in fwd_active]) / len(fwd_active)
    fwd_ekf_vy = sum([r['ekf_vy'] for r in fwd_active]) / len(fwd_active)
    
    fwd_zed_ax = sum([r['zed_ax'] for r in fwd_active]) / len(fwd_active)
    fwd_cube_ax = sum([r['cube_ax'] for r in fwd_active]) / len(fwd_active)
    
    print(f"When FWD thrust > 1550 (mean {fwd_mean_pwm:.0f}):")
    print(f"  ZED avg velocity (X, Y): ({fwd_zed_vx:.3f} m/s, {fwd_zed_vy:.3f} m/s)")
    print(f"  EKF avg velocity (X, Y): ({fwd_ekf_vx:.3f} m/s, {fwd_ekf_vy:.3f} m/s)")
    print(f"  ZED measured Acceleration X: {fwd_zed_ax:.3f}")
    print(f"  Cube measured Acceleration X: {fwd_cube_ax:.3f}")

yaw_active = [r for r in data if abs(r['yaw_pwm'] - 1500) > 50 and r['dt'] > 0]
if yaw_active:
    zed_yaw_rate = sum([abs(yaw_active[i]['zed_yaw_deg'] - yaw_active[i-1]['zed_yaw_deg'])/yaw_active[i]['dt'] for i in range(1, len(yaw_active))]) / (len(yaw_active)-1)
    ekf_yaw_rate = sum([abs(yaw_active[i]['ekf_yaw_deg'] - yaw_active[i-1]['ekf_yaw_deg'])/yaw_active[i]['dt'] for i in range(1, len(yaw_active))]) / (len(yaw_active)-1)
    
    print(f"When YAW thrust is active:")
    print(f"  ZED yaw avg absolute change rate: {zed_yaw_rate:.3f} deg/s")
    print(f"  EKF yaw avg absolute change rate: {ekf_yaw_rate:.3f} deg/s")
    
# Generate summary text for saving
summary = []
summary.append("--- KEY DISCREPANCIES FINDINGS ---")

dx_total = abs(data[-1]['zed_x'] - data[-1]['ekf_x'])
dy_total = abs(data[-1]['zed_y'] - data[-1]['ekf_y'])
summary.append(f"Final Position Drift (ZED vs EKF): X={dx_total:.2f}m, Y={dy_total:.2f}m")

# Acceleration noise check
cube_ax_mean = sum([r['cube_ax'] for r in data])/len(data)
zed_ax_mean = sum([r['zed_ax'] for r in data])/len(data)
cube_ax_var = sum([(r['cube_ax'] - cube_ax_mean)**2 for r in data])/len(data)
zed_ax_var = sum([(r['zed_ax'] - zed_ax_mean)**2 for r in data])/len(data)

summary.append(f"Acceleration Variance (Cube vs ZED): Cube={cube_ax_var:.3f}, ZED={zed_ax_var:.3f}")
if zed_ax_var > cube_ax_var * 2:
    summary.append("  -> ZED IMU is much noisier than Cube IMU.")

print("\n".join(summary))

with open('/home/femre/ros-ws/analysis_stats.txt', 'w') as f:
    f.write("\n".join(summary))
    f.write("\nZED output effectively analyzed comparing with EKF and PWM.")
