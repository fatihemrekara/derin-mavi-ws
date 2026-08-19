import csv
import glob
import os
import sys

def parse_logs():
    logs = sorted(glob.glob('/home/femre/ros-ws/test_logs/*.csv'))
    
    for log_path in logs:
        filename = os.path.basename(log_path)
        print(f"\n" + "="*70)
        print(f"ANALYZING: {filename}")
        
        with open(log_path, 'r') as f:
            reader = csv.DictReader(f)
            rows = list(reader)
            
        if not rows:
            print("Empty log.")
            continue
            
        div_rows = [r for r in rows if r['state'] == 'DIVING']
        fwd_rows = [r for r in rows if r['state'] == 'FORWARD']
        
        print(f"Total entries: {len(rows)}, DIVING: {len(div_rows)}, FORWARD: {len(fwd_rows)}")
        
        depths = [float(r['rel_alt']) for r in rows]
        print(f"Depth Stats -> Min (Deepest): {min(depths):.2f}m, Max: {max(depths):.2f}m")
            
        if fwd_rows:
            head_start = float(fwd_rows[0]['heading_deg'])
            head_end = float(fwd_rows[-1]['heading_deg'])
            min_head = min([float(r['heading_deg']) for r in fwd_rows])
            max_head = max([float(r['heading_deg']) for r in fwd_rows])
            print(f"During FORWARD -> Heading starts at {head_start:.1f} and ends at {head_end:.1f}. (Min: {min_head}, Max: {max_head})")
            
            p_yaw = [float(r['yaw_pwm']) for r in fwd_rows]
            p_fwd = [float(r['fwd_pwm']) for r in fwd_rows]
            p_thr = [float(r['thr_pwm']) for r in fwd_rows]
            print(f"During FORWARD -> FWD PWM: Min={min(p_fwd)}, Max={max(p_fwd)}")
            print(f"During FORWARD -> YAW PWM: Min={min(p_yaw)}, Max={max(p_yaw)}")
            print(f"During FORWARD -> THR PWM: Min={min(p_thr)}, Max={max(p_thr)}")
        
        end_states = [r['state'] for r in rows[-5:]]
        print(f"Last 5 states: {end_states}")

if __name__ == '__main__':
    parse_logs()
