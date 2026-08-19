#!/usr/bin/env python3
"""
Tüm CSV loglarının kapsamlı analizi.
ZED konum doğruluğu, PWM-hareket korelasyonu, sensör karşılaştırması.
"""

import csv
import math
import os
import sys
from collections import defaultdict

# ============================================================
# UTILITY FUNCTIONS
# ============================================================

def load_csv(path):
    """CSV dosyasını sözlük listesi olarak yükle, sayıları float'a çevir."""
    if not os.path.exists(path):
        return []
    rows = []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            for k in row:
                try:
                    row[k] = float(row[k])
                except (ValueError, TypeError):
                    pass
            rows.append(row)
    return rows

def normalize_angle(a):
    while a > 180: a -= 360
    while a <= -180: a += 360
    return a

def stats(values):
    """Min, max, ortalama, std sapma hesapla."""
    if not values:
        return {'n': 0, 'mean': 0, 'std': 0, 'min': 0, 'max': 0}
    n = len(values)
    mean = sum(values) / n
    var = sum((v - mean)**2 for v in values) / max(n - 1, 1)
    return {'n': n, 'mean': mean, 'std': math.sqrt(var), 'min': min(values), 'max': max(values)}

def cumulative_distance(xs, ys):
    """Noktalar arası toplam mesafe (yol uzunluğu)."""
    total = 0.0
    for i in range(1, len(xs)):
        total += math.hypot(xs[i] - xs[i-1], ys[i] - ys[i-1])
    return total

def displacement(xs, ys):
    """Kuş uçuşu başlangıç-bitiş mesafesi."""
    if len(xs) < 2:
        return 0.0
    return math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])

# ============================================================
# 1. BLIND FOLLOWER LOG ANALYSIS
# ============================================================

def analyze_blind_follower(path, label):
    data = load_csv(path)
    if len(data) < 5:
        print(f"\n{'='*70}")
        print(f"  {label}: Yetersiz veri ({len(data)} satır), ATLANACAK")
        print(f"{'='*70}")
        return

    print(f"\n{'='*70}")
    print(f"  {label}")
    print(f"  Dosya: {os.path.basename(path)}")
    print(f"  Toplam satır: {len(data)}")
    print(f"{'='*70}")

    # ---- Zaman bilgisi ----
    t0 = data[0]['timestamp']
    tN = data[-1]['timestamp']
    duration = tN - t0
    print(f"\n  Toplam süre: {duration:.1f} saniye")

    # ---- Durum (state) dağılımı ----
    state_times = defaultdict(float)
    for i in range(1, len(data)):
        dt = data[i]['timestamp'] - data[i-1]['timestamp']
        state_times[data[i-1]['state']] += dt
    
    print(f"\n  --- Durum Dağılımı ---")
    for st, t in sorted(state_times.items(), key=lambda x: -x[1]):
        pct = 100.0 * t / max(duration, 0.01)
        print(f"    {st:25s}: {t:7.1f}s  ({pct:5.1f}%)")

    # ---- ZED Pozisyon Analizi ----
    zed_xs = [r['zed_x'] for r in data if isinstance(r.get('zed_x'), float)]
    zed_ys = [r['zed_y'] for r in data if isinstance(r.get('zed_y'), float)]
    
    if zed_xs and zed_ys:
        zed_cum_dist = cumulative_distance(zed_xs, zed_ys)
        zed_disp = displacement(zed_xs, zed_ys)
        
        print(f"\n  --- ZED VIO Pozisyon ---")
        print(f"    Başlangıç  (X, Y):  ({zed_xs[0]:.3f}, {zed_ys[0]:.3f}) m")
        print(f"    Bitiş      (X, Y):  ({zed_xs[-1]:.3f}, {zed_ys[-1]:.3f}) m")
        print(f"    Kuş uçuşu mesafe:   {zed_disp:.2f} m")
        print(f"    Toplam yol uzunluğu: {zed_cum_dist:.2f} m")
        print(f"    X aralığı:          [{min(zed_xs):.2f}, {max(zed_xs):.2f}] m (genişlik: {max(zed_xs)-min(zed_xs):.2f})")
        print(f"    Y aralığı:          [{min(zed_ys):.2f}, {max(zed_ys):.2f}] m (genişlik: {max(zed_ys)-min(zed_ys):.2f})")

    # ---- EKF Pozisyon Analizi ----
    ekf_xs = [r['ekf_x'] for r in data if isinstance(r.get('ekf_x'), float)]
    ekf_ys = [r['ekf_y'] for r in data if isinstance(r.get('ekf_y'), float)]

    if ekf_xs and ekf_ys:
        ekf_cum_dist = cumulative_distance(ekf_xs, ekf_ys)
        ekf_disp = displacement(ekf_xs, ekf_ys)
        ekf_all_zero = all(abs(x) < 0.001 and abs(y) < 0.001 for x, y in zip(ekf_xs, ekf_ys))
        
        print(f"\n  --- EKF (Pixhawk) Pozisyon ---")
        if ekf_all_zero:
            print(f"    ⚠ EKF çıktısı TAMAMEN SIFIR — EKF çalışmıyor veya veri almıyor!")
        else:
            print(f"    Başlangıç  (X, Y):  ({ekf_xs[0]:.3f}, {ekf_ys[0]:.3f}) m")
            print(f"    Bitiş      (X, Y):  ({ekf_xs[-1]:.3f}, {ekf_ys[-1]:.3f}) m")
            print(f"    Kuş uçuşu mesafe:   {ekf_disp:.2f} m")
            print(f"    Toplam yol uzunluğu: {ekf_cum_dist:.2f} m")

    # ---- PWM → ZED Hız Korelasyonu ----
    # Her durum için ortalama ZED hızı hesapla
    print(f"\n  --- PWM → ZED Hız İlişkisi (Durum bazında) ---")
    
    state_speeds = defaultdict(list)
    state_pwms = defaultdict(lambda: {'fwd': [], 'yaw': []})
    
    for i in range(1, len(data)):
        dt = data[i]['timestamp'] - data[i-1]['timestamp']
        if dt <= 0 or dt > 1.0:
            continue
        dx = data[i]['zed_x'] - data[i-1]['zed_x']
        dy = data[i]['zed_y'] - data[i-1]['zed_y']
        speed = math.hypot(dx, dy) / dt
        state = data[i]['state']
        state_speeds[state].append(speed)
        
        fwd_pwm = data[i].get('fwd_pwm', 1500)
        yaw_pwm = data[i].get('yaw_pwm', 1500)
        if isinstance(fwd_pwm, (int, float)):
            state_pwms[state]['fwd'].append(float(fwd_pwm))
        if isinstance(yaw_pwm, (int, float)):
            state_pwms[state]['yaw'].append(float(yaw_pwm))

    for state in ['FORWARD', 'ROTATING', 'DIVING', 'ARMING', 'DONE']:
        if state not in state_speeds:
            continue
        ss = stats(state_speeds[state])
        fwd_s = stats(state_pwms[state]['fwd']) if state_pwms[state]['fwd'] else None
        yaw_s = stats(state_pwms[state]['yaw']) if state_pwms[state]['yaw'] else None
        
        fwd_str = f"FWD_PWM={fwd_s['mean']:.0f}" if fwd_s else "FWD_PWM=N/A"
        yaw_str = f"YAW_PWM={yaw_s['mean']:.0f}" if yaw_s else "YAW_PWM=N/A"
        
        print(f"    {state:12s}: ZED hız ort={ss['mean']:.3f} m/s (std={ss['std']:.3f}), {fwd_str}, {yaw_str}")

    # ---- FORWARD durumundayken detaylı PWM → hareket analizi ----
    fwd_rows = [r for r in data if r.get('state') == 'FORWARD']
    if len(fwd_rows) > 2:
        fwd_zed_xs = [r['zed_x'] for r in fwd_rows]
        fwd_zed_ys = [r['zed_y'] for r in fwd_rows]
        fwd_dist = cumulative_distance(fwd_zed_xs, fwd_zed_ys)
        fwd_duration_rows = fwd_rows[-1]['timestamp'] - fwd_rows[0]['timestamp']
        fwd_avg_speed = fwd_dist / max(fwd_duration_rows, 0.01)
        
        fwd_pwm_val = fwd_rows[0].get('fwd_pwm', 'N/A')
        
        print(f"\n  --- FORWARD Modu Detaylı ---")
        print(f"    FWD PWM değeri:       {fwd_pwm_val}")
        print(f"    FORWARD süre:         {fwd_duration_rows:.1f} s")
        print(f"    FORWARD ZED mesafe:   {fwd_dist:.2f} m")
        print(f"    FORWARD ZED ort hız:  {fwd_avg_speed:.3f} m/s")
        
        # Yaw sapması (FORWARD sırasında ZED yaw'un ne kadar değiştiği)
        fwd_yaws = [r['zed_yaw_deg'] for r in fwd_rows if isinstance(r.get('zed_yaw_deg'), float)]
        if fwd_yaws:
            yaw_drift = fwd_yaws[-1] - fwd_yaws[0]
            print(f"    FORWARD sırasında ZED yaw sapması: {yaw_drift:.1f}°")
    
    # ---- Heading (Pusula) vs ZED Yaw ----
    heading_vals = [r['heading_deg'] for r in data if isinstance(r.get('heading_deg'), float)]
    zed_yaw_vals = [r['zed_yaw_deg'] for r in data if isinstance(r.get('zed_yaw_deg'), float)]
    
    if heading_vals and zed_yaw_vals and len(heading_vals) == len(zed_yaw_vals):
        diffs = [normalize_angle(h - z) for h, z in zip(heading_vals, zed_yaw_vals)]
        ds = stats(diffs)
        print(f"\n  --- Pusula vs ZED Yaw Farkı ---")
        print(f"    Ortalama fark: {ds['mean']:.1f}° (std: {ds['std']:.1f}°)")
        print(f"    Fark aralığı:  [{ds['min']:.1f}°, {ds['max']:.1f}°]")

    # ---- ZED FPS ----
    fps_vals = [r['zed_fps'] for r in data if isinstance(r.get('zed_fps'), float) and r['zed_fps'] > 0]
    if fps_vals:
        fs = stats(fps_vals)
        print(f"\n  --- ZED FPS ---")
        print(f"    Ortalama: {fs['mean']:.1f}, Min: {fs['min']:.1f}, Max: {fs['max']:.1f}")

    # ---- ZED Diagnostik Hata Kontrolü ----
    diag_issues = [r['zed_diag'] for r in data if r.get('zed_diag') not in ('OK', 'ok', '')]
    if diag_issues:
        unique_issues = set(str(d) for d in diag_issues)
        print(f"\n  --- ZED Diagnostik Sorunlar ---")
        for iss in unique_issues:
            print(f"    ⚠ {iss}")
    else:
        print(f"\n  --- ZED Diagnostik: Tüm test boyunca OK ✓ ---")

    # ---- Derinlik (rel_alt) Analizi ----
    alt_vals = [r['rel_alt'] for r in data if isinstance(r.get('rel_alt'), float)]
    if alt_vals:
        als = stats(alt_vals)
        print(f"\n  --- Derinlik (rel_alt) ---")
        print(f"    Ort: {als['mean']:.2f}m, Min: {als['min']:.2f}m, Max: {als['max']:.2f}m")

    print()

# ============================================================
# 2. ZED POSE LOG ANALYSIS
# ============================================================

def analyze_zed_pose_log(path, label):
    data = load_csv(path)
    if len(data) < 5:
        print(f"\n{'='*70}")
        print(f"  {label}: Yetersiz veri ({len(data)} satır), ATLANACAK")
        print(f"{'='*70}")
        return

    print(f"\n{'='*70}")
    print(f"  {label}")
    print(f"  Dosya: {os.path.basename(path)}")
    print(f"  Toplam satır: {len(data)}")
    print(f"{'='*70}")

    # Sütun isimleri: t_sec, x_m, y_m, z_m, yaw_deg, dist_from_origin_m
    xs = [r['x_m'] for r in data if isinstance(r.get('x_m'), float)]
    ys = [r['y_m'] for r in data if isinstance(r.get('y_m'), float)]
    zs = [r['z_m'] for r in data if isinstance(r.get('z_m'), float)]
    ts = [r['t_sec'] for r in data if isinstance(r.get('t_sec'), float)]
    yaws = [r['yaw_deg'] for r in data if isinstance(r.get('yaw_deg'), float)]
    dists = [r['dist_from_origin_m'] for r in data if isinstance(r.get('dist_from_origin_m'), float)]

    duration = ts[-1] - ts[0] if ts else 0
    print(f"\n  Kayıt süresi: {duration:.1f} saniye")
    
    if xs and ys:
        cum_dist = cumulative_distance(xs, ys)
        disp_xy = displacement(xs, ys)
        
        print(f"\n  --- ZED XY Pozisyon ---")
        print(f"    Başlangıç (X, Y): ({xs[0]:.3f}, {ys[0]:.3f}) m")
        print(f"    Bitiş     (X, Y): ({xs[-1]:.3f}, {ys[-1]:.3f}) m")
        print(f"    Kuş uçuşu mesafe:    {disp_xy:.3f} m")
        print(f"    Toplam yol uzunluğu: {cum_dist:.3f} m")
        print(f"    X aralığı: [{min(xs):.3f}, {max(xs):.3f}] m")
        print(f"    Y aralığı: [{min(ys):.3f}, {max(ys):.3f}] m")

    if zs:
        zst = stats(zs)
        print(f"\n  --- Z Ekseni (Derinlik/Yükseklik) ---")
        print(f"    Ort: {zst['mean']:.3f}m, Min: {zst['min']:.3f}m, Max: {zst['max']:.3f}m")

    if yaws:
        yst = stats(yaws)
        print(f"\n  --- Yaw (Açı) ---")
        print(f"    Ort: {yst['mean']:.1f}°, Min: {yst['min']:.1f}°, Max: {yst['max']:.1f}°")
        print(f"    Toplam yaw değişimi: {yaws[-1] - yaws[0]:.1f}°")

    if dists:
        print(f"\n  --- Orijinden Maksimum Uzaklaşma ---")
        print(f"    Maks: {max(dists):.3f} m (son: {dists[-1]:.3f} m)")

    # ZED drift analizi - kamera hareketsizken ne kadar sapıyor?
    # İlk ve son 10 saniyedeki pozisyon kararlılığı
    if ts and duration > 20:
        early = [(r['x_m'], r['y_m']) for r in data if isinstance(r.get('t_sec'), float) and r['t_sec'] < 10]
        late = [(r['x_m'], r['y_m']) for r in data if isinstance(r.get('t_sec'), float) and r['t_sec'] > duration - 10]
        
        if early:
            ex = stats([p[0] for p in early])
            ey = stats([p[1] for p in early])
            print(f"\n  --- İlk 10s Kararlılık (drift göstergesi) ---")
            print(f"    X std sapma: {ex['std']:.4f} m")
            print(f"    Y std sapma: {ey['std']:.4f} m")

    # Hız profili
    if len(xs) > 1 and len(ts) > 1:
        speeds = []
        for i in range(1, len(data)):
            dt_s = data[i]['t_sec'] - data[i-1]['t_sec']
            if dt_s > 0 and dt_s < 2:
                dx = data[i]['x_m'] - data[i-1]['x_m']
                dy = data[i]['y_m'] - data[i-1]['y_m']
                speeds.append(math.hypot(dx, dy) / dt_s)
        
        if speeds:
            sp = stats(speeds)
            print(f"\n  --- ZED'e Göre Hız Profili ---")
            print(f"    Ort hız: {sp['mean']:.4f} m/s, Maks: {sp['max']:.4f} m/s")

    print()

# ============================================================
# 3. EKF / SENSOR EVALUATION LOG ANALYSIS
# ============================================================

def analyze_ekf_sensor_log(path, label):
    data = load_csv(path)
    if len(data) < 5:
        print(f"\n{'='*70}")
        print(f"  {label}: Yetersiz veri ({len(data)} satır), ATLANACAK")
        print(f"{'='*70}")
        return

    print(f"\n{'='*70}")
    print(f"  {label}")
    print(f"  Dosya: {os.path.basename(path)}")
    print(f"  Toplam satır: {len(data)}")
    print(f"{'='*70}")

    # Sütunlar: Time(s), State, RelAlt(m), Compass_Hdg(deg), EKF_X/Y/Z, ZED_X/Y/Z, IMU_AccX/Y/Z
    ts_key = 'Time(s)'
    ts = [r[ts_key] for r in data if isinstance(r.get(ts_key), float)]
    duration = ts[-1] - ts[0] if len(ts) > 1 else 0
    print(f"\n  Kayıt süresi: {duration:.1f} saniye")

    # Durum dağılımı
    if 'State' in data[0]:
        state_times = defaultdict(float)
        for i in range(1, len(data)):
            dt = data[i][ts_key] - data[i-1][ts_key] if isinstance(data[i].get(ts_key), float) and isinstance(data[i-1].get(ts_key), float) else 0
            state_times[str(data[i-1].get('State', 'N/A'))] += dt
        
        if state_times:
            print(f"\n  --- Durum Dağılımı ---")
            for st, t in sorted(state_times.items(), key=lambda x: -x[1]):
                print(f"    {st:20s}: {t:.1f}s")

    # ZED Pozisyon
    zed_xs = [r['ZED_X'] for r in data if isinstance(r.get('ZED_X'), float)]
    zed_ys = [r['ZED_Y'] for r in data if isinstance(r.get('ZED_Y'), float)]
    zed_zs = [r['ZED_Z'] for r in data if isinstance(r.get('ZED_Z'), float)]

    if zed_xs and zed_ys:
        zed_all_zero = all(abs(x) < 0.001 and abs(y) < 0.001 for x, y in zip(zed_xs, zed_ys))
        cum_dist = cumulative_distance(zed_xs, zed_ys)
        disp = displacement(zed_xs, zed_ys)
        
        print(f"\n  --- ZED Pozisyon ---")
        if zed_all_zero:
            print(f"    ⚠ ZED çıktısı TAMAMEN SIFIR!")
        else:
            print(f"    Başlangıç (X, Y, Z): ({zed_xs[0]:.3f}, {zed_ys[0]:.3f}, {zed_zs[0]:.3f}) m")
            print(f"    Bitiş     (X, Y, Z): ({zed_xs[-1]:.3f}, {zed_ys[-1]:.3f}, {zed_zs[-1]:.3f}) m")
            print(f"    Kuş uçuşu:           {disp:.2f} m")
            print(f"    Toplam yol:           {cum_dist:.2f} m")

    # EKF Pozisyon
    ekf_xs = [r['EKF_X'] for r in data if isinstance(r.get('EKF_X'), float)]
    ekf_ys = [r['EKF_Y'] for r in data if isinstance(r.get('EKF_Y'), float)]
    ekf_zs = [r['EKF_Z'] for r in data if isinstance(r.get('EKF_Z'), float)]

    if ekf_xs and ekf_ys:
        ekf_all_zero = all(abs(x) < 0.001 and abs(y) < 0.001 for x, y in zip(ekf_xs, ekf_ys))
        
        print(f"\n  --- EKF Pozisyon ---")
        if ekf_all_zero:
            print(f"    ⚠ EKF çıktısı TAMAMEN SIFIR — EKF çalışmıyor!")
        else:
            ekf_cum = cumulative_distance(ekf_xs, ekf_ys)
            ekf_disp = displacement(ekf_xs, ekf_ys)
            print(f"    Başlangıç (X, Y, Z): ({ekf_xs[0]:.3f}, {ekf_ys[0]:.3f}, {ekf_zs[0]:.3f}) m")
            print(f"    Bitiş     (X, Y, Z): ({ekf_xs[-1]:.3f}, {ekf_ys[-1]:.3f}, {ekf_zs[-1]:.3f}) m")
            print(f"    Kuş uçuşu:           {ekf_disp:.2f} m")
            print(f"    Toplam yol:           {ekf_cum:.2f} m")

    # ZED vs EKF karşılaştırma
    if zed_xs and ekf_xs and not zed_all_zero and not ekf_all_zero:
        n = min(len(zed_xs), len(ekf_xs))
        pos_errs = [math.hypot(zed_xs[i] - ekf_xs[i], zed_ys[i] - ekf_ys[i]) for i in range(n)]
        pes = stats(pos_errs)
        print(f"\n  --- ZED vs EKF Sapma ---")
        print(f"    Ort pozisyon farkı: {pes['mean']:.3f} m")
        print(f"    Maks pozisyon farkı: {pes['max']:.3f} m")

    # IMU verileri
    imu_ax = [r['IMU_AccX'] for r in data if isinstance(r.get('IMU_AccX'), float)]
    imu_ay = [r['IMU_AccY'] for r in data if isinstance(r.get('IMU_AccY'), float)]
    imu_az = [r['IMU_AccZ'] for r in data if isinstance(r.get('IMU_AccZ'), float)]

    if imu_az:
        axs = stats(imu_ax)
        ays = stats(imu_ay)
        azs = stats(imu_az)
        print(f"\n  --- IMU İvme ---")
        print(f"    AccX ort: {axs['mean']:.3f} (std: {axs['std']:.3f})")
        print(f"    AccY ort: {ays['mean']:.3f} (std: {ays['std']:.3f})")
        print(f"    AccZ ort: {azs['mean']:.3f} (std: {azs['std']:.3f})")

    # Derinlik
    alts = [r['RelAlt(m)'] for r in data if isinstance(r.get('RelAlt(m)'), float)]
    if alts:
        alt_s = stats(alts)
        print(f"\n  --- Derinlik ---")
        print(f"    Ort: {alt_s['mean']:.2f}m, Min: {alt_s['min']:.2f}m, Max: {alt_s['max']:.2f}m")

    print()

# ============================================================
# 4. PASSIVE SENSOR LOG ANALYSIS
# ============================================================

def analyze_passive_sensor_log(path, label):
    data = load_csv(path)
    if len(data) < 5:
        print(f"\n{'='*70}")
        print(f"  {label}: Yetersiz veri ({len(data)} satır), ATLANACAK")
        print(f"{'='*70}")
        return

    print(f"\n{'='*70}")
    print(f"  {label}")
    print(f"  Dosya: {os.path.basename(path)}")
    print(f"  Toplam satır: {len(data)}")
    print(f"{'='*70}")

    # Belirle: hangi sütunlar mevcut
    cols = list(data[0].keys())
    print(f"  Sütunlar: {', '.join(cols)}")

    # Zaman sütununu bul
    time_key = None
    for k in cols:
        if 'time' in k.lower() or 't_sec' in k.lower():
            time_key = k
            break
    
    if time_key:
        ts = [r[time_key] for r in data if isinstance(r.get(time_key), float)]
        if ts:
            duration = ts[-1] - ts[0]
            print(f"\n  Kayıt süresi: {duration:.1f} saniye")

    # ZED verileri
    zed_x_key = next((k for k in cols if 'zed_x' in k.lower() or k == 'ZED_X'), None)
    zed_y_key = next((k for k in cols if 'zed_y' in k.lower() or k == 'ZED_Y'), None)
    
    if zed_x_key and zed_y_key:
        zxs = [r[zed_x_key] for r in data if isinstance(r.get(zed_x_key), float)]
        zys = [r[zed_y_key] for r in data if isinstance(r.get(zed_y_key), float)]
        
        if zxs and zys:
            zed_all_zero = all(abs(x) < 0.001 and abs(y) < 0.001 for x, y in zip(zxs, zys))
            print(f"\n  --- ZED Pozisyon ---")
            if zed_all_zero:
                print(f"    ⚠ ZED çıktısı TAMAMEN SIFIR!")
            else:
                cum = cumulative_distance(zxs, zys)
                disp = displacement(zxs, zys)
                print(f"    Başlangıç (X, Y): ({zxs[0]:.3f}, {zys[0]:.3f}) m")
                print(f"    Bitiş     (X, Y): ({zxs[-1]:.3f}, {zys[-1]:.3f}) m")
                print(f"    Kuş uçuşu:        {disp:.3f} m")
                print(f"    Toplam yol:        {cum:.3f} m")

    # EKF verileri
    ekf_x_key = next((k for k in cols if 'ekf_x' in k.lower() or k == 'EKF_X'), None)
    ekf_y_key = next((k for k in cols if 'ekf_y' in k.lower() or k == 'EKF_Y'), None)

    if ekf_x_key and ekf_y_key:
        exs = [r[ekf_x_key] for r in data if isinstance(r.get(ekf_x_key), float)]
        eys = [r[ekf_y_key] for r in data if isinstance(r.get(ekf_y_key), float)]
        
        if exs and eys:
            ekf_all_zero = all(abs(x) < 0.001 and abs(y) < 0.001 for x, y in zip(exs, eys))
            print(f"\n  --- EKF Pozisyon ---")
            if ekf_all_zero:
                print(f"    ⚠ EKF çıktısı TAMAMEN SIFIR!")
            else:
                cum = cumulative_distance(exs, eys)
                disp = displacement(exs, eys)
                print(f"    Başlangıç (X, Y): ({exs[0]:.3f}, {eys[0]:.3f}) m")
                print(f"    Bitiş     (X, Y): ({exs[-1]:.3f}, {eys[-1]:.3f}) m")
                print(f"    Kuş uçuşu:        {disp:.3f} m")
                print(f"    Toplam yol:        {cum:.3f} m")

    # Bridge verileri (varsa)
    bridge_x_key = next((k for k in cols if 'bridge_x' in k.lower()), None)
    bridge_y_key = next((k for k in cols if 'bridge_y' in k.lower()), None)
    
    if bridge_x_key and bridge_y_key:
        bxs = [r[bridge_x_key] for r in data if isinstance(r.get(bridge_x_key), float)]
        bys = [r[bridge_y_key] for r in data if isinstance(r.get(bridge_y_key), float)]
        
        if bxs and bys:
            print(f"\n  --- Bridge (ZED→MAVROS) Pozisyon ---")
            cum = cumulative_distance(bxs, bys)
            disp = displacement(bxs, bys)
            print(f"    Başlangıç (X, Y): ({bxs[0]:.3f}, {bys[0]:.3f}) m")
            print(f"    Bitiş     (X, Y): ({bxs[-1]:.3f}, {bys[-1]:.3f}) m")
            print(f"    Kuş uçuşu:        {disp:.3f} m")
            print(f"    Toplam yol:        {cum:.3f} m")

    # IMU karşılaştırma
    zed_imu_az = next((k for k in cols if 'zed_imu_az' in k.lower()), None)
    cube_imu_az = next((k for k in cols if 'cube_imu_az' in k.lower()), None)
    
    if zed_imu_az and cube_imu_az:
        zaz = [r[zed_imu_az] for r in data if isinstance(r.get(zed_imu_az), float)]
        caz = [r[cube_imu_az] for r in data if isinstance(r.get(cube_imu_az), float)]
        
        if zaz and caz:
            zs = stats(zaz)
            cs = stats(caz)
            print(f"\n  --- IMU Z-Ekseni Karşılaştırma ---")
            print(f"    ZED  IMU Az: ort={zs['mean']:.3f}, std={zs['std']:.3f}")
            print(f"    Cube IMU Az: ort={cs['mean']:.3f}, std={cs['std']:.3f}")

    print()


# ============================================================
# MAIN - TÜM DOSYALARI SIRAYLA ANALİZ ET
# ============================================================

if __name__ == '__main__':
    BASE = '/home/femre/ros-ws'

    print("=" * 70)
    print("  TÜM CSV LOG DOSYALARI — KAPSAMLI ANALİZ RAPORU")
    print(f"  Tarih: 2026-08-19")
    print("=" * 70)

    # ---- BLIND FOLLOWER LOGS ----
    print("\n" + "▓" * 70)
    print("  BÖLÜM 1: BLIND FOLLOWER LOGLARI")
    print("▓" * 70)

    blind_files = [
        ('blind_follower_log_20260819_071219.csv', 'Blind Follower #1 (07:12)'),
        ('blind_follower_log_20260819_071512.csv', 'Blind Follower #2 (07:15)'),
        ('blind_follower_log_20260819_071752.csv', 'Blind Follower #3 (07:17)'),
        ('blind_follower_log_20260819_072747.csv', 'Blind Follower #4 (07:27)'),
        ('blind_follower_log_20260819_072820.csv', 'Blind Follower #5 (07:28)'),
    ]
    for fname, lbl in blind_files:
        analyze_blind_follower(os.path.join(BASE, fname), lbl)

    # ---- ZED POSE LOGS ----
    print("\n" + "▓" * 70)
    print("  BÖLÜM 2: ZED POSE LOGLARI (Eski testler - 06 Temmuz)")
    print("▓" * 70)

    zed_files = [
        ('zed_pose_log_20260706_115018.csv', 'ZED Pose Log #1 (11:50)'),
        ('zed_pose_log_20260706_115417.csv', 'ZED Pose Log #2 (11:54)'),
        ('zed_pose_log_20260706_180129.csv', 'ZED Pose Log #3 (18:01)'),
    ]
    for fname, lbl in zed_files:
        analyze_zed_pose_log(os.path.join(BASE, fname), lbl)

    # ---- EKF / SENSOR EVALUATION LOGS ----
    print("\n" + "▓" * 70)
    print("  BÖLÜM 3: EKF SENSÖR DEĞERLENDİRME LOGLARI")
    print("▓" * 70)

    ekf_files = [
        ('ekf_sensor_evaluation_log.csv', 'EKF Sensor Eval (kök dizin)'),
        ('src/zed_pose_test/zed_pose_test/ASIL_JETSON_VERISI_ekf_sensor_evaluation_log.csv', 'EKF Sensor Eval (ASIL JETSON VERİSİ)'),
        ('src/zed_pose_test/zed_pose_test/ekf_circle_evaluation_log.csv', 'EKF Circle Evaluation'),
        ('src/zed_pose_test/zed_pose_test/ekf_sensor_evaluation_log.csv', 'EKF Sensor Eval (paket içi)'),
    ]
    for fname, lbl in ekf_files:
        analyze_ekf_sensor_log(os.path.join(BASE, fname), lbl)

    # ---- PASSIVE SENSOR LOGS ----
    print("\n" + "▓" * 70)
    print("  BÖLÜM 4: PASİF SENSÖR LOGLARI")
    print("▓" * 70)

    passive_files = [
        ('src/passive_sensor_log.csv', 'Passive Sensor Log (src/)'),
        ('src/zed_pose_test/zed_pose_test/passive_sensor_log.csv', 'Passive Sensor Log (paket içi)'),
    ]
    for fname, lbl in passive_files:
        analyze_passive_sensor_log(os.path.join(BASE, fname), lbl)

    # ---- ÖZET ----
    print("\n" + "▓" * 70)
    print("  BÖLÜM 5: GENEL ÖZET & SONUÇLAR")
    print("▓" * 70)
    print("""
  Bu özet bölüm script çıktısı tamamlandıktan sonra
  sonuçlara bakılarak oluşturulacaktır.
  
  Analiz scripti tamamlandı.
""")
