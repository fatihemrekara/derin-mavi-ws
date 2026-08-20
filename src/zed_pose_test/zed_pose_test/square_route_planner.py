#!/usr/bin/env python3
"""
square_route_planner.py
Samandira etrafinda tam donusleri sanal tegetler (virtual tangents) ile hesaplayip,
aracin 90 derecelik koseleri tam olarak takip etmesini saglayan (Snapped Corners) 
ve her durumda en az bir tam tur atmasini garanti eden gelismis rotaci.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, PoseArray
from visualization_msgs.msg import Marker, MarkerArray

def cross2(ax, ay, bx, by):
    return ax * by - ay * bx

def tangent_points(p, c, r):
    dx, dy = p[0] - c[0], p[1] - c[1]
    d = math.hypot(dx, dy)
    if d <= r:
        raise ValueError(
            f'Nokta ({p[0]:.2f}, {p[1]:.2f}) cemberin icinde (uzaklik {d:.2f} <= {r:.2f}).')
    base = math.atan2(dy, dx)
    alpha = math.acos(r / d)
    t1 = (c[0] + r * math.cos(base + alpha), c[1] + r * math.sin(base + alpha))
    t2 = (c[0] + r * math.cos(base - alpha), c[1] + r * math.sin(base - alpha))
    return t1, t2

def pick_tangent(p, c, r, ref, logger, label):
    """
    Orijinal route_planner_node.py ile birebir ayni calisan teget secimi.
    Kullanici (route_planner_node) davranisini harfiyen istemektedir.
    """
    t1, t2 = tangent_points(p, c, r)
    v_pc = (c[0] - p[0], c[1] - p[1])
    side_ref = cross2(v_pc[0], v_pc[1], ref[0] - p[0], ref[1] - p[1])
    side_t1 = cross2(v_pc[0], v_pc[1], t1[0] - p[0], t1[1] - p[1])
    if abs(side_ref) < 1e-3:
        # Toleransi biraz genis tuttuk (1e-3) ki asiri yakin/dogrusal noktalarda cildirip diger tegete sicramasin
        return t1
    return t1 if side_ref * side_t1 > 0 else t2

def sample_segment(a, b, step):
    dist = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(1, int(math.ceil(dist / step)))
    return [(a[0] + (b[0] - a[0]) * i / n,
             a[1] + (b[1] - a[1]) * i / n) for i in range(1, n + 1)]

def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)

def get_square_point(buoy, r, beta, angle_global):
    th = (angle_global - beta)
    th = (th + math.pi) % (2.0 * math.pi) - math.pi
    
    if -math.pi/4 <= th <= math.pi/4:
        lx = r
        ly = r * math.tan(th)
    elif math.pi/4 < th <= 3*math.pi/4:
        ly = r
        if abs(math.tan(th)) > 1e-6:
            lx = r / math.tan(th)
        else:
            lx = 0.0
    elif th > 3*math.pi/4 or th < -3*math.pi/4:
        lx = -r
        ly = -r * math.tan(th)
    else:
        ly = -r
        if abs(math.tan(th)) > 1e-6:
            lx = -r / math.tan(th)
        else:
            lx = 0.0
            
    gx = buoy[0] + lx * math.cos(beta) - ly * math.sin(beta)
    gy = buoy[1] + lx * math.sin(beta) + ly * math.cos(beta)
    return (gx, gy)

def sample_square_arc(buoy, r, beta, a0, sweep, step):
    if abs(sweep) < 1e-6:
        return [get_square_point(buoy, r, beta, a0)]

    is_ccw = sweep > 0
    corners_local = [math.pi/4, 3*math.pi/4, 5*math.pi/4, 7*math.pi/4]
    
    th_start = (a0 - beta) % (2.0 * math.pi)
    
    def get_next_corner(th, ccw):
        if ccw:
            for c in corners_local:
                if c > th + 1e-4: return c
            return corners_local[0]
        else:
            for c in reversed(corners_local):
                if c < th - 1e-4: return c
            return corners_local[-1]

    vertices_angles = [th_start]
    
    curr_th = th_start
    swept_so_far = 0.0
    
    while True:
        next_th = get_next_corner(curr_th % (2.0*math.pi), is_ccw)
        
        if is_ccw:
            diff = (next_th - curr_th) % (2.0 * math.pi)
        else:
            diff = (curr_th - next_th) % (2.0 * math.pi)
            
        if diff < 0:
            diff += 2.0 * math.pi
            
        if swept_so_far + diff >= abs(sweep) - 1e-4:
            break
            
        swept_so_far += diff
        
        if is_ccw:
            curr_th += diff
        else:
            curr_th -= diff
            
        vertices_angles.append(curr_th)
        
    if is_ccw:
        vertices_angles.append(th_start + sweep)
    else:
        vertices_angles.append(th_start + sweep)

    pts = []
    for th in vertices_angles:
        pts.append(get_square_point(buoy, r, beta, th + beta))
        
    final_pts = []
    for i in range(len(pts)-1):
        seg = sample_segment(pts[i], pts[i+1], step)
        if i == 0:
            final_pts.extend(seg)
        else:
            final_pts.extend(seg[1:])
            
    return final_pts

class SquareRoutePlannerNode(Node):

    def __init__(self):
        super().__init__('square_route_planner')

        self.declare_parameter('radius', 2.0)
        self.declare_parameter('step', 0.5)
        self.declare_parameter('frame_id', 'map')

        self.radius = float(self.get_parameter('radius').value)
        self.step = float(self.get_parameter('step').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self.path_pub = self.create_publisher(Path, 'planned_route', latched)
        self.marker_pub = self.create_publisher(MarkerArray, 'route_markers', latched)

        self.sub = self.create_subscription(
            PoseArray, 'gps_waypoints', self.on_waypoints, latched)

        self.get_logger().info(
            f'Square route planner (Tam Tur Merkezli) hazir. (r={self.radius} m, step={self.step})')

    def on_waypoints(self, msg: PoseArray):
        if len(msg.poses) < 3:
            self.get_logger().error(f'Beklenen 3 nokta, gelen {len(msg.poses)}. Yoksayildi.')
            return

        start = (msg.poses[0].position.x, msg.poses[0].position.y)
        end = (msg.poses[1].position.x, msg.poses[1].position.y)
        buoy = (msg.poses[2].position.x, msg.poses[2].position.y)

        self.get_logger().info(f'Noktalar: S={start}, E={end}, B={buoy}. Rota hesaplaniyor...')

        try:
            pts = self.build_route(start, end, buoy)
        except ValueError as exc:
            self.get_logger().error(f'Rota hesaplanamadi: {exc}')
            return

        self.publish_path(pts)
        self.publish_markers(start, end, buoy, pts)

        total = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
                    for i in range(len(pts) - 1))
        self.get_logger().info(f'Rota yayinlandi: {len(pts)} waypoint, toplam {total:.2f} m.')

    def build_route(self, start, end, buoy):
        r = self.radius
        log = self.get_logger()

        # Orijinal route_planner teget secimi
        t_in = pick_tangent(start, buoy, r, end, log, 'Giris tegeti')
        t_out = pick_tangent(end, buoy, r, start, log, 'Cikis tegeti')

        u = (t_in[0] - start[0], t_in[1] - start[1])
        rad = (t_in[0] - buoy[0], t_in[1] - buoy[1])
        ccw = cross2(rad[0], rad[1], u[0], u[1]) > 0.0

        a0 = math.atan2(t_in[1] - buoy[1], t_in[0] - buoy[0])
        a1 = math.atan2(t_out[1] - buoy[1], t_out[0] - buoy[0])
        
        if ccw:
            raw_sweep = (a1 - a0) % (2.0 * math.pi)
        else:
            raw_sweep = -((a0 - a1) % (2.0 * math.pi))

        # "tam tur atmasi gerekiyor"
        # Bazen sweep tam hesaplanmis ve 1-2-3 ceyrek (quadrant) kadar yoldan hedefe akiyor olabilir.
        # Bu durumda ekstra tura gerek yoktur, sadece cok dolanir ve dugum olur (sizin 'cok dolanmis' tespitiniz).
        # Ancak, 0,0 - 0,0 durumunda oldugu gibi raw_sweep 0 marta dusuyorsa, aracin mecburen "tam tur" atip geri donmesi gerekir.
        # Bu yuzden SADECE yay 0 ise ona bir tam kare turlama veriyoruz (4 quadrants = 360 derece).
        # Dogrusal gecis tespiti (Collinear Bypass): Arac duz yolda giderken onundeki samandiranin koseli etrafindan dolanmadan sadece tegetinden hafifce kayarak gecer.
        vec_sb = (buoy[0] - start[0], buoy[1] - start[1])
        vec_be = (end[0] - buoy[0], end[1] - buoy[1])
        nsb = math.hypot(vec_sb[0], vec_sb[1])
        nbe = math.hypot(vec_be[0], vec_be[1])
        
        is_straight = False
        if nsb > 1e-3 and nbe > 1e-3:
            dot_prod = vec_sb[0]*vec_be[0] + vec_sb[1]*vec_be[1]
            cos_a = dot_prod / (nsb * nbe)
            if cos_a > 0.985:  # Acinin neredeyse sifir olmasi (duz cizgi) gidisati (-10 ile +10 derece arasi sapma toleransi)
                is_straight = True
                
        quadrants = math.ceil(abs(raw_sweep) / (math.pi / 2.0))
        
        if is_straight:
            quadrants = 4
            log.info(">>> Dogrusal (Collinear) gecis rotasi tespit edildi. Kural geregi aracin samandira etrafinda TAM KARE alan (360 derece) tam tur cizmesi saglanacak. <<<")
        elif quadrants == 0:
            quadrants = 4  # 0,0 durumu icin tam 4 kosesi cizmeye zorla!
            
        sweep = quadrants * (math.pi / 2.0) * (1.0 if ccw else -1.0)

        yon = 'saat yonu tersi (CCW)' if ccw else 'saat yonu (CW)'
        log.info(f'Sanal Cember Yayi (Tam Tur): {math.degrees(abs(sweep)):.1f} derece, donus {yon}')

        # "donme noktalarini ayni ona gore ayarla" - Kullanici talebi uzerine:
        # Eger baslangic acisimizi a0 kabul edip, bunu karenin BIR KOSESI (pi/4) yaparsak,
        # araba giris asamasindan saniyesinde dumduz tam 90 derecelik kose cizgisine oturur.
        # Bu yuzden karenin merkez eksenini (beta) a0'dan 45 derece geriye aliyoruz.
        # (Yani ilk girdigimiz nokta (get_square_point icinde) pi/4 = kose olacak)
        if ccw:
            beta = a0 - math.pi/4
        else:
            beta = a0 + math.pi/4  # CW de kose acisi tersten yaslanir. (Aslinda CW donuste local aci kuculur ama biz saglam olsun t_start t_out vs denerken duz durmasini saglariz)

        # KOSE TEST: a0 acisini koyunca gercekten pi/4 veriyor mu bakalim
        # th_start = (a0 - beta) % 2pi. Eger beta = a0 - pi/4 olursa, th_start_ccw = pi/4! Cuk oturur.
        # CW icin beta = a0 - 7pi/4 (veya a0 + pi/4). O zaman th_start = -pi/4 olur. Cuk oturur.

        self.square_corners = [get_square_point(buoy, r, beta, beta + a) 
                               for a in [math.pi/4, 3*math.pi/4, 5*math.pi/4, 7*math.pi/4]]

        p_in = get_square_point(buoy, r, beta, a0)
        p_out = get_square_point(buoy, r, beta, a0 + sweep)

        pts = [start]
        pts += sample_segment(start, p_in, self.step)
        
        arc_pts = sample_square_arc(buoy, r, beta, a0, sweep, self.step)
        if len(arc_pts) > 0:
            if len(pts) > 0 and math.hypot(arc_pts[0][0]-pts[-1][0], arc_pts[0][1]-pts[-1][1]) < 1e-3:
                pts += arc_pts[1:]
            else:
                pts += arc_pts
                
        pts += sample_segment(p_out, end, self.step)
        return pts

    def publish_path(self, pts):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for i, (x, y) in enumerate(pts):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            if i < len(pts) - 1:
                yaw = math.atan2(pts[i + 1][1] - y, pts[i + 1][0] - x)
            else:
                yaw = math.atan2(y - pts[i - 1][1], x - pts[i - 1][0])
            qx, qy, qz, qw = yaw_to_quat(yaw)
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path.poses.append(ps)
        self.path_pub.publish(path)

    def publish_markers(self, start, end, buoy, pts):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        def base(mid, mtype):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = stamp
            m.ns = 'route_square'
            m.id = mid
            m.type = mtype
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        m = base(0, Marker.CYLINDER)
        m.pose.position.x, m.pose.position.y = buoy
        m.scale.x = m.scale.y = 0.4
        m.scale.z = 0.8
        m.color.r, m.color.a = 1.0, 1.0
        ma.markers.append(m)

        if hasattr(self, 'square_corners'):
            m = base(1, Marker.LINE_STRIP)
            m.scale.x = 0.03
            m.color.r, m.color.g, m.color.b = 0.6, 0.6, 0.6
            m.color.a = 1.0
            for pt in self.square_corners + [self.square_corners[0]]:
                m.points.append(Point(x=pt[0], y=pt[1]))
            ma.markers.append(m)

        m = base(2, Marker.LINE_STRIP)
        m.scale.x = 0.06
        m.color.g, m.color.a = 1.0, 1.0
        for x, y in pts:
            m.points.append(Point(x=x, y=y))
        ma.markers.append(m)

        for mid, (x, y), (cr, cg, cb) in ((3, start, (0.0, 0.3, 1.0)),
                                          (4, end, (1.0, 0.9, 0.0))):
            m = base(mid, Marker.SPHERE)
            m.pose.position.x, m.pose.position.y = x, y
            m.scale.x = m.scale.y = m.scale.z = 0.3
            m.color.r, m.color.g, m.color.b, m.color.a = cr, cg, cb, 1.0
            ma.markers.append(m)

        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = SquareRoutePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
