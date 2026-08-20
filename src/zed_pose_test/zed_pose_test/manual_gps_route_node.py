#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseArray, Pose

_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2 - _F)

def geodetic_to_ecef(lat_deg, lon_deg, alt=0.0):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    s = math.sin(lat)
    N = _A / math.sqrt(1 - _E2 * s * s)
    x = (N + alt) * math.cos(lat) * math.cos(lon)
    y = (N + alt) * math.cos(lat) * math.sin(lon)
    z = (N * (1 - _E2) + alt) * s
    return x, y, z

def ecef_to_enu(x, y, z, lat0_deg, lon0_deg, alt0=0.0):
    x0, y0, z0 = geodetic_to_ecef(lat0_deg, lon0_deg, alt0)
    dx, dy, dz = x - x0, y - y0, z - z0
    lat0 = math.radians(lat0_deg)
    lon0 = math.radians(lon0_deg)
    sl, cl = math.sin(lat0), math.cos(lat0)
    so, co = math.sin(lon0), math.cos(lon0)
    east = -so * dx + co * dy
    north = -sl * co * dx - sl * so * dy + cl * dz
    up = cl * co * dx + cl * so * dy + sl * dz
    return east, north, up

def gps_to_local(lat0, lon0, lat, lon):
    x, y, z = geodetic_to_ecef(lat, lon)
    e, n, _ = ecef_to_enu(x, y, z, lat0, lon0)
    return e, n

class GpsWaypointsParamNode(Node):
    def __init__(self):
        super().__init__('manual_gps_route_node')
        
        # Varsayilan Degerler - Kucukcekmece (Eski Senaryo 4 tarzi)
        self.declare_parameter('start_lat', 41.0082000)
        self.declare_parameter('start_lon', 28.9784000)
        
        self.declare_parameter('buoy_lat', 41.0081113)
        self.declare_parameter('buoy_lon', 28.9784186)
        
        self.declare_parameter('end_lat', 41.0082000)
        self.declare_parameter('end_lon', 28.9784000)
        
        self.declare_parameter('frame_id', 'map')
        
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
            
        self.pub = self.create_publisher(PoseArray, 'gps_waypoints', latched)
        
        self.frame_id = str(self.get_parameter('frame_id').value)
        
        # Oku
        self.start_lat = self.get_parameter('start_lat').value
        self.start_lon = self.get_parameter('start_lon').value
        self.buoy_lat = self.get_parameter('buoy_lat').value
        self.buoy_lon = self.get_parameter('buoy_lon').value
        self.end_lat = self.get_parameter('end_lat').value
        self.end_lon = self.get_parameter('end_lon').value
        
        self.get_logger().info('GPS Parametre Düğümü Başlatıldı.')
        self.publish_scenario()
        
    def publish_scenario(self):
        # Ortak Başlangıç Noktası (Orijin kendi üstü)
        # gps_to_local(start_lat, start_lon, start_lat, start_lon) otomatik 0.0, 0.0 donecek.
        e_start, n_start = gps_to_local(self.start_lat, self.start_lon, self.start_lat, self.start_lon)
        
        # Bitiş (route_planner_node Poses[1]'in Bitiş olmasını bekliyor)
        e_end, n_end = gps_to_local(self.start_lat, self.start_lon, self.end_lat, self.end_lon)
        
        # Samandira (route_planner_node Poses[2]'in Samandira olmasını bekliyor)
        e_buoy, n_buoy = gps_to_local(self.start_lat, self.start_lon, self.buoy_lat, self.buoy_lon)

        # Siralama: [BASLANGIC, BITIS, SAMANDIRA]
        points = [
            ("Başlangıç (Orijin)", e_start, n_start),
            ("Bitiş Noktası", e_end, n_end),
            ("Şamandıra Noktası", e_buoy, n_buoy)
        ]

        pa = PoseArray()
        pa.header.frame_id = self.frame_id
        pa.header.stamp = self.get_clock().now().to_msg()
        
        self.get_logger().info('='*60)
        self.get_logger().info(f' GÖNDERİLEN GPS KOORDİNATLARI (Metre Dönüşümleriyle)')
        self.get_logger().info('='*60)
        
        for name, px, py in points:
            p = Pose()
            p.position.x = float(px)
            p.position.y = float(py)
            p.position.z = 0.0
            p.orientation.w = 1.0
            pa.poses.append(p)
            self.get_logger().info(f'{name:<20}: X= {px:>7.2f}m , Y= {py:>7.2f}m')
            
        self.pub.publish(pa)
        self.get_logger().info('='*60)
        self.get_logger().info('Noktalar /gps_waypoints topic\'ine bırakıldı!')

def main(args=None):
    rclpy.init(args=args)
    node = GpsWaypointsParamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
