#!/usr/bin/env python3
"""
GPS -> Yerel metre donusturucu (Publisher versiyonu)

Onceden belirlenmis 3 GPS noktasini (baslangic, bitis, samandira) kullanir,
baslangic noktasini referans (0,0) kabul edip ENU metre degerlerine cevirir
ve /gps_waypoints topic'inden geometry_msgs/PoseArray olarak yayinlar.

PoseArray icindeki poz sirasi: [BASLANGIC, BITIS, SAMANDIRA]
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from geometry_msgs.msg import PoseArray, Pose


# ------------------ WGS84 ellipsoid constants ------------------
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


class GpsToLocalNode(Node):

    def __init__(self):
        super().__init__('gps_to_local')

        self.declare_parameter('frame_id', 'map')
        self.frame_id = str(self.get_parameter('frame_id').value)

        # "Latched" QoS: subscriber sonradan bile baglansa son yayini alsin
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.pub = self.create_publisher(PoseArray, 'gps_waypoints', qos)

        print('=' * 52)
        print(' GPS -> Yerel Metre Donusturucu (PUBLISHER)')
        print(' Sabit noktalar dogrudan yukleniyor...')
        print('=' * 52)

        # Kullanicidan istemek yerine dogrudan belirlenen 6 enlem-boylam degeri (3 nokta)
        start = (41.008200, 28.978400)      # BASLANGIC (Nokta A)
        end = (41.008350, 28.978550)        # BITIS (Nokta B)
        buoy = (41.008120, 28.978600)       # SAMANDIRA (Nokta C)

        lat0, lon0 = start
        points_deg = [('Baslangic', start), ('Bitis', end), ('Samandira', buoy)]

        pa = PoseArray()
        pa.header.frame_id = self.frame_id
        pa.header.stamp = self.get_clock().now().to_msg()

        print('\n' + '=' * 52)
        print(' SONUCLAR (metre, ENU)')
        print('=' * 52)
        for name, (lat, lon) in points_deg:
            e, n = gps_to_local(lat0, lon0, lat, lon)
            dist = math.hypot(e, n)
            print(f'{name:<11}: Dogu={e:+9.2f} m, Kuzey={n:+9.2f} m '
                  f'(baslangica uzaklik: {dist:8.2f} m)')
            p = Pose()
            p.position.x = e
            p.position.y = n
            p.position.z = 0.0
            p.orientation.w = 1.0
            pa.poses.append(p)

        self.pub.publish(pa)
        self.get_logger().info(
            '3 sabit nokta /gps_waypoints topic\'ine yayinlandi. '
            'Node acik kaldigi surece yeni subscriber\'lar da alabilir.')


def main(args=None):
    rclpy.init(args=args)
    node = GpsToLocalNode()
    try:
        rclpy.spin(node)  # transient_local yayin icin acik kalmali
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()