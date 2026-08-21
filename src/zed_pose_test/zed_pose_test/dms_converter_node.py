#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import re
import sys
import select

def parse_dms(dms_input):
    """
    Parses a DMS coordinate string and returns Decimal Degrees (DD).
    Example input: 40°43'20.4"N 29°49'49.5"E
    """
    # Matches patterns like 40°43'20.4"N or 29°49'49.5"E
    # Group 1: Degrees, Group 2: Minutes, Group 3: Seconds, Group 4: Direction (N,S,E,W)
    pattern = r"(\d+)[^\d]+(\d+)[^\d]+([\d.]+)[^\d]*([NSEWnsew])"
    matches = list(re.finditer(pattern, dms_input))
    
    if not matches:
        return None, None

    lat = None
    lon = None
    
    for match in matches:
        deg = float(match.group(1))
        min_ = float(match.group(2))
        sec = float(match.group(3))
        dir_ = match.group(4).upper()
        
        # Dönüşüm Formülü: Ondalık Derece = Derece + (Dakika / 60) + (Saniye / 3600)
        dd = deg + (min_ / 60) + (sec / 3600)
        
        # Güney (S) ve Batı (W) için negatif değerler
        if dir_ in ['S', 'W']:
            dd = -dd
            
        if dir_ in ['N', 'S']:
            lat = dd
        elif dir_ in ['E', 'W']:
            lon = dd
            
    return lat, lon

class DmsConverterNode(Node):
    def __init__(self):
        super().__init__('dms_converter_node')
        self.get_logger().info("DMS to DD Dönüştürücü Başlatıldı.")
        self.get_logger().info("Örnek girdi formatı: 40°43'20.4\"N 29°49'49.5\"E")
        self.get_logger().info("Çıkmak için CTRL+C yapabilirsiniz.\n")
        
        # ROS 2 spin() mekanizmasını engellemeden terminalden input almak için timer
        self.timer = self.create_timer(1.0, self.check_input)
        
    def check_input(self):
        print("\n[DmsConverter] Dönüştürülecek koordinatı yapıştırın ve Enter'a basın: ", end='', flush=True)
        
        # select ile input bekle, 1 saniye timeout ver ki arka planda ROS çalışmaya devam etsin
        i, o, e = select.select([sys.stdin], [], [], 1.0)
        if (i):
            dms_input = sys.stdin.readline().strip()
            if not dms_input:
                return
            
            lat, lon = parse_dms(dms_input)
            
            print("=" * 60)
            if lat is not None and lon is not None:
                print(f"Girdi: {dms_input}\n")
                print(f"Enlem (Latitude) : {lat:.6f}")
                print(f"Boylam (Longitude): {lon:.6f}\n")
                print(f"manual_gps_route_node için kopyalanacak değerler:\nlat = {lat:.6f}\nlon = {lon:.6f}")
            else:
                self.get_logger().warn("Geçersiz veya eksik format! ' 40°43'20.4\"N 29°49'49.5\"E ' şeklinde girmeyi deneyin.")
            print("=" * 60)

def main(args=None):
    rclpy.init(args=args)
    node = DmsConverterNode()
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
