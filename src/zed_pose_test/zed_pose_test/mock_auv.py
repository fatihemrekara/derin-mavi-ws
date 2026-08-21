#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn, VfrHud
from std_msgs.msg import Float64

class MockAuvNode(Node):
    def __init__(self):
        super().__init__('mock_auv')
        
        self.rc_sub = self.create_subscription(OverrideRCIn, '/mavros/rc/override', self.on_rc, 10)
        
        self.vfr_pub = self.create_publisher(VfrHud, '/mavros/vfr_hud', 10)
        self.alt_pub = self.create_publisher(Float64, '/mavros/global_position/rel_alt', 10)
        
        self.timer = self.create_timer(0.1, self.loop)
        
        # Initial states
        self.heading_deg = 90.0 # Start facing East
        self.rel_alt = 0.0      # Start at surface
        
        self.yaw_pwm = 1500
        self.thr_pwm = 1500
        
        self.get_logger().info("Mock AUV Simulator started! Waiting for RC override cmds...")

    def on_rc(self, msg: OverrideRCIn):
        if len(msg.channels) >= 6:
            self.thr_pwm = msg.channels[2]
            self.yaw_pwm = msg.channels[3]
            
    def loop(self):
        # Handle ignore flags or uninitialized values
        if self.thr_pwm == 65535 or self.thr_pwm == 0:
            eff_thr = 1500
        else:
            eff_thr = self.thr_pwm
            
        if self.yaw_pwm == 65535 or self.yaw_pwm == 0:
            eff_yaw = 1500
        else:
            eff_yaw = self.yaw_pwm

        # Altitude physics
        # Values < 1500 means dive (rel_alt should decrease)
        alt_velocity = (eff_thr - 1500) / 100.0
        self.rel_alt += alt_velocity * 0.1 # dt = 0.1
        
        # Heading physics
        # Values < 1500 means CW target in heading_rad, which means CW in compass (decreasing msg.heading)
        yaw_velocity = (eff_yaw - 1500) / 5.0 # degrees per sec
        self.heading_deg += yaw_velocity * 0.1 # dt = 0.1
        
        # Normalize heading to 0-359
        self.heading_deg %= 360.0
        if self.heading_deg < 0:
            self.heading_deg += 360.0
            
        # Publish
        vfr = VfrHud()
        vfr.heading = int(self.heading_deg)
        vfr.altitude = self.rel_alt
        self.vfr_pub.publish(vfr)
        
        alt_msg = Float64()
        alt_msg.data = self.rel_alt
        self.alt_pub.publish(alt_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockAuvNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
