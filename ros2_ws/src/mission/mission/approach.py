import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from rclpy.qos import qos_profile_sensor_data
from pymavlink import mavutil
import time
from helper_func import *

class TargetApproachMavlink(Node):
    def __init__(self):
        super().__init__('target_approach')
        self.start_time = time.time()

        self.subscription = self.create_subscription(
            Point,
            '/target_position',
            self.target_callback,
            qos_profile_sensor_data
        )

        connection = 'udp:127.0.0.1:14550'
        self.mav = pymav()
        self.mav.connect(connection)

        self.get_logger().info("Approach node initialised, mavlink connection successful")



    def target_callback(self, msg):
        x, y, z = msg.x, msg.y, msg.z
        self.get_logger().info(f"Target relative position: x={x:.2f}, y={y:.2f}, z={z:.2f}")
        if z<3:
            pass
            #Could implement some king of logic here for low altitude flight
            #RC override was used to control a camera's gimbal which we wont have in real scenario
            '''self.send_rc_override(8,1500)
            self.send_rc_override(6,1500)
            self.send_rc_override(7, 1300)'''
        if z > 4:
            '''for i in range(6,9):
                self.send_rc_override(i,1500)'''
            Kpy = 0.05
            Kpx = 0.75
            vx = Kpx * z
            vy = Kpy * x
            vz = 0

            vx = max(min(vx, 1.0), -1.0)
            vy = max(min(vy, 1.0), -1.0)
            vz = max(min(vz, 0.5), -0.5)

            self.mav.send_velocity(vx, vy, vz)
            self.get_logger().info(f"Sending velocity command: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}")

        else:
            Kp = 0.15
            vx = -Kp * y #if z > 2.5 else y
            vy = Kp * x #if z > 2.5 else x
            vz = Kp * z if z > 2.5 else 0

            vx = max(min(vx, 1.0), -1.0)
            vy = max(min(vy, 1.0), -1.0)
            vz = max(min(vz, 0.5), -0.5)

            self.mav.send_velocity(vx, vy, vz)
            self.get_logger().info(f"Sending velocity command: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = TargetApproachMavlink()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
