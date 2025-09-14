import rclpy
import time
import math
import numpy as np
from zenmav.core import Zenmav
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class GoAlignPublisher(Node):
    def __init__(self):
        super().__init__('target_baselink_publisher')

        time.sleep(1)
        self.get_logger().info(f"Zenmav port target baselink va s'initialiser")
        self.drone = Zenmav(ip='tcp:127.0.0.1:14554') # Se connecte à un des splits ports de Zenmav de abort_brake node
        self.get_logger().info(f'Zenmav port target baselink est connecté!')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=8
        )
        qos_profile_BE = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=8
        )

        self.subscriber_atg = self.create_subscription(String, '/approach_target_graph', self.atg_callback, qos_profile)
        self.publisher_target = self.create_publisher(String, '/go_target_baselink', qos_profile)
        self.subscriber_ab_call = self.create_subscription(String, '/close', self.close_callback, 10)
        self.position_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos_profile_BE)

        ### TARGETS TEST VOL I
        self.timer_period_between_target_switch = 14.0 # sec
        nbr_targets = 4 # Faire varier

        self.target_1 = [0,0,10,0] # North, East, Haut| yaw sont en degrés!
        self.target_2 = [0,0,10,170]
        self.target_3 = [0,0,10,0]
        self.target_4 = [0,0,10,-170]
        #self.target_5 = [-2,2,10,0]
        #self.target_6 = [2,-2,10,0]
        #self.target_7 = [10,10,20,np.pi/16] # to trigger failsafe : target baselink too far
        
        self.targets = []
        for i in range(1, nbr_targets+1):
            self.targets.append(getattr(self, f"target_{i}"))
        ### FIN TARGETS TEST DE VOL I

        self.i = 0
        self.last_target = ""
        self.last_record_time_ct = time.time()
    
        self.get_logger().info(f"Publisher initialized, will publish to '/go_target_baselink' every {self.timer_period_between_target_switch:.3f} seconds when align start")

    def timer_callback(self):
        msg = String()

        # Target, composantes linéaires
        tar = (self.i) % len(self.targets)      # liste et index circulaire
        target_local = np.array(self.targets[tar])
        target_local_ny = target_local[0:3] # sans le yaw (ny)
        #self.get_logger().info(f'target_local_ny est : {target_local_ny}')
        position_actuelle_locale_ny = np.array([self.curr_pos.x, self.curr_pos.y, self.curr_pos.z])
        self.get_logger().info(f'position_actuelle_locale_ny est : {position_actuelle_locale_ny}')
        target_baselink_ny = target_local_ny - position_actuelle_locale_ny # sans yaw (ny)
        #self.get_logger().info(f'target_baselink_ny est : {target_baselink_ny} en NEU')

        # Target TF [N,E,U] --> [F,L,U]
        hdg_deg = self.drone.get_global_pos(heading=True).hdg # degrés
        hdg_rad = np.deg2rad(hdg_deg)
        ang_rota = np.pi/2 - hdg_rad

        R = np.array([[np.cos(ang_rota),  np.sin(ang_rota)],[-np.sin(ang_rota), np.cos(ang_rota)]])
        EN = [target_baselink_ny[1], target_baselink_ny[0]]
        FL = R @ EN

        epsilon = (FL[0])**2 + (FL[1])**2 - (EN[0])**2 - (EN[1])**2
        self.get_logger().info(f'epsilon FL-EN : {epsilon:.3f}')

        # Yaw Baselink
        yaw_bl = target_local[3] - hdg_deg
        if abs(yaw_bl) > 180:
            yaw_bl = target_local[3] + 360 - hdg_deg

        target_baselink = f"{FL[0]:.3f},{FL[1]:.3f},{target_baselink_ny[2]:.3f},{yaw_bl:.3f}"
        self.get_logger().info(f'Venant de la target locale "{target_local}", le yaw du drone est : "{hdg_deg:.3f}" degrés')
        self.get_logger().info(f'target_baselink est : {target_baselink}, en FLU')

        msg.data = target_baselink
        self.publisher_target.publish(msg)

        # Section de contrôle de changement de target
        current_time = time.time()
        elapsed_change_target = current_time - self.last_record_time_ct

        if elapsed_change_target > self.timer_period_between_target_switch:
            self.i += 1
            self.last_record_time_ct = current_time

    def pose_callback(self, msg):
        self.curr_pos = msg.pose.position

    def atg_callback(self, msg):
        if msg.data == "GO!":
            self.get_logger().info(f'message {msg.data} received to start FLU target baselink')
            self.Hertz = 20
            self.timer = self.create_timer(1/self.Hertz, self.timer_callback)

        else:
            self.get_logger().info(f'Error in message received for FLU target baselink, GO! to start : {msg.data}')

    def close_callback(self, msg):
        if msg.data == "close":
            self.destroy_node()
            rclpy.shutdown()
        
def main(args=None):
    rclpy.init(args=args)
    node = GoAlignPublisher()
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