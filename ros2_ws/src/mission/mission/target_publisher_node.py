import rclpy
import time
import math
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class GoApproachPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')

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
        self.publisher_target = self.create_publisher(String, '/go_target', qos_profile)
        self.subscriber_ab_call = self.create_subscription(String, '/close', self.close_callback, 10)
        self.subscription_pose = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos_profile_BE)

        """### TARGETS TEST VOL I
        self.timer_period_between_target_switch = 10.0 # sec

        self.target_1 = "0,0,7" # répétabilité en x
        self.target_2 = "3.5,0,7"
        self.target_3 = "0,0,7" 
        self.target_4 = "3.5,0,7"
        self.target_5 = "0,0,7"
        self.target_6 = "3.5,0,7"

        self.target_7 = "0,0,7" # y
        self.target_8 = "0,3.5,7"

        self.target_9 = "0,0,7" # z haut
        self.target_10 = "0,0,15"

        self.target_11 = "0,0,7" # répétabilité en z
        self.target_12 = "0,0,12"
        self.target_13 = "0,0,7"
        self.target_14 = "0,0,12"
        self.target_15 = "0,0,7"
        self.target_16 = "0,0,12"

        self.target_17 = "3.5,0,12" # Z for Zenith
        self.target_18 = "0,0,7"
        self.target_19 = "3.5,0,7"
        self.target_20 = "0,0,7"
        self.target_21 = "3.5,0,12"
        self.target_22 = "0,0,12" # Z for Zenith again
        self.target_23 = "3.5,0,12" 
        self.target_24 = "0,0,7"
        self.target_25 = "3.5,0,7"

        self.target_26 = "3.5,15,7" # to trigger failsafe : target too far
        ### FIN TARGETS TEST DE VOL I
        """
        """### TARGETS TEST DE VOL II
        self.timer_period_between_target_switch = 6.0 # sec

        self.target_1 = "0,0,8"
        self.target_2 = "3,0,8"
        self.target_3 = "0,0,8" 
        self.target_4 = "3,0,8"

        self.target_5 = "0,0,8"
        self.target_6 = "3,3,8"
        self.target_7 = "0,0,8" 
        self.target_8 = "3,3,8"

        self.target_9 = "0,0,8"
        self.target_10 = "3,3,11"
        self.target_11 = "0,0,8" 
        self.target_12 = "3,3,11"

        self.target_13 = "3,3,10"
        self.target_14 = "3,3,9"
        self.target_15 = "3,3,8"
        self.target_16 = "3,3,7"
        self.target_17 = "3,3,10"
        self.target_18 = "3,3,9"
        self.target_19 = "3,3,8"
        self.target_20 = "3,3,7"
        ### FIN TARGETS TEST DE VOL II"""
        """### TARGETS TEST VOL III
        self.timer_period_between_target_switch = 10.0 # sec

        self.target_1 = "0,0,12" # Série de trois approches; avec z, xz puis xyz
        self.target_2 = "5,0,7"
        self.target_3 = "0,0,12" 
        self.target_4 = "5,0,7"
        self.target_5 = "0,0,12"
        self.target_6 = "5,0,7"
        ### FIN TARGETS TEST DE VOL III"""

        # Pour test de vol I, II ou III :
        """self.targets = []
        for i in range(1, 7):  # (1, N+1) À CHANGER DÉPENDANT DE LA MISSION
            self.targets.append(getattr(self, f"target_{i}"))""" # À inclure pour test de vol targets distinctes

        ### TARGETS TEST VOL IV - Cercle
        nombre_de_points_de_cercle = 60
        cercle_chrono = 120 #secondes
        self.timer_period_between_target_switch = cercle_chrono/nombre_de_points_de_cercle # sec
        
        def circular_target(index, radius):
            angle = (- (index - 1) * 2 * math.pi / nombre_de_points_de_cercle)  # sens horaire
            x = round(radius * math.cos(angle), 3)
            y = round(radius * math.sin(angle), 3)
            altitude = round(10 + math.sin(angle), 3)

            target_look_at = [0, 0]
            #direction_look = np.array(target_look_at) - np.array([self.position_actuelle_x, self.position_actuelle_y]) # peut pas être implémenté actuellement car target définit initiallement
            direction_look = np.array(target_look_at) - np.array([x, y]) 
            angle_dir_look = math.atan2(direction_look[1], direction_look[0])
            yaw = round(angle_dir_look, 3)

            targ = f"{x}, {y}, {altitude}, {yaw}"
            self.get_logger().info(f'New targ added : {targ} at index : {index}') # Pour vérif, peut être enlevé sinon
            return self.targets.append(targ)

        self.targets = []
        for index in range(1, nombre_de_points_de_cercle+1):
            circular_target(index, radius=5)
        self.get_logger().info(f'All targets added for test de vol IV - Cercle!')

        ### FIN TARGETS TEST DE VOL IV - Cercle

        self.i = 0
        self.last_target = ""
        self.last_record_time_ct, self.last_record_time_glt = time.time(), time.time()
    
        self.get_logger().info(f"Publisher initialized, will publish to '/go_target' every {self.timer_period_between_target_switch:.3f} seconds when approach start")

    def pose_callback(self, msg):
        try:
            self.position_actuelle_x = msg.pose.position.x
            self.position_actuelle_y = msg.pose.position.y
        except:
            self.position_actuelle_x = 0.0
            self.position_actuelle_y = 0.0

    def timer_callback(self):
        msg = String()
        self.tar = (self.i) % len(self.targets)      # liste et index circulaire
        msg.data = self.targets[self.tar]
        self.publisher_target.publish(msg)

        # Section de contrôle de changement de target et de print terminaux de celle-ci
        current_time = time.time()
        elapsed_change_target = current_time - self.last_record_time_ct
        elapsed_get_logger_target = current_time - self.last_record_time_glt

        if elapsed_change_target > self.timer_period_between_target_switch:
            self.i += 1
            self.last_record_time_ct = current_time

        if elapsed_get_logger_target > 0.5:
            self.get_logger().info(f'Published target at {self.Hertz} Hz: "{msg.data}"')
            self.last_record_time_glt = current_time

    def atg_callback(self, msg):
        if msg.data == "GO!":
            self.get_logger().info(f'message {msg.data} received to start target')
            self.Hertz = 15
            self.timer = self.create_timer(1/self.Hertz, self.timer_callback)

        else:
            self.get_logger().info(f'Error in message received for target, GO! to start : {msg.data}')

    def close_callback(self, msg):
        if msg.data == "close":
            self.destroy_node()
            rclpy.shutdown()
        
def main(args=None):
    rclpy.init(args=args)
    node = GoApproachPublisher()
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