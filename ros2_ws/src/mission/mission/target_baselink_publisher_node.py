import rclpy
import time
import math
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class GoAlignPublisher(Node):
    def __init__(self):
        super().__init__('target_baselink_publisher')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=8
        )
        
        self.subscriber_atg = self.create_subscription(String, '/approach_target_graph', self.atg_callback, qos_profile)
        self.publisher_target = self.create_publisher(String, '/go_target_baselink', qos_profile)
        self.subscriber_ab_call = self.create_subscription(String, '/close', self.close_callback, 10)

        ### TARGETS TEST VOL I
        self.timer_period_between_target_switch = 8.0 # sec

        self.target_1 = "0,0,10"
        self.target_2 = "2,0,10"
        self.target_3 = "0,2,12" 
        self.target_4 = "2,0,12"
        self.target_5 = "0,2,10"
        self.target_6 = "2,0,10"

        self.target_7 = "10,10,20" # to trigger failsafe : target baselink too far
        ### FIN TARGETS TEST DE VOL I
        
        # Pour test de vol I :
        self.targets = []
        for i in range(1, 8):  # (1, N+1) À CHANGER DÉPENDANT DE LA MISSION
            self.targets.append(getattr(self, f"target_{i}"))

        """ EX ### TARGETS TEST VOL IV - Cercle
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

        ### FIN TARGETS TEST DE VOL IV - Cercle"""

        self.i = 0
        self.last_target = ""
        self.last_record_time_ct, self.last_record_time_glt = time.time(), time.time()
    
        self.get_logger().info(f"Publisher initialized, will publish to '/go_target_baselink' every {self.timer_period_between_target_switch:.3f} seconds when align start")

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
            self.get_logger().info(f'Published baselink target at {self.Hertz} Hz: "{msg.data}"')
            self.last_record_time_glt = current_time

    def atg_callback(self, msg):
        if msg.data == "GO!":
            self.get_logger().info(f'message {msg.data} received to start target baselink')
            self.Hertz = 15
            self.timer = self.create_timer(1/self.Hertz, self.timer_callback)

        else:
            self.get_logger().info(f'Error in message received for target baselink, GO! to start : {msg.data}')

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