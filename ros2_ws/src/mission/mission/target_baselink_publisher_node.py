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
        self.timer_period_between_target_switch = 10.0 # sec

        self.target_1 = [0,0,10,0] # Tout isoler, vérif breaking point (mth bissection), calculer a la main, et vérif des cas très simple, style rester au même point et jarter align pour juste analyser le output target_baselink!
        #self.target_2 = [-5,0,10,0]
        #self.target_3 = [-2,2,0,0] 
        #self.target_4 = [2,-2,0,0]
        #self.target_5 = [-2,2,10,0]
        #self.target_6 = [2,-2,10,0]

        #self.target_7 = [10,10,20,np.pi/16] # to trigger failsafe : target baselink too far
        ### FIN TARGETS TEST DE VOL I
        
        # Pour test de vol I :
        self.targets = []
        for i in range(1, 2):  # (1, N+1) À CHANGER DÉPENDANT DE LA MISSION
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
        self.target_local = np.array(self.targets[self.tar])
        self.target_local_ny = self.target_local[0:3] # sans le yaw
        self.get_logger().info(f'self.target_local_ny EST LA SUIVANTE : "{self.target_local_ny}", donc sans le yaw') ##
        
        self.position_actuelle_locale_ny = np.array([self.curr_pos.x, self.curr_pos.y, self.curr_pos.z])
        self.get_logger().info(f'self.position_actuelle_locale_ny EST LA SUIVANTE : "{self.position_actuelle_locale_ny}", donc pos sans le yaw') ##
        self.target_baselink_ny = self.target_local_ny - self.position_actuelle_locale_ny # sans yaw (ny)
        self.get_logger().info(f'self.target_baselink_ny EST LA SUIVANTE : "{self.target_baselink_ny}", donc pos baselink sans le yaw') ##

        # TF [N,E,U] --> [F,L,U]
        hdg_rad = self.drone.get_global_pos(heading=True).hdg # rad
        F_1 = self.target_baselink_ny[0] * np.cos(hdg_rad)
        L_1 = self.target_baselink_ny[0] * np.sin(hdg_rad)
        F_2 = self.target_baselink_ny[1] * np.sin(hdg_rad)
        L_2 = self.target_baselink_ny[1] * np.cos(hdg_rad + np.pi)
        self.get_logger().info(f'L2') ##

        self.get_logger().info(f'YAW de la target de linstant EST LA SUIVANTE : "{self.Y:.3f}" degrés') ##
        # yaw baselink
        self.Y = self.target_local[3] - hdg_rad

        self.target_baselink = f"{F_1 + F_2},{L_1 + L_2},{self.target_baselink_ny[2]},{self.Y}"
        #self.get_logger().info(f'YAW de la target de linstant EST LA SUIVANTE : "{self.Y*180/np.pi:.3f}" degrés') ## car Zenmav renvoit des degrés
        self.get_logger().info(f'YAW de la target de linstant EST LA SUIVANTE : "{self.Y:.3f}" degrés') ##
        self.get_logger().info(f'self.target_baselink EST LA SUIVANTE : "{self.target_baselink}", donc tar baselink') ##

        msg.data = self.target_baselink
        self.publisher_target.publish(msg)

        # Section de contrôle de changement de target et de print terminaux de celle-ci
        current_time = time.time()
        elapsed_change_target = current_time - self.last_record_time_ct
        elapsed_get_logger_target = current_time - self.last_record_time_glt

        if elapsed_change_target > self.timer_period_between_target_switch:
            self.i += 1
            self.last_record_time_ct = current_time

        if elapsed_get_logger_target > 0.2: # 0.5
            self.get_logger().info(f'Published baselink target at {self.Hertz} Hz: "{msg.data}"')
            self.last_record_time_glt = current_time

    def pose_callback(self, msg):
        self.curr_pos = msg.pose.position
        self.get_logger().info(f'Current position du drone en LOCAL :  {self.curr_pos}')

    def atg_callback(self, msg):
        if msg.data == "GO!":
            self.get_logger().info(f'message {msg.data} received to start target baselink')
            self.Hertz = 20
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