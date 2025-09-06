import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import pandas as pd
import time
import os
from zenmav.core import Zenmav
from datetime import datetime
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class GraphNode_baselink(Node):
    def __init__(self):
        super().__init__('graph_baselink')
        self.last_record_time = 0

        time.sleep(1)
        self.get_logger().info(f"Zenmav port graph_baselink va s'initialiser")
        self.drone = Zenmav(ip='tcp:127.0.0.1:14553') # Se connecte à un des splits ports de Zenmav de abort_brake node
        self.get_logger().info(f'Zenmav port graph_baselink est connecté!')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=8
        )
        
        self.subscriber_atg = self.create_subscription(String, '/approach_target_graph', self.atg_callback, qos_profile)
        self.subscription_target = self.create_subscription(String, '/go_target', self.go_target_callback, qos_profile)
        self.subscriber_ab_call = self.create_subscription(String, '/close', self.close_callback, 10)

        self.current_target = {'F': None, 'L': None, 'U': None, 'yaw_b': None}
        self.last_log_time = -1
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = f"data/pos_FLU_yawb_{timestamp_str}.csv"
        os.makedirs("data", exist_ok=True)

        self.df = pd.DataFrame(columns=['tar_x', 'tar_y', 'tar_z','tar_yawb', 'time'])
        self.df.to_csv(self.csv_file, index=False)

    def atg_callback(self, msg):
        if msg.data == "GO!":
            self.get_logger().info(f'message {msg.data} received to start graph baselink')
            self.start_time = time.time()
        else:
            self.get_logger().info(f'message GO! not received for graph baselink : {msg.data}')

    def go_target_callback(self, msg):
        try:
            # Format attendu : "-1,2,0,pi/8"
            parts = msg.data.split(',')
            self.current_target['F'] = float(parts[0])
            self.current_target['L'] = float(parts[1])
            self.current_target['U'] = float(parts[2])
            self.current_target["yawb"] = ((float(parts[3]))*180/np.pi)
        except Exception as e:
            self.get_logger().error(f'Erreur de parsing du message target: {e}')

        current_time = time.time()

        # Vérifie que les coordonnées cibles sont bien définies
        if None in self.current_target.values():
            if current_time - self.last_log_time >= 2:
                self.get_logger().warn("Cible non encore définie, ligne ignorée")
                self.last_log_time = current_time
                return
            else:
                return

        elapsed = current_time - self.last_record_time

        self.timer_elapsed = 0.2 # secondes
        if elapsed < self.timer_elapsed:
            return  # ignore si moins de "self.timer_elapsed" depuis le dernier enregistrement de données

        self.last_record_time = current_time

        tar_F = self.current_target['F']
        tar_L = self.current_target['L']
        tar_U = self.current_target['U']

        yaw = self.current_target['yawb']
        yaw = yaw - 90 # décalage pour que 0° = Nord
        yaw = 360 - yaw # conversion horaire -> antihoraire
        while yaw < 0:
            yaw += 360
        while yaw > 360:
            yaw -= 360
        tar_YAW_B = round((yaw), 4)


        timestamp = round(time.time() - self.start_time, 2)

        row = {'tar_F': tar_F, 'tar_L': tar_L, 'tar_U': tar_U, 'tar_yawb': tar_YAW_B, 'time': timestamp}

        # Append à la volée à chaque "self.timer_elapsed"
        pd.DataFrame([row]).to_csv(self.csv_file, mode='a', header=False, index=False)
        self.get_logger().info(f'Donnée enregistrée à t={timestamp:.2f}s')

    def close_callback(self, msg):
        if msg.data == "close":
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = GraphNode_baselink()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()