import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import pandas as pd
import time
import os
from datetime import datetime
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class GraphNode(Node):
    def __init__(self):
        super().__init__('graph')
        self.last_record_time = 0
        self.first_time_AUTO = True

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscriber_ = self.create_subscription(String, '/manual', self.manual_callback, qos_profile)

        self.subscription_pose = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback, qos_profile)

        self.subscription_target = self.create_subscription(
            String,
            '/go_approach',
            self.target_callback, qos_profile)

        self.current_target = {'x': None, 'y': None, 'z': None}
        self.last_log_time = -1  # Initialisation du dernier temps de log
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = f"data/pos_xyz_{timestamp_str}.csv"
        os.makedirs("data", exist_ok=True)

        # Initialise le fichier CSV avec les en-têtes
        self.df = pd.DataFrame(columns=['pos_x', 'pos_y', 'pos_z', 'tar_x', 'tar_y', 'tar_z', 'time'])
        self.df.to_csv(self.csv_file, index=False)

    def manual_callback(self, msg):
        if msg.data == "AUTO":
            self.get_logger().info(f'message AUTO received for graph')
            if self.first_time_AUTO == True:
                self.start_time = time.time()
        if msg.data == "MANUAL":
            self.get_logger().info(f'message MANUAL received to stop graph')
            self.first_time_AUTO = False
        else:
            self.get_logger().info(f'message AUTO not received for graph : {msg.data}')

    def target_callback(self, msg):
        try:
            # Format attendu : "12,34,56"
            parts = msg.data.split(',')
            self.current_target['x'] = float(parts[0])
            self.current_target['y'] = float(parts[1])
            self.current_target['z'] = float(parts[2])
            self.get_logger().info(f'Nouvelle cible reçue : {self.current_target}')
        except Exception as e:
            self.get_logger().error(f'Erreur de parsing du message target: {e}')

    def pose_callback(self, msg):
        # ✅ Vérifie que les coordonnées cibles sont bien définies
        current_time = time.time()
        if None in self.current_target.values():
            if current_time - self.last_log_time >= 2:
                self.get_logger().warn("Cible non encore définie, ligne ignorée")
                self.last_log_time = current_time
                return
            else:
                return

        elapsed = current_time - self.last_record_time

        if elapsed < 0.2:
            return  # ignore si moins de 0.2s depuis le dernier enregistrement

        self.last_record_time = current_time

        pos_x = msg.pose.position.x
        pos_y = msg.pose.position.y
        pos_z = msg.pose.position.z
        tar_x = self.current_target['x']
        tar_y = self.current_target['y']
        tar_z = self.current_target['z']
        timestamp = time.time() - self.start_time

        row = {
            'pos_x': pos_x,
            'pos_y': pos_y,
            'pos_z': pos_z,
            'tar_x': tar_x,
            'tar_y': tar_y,
            'tar_z': tar_z,
            'time': timestamp
        }

        # Append à la volée
        pd.DataFrame([row]).to_csv(self.csv_file, mode='a', header=False, index=False)
        self.get_logger().info(f'Donnée enregistrée à t={timestamp:.2f}s')

def main(args=None):
    rclpy.init(args=args)
    node = GraphNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()