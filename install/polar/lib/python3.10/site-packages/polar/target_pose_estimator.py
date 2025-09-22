import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, TwistStamped
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException
import numpy as np

MODE = "innovation"  # "origin_baselink" or "innovation"

class PoseTransformer(Node):
    def __init__(self):
        super().__init__('pose_transformer')

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subs
        self.pose_sub = self.create_subscription(
            PoseStamped, '/pose_in_baselink', self.pose_callback, 10
        )
        self.speed_sub = self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local', self.speed_callback, 10
        )

        # Pub
        self.pose_pub = self.create_publisher(PoseStamped, '/pose_in_map', 10)

        # State
        self.estimate = None            # np.array([x,y,z]) in map
        self.weight_total = 0.0
        self.moving = False
        self.current_local_target = None
        self.pub_timer = None

    def speed_callback(self, msg: TwistStamped):
        v = msg.twist.linear
        speed = float(np.linalg.norm([v.x, v.y, v.z]))
        self.moving = speed > 0.5

    def pose_callback(self, pose_msg: PoseStamped):
        try:
            # Transform pose to map
            transform = self.tf_buffer.lookup_transform(
                'map', pose_msg.header.frame_id, pose_msg.header.stamp,
                timeout=Duration(seconds=1.0)
            )
            transformed = tf2_geometry_msgs.do_transform_pose(pose_msg, transform)
            p = transformed.pose.position
            self.current_local_target = np.array([p.x, p.y, p.z], dtype=float)

            # Base_link origin distance, if needed for weighting mode
            bl = pose_msg.pose.position
            base_link_vec = np.array([bl.x, bl.y, bl.z], dtype=float)

            self.fuse_estimate(base_link_vec)
        except TransformException as ex:
            self.get_logger().error(f'Could not transform pose: {ex}')

    def fuse_estimate(self, base_link_vec: np.ndarray):
        if self.moving or self.current_local_target is None:
            return

        if self.estimate is None:
            self.estimate = self.current_local_target.copy()
            self.weight_total = 1.0
            if self.pub_timer is None:
                self.pub_timer = self.create_timer(1.0, self.pub_estimate)
            return

        # ----- Choose diff based on mode -----
        if MODE == "origin_baselink":
            # “More confidence when pose in base_link is small”
            diff = float(np.linalg.norm(base_link_vec))
        else:  # MODE == "innovation" (recommended)
            diff = float(np.linalg.norm(self.current_local_target - self.estimate))
        # -------------------------------------

        # Weight and update
        eps = 1e-4
        w = 1.0 / max(diff, eps)     # closer -> larger weight
        w = min(w, 100.0)            # cap a single sample’s influence

        alpha = w / (self.weight_total + w)
        self.estimate = (1.0 - alpha) * self.estimate + alpha * self.current_local_target
        self.weight_total += w

    def pub_estimate(self):
        if self.estimate is None:
            return
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.estimate.tolist()
        self.pose_pub.publish(msg)
