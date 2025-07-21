import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from std_msgs.msg import String
import time

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )

        self.go_sub = self.create_subscription(
            String,
            'go_vision',
            self.go_callback,
            10
        )

        self.publisher_ = self.create_publisher(Point, '/target_position', 10)

        self.bridge = CvBridge()

        # Load YOLO model
        self.model = YOLO("/home/haipy/ardu_ws/src/src/my_camera_pkg/my_camera_pkg/my_model_v8n.pt")

        #Control parameter to stop the commends sens to the pd approach
        self.active = False

        # Real bucket dimensions (in meters)
        self.bucket_diameter = 0.58
        self.bucket_height = 1.42

        # Regular object points for side detection (PnP)
        self.side_object_points = np.array([
            [-self.bucket_diameter / 2, 0, 0],
            [self.bucket_diameter / 2, 0, 0],
            [-self.bucket_diameter / 2, 0, self.bucket_height],
            [self.bucket_diameter / 2, 0, self.bucket_height]
        ], dtype=np.float32)

        # Top view object points (as a flat disk at z=0)
        self.top_object_points = np.array([
            [-self.bucket_diameter / 2, 0, 0],  # Left
            [self.bucket_diameter / 2, 0, 0],   # Right
            [0, -self.bucket_diameter / 2, 0],  # Top
            [0, self.bucket_diameter / 2, 0]    # Bottom
        ], dtype=np.float32)

        # Gazebo camera intrinsics
        self.camera_matrix = np.array([
            [205.47, 0, 320],
            [0, 205.47, 240],
            [0, 0, 1]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((4, 1))

    def go_callback(self, msg):
        go_message = msg.data

        if go_message == 'SOURCE':
            self.active = True
            self.get_logger().info(f"BEGINNING SOURCE VISION")
            #Could implement a parameter here to control the model and the approach type
        elif go_message == 'BUCKET':
            self.active = True
            self.get_logger().info(f"BEGINNING BUCKET VISION")
            #Could implement a parameter here to control the model and the approach type
        else:
            self.get_logger().warning(f"INVALID COMMAND MESSAGE RECEIVED : {go_message}")

    def image_callback(self, msg):
        if self.active:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f'Failed to convert image: {e}')
                return

            results = self.model(cv_image)

            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])

                    # Compute image points for side PnP
                    image_points_side = np.array([
                        [x1, y2],
                        [x2, y2],
                        [x1, y1],
                        [x2, y1]
                    ], dtype=np.float32)

                    success, rvec, tvec = cv2.solvePnP(
                        self.side_object_points, image_points_side, self.camera_matrix, self.dist_coeffs)

                    if success:
                        x, y, z = tvec.flatten()

                        if z >= 3:
                            # Display PnP position
                            position_text = f"PnP: x={x:.2f}m, y={y:.2f}m, z={z:.2f}m"
                            cv2.putText(cv_image, position_text, (x1, y1 - 20),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                            # Publish the PnP position
                            position_msg = Point()
                            position_msg.x = float(x)
                            position_msg.y = float(y)
                            position_msg.z = float(z)
                            self.publisher_.publish(position_msg)

                            if np.linalg.norm(x,y,z) < 0.1:
                                if not self.arrived:
                                    self.arrived = True
                                    self.arrival_time = time.time()
                                elif self.arrived:
                                    if time.time() - self.arrival_time > 10:
                                        self.active = False
                            else:
                                self.arrived = False


                        else:
                            # Use top-view PnP instead of centroid offset
                            image_points_top = np.array([
                                [(x1 + x2) / 2, y1],    # Top-center
                                [(x1 + x2) / 2, y2],    # Bottom-center
                                [x1, (y1 + y2) / 2],    # Left-center
                                [x2, (y1 + y2) / 2]     # Right-center
                            ], dtype=np.float32)

                            success_top, rvec_top, tvec_top = cv2.solvePnP(
                                self.top_object_points, image_points_top, self.camera_matrix, self.dist_coeffs)

                            if success_top:
                                x_top, y_top, z_top = tvec_top.flatten()

                                # Display top-view PnP position
                                position_text = f"TopPnP: x={x_top:.2f}m, y={y_top:.2f}m, z={z_top:.2f}m"
                                cv2.putText(cv_image, position_text, (x1, y1 - 20),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

                                # Publish top-view PnP position
                                position_msg = Point()
                                position_msg.x = float(x_top)
                                position_msg.y = float(y_top)
                                position_msg.z = float(z_top)
                                self.publisher_.publish(position_msg)

                    # Draw bounding box and label
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(cv_image, f"{self.model.names[int(box.cls)]} {float(box.conf):.2f}",
                                (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            cv2.imshow("YOLOv8 Detection with PnP/Centroid", cv_image)
            cv2.waitKey(1)
        else:
            pass




def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    camera_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
