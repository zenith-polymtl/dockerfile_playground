#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from mission_interfaces.msg import ApproachInfo
from mission_interfaces.srv import GetPose

class VisionNode(Node):
    def __init__(self):
        super().__init__("Vision_Node")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Try RELIABLE if publisher is strict
            history=QoSHistoryPolicy.KEEP_LAST,  # Change to KEEP_ALL if missing messages
            depth=10
        )

        self.go_vision_sub = self.create_subscription(
            String,
            'go_vision',
            self.go_callback,
            qos_profile)

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )

        self.go_publisher = self.create_publisher(ApproachInfo, 'go_approach', qos_profile)
        self.client = self.create_client(GetPose, 'get_pose')

        self.image_pub = self.create_publisher(Image, '/camera/image_modified', 10)

        self.status = False
        self.first = True
        
        self.get_logger().info("Vision Node initialized")

          
    def go_callback(self, msg):
        go_message = msg.data

        if go_message == 'SOURCE':
            self.status = True
            self.type = "source"
            self.get_logger().info(f"BEGINNING SOURCE VISION")
        elif go_message == 'BUCKET':
            self.status = True
            self.type = "bucket"
            self.get_logger().info(f"BEGINNING BUCKET VISION")
        else:
            self.get_logger().warning(f"INVALID COMMAND MESSAGE RECEIVED : {go_message}")


    def image_callback(self, msg):
        if self.status:
            if self.first:
                self.i = 0 #Initialisation of whatever you want yo to do in your analysis
                self.first = False

            if self.type == 'source':
                #Then do this
                pass
            if self.type == 'bucket':
                #Then do that
                pass
                
            self.get_logger().info(f"IMAGE RECEIVED")
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Convert to grayscale
            modified_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Convert back to ROS2 Image message
            ros_image = self.bridge.cv2_to_imgmsg(modified_image, "mono8")
            ros_image.header.frame_id = "camera_frame"  # Important for RViz


            self.image_pub.publish(ros_image)
            self.get_logger().info(f"IMAGE SENT")

            self.i +=1
            if self.i > 30: #Here i is juste taken as an example for a criteria  to publish a setpoint
                msg = ApproachInfo()
                msg.x = 100.
                msg.y = 100.
                msg.z = 3.
                msg.status = 'Final'
                self.go_publisher.publish(msg)
                self.get_logger().info(f"Published setpoint for approach")
                self.status = False


                

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()