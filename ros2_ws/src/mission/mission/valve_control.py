#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import board
import busio
from adafruit_pca9685 import PCA9685

class ValveNode(Node):
    def __init__(self):
        super().__init__("bucket_valve_Node")

        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c, address=0x40)
        self.pca.frequency = 50  # 50Hz for standard servos
        self.servo_channel = self.pca.channels[1]
        
        self.bucketsQty = None
        self.waterVolume = 0
        self.openTime = None
        self.isClosed = True

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Ensures message delivery
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscriber_ = self.create_subscription(String, '/go_bucket_valve', self.go_callback, qos_profile)
        self.bucket_number_sub = self.create_subscription(Int32, '/bucket_number', self.bucket_number_callback, qos_profile)
        self.water_sub = self.create_subscription(Int32, '/water_qty', self.water_sub_callback, 2)
        self.state_sub = self.create_subscription(String, '/valve_state', self.state_callback, 10)

        self.get_logger().info(f"Valve Initialized")

    def set_servo_pulse_us(self, pulse_us):
        pulse_length = 1000000 / self.pca.frequency / 4096  # microseconds per bit
        pulse = int(pulse_us / pulse_length)
        # Scale the 12-bit pulse value to the 16-bit duty cycle range
        self.servo_channel.duty_cycle = pulse * 16
        
    def state_callback(self, msg):
        if msg.data == "OPEN":
            self.open_valve()
        elif msg.data == "CLOSE":
            self.close_valve()
        else:
            self.get_logger().info(f"Unexpected command sent")

    
    def bucket_number_callback(self, msg):
        self.bucketsQty = int(msg.data)
        self.get_logger().info(f"Set number of buckets to {self.bucketsQty}")
        self.last_logged_bucketsQty = self.bucketsQty

        
    def open_valve(self):
        if self.isClosed:
            self.set_servo_pulse_us(500) # Applique la position calculée
            self.get_logger().info(f"Opened Valve")
            self.isClosed = False

    def close_valve(self):
        if not self.isClosed:
            self.get_logger().info(f"Closing Valve")
            self.set_servo_pulse_us(2500)
            self.detach_timer = self.create_timer(0.5, self.detach_servo)
            self.get_logger().info(f"Closed Valve")
            self.isClosed = True

    def detach_servo(self):
        self.servo_channel.duty_cycle = 0

        if hasattr(self, 'detach_timer') and self.detach_timer is not None:
            try:
                self.destroy_timer(self.detach_timer)
                self.detach_timer = None
                self.get_logger().info("Timer destroyed")
            except Exception as e:
                self.get_logger().info(f"Could not destroy timer: {e}")
        else:
            self.get_logger().info("No detach timer found or already destroyed")

        self.get_logger().info("Detached Servo")


    def go_callback(self, msg):
        if msg.data == 'RELEASE' and self.isClosed:
            self.calculate_open_time()
            self.get_logger().info(f"Opening Valve for {self.openTime}")
            self.start_timer()
            self.bucketsQty -= 1
        elif msg.data == "REFILL":
            self.get_logger().info(f"REFILLING")
            self.openTime = 30
            self.start_timer()
            self.bucketsQty = self.last_logged_bucketsQty

    def water_sub_callback(self, msg):
        self.waterVolume = msg.data
        self.get_logger().info(f"Water volume updated to {self.waterVolume}")


    def calculate_open_time(self):
        """Calculates the time the valve should stay open based on the amount of water left"""
        if self.waterVolume <= 400:
            self.get_logger().info(f"Not enough water to open the valve")
            self.openTime = None
            return
        volume_par_ecoulement = self.waterVolume / self.bucketsQty

        debit_moyen= self.fonction_debit(self.waterVolume, volume_par_ecoulement)
        temps_s = (volume_par_ecoulement / debit_moyen) * 60  # t en secondes
        self.openTime = temps_s
        self.get_logger().info(f"Valve open time calculated to {self.openTime:.2f} seconds")

        #Safety logic to make sure the open time is not too long, or too short
        min_bound = 0.2
        max_bound = 30
        if not (self.openTime > min_bound) and (self.openTime < max_bound):
            val = self.openTime
            self.openTime = None
            self.get_logger().info(f"Valve open time set to 0 seconds, cause out of bounds")
            self.get_logger().info(f"Current val : {val}, bounds : {min_bound} - {max_bound}")

    def fonction_debit(self, volume_restant, volume_par_ecoulement):
        """Calculates the flow rate based on the remaining volume"""
        # Q = 4.0032 * (Vr - Delta_V/2) + 10.55
        return (4.0032 * (volume_restant - volume_par_ecoulement/2) + 10.55)

    def start_timer(self):
        if self.openTime is not None:
            self.open_valve()
            self.create_timer(self.openTime, self.timer_callback)

    def timer_callback(self):
        self.close_valve()
        self.get_logger().info(f"Valve closed after {self.openTime:.2f} seconds")	 

    def end_servo(self):
        self.pca.deinit()
        self.get_logger().info(f"Servo deinitialized")

        
def main(args=None):
    rclpy.init(args=args)
    node = ValveNode()
    rclpy.spin(node)
    node.end_servo()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()