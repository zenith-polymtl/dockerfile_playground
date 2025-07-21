#!/usr/bin/env python3

import can
import sys
import time
from threading import Thread
from queue import Queue

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class CANWinchNode(Node):
    """ROS2 node for controlling a winch via CAN bus."""

    def __init__(self):
        super().__init__('can_winch_node')
        
        # Configuration parameters
        self.declare_parameter('device', '/dev/ttyUSB0')
        self.declare_parameter('can_speed', 500000)  # 500kbps
        self.declare_parameter('baudrate', 2000000)  # 2Mbps
        self.declare_parameter('motor_id', 1)
        self.declare_parameter('movement_time', 2.0)  # Time for movement in seconds
        
        # Get parameters
        self.device = self.get_parameter('device').value
        self.can_speed = self.get_parameter('can_speed').value
        self.baudrate = self.get_parameter('baudrate').value
        self.motor_id = self.get_parameter('motor_id').value
        self.movement_time = self.get_parameter('movement_time').value
        
        # Create callback groups for timer separation
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.subscription_callback_group = MutuallyExclusiveCallbackGroup()
        
        # State variables
        self.bus = None
        self.response_queue = Queue()
        self.last_command = None
        self.current_operation = None
        self.operation_step = 0
        self.operation_data = {}
        self.timer = None
        self.last_command_time = 0  # Track when the last command was sent
        
        # Create reliable QoS profile
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # Create subscription
        self.go_sub = self.create_subscription(
            String, '/go_winch', self.go_callback, reliable_qos,
            callback_group=self.subscription_callback_group)
        
        # Initialize CAN bus
        self.setup_can_bus()
        
        # Start receiver thread
        if self.bus:
            self.receiver_thread = Thread(target=self.receive_messages, daemon=True)
            self.receiver_thread.start()
            self.get_logger().info("CAN message receiver thread started")
        
        self.get_logger().info("CAN Winch Node initialized")

    def setup_can_bus(self):
        """Initialize the CAN bus connection"""
        try:
            self.bus = can.interface.Bus(
                interface='seeedstudio',
                channel=self.device,
                bitrate=self.can_speed,
                baudrate=self.baudrate
            )
            self.get_logger().info(f"Connected to CAN bus on {self.device}")
        except Exception as e:
            self.get_logger().error(f"Error setting up CAN bus: {e}")
            self.bus = None

    def format_can_data(self, data):
        """Format CAN data as space-separated hex bytes"""
        return " ".join([f"{byte:02X}" for byte in data])

    def send_message(self, data, arbitration_id=None):
        """Send a message with the specified data to the CAN bus"""
        if arbitration_id is None:
            arbitration_id = self.motor_id
            
        if len(data) != 8:
            self.get_logger().error("Error: Message must be exactly 8 bytes")
            return False
        
        # Clear the response queue before sending a new command
        while not self.response_queue.empty():
            self.response_queue.get()
        
        message = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=False
        )
        
        try:
            self.last_command = data[:2]  # Store the first 2 bytes for response matching
            self.last_command_time = time.time()  # Record when we sent the command
            self.bus.send(message)
            self.get_logger().info(f"Sent message to ID {arbitration_id}: {self.format_can_data(data)}")
            return True
        except Exception as e:
            self.get_logger().error(f"Error sending message: {e}")
            return False

    def receive_messages(self):
        """Continuously receive messages from the CAN bus"""
        self.get_logger().info("Listening for CAN messages...")
        
        while rclpy.ok() and self.bus:
            try:
                message = self.bus.recv(0.1)  # Use timeout to allow clean thread exit
                if message:
                    current_time = time.time()
                    time_since_command = current_time - self.last_command_time
                    self.get_logger().info(f"Received from ID {message.arbitration_id}: {self.format_can_data(message.data)} (delay: {time_since_command:.3f}s)")
                    
                    # Add any message received to the queue if we're in an operation
                    # The check_for_response method will filter as needed
                    if self.current_operation is not None:
                        self.response_queue.put(message)
            except Exception as e:
                self.get_logger().error(f"Error in receiver thread: {e}")
                # Brief pause to avoid tight loop in case of persistent errors
                time.sleep(0.1)

    def check_for_response(self, wait_time=0.2):
        """Check if there's a response in the queue, with option to wait briefly for response"""
        try:
            start_time = time.time()
            while time.time() - start_time < wait_time:
                if not self.response_queue.empty():
                    return self.response_queue.get(block=False)
                time.sleep(0.01)  # Small sleep to prevent tight loop
        except:
            pass
        return None

    def parse_byte_string(self, byte_string):
        """Parse a string of hex bytes like '00 FF 12 34 56 78 9A BC' into bytes"""
        try:
            # Remove any extra whitespace and split by spaces
            parts = byte_string.strip().split()
            
            # Check if we have exactly 8 parts
            if len(parts) != 8:
                self.get_logger().error("Error: Please provide exactly 8 bytes")
                return None
            
            # Convert each part to an integer
            data = [int(part, 16) for part in parts]
            return bytes(data)
        except ValueError as e:
            self.get_logger().error(f"Error parsing input: {e}")
            return None

    def go_callback(self, msg):
        """Handles 'UP' or 'DOWN' command messages."""
        self.get_logger().info(f"GO Command Received: {msg.data}")
        # Check if CAN bus is available
        if not self.bus:
            self.get_logger().error("CAN bus not available for GO command.")
            return
        # Prevent starting a new movement if one is already in progress
        if self.current_operation is not None:
            self.get_logger().warn(f"Ignoring GO '{msg.data}': Operation '{self.current_operation}' in progress.")
            return

        # Determine which command to send based on message data
        if msg.data == 'UP':
            self.execute_command_sequence_up()
        elif msg.data == 'DOWN':
            self.execute_command_sequence_down()
        else:
            # Log if the command is unrecognized
            self.get_logger().warn(f"Unknown GO command: '{msg.data}'. Expected 'UP' or 'DOWN'.")

    def execute_command_sequence_up(self):
        """Start the UP command sequence using timers"""
        self.current_operation = 'UP'
        self.operation_step = 1
        self.operation_data = {}
        
        # Start the sequence with the first step
        self.get_logger().info("Starting UP sequence")
        self.execute_next_step()

    def execute_command_sequence_down(self):
        """Start the DOWN command sequence using timers"""
        self.current_operation = 'DOWN'
        self.operation_step = 1
        self.operation_data = {}
        
        # Start the sequence with the first step
        self.get_logger().info("Starting DOWN sequence")
        self.execute_next_step()

    def execute_next_step(self):
        """Execute the next step in the current operation sequence"""
        if not self.current_operation:
            return
            
        if self.current_operation == 'UP':
            if self.operation_step == 1:
                # Step 1: Send command 94 00 00 A0 C1 D0 07 00
                self.get_logger().info("Step 1: Sending initial UP command")
                data = self.parse_byte_string("94 00 00 A0 C1 D0 07 00")
                if data and self.send_message(data):
                    # Give some time to capture response before moving to next step
                    # We'll check for this response in the next step
                    self.create_timer_for_next_step(0.05)
                else:
                    self.reset_operation()
                    
            elif self.operation_step == 2:
                # Check for response to previous command
                response = self.check_for_response()
                if not response:
                    self.get_logger().warn("No response to UP command 1, continuing anyway")
                else:
                    self.get_logger().info(f"Response to command 1: {self.format_can_data(response.data)}")
                
                # Step 2: Send command 91 00 00 00 00 00 00 00
                self.get_logger().info("Step 2: Sending second UP command")
                data = self.parse_byte_string("91 00 00 00 00 00 00 00")
                if data and self.send_message(data):
                    # Set timer to check for response and move to next step
                    self.create_timer_for_next_step(0.05)
                else:
                    self.reset_operation()
                    
            elif self.operation_step == 3:
                # Check for response to previous command
                response = self.check_for_response()
                if not response:
                    self.get_logger().warn("No response to UP command 2, continuing anyway")
                else:
                    self.get_logger().info(f"Response to command 2: {self.format_can_data(response.data)}")
                
                # Wait for movement_time seconds before proceeding
                self.get_logger().info(f"Waiting {self.movement_time} seconds for UP movement...")
                self.create_timer_for_next_step(self.movement_time)
                
            elif self.operation_step == 4:
                # Step 3: Send command B4 13 00 00 00 00 00 00
                self.get_logger().info("Step 3: Sending command to get data for UP")
                data = self.parse_byte_string("B4 13 00 00 00 00 00 00")
                if data and self.send_message(data):
                    # IMPORTANT: Use a short delay to check for response to B4 command
                    # This ensures we don't miss the response which may come quickly
                    self.create_timer_for_next_step(0.1)
                else:
                    self.reset_operation()
                    
            elif self.operation_step == 5:
                # Check for response to B4 command - wait a bit longer to ensure we catch it
                self.get_logger().info("Waiting for response to B4 command...")
                response = self.check_for_response(wait_time=0.5)
                
                if response:
                    self.get_logger().info(f"Received response to B4: {self.format_can_data(response.data)}")
                    # Extract last 4 bytes from response
                    last_four_bytes = response.data[-4:]
                    self.get_logger().info(f"Extracted 4 bytes: {self.format_can_data(last_four_bytes)}")
                    self.operation_data['last_four_bytes'] = last_four_bytes
                    
                    # Step 4: Send command 95 [4 bytes] 32 14 00
                    self.get_logger().info("Step 4: Sending final UP command with extracted bytes")
                    final_command = bytes([0x95]) + last_four_bytes + bytes([0x32, 0x14, 0x00])
                    
                    # Ensure it's exactly 8 bytes
                    if len(final_command) > 8:
                        final_command = final_command[:8]
                    elif len(final_command) < 8:
                        final_command = final_command + bytes([0x00] * (8 - len(final_command)))
                    
                    self.get_logger().info(f"Final UP command: {self.format_can_data(final_command)}")
                    if self.send_message(final_command):
                        # Set timer to check for response and finish
                        self.create_timer_for_next_step(0.2)
                    else:
                        self.reset_operation()
                else:
                    # Try looking for any message in the queue that might have the format we need
                    self.get_logger().error("Error: No direct response to UP command B4")
                    self.get_logger().info("Checking if there are any messages we can use...")
                    
                    # If no response, check if we have any received messages from ID 0
                    # Send a dummy command to ping the device and get any response
                    data = self.parse_byte_string("91 00 00 00 00 00 00 00")
                    if data and self.send_message(data):
                        # Wait a bit to see if we get any response
                        response = self.check_for_response(wait_time=0.3)
                        if response:
                            self.get_logger().info(f"Found usable response: {self.format_can_data(response.data)}")
                            # Try to use this response
                            last_four_bytes = response.data[-4:]
                            self.operation_data['last_four_bytes'] = last_four_bytes
                            
                            # Continue with final command
                            final_command = bytes([0x95]) + last_four_bytes + bytes([0x32, 0x14, 0x00])
                            if len(final_command) > 8:
                                final_command = final_command[:8]
                            elif len(final_command) < 8:
                                final_command = final_command + bytes([0x00] * (8 - len(final_command)))
                            
                            self.get_logger().info(f"Final UP command (from alternate response): {self.format_can_data(final_command)}")
                            if self.send_message(final_command):
                                self.create_timer_for_next_step(0.2)
                                return
                    
                    self.get_logger().error("Could not find any usable response data")
                    self.reset_operation()
                    
            elif self.operation_step == 6:
                # Check for response to final command
                response = self.check_for_response()
                if response:
                    self.get_logger().info(f"Final UP response: {self.format_can_data(response.data)}")
                    self.get_logger().info("UP command sequence completed successfully")
                else:
                    self.get_logger().warn("No response to final UP command")
                
                # Operation complete
                self.reset_operation()
                
        elif self.current_operation == 'DOWN':
            if self.operation_step == 1:
                # Step 1: Send command 94 00 00 A0 41 D0 07 00
                self.get_logger().info("Step 1: Sending initial DOWN command")
                data = self.parse_byte_string("94 00 00 A0 41 D0 07 00")
                if data and self.send_message(data):
                    # Give some time to capture response before moving to next step
                    self.create_timer_for_next_step(0.2)
                else:
                    self.reset_operation()
                    
            elif self.operation_step == 2:
                # Check for response to previous command
                response = self.check_for_response()
                if not response:
                    self.get_logger().warn("No response to DOWN command 1, continuing anyway")
                else:
                    self.get_logger().info(f"Response to command 1: {self.format_can_data(response.data)}")
                
                # Step 2: Send command 91 00 00 00 00 00 00 00
                self.get_logger().info("Step 2: Sending second DOWN command")
                data = self.parse_byte_string("91 00 00 00 00 00 00 00")
                if data and self.send_message(data):
                    # Set timer to check for response and move to next step
                    self.create_timer_for_next_step(0.2)
                else:
                    self.reset_operation()
                    
            elif self.operation_step == 3:
                # Check for response to previous command
                response = self.check_for_response()
                if not response:
                    self.get_logger().warn("No response to DOWN command 2, continuing anyway")
                else:
                    self.get_logger().info(f"Response to command 2: {self.format_can_data(response.data)}")
                
                # Wait for movement_time seconds before proceeding
                self.get_logger().info(f"Waiting {self.movement_time} seconds for DOWN movement...")
                self.create_timer_for_next_step(self.movement_time)
                
            elif self.operation_step == 4:
                # Step 3: Send command B4 13 00 00 00 00 00 00
                self.get_logger().info("Step 3: Sending command to get data for DOWN")
                data = self.parse_byte_string("B4 13 00 00 00 00 00 00")
                if data and self.send_message(data):
                    # IMPORTANT: Use a short delay to check for response to B4 command
                    # This ensures we don't miss the response which may come quickly
                    self.create_timer_for_next_step(0.1)
                else:
                    self.reset_operation()
                    
            elif self.operation_step == 5:
                # Check for response to B4 command - wait a bit longer to ensure we catch it
                self.get_logger().info("Waiting for response to B4 command...")
                response = self.check_for_response(wait_time=0.5)
                
                if response:
                    self.get_logger().info(f"Received response to B4: {self.format_can_data(response.data)}")
                    # Extract last 4 bytes from response
                    last_four_bytes = response.data[-4:]
                    self.get_logger().info(f"Extracted 4 bytes: {self.format_can_data(last_four_bytes)}")
                    self.operation_data['last_four_bytes'] = last_four_bytes
                    
                    # Step 4: Send command 95 [4 bytes] 32 14 00
                    self.get_logger().info("Step 4: Sending final DOWN command with extracted bytes")
                    final_command = bytes([0x95]) + last_four_bytes + bytes([0x32, 0x14, 0x00])
                    
                    # Ensure it's exactly 8 bytes
                    if len(final_command) > 8:
                        final_command = final_command[:8]
                    elif len(final_command) < 8:
                        final_command = final_command + bytes([0x00] * (8 - len(final_command)))
                    
                    self.get_logger().info(f"Final DOWN command: {self.format_can_data(final_command)}")
                    if self.send_message(final_command):
                        # Set timer to check for response and finish
                        self.create_timer_for_next_step(0.2)
                    else:
                        self.reset_operation()
                else:
                    # Try looking for any message in the queue that might have the format we need
                    self.get_logger().error("Error: No direct response to DOWN command B4")
                    self.get_logger().info("Checking if there are any messages we can use...")
                    
                    # If no response, check if we have any received messages from ID 0
                    # Send a dummy command to ping the device and get any response
                    data = self.parse_byte_string("91 00 00 00 00 00 00 00")
                    if data and self.send_message(data):
                        # Wait a bit to see if we get any response
                        response = self.check_for_response(wait_time=0.3)
                        if response:
                            self.get_logger().info(f"Found usable response: {self.format_can_data(response.data)}")
                            # Try to use this response
                            last_four_bytes = response.data[-4:]
                            self.operation_data['last_four_bytes'] = last_four_bytes
                            
                            # Continue with final command
                            final_command = bytes([0x95]) + last_four_bytes + bytes([0x32, 0x14, 0x00])
                            if len(final_command) > 8:
                                final_command = final_command[:8]
                            elif len(final_command) < 8:
                                final_command = final_command + bytes([0x00] * (8 - len(final_command)))
                            
                            self.get_logger().info(f"Final DOWN command (from alternate response): {self.format_can_data(final_command)}")
                            if self.send_message(final_command):
                                self.create_timer_for_next_step(0.2)
                                return
                    
                    self.get_logger().error("Could not find any usable response data")
                    self.reset_operation()
                    
            elif self.operation_step == 6:
                # Check for response to final command
                response = self.check_for_response()
                if response:
                    self.get_logger().info(f"Final DOWN response: {self.format_can_data(response.data)}")
                    self.get_logger().info("DOWN command sequence completed successfully")
                else:
                    self.get_logger().warn("No response to final DOWN command")
                
                # Operation complete
                self.reset_operation()

    def create_timer_for_next_step(self, delay):
        """Create a one-shot timer to execute the next step after a delay"""
        # Cancel any existing timer
        if self.timer:
            self.timer.cancel()
        
        # Increment step counter
        self.operation_step += 1
        
        # Create new timer
        self.timer = self.create_timer(
            delay, 
            self.timer_callback,
            callback_group=self.timer_callback_group
        )

    def timer_callback(self):
        """Called when the timer expires"""
        # Cancel the timer since we only want it to fire once
        self.timer.cancel()
        self.timer = None
        
        # Execute the next step in the sequence
        self.execute_next_step()

    def reset_operation(self):
        """Reset the operation state"""
        self.current_operation = None
        self.operation_step = 0
        self.operation_data = {}
        
        # Cancel any pending timer
        if self.timer:
            self.timer.cancel()
            self.timer = None

    def shutdown(self):
        """Clean shutdown of the node"""
        self.get_logger().info("Shutting down CAN Winch Node")
        
        # Cancel any active timer
        if self.timer:
            self.timer.cancel()
            self.timer = None
        
        # Close CAN bus connection
        if self.bus:
            self.get_logger().info("Closing CAN bus connection")
            self.bus.shutdown()
            self.bus = None

def main(args=None):
    rclpy.init(args=args)
    
    # Create the node
    node = CANWinchNode()
    
    # Use a MultiThreadedExecutor to handle callbacks from multiple sources
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        # Spin the node to execute callbacks
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        node.shutdown()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()