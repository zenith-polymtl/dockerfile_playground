import can
import time
import struct
from collections import deque
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float32


class WaterNode(Node):
    def __init__(self):
        super().__init__('water_node')

        # --- ROS2 Publishers ---
        # Publisher for the calculated water quantity (integer in mL)
        self.water_qty_pub = self.create_publisher(Int32, '/water_qty', 2)
        # Publisher for the filtered motor torque (float in Nm)
        self.torque_pub = self.create_publisher(Float32, '/torque', 2)
        # *** NEW: Publisher for the motor position (float, units depend on motor) ***
        self.position_pub = self.create_publisher(
            Float32, '/motor_position', 2
        )

        # --- Configuration Parameters ---
        # These parameters allow configuring the node without changing the code
        # They can be set via launch files or command line arguments
        self.declare_parameter('device', '/dev/ttyUSB0') # Serial device for CAN adapter
        self.declare_parameter('can_speed', 500000)     # CAN bus speed in bps
        self.declare_parameter('baudrate', 2000000)    # Serial baudrate for CAN adapter
        self.declare_parameter('motor_id', 1)          # CAN ID of the target motor
        self.declare_parameter('polling_rate', 2.0)    # How often to poll the motor (Hz)
        self.declare_parameter('filter_window', 11)    # Size of the moving average filter window
        self.declare_parameter('debug_level', 1)       # Verbosity: 0=minimal, 1=normal, 2=verbose

        # --- Physics Constants ---
        # These parameters define the physical properties of the system
        self.declare_parameter('torque_constant', 0.065) # Motor Kt (Nm/A)
        self.declare_parameter('drum_radius', 0.0235)    # Radius of the winch drum (m)
        self.declare_parameter('gear_ratio', 10.0)       # Gearbox reduction ratio (e.g., 10:1)
        self.declare_parameter('dead_weight', 0.74)      # Weight of the empty sampler (kg)
        self.declare_parameter('water_density', 1.0)     # Density of water (kg/L or g/mL)

        # --- Get Parameters ---
        # Retrieve the parameter values defined above
        self.device = self.get_parameter('device').value
        self.can_speed = self.get_parameter('can_speed').value
        self.baudrate = self.get_parameter('baudrate').value
        self.motor_id = self.get_parameter('motor_id').value
        self.polling_rate = self.get_parameter('polling_rate').value
        self.filter_window = self.get_parameter('filter_window').value
        self.debug_level = self.get_parameter('debug_level').value
        self.torque_constant = self.get_parameter('torque_constant').value
        self.drum_radius = self.get_parameter('drum_radius').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.dead_weight = self.get_parameter('dead_weight').value
        self.water_density = self.get_parameter('water_density').value

        # --- Initialize State Variables ---
        self.current = 0.0  # Last measured motor current (Amps)
        self.torque = 0.0   # Last calculated filtered torque (Nm)
        self.volume = 0     # Last calculated water volume (mL)
        self.position = 0.0 # *** NEW: Last measured motor position ***

        # Initialize the filter buffer using a deque for efficient additions/removals
        self.torque_buffer = deque(maxlen=self.filter_window)

        # --- CAN Commands ---
        # Define the byte sequences for the CAN commands we need to send
        # Command to request the motor's Iq current (related to torque)
        self.IQ_COMMAND = bytes(
            [0xB4, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        )
        # *** NEW: Command to request the motor's position ***
        # IMPORTANT: Verify that 0x13 is the correct indicator byte for position
        # and that the response format is the same (float in bytes 4-7)
        self.GET_POSITION_COMMAND = bytes(
            [0xB4, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        )

        # --- Alternation Flag ---
        # *** NEW: Flag to control which command is sent next ***
        self.send_iq_next = True # Start by sending the Iq command

        # Print configuration summary
        self.debug_print(
            f"Configuration: torque_constant={self.torque_constant}, "
            f"drum_radius={self.drum_radius}, gear_ratio={self.gear_ratio}, "
            f"dead_weight={self.dead_weight}",
            level=0,
        )

        # Initialize CAN bus connection
        self.bus = self.setup_can_bus()

        # Create a timer that calls poll_motor_data periodically
        self.timer = self.create_timer(
            1.0 / self.polling_rate, self.poll_motor_data # Renamed for clarity
        )

        self.get_logger().info('Water Node initialized and polling started')

    def debug_print(self, message, level=1):
        """Prints a message if the configured debug level is high enough."""
        if self.debug_level >= level:
            # Use ROS2 logger for consistent output
            self.get_logger().info(f"DEBUG: {message}")

    def format_can_data(self, data):
        """Formats CAN message data bytes into a readable hex string."""
        return " ".join([f"{byte:02X}" for byte in data])

    def setup_can_bus(self):
        """Initializes and returns the CAN bus interface object."""
        try:
            # Use python-can library with the specified interface and settings
            bus = can.interface.Bus(
                interface='seeedstudio', # Specific adapter type
                channel=self.device,     # Serial port
                bitrate=self.can_speed,  # CAN bus speed
                baudrate=self.baudrate,  # Serial port speed for adapter
            )
            self.get_logger().info(
                f"Successfully connected to CAN bus on {self.device}"
            )
            return bus
        except Exception as e:
            # Log error and shutdown if CAN connection fails
            self.get_logger().error(f"Fatal Error setting up CAN bus: {e}")
            rclpy.shutdown() # Stop the ROS2 node if CAN fails
            return None # Should not be reached if shutdown works

    def send_message(self, data, arbitration_id=None):
        """Constructs and sends a CAN message."""
        # Default to the configured motor ID if none is provided
        if arbitration_id is None:
            arbitration_id = self.motor_id

        # Ensure data payload is the standard 8 bytes for CAN
        if len(data) != 8:
            self.get_logger().error(
                f"Error: CAN message data must be exactly 8 bytes, got {len(data)}"
            )
            return False

        # Create the CAN message object
        message = can.Message(
            arbitration_id=arbitration_id, # The CAN ID
            data=data,                     # The 8-byte payload
            is_extended_id=False,          # Use standard 11-bit IDs
        )

        try:
            # Send the message on the bus
            self.bus.send(message)
            self.debug_print(
                f"Sent message: ID={arbitration_id}, Data={self.format_can_data(data)}",
                level=2, # Verbose logging for sent messages
            )
            return True
        except Exception as e:
            self.get_logger().error(f"Error sending CAN message: {e}")
            return False

    def parse_float_from_response(self, data):
        """
        Parses a little-endian IEEE 754 float from bytes 4-7 of CAN data.
        Assumes the motor sends float values in this specific format.
        """
        # Check if data is long enough
        if len(data) < 8:
            self.get_logger().error(f"Error parsing float: Data too short ({len(data)} bytes)")
            return None
        try:
            # Extract bytes 4, 5, 6, 7
            float_bytes = bytes(data[4:8])
            # Unpack as a little-endian float ('<f')
            value = struct.unpack('<f', float_bytes)[0]
            self.debug_print(
                f"Parsed float: {value:.4f} from bytes {self.format_can_data(float_bytes)}",
                level=2, # Verbose
            )
            return value
        except struct.error as e:
            self.get_logger().error(f"Error unpacking float: {e}. Data: {self.format_can_data(data)}")
            return None
        except Exception as e:
            self.get_logger().error(f"Unexpected error parsing float: {e}")
            return None

    def apply_filter(self, new_value):
        """Applies a simple moving average filter to the torque value."""
        self.torque_buffer.append(new_value)
        # Calculate the average of the values currently in the buffer
        # The deque automatically handles the window size
        filtered_value = sum(self.torque_buffer) / len(self.torque_buffer)
        self.debug_print(
            f"Filter: input={new_value:.4f}, buffer_size={len(self.torque_buffer)}, output={filtered_value:.4f}",
            level=2 # Verbose
        )
        return filtered_value

    def calculate_water_volume(self, torque):
        """Calculates the estimated water volume based on filtered torque."""
        # --- Torque to Force Calculation ---
        # Torque at the drum = Motor Torque * Gear Ratio
        # Force = Torque at the drum / Drum Radius
        # We use abs(torque) because we only care about the magnitude for weight
        force_newtons = abs(torque) * self.gear_ratio / self.drum_radius

        # --- Force to Weight Calculation ---
        # Force = mass * gravity (F=mg), so mass = Force / g
        # Convert mass from kg to g (multiply by 1000)
        total_weight_grams = (force_newtons / 9.81) * 1000

        # --- Subtract Dead Weight ---
        # Calculate the weight of the water alone
        water_weight_grams = total_weight_grams - (self.dead_weight * 1000)

        # --- Weight to Volume Calculation ---
        # Volume = Mass / Density
        # Using water_density (e.g., 1.0 g/mL)
        volume_ml = water_weight_grams / self.water_density

        # Debug print for intermediate steps
        self.debug_print(
            f"Water Calc: Torque={torque:.4f} Nm -> Force={force_newtons:.2f} N -> "
            f"Total Weight={total_weight_grams:.1f} g -> Water Weight={water_weight_grams:.1f} g -> "
            f"Volume={volume_ml:.1f} mL",
            level=1, # Normal debug level
        )

        # Round the volume to the nearest integer mL
        rounded_volume = round(volume_ml)

        # Ensure volume is not negative (can happen with noise or upward movement)
        # Return 0 if calculated volume is negative
        result = max(0, rounded_volume)

        # Optional: Log if a negative volume was calculated before clamping
        # if rounded_volume < 0:
        #     self.debug_print(
        #         f"NOTE: Negative water volume calculated ({volume_ml:.1f}mL), reporting 0.",
        #         level=0, # Minimal level - important note
        #     )

        return result

    def poll_motor_data(self):
        """
        Alternately polls the motor for Iq (current) and Position,
        processes the response, and publishes the data.
        """
        command_to_send = None
        expected_indicator = None # The expected second byte in the response

        # --- Determine Command for this Poll Cycle ---
        if self.send_iq_next:
            command_to_send = self.IQ_COMMAND
            expected_indicator = 0x09 # Expected response indicator for Iq
            self.debug_print("Polling for Iq (Current)", level=1)
        else:
            command_to_send = self.GET_POSITION_COMMAND
            expected_indicator = 0x13 # Expected response indicator for Position
            self.debug_print("Polling for Position", level=1)

        # --- Send the Command ---
        if not self.send_message(command_to_send):
            self.debug_print("Failed to send command, skipping poll cycle.", level=0)
            # Flip the flag anyway to try the other command next time
            self.send_iq_next = not self.send_iq_next
            return # Exit this poll cycle

        # --- Wait for Response ---
        # Use a timeout (e.g., 200ms) to avoid blocking indefinitely
        message = self.bus.recv(0.2)

        # --- Process Response ---
        if message:
            self.debug_print(
                f"Received message: ID={message.arbitration_id}, Data={self.format_can_data(message.data)}",
                level=2, # Verbose
            )

            # Check if it's a response to our command type (first byte should be 0xB4)
            if message.data[0] == 0xB4:
                received_indicator = message.data[1]

                # Check if the indicator matches the command we sent
                if received_indicator == expected_indicator:
                    # Parse the float value (assuming same format for both Iq and Position)
                    # *** IMPORTANT: If position is not a float in bytes 4-7, this needs modification ***
                    value_raw = self.parse_float_from_response(message.data)

                    if value_raw is not None:
                        # --- Process based on the command sent ---
                        if expected_indicator == 0x09: # Processing Iq response
                            current_raw = value_raw
                            # Calculate raw torque
                            torque_raw = current_raw * self.torque_constant
                            # Apply filter to torque
                            self.torque = self.apply_filter(torque_raw)
                            self.current = current_raw # Store raw current

                            # Debug torque calculation
                            self.debug_print(
                                f"Torque Update: Current={current_raw:.4f} A -> Raw Torque={torque_raw:.4f} Nm -> Filtered Torque={self.torque:.4f} Nm",
                                level=1,
                            )

                            # Calculate water volume from filtered torque
                            self.volume = self.calculate_water_volume(self.torque)

                            # Log summary and publish
                            self.debug_print(
                                f"Result: Current={self.current:.2f}A | Torque={self.torque:.4f}Nm | Water={self.volume}mL",
                                level=0, # Minimal level - show final results
                            )
                            self.send_torque()
                            self.send_water_volume()

                        elif expected_indicator == 0x13: # Processing Position response
                            position_raw = value_raw
                            # *** TODO: Consider adding filtering for position if needed ***
                            self.position = position_raw # Store raw position

                            # Log summary and publish
                            self.debug_print(
                                f"Result: Position={self.position:.4f}", # Units depend on motor config
                                level=0, # Minimal level
                            )
                            self.send_position()

                        # This else shouldn't be reached if logic is correct
                        # else:
                        #    self.debug_print(f"Logic Error: Unexpected indicator {received_indicator} passed checks.", level=0)

                    else:
                        # Float parsing failed
                        self.debug_print(
                            f"Failed to parse float from response for indicator {received_indicator}",
                            level=1,
                        )
                else:
                    # Indicator mismatch (e.g., sent Iq request, got position response)
                    self.debug_print(
                        f"Indicator mismatch: Sent command expecting {expected_indicator:02X}, received response with {received_indicator:02X}",
                        level=1,
                    )
            else:
                # Received a message, but not the expected response type (first byte != 0xB4)
                self.debug_print(
                    f"Received unexpected message type (first byte != 0xB4): {message.data[0]:02X}",
                    level=1,
                )
        else:
            # Timeout - no message received
            self.debug_print(
                f"No CAN response received within timeout for command expecting indicator {expected_indicator:02X}",
                level=1,
            )

        # --- Flip the Flag for Next Poll ---
        # Always flip the flag after attempting a poll (success or fail)
        # This ensures the commands alternate consistently.
        self.send_iq_next = not self.send_iq_next

    def send_water_volume(self):
        """Publishes the calculated water volume to the /water_qty topic."""
        msg = Int32()
        msg.data = self.volume
        self.water_qty_pub.publish(msg)
        # Optional: Add a log message here if needed, but it might be noisy
        # self.get_logger().info(f"Published Water Volume: {self.volume} mL")

    def send_torque(self):
        """Publishes the filtered torque value to the /torque topic."""
        msg = Float32()
        msg.data = self.torque
        self.torque_pub.publish(msg)
        # Optional: Add a log message
        # self.get_logger().info(f"Published Torque: {self.torque:.4f} Nm")

    def send_position(self):
        """*** NEW: Publishes the motor position to the /motor_position topic. ***"""
        msg = Float32()
        # Check if self.position has been set (it might not have if the first poll failed)
        # Using hasattr is safer than assuming it exists.
        if hasattr(self, 'position'):
            msg.data = self.position
            self.position_pub.publish(msg)
            # Optional: Add a log message
            # self.get_logger().info(f"Published Position: {self.position:.4f}")
        else:
             self.get_logger().warn("Attempted to publish position, but self.position is not yet available.")


    def shutdown(self):
        """Performs cleanup actions when the node is shutting down."""
        self.get_logger().info("Shutting down Water Node...")
        # Ensure the CAN bus object exists before trying to shut it down
        if hasattr(self, 'bus') and self.bus:
            try:
                self.bus.shutdown()
                self.get_logger().info("CAN bus shut down.")
            except Exception as e:
                self.get_logger().error(f"Error shutting down CAN bus: {e}")
        # The timer is managed by rclpy, no explicit shutdown needed here


def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create an instance of the WaterNode
    node = WaterNode()

    try:
        # Keep the node running, processing callbacks (like the timer)
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        node.get_logger().info("KeyboardInterrupt received, shutting down...")
    except Exception as e:
        # Log any other unexpected exceptions
        node.get_logger().error(f"Unhandled exception in main loop: {e}")
    finally:
        # --- Cleanup ---
        # Call the node's shutdown method for custom cleanup (like CAN bus)
        node.shutdown()
        # Destroy the node explicitly
        node.destroy_node()
        # Shutdown the ROS2 client library
        rclpy.shutdown()


if __name__ == '__main__':
    # This ensures the main() function is called when the script is executed
    main()
