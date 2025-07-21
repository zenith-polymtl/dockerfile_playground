#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import zenmav.core as Zenmav
import itertools
import numpy as np


def func_distance(pos1, pos2):
    '''Possible de l'utiliser pour des données métriques PAS GPS'''
    ''' Voir si ça semble good'''
    return np.linalg.norm(np.array(pos1) - np.array(pos2))


def calculate_cost(route, distances):
    total_cost = 0
    n = len(route)
    for i in range(n):
        current_pt = route[i][0]
        next_pt = route[(i + 1) % n][0]  # wrap around
        if (current_pt, next_pt) in distances:
            total_cost += distances[(current_pt, next_pt)]
        else:
            total_cost += distances[(next_pt, current_pt)]
    return total_cost


def tmp_solution(buckets):
    distances = {}
    # Build up a dictionary of pairwise distances
    for i in range(len(buckets) - 1):
        for j in range(i + 1, len(buckets)):
            d = func_distance(buckets[i][1], buckets[j][1])
            distances[(buckets[i][0], buckets[j][0])] = d

    all_permutations = itertools.permutations(buckets)
    min_cost = float("inf")
    optimal_route = None
    for perm in all_permutations:
        cost = calculate_cost(perm, distances)
        if cost < min_cost:
            min_cost = cost
            optimal_route = perm

    # Return the best route plus the distance dict
    return list(optimal_route), distances


# ---------------------------
#   Define mission STATES
# ---------------------------


flying_height = 20


class MissionState:
    IDLE = 0
    WAIT_FOR_TAKEOFF = 1
    GOTO_WATER_SOURCE = 2
    WAIT_FOR_TRAVEL = 3
    LOWERING_DRONE = 4
    WAIT_FOR_LOWER = 5
    APPROACH = 6
    WAIT_FINISH = 7
    GOTO_NEXT_BUCKET = 8
    MANUAL = 9
    FINISHED = 99
    # You can add states for if you want


class Target_info:
    def __init__(self, name, approach_height, msg, is_bucket):
        self.name = name
        self.approach_height = approach_height
        self.vision_message = msg
        self.is_bucket = is_bucket
        


class StateNode(Node):
    def __init__(self, position_dict):
        super().__init__("State_node")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        #TODO Use pymavlink global coordinates instead of local
        self.source = Target_info("source", 8, "SOURCE", is_bucket=False)
        self.bucket = Target_info("bucket", 5, "BUCKET", is_bucket=True)

        # TMP route setup
        self.position_dict = position_dict
        self.optimal_route, self.distances = tmp_solution(self.position_dict)

        # Some fixed references
        self.flying_height = flying_height
        self.ground_station = ("ground station", [0, 0, 0])
        self.water_source = ("water source", [50.1013, -110.734, self.flying_height])
        self.current_pos = None  # store last known drone position
        self.current_target = None  # whichever waypoint we’re currently trying to reach

        # We track mission states
        self.state = MissionState.IDLE
        self.finished_manual_approach = False
        self.manual = True
        self.taken_off = False
        self.reached_wp = False
        self.ready_to_fly = False
        #TODO Get message from another node to tell the state node if the drone is empty
        self.empty = False
        #TODO Compute the number of loops needed based of the drone's and buckets' water capacity
        self.num_of_loop = 2

        

        # Battery info
        self.drone_battery = 100.0
        self.drone_travel_efficiency = 2.0  # distance units per 1% battery
        #TODO remove this efficency metrics, wont have enough time to test it
        #Implement a battery check function that will return the battery level of the drone...

        # Create Publishers / Subscribers
        self.publisher_vision = self.create_publisher(String, "/go_vision", qos_profile)
        self.manual_sub = self.create_subscription(
            String, "/manual", self.manual_callback, qos_profile
        )
        self.finished_manual_sub = self.create_subscription(
            String, "/task_end", self.end_approach_callback, qos_profile
        )
        self.battery_sub = self.create_subscription(
            String, "/battery_changed", self.notify_battery, qos_profile
        )
        self.abort_sub = self.create_subscription(
            String, "/abort_state", self.abort, qos_profile
        )
        self.armed_sub = self.create_subscription(
            String, "/confirm_arming", self.confirm_arming, qos_profile
        )

        # MAVLink Connection
        self.mav = Zenmav("udp:127.0.0.1:14551")

        # Start a short, frequent timer for mission logic
        self.main_timer = self.create_timer(0.5, self.mission_step)
        # Start a less frequent timer for battery checks
        self.batt_timer = self.create_timer(30.0, self.charge_opportunity)

        self.get_logger().info("✅ State node started.")

    ###########
    # callbacks
    ###########

    def abort(self, msg):
        self.get_logger().info("Shutting down node...")

        # Close the node after command sent to ensure cutoff from further autonomous commands from state node
        # Vision possible if accesed via manual control
        self.destroy_node()
        rclpy.shutdown()

    def notify_battery(self, msg):
        if msg.data == "CHANGED":
            self.drone_battery = 100.0

    def manual_callback(self, msg):
        if msg.data == "MANUAL":
            self.manual = True
        elif msg.data == "AUTO":
            self.manual = False

    def end_approach_callback(self, msg):
        if msg.data == "END":
            self.finished_manual_approach = True

    def confirm_arming(self, msg):
        if msg.data == "ARM":
            self.ready_to_fly = True

    # -----------------------------------------------------
    #  The main state machine logic runs in mission_step()
    # -----------------------------------------------------
    def mission_step(self):
        # At each timer tick, we check which state we are in,
        # see if we have completed it, or if we need to do something else.

        # Example manual override check:
        if self.state == MissionState.MANUAL:
            self.get_logger().info(
                    "🚀 DRONE"
                )
            pass

        if self.state == MissionState.IDLE:
            self.get_logger().info("🚀 Waiting for arming and guided.")
            # We want to start by takeoff
            if self.ready_to_fly:
                self.mav.takeoff(
                    self.flying_height, wait_to_takeoff=False
                )  # Non-blocking call
                self.state = MissionState.WAIT_FOR_TAKEOFF
                self.get_logger().info(
                    "🚀 Takeoff initiated. Transition to WAIT_FOR_TAKEOFF."
                )

        elif self.state == MissionState.WAIT_FOR_TAKEOFF:
            # Check if we are at ~alt=20
            N, E, D = self.mav.get_local_pos()
            # Suppose we consider "taken_off" if alt >=  tkf height -1, negative convetion for up, so inverted 
            if -self.flying_height + 1 >= D:
                self.taken_off = True
                self.get_logger().info(
                    "Takeoff complete! Transition to GOTO_WATER_SOURCE."
                )
                self.state = MissionState.GOTO_WATER_SOURCE

        elif self.state == MissionState.GOTO_WATER_SOURCE:
            # we just set a global target
            self.current_target = self.water_source
            self.target_type = self.source
            self.send_global_target(self.water_source, label="WATER SOURCE")
            self.state = MissionState.WAIT_FOR_TRAVEL

        elif self.state == MissionState.WAIT_FOR_TRAVEL:
            # Check if near the current_target
            self.get_logger().info("Flying towards target")
            #TODO implement the AEAC-mission 1 threshold logic
            if self.mav.is_near_waypoint(
                self.current_target[1], self.mav.get_global_pos(), 0.00003
            ):
                self.get_logger().info(f"Reached {self.target_type}. Next: VISION/APPROACH.")
                self.state = MissionState.LOWERING_DRONE

        elif self.state == MissionState.LOWERING_DRONE:
            #TODO Remove all instances of local position and replace with global position
            self.current_pos = self.mav.get_local_pos()
            self.mav.local_target(
                [self.current_pos[0], self.current_pos[1], self.target_type.approach_height], wait_to_reach=False
            )
            self.get_logger().info(f"Lowering down to {self.current_target[0]}")
            self.state = MissionState.WAIT_FOR_LOWER

        elif self.state == MissionState.WAIT_FOR_LOWER:

            if self.mav.is_near_waypoint(
                self.mav.get_local_pos()[2],
                self.target_type.approach_height,
                0.5
            ):
                self.get_logger().info(f"Into position, beginning Vision approach.")
                self.state = MissionState.APPROACH

        elif self.state == MissionState.APPROACH:
            # Start vision, or do manual approach again
            if not self.manual:
                self.finished_bucket = False
                self.start_vision()

                self.state = MissionState.WAIT_FINISH
            else:
                # If manual, we wait for /task_end:
                if self.finished_manual_approach:
                    self.state = MissionState.GOTO_NEXT_BUCKET
                    self.finished_manual_approach = False
                    
                    # If the drone just filled a bucket, checking if the drone is empty and going to water source if it is
                    # If the drone just went to a water source, registering that it's now not empty
                    if self.target_type.is_bucket:
                        self.optimal_route.pop(0)
                        if self.empty:
                            if not self.possible_movement(self.water_source[1]):
                                self.get_logger().warn(
                                    "Not enough battery to reach source + return; RTL now."
                                )
                                self.mav.RTL()
                                self.state = MissionState.FINISHED
                                return
                            else:
                                self.state = MissionState.GOTO_WATER_SOURCE
                    elif self.target_type == self.source:
                        self.empty = False
                else:
                    self.get_logger().info("Waiting for manual approach to end...")

        elif self.state == MissionState.WAIT_FINISH:
            if self.finished_bucket:

                self.get_logger().info("Target treated successfully")
                self.state = MissionState.GOTO_NEXT_BUCKET
                self.finished_bucket = False

                # If the drone just filled a bucket, checking if the drone is empty and going to water source if it is
                # If the drone just went to a water source, registering that it's now not empty
                if self.target_type.is_bucket:
                    self.optimal_route.pop(0)
                    if self.empty:
                        if not self.possible_movement(self.water_source[1]):
                            self.get_logger().warn(
                                "Not enough battery to reach source + return; RTL now."
                            )
                            self.mav.RTL()
                            self.state = MissionState.FINISHED
                            return
                        else:
                            self.state = MissionState.GOTO_WATER_SOURCE
                elif self.target_type == self.source:
                    self.empty = False

        elif self.state == MissionState.GOTO_NEXT_BUCKET:
            if len(self.optimal_route) == 0:
                # No more buckets, we have done one full loop 
                self.num_of_loop -= 1
                if self.num_of_loop == 0:
                    # No more buckets and no more loops, we are done
                    self.get_logger().info("No more buckets => Return to base (RTL).")
                    self.mav.RTL()
                    self.state = MissionState.FINISHED
                    return
                else:
                    # Recalculating the optimal route and going to get water to start a new loop
                    self.get_logger().info("Starting a new loop")
                    self.optimal_route, self.distances = tmp_solution(self.position_dict)
                    self.state = MissionState.GOTO_WATER_SOURCE
            # Grab the next bucket
            next_bucket = self.optimal_route[0]
            if not self.possible_movement(next_bucket):
                self.get_logger().warn(
                    "Not enough battery to reach next bucket + return; RTL now."
                )
                self.mav.RTL()
                self.state = MissionState.FINISHED
                return

            # We do have enough battery, so go
            self.current_target = next_bucket
            self.target_type = self.bucket

            self.send_global_target(next_bucket, label=next_bucket[0])
            self.state = MissionState.WAIT_FOR_TRAVEL

        elif self.state == MissionState.FINISHED:
            # We do nothing. The mission is ended. Possibly keep spinning
            pass

        else:
            # No other states
            pass

    # -----------
    #  Helpers
    # -----------

    def send_global_target(self, waypoint, label=""):
        """
        Non-blocking: just sends the target once via helper_func.
        """
        coords = waypoint[1]  # (lat, lon, alt)
        self.get_logger().info(f"Moving to target {label} => {coords}")
        self.mav.global_target(coords, wait_to_reach=False)

    def start_vision(self):
        msg = String()
        msg.data = self.target_type.vision_message
        self.publisher_vision.publish(msg)
        self.get_logger().info("VISION GO => " + self.target_type.name)

    def charge_opportunity(self):
        # This is the 10 second timer for battery checks

        # For example:
        if self.drone_battery <= 15:
            self.get_logger().warn("Battery low => returning to launch for charge.")
            self.mav.RTL()
            self.state = MissionState.FINISHED
            # If you want to do a real charge cycle, you can set self.drone_battery = 100 later
        # Otherwise do nothing

    def possible_movement(self, target_pos):
        # This code is essentially your same function
        # but we skip the while loops
        try:
            next_distance = self.distances[(self.current_target[0], target_pos[0])]
        except KeyError:
            # fallback
            next_distance = func_distance(self.current_target[1], target_pos[1])

        distance_target_to_base = func_distance(target_pos[1], self.ground_station[1])
        # approximate battery usage
        battery_at_target = self.drone_battery - (
            next_distance / self.drone_travel_efficiency
        )
        autonomy_at_target = battery_at_target * self.drone_travel_efficiency
        return autonomy_at_target > distance_target_to_base


def main(args=None):
    rclpy.init(args=args)
    buckets = [
        ("bucket_1", [50.101222, -110.738856, flying_height]),
        ("bucket_2", [50.101796, -110.739031, flying_height]),
        ("bucket_3", [50.101195, -110.738814, flying_height]),
        ("bucket_4", [50.101195, -110.738554, flying_height]),
        ("bucket_5", [50.101928, -110.738858, flying_height]),
        ("bucket_6", [50.101876, -110.738629, flying_height]),
    ]
    node = StateNode(buckets)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
