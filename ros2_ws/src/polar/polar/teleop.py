#!/usr/bin/env python3
import sys, termios, tty, select
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import TargetPosePolar

HELP = """
Keyboard:
  w / s  -> r += 0.1 / r -= 0.1   (m)
  d / a  -> v_theta += 0.1 / -= 0.1   (m/s along tangent you use)
  z / x  -> yaw_rate += 0.1 / -= 0.1  
  space  -> zero r and v_theta
  h      -> help
  x      -> exit
Publishing /goal_pose_polar at 20 Hz with relative=True
"""

class TargetPoseKeyboard(Node):
    def __init__(self):
        super().__init__('target_pose_keyboard')
        self.pub = self.create_publisher(TargetPosePolar, '/goal_pose_polar', 10)
        self.dt = 1.0/20.0
        self.timer = self.create_timer(self.dt, self.tick)

        # state
        self.r = 0.0
        self.v_theta = 0.0
        self.step = 0.5  # increment per key press
        self.v_z = 0.0 
        self.yaw_rate = 0.0
        self.yaw_step = 0.02

        # non-blocking keyboard setup
        self.fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        print(HELP)

    def destroy_node(self):
        try:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self._old)
        except Exception:
            pass
        return super().destroy_node()

    def _read_key(self):
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        if dr:
            return sys.stdin.read(1)
        return None

    def _handle_key(self, k):
        if k in ('\x03', '\x04'):  # x / Ctrl-C / Ctrl-D
            rclpy.shutdown()
            return
        if k == 'h':
            print(HELP, end='')
        elif k == ' ':
            self.r = 0.0
            self.v_theta = 0.0
            self.v_z = 0.0
            self.yaw_rate = 0.0
        elif k == 'w':
            self.r += self.step
        elif k == 's':
            self.r -= self.step
        elif k == 'd':
            self.v_theta -= self.step
        elif k == 'a':
            self.v_theta += self.step
        elif k == 'r':
            self.v_z += self.step
        elif k == 'f':
            self.v_z -= self.step
        elif k == 'x':
            self.yaw_rate -= self.yaw_step
        elif k == 'z':
            self.yaw_rate += self.yaw_step
        # echo compact status on any change
        if k in ('w','s','a','d','x', 'z',' ', 'r', 'f'):
            print(f"\r r={self.r:.2f}  v_theta={self.v_theta:.2f}, v_z={self.v_z:.2f}, yaw_rate = {self.yaw_rate:.2f}", end='', flush=True)

    def tick(self):
        # process any pending keystrokes (allow multiple per cycle)
        while True:
            k = self._read_key()
            if not k:
                break
            self._handle_key(k)

        # build & publish message
        msg = TargetPosePolar()
        msg.relative = True
        msg.v_theta = float(self.v_theta)
        msg.v_r = float(self.r)
        msg.v_z = float(self.v_z)
        msg.yaw_rate = float(self.yaw_rate)

        # keep the rest simple/neutral; adjust if your interface needs other fields
        msg.r = 0.0  # always zero; we use v_r instead
        msg.z = 0.0
        msg.theta = 0.0
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = TargetPoseKeyboard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
