#!/usr/bin/env python3
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

msg = """
---------------------------
Cartesian Control Mode
---------------------------
w/s: +/- X
a/d: +/- Y
q/e: +/- Z
u/j: +/- Roll
i/k: +/- Pitch
o/l: +/- Yaw
x: Exit
---------------------------
"""

moveBindings = {
    'w': (1, 0, 0, 0, 0, 0),
    's': (-1, 0, 0, 0, 0, 0),
    'a': (0, 1, 0, 0, 0, 0),
    'd': (0, -1, 0, 0, 0, 0),
    'q': (0, 0, 1, 0, 0, 0),
    'e': (0, 0, -1, 0, 0, 0),
    'u': (0, 0, 0, 1, 0, 0),
    'j': (0, 0, 0, -1, 0, 0),
    'i': (0, 0, 0, 0, 1, 0),
    'k': (0, 0, 0, 0, -1, 0),
    'o': (0, 0, 0, 0, 0, 1),
    'l': (0, 0, 0, 0, 0, -1),
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class CartesianTeleop(Node):
    def __init__(self):
        super().__init__('cartesian_keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cartesian_cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)

    def run(self):
        print(msg)
        try:
            while True:
                key = getKey(self.settings)
                if key in moveBindings.keys():
                    x = moveBindings[key][0]
                    y = moveBindings[key][1]
                    z = moveBindings[key][2]
                    thx = moveBindings[key][3]
                    thy = moveBindings[key][4]
                    thz = moveBindings[key][5]

                    twist = Twist()
                    # We use the values as "steps" (e.g. 1 means 1 step positive)
                    # The C++ node will define the step size (e.g. 0.02m)
                    twist.linear.x = float(x)
                    twist.linear.y = float(y)
                    twist.linear.z = float(z)
                    twist.angular.x = float(thx)
                    twist.angular.y = float(thy)
                    twist.angular.z = float(thz)
                    
                    self.publisher_.publish(twist)
                    print(f"Sent command: {key}")

                elif key == 'x':
                    break
                else:
                    if (key == '\x03'):
                        break

        except Exception as e:
            print(e)

        finally:
            twist = Twist()
            self.publisher_.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = CartesianTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
