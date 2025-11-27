#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c

t/g : increase/decrease max speeds by 10%
y/h : increase/decrease only linear speed by 10%
u/j : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0, 0, 0, 0, 0),
    's': (-1, 0, 0, 0, 0, 0),
    'a': (0, 1, 0, 0, 0, 0),
    'd': (0, -1, 0, 0, 0, 0),
    'q': (0, 0, 1, 0, 0, 0),
    'e': (0, 0, -1, 0, 0, 0),
    'z': (0, 0, 0, 0, 0, 1),
    'c': (0, 0, 0, 0, 0, -1),
    'x': (0, 0, 0, 0, 0, 0),
}

speedBindings = {
    't': (1.1, 1.1),
    'g': (0.9, 0.9),
    'y': (1.1, 1.0),
    'h': (0.9, 1.0),
    'u': (1.0, 1.1),
    'j': (1.0, 0.9),
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main():
    settings = saveTerminalSettings()
    rclpy.init()
    
    node = rclpy.create_node('servo_keyboard_input')
    
    # Matches servo_parameters.yaml: cartesian_command_in_topic: "~/delta_twist_cmds"
    # Since the node name in launch file is "servo_node", this topic is likely /servo_node/delta_twist_cmds
    # But we can remap or just publish to the absolute path if we know it.
    # Let's assume the servo node is namespaced or we publish to the relative topic if we were in the same namespace.
    # Ideally, we publish to `/servo_node/delta_twist_cmds`.
    
    pub = node.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

    speed = 0.5
    turn = 0.5
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0

    try:
        print(msg)
        print(f"currently:\tspeed {speed}\tturn {turn}")
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][5]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                print(f"currently:\tspeed {speed}\tturn {turn}")
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if (key == '\x03'):
                    break

            twist = TwistStamped()
            twist.header.stamp = node.get_clock().now().to_msg()
            twist.header.frame_id = 'base_link' # Or 'world', or 'ee_link' depending on config. 
                                                # servo_parameters.yaml says: apply_twist_commands_about_ee_frame: true
                                                # usually we send commands in the frame we want to move relative to.
                                                # If we want to move "forward" in EE frame, we send X in EE frame.
                                                # If we want to move "up" in World frame, we send Z in World frame.
                                                # Let's default to 'base_link' for general movement or 'ee_link' if the user wants tool-centric.
                                                # For this simple teleop, let's use 'base_link' (robot base).

            twist.twist.linear.x = x * speed
            twist.twist.linear.y = y * speed
            twist.twist.linear.z = z * speed
            twist.twist.angular.x = 0.0
            twist.twist.angular.y = 0.0
            twist.twist.angular.z = th * turn

            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = TwistStamped()
        twist.header.stamp = node.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        twist.twist.linear.x = 0.0
        twist.twist.linear.y = 0.0
        twist.twist.linear.z = 0.0
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0
        pub.publish(twist)

        restoreTerminalSettings(settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
