#!/usr/bin/env python3
"""
Simplified MTC example that works with MoveIt 2 Jazzy
Just demonstrates the API without complex robot model loading
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
import time


class SimpleMoveSequence(Node):
    """
    Simple example that demonstrates sequential motion planning
    without using MTC (Task Constructor has Python binding issues in Jazzy)
    """
    
    def __init__(self):
        super().__init__('simple_move_sequence')
        self.get_logger().info("Simple Move Sequence Node starting...")
        self.get_logger().info("This is a simplified example due to MTC Python API limitations")
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publisher for joint commands
        self.joint_pub = self.create_publisher(JointState, '/joint_command', 10)
        
        self.current_joints = None
        self.get_logger().info("Node initialized. Waiting for joint states...")
        
    def joint_state_callback(self, msg):
        """Store current joint states"""
        if self.current_joints is None:
            self.get_logger().info("Received first joint state")
        self.current_joints = msg
        
    def move_to_joints(self, joint_values, duration=2.0):
        """Move to target joint positions"""
        if self.current_joints is None:
            self.get_logger().error("No joint state received yet!")
            return False
            
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['L1', 'L2', 'L3', 'L4', 'L5', 'L6']
        msg.position = joint_values
        
        self.get_logger().info(f"Moving to: {joint_values}")
        self.joint_pub.publish(msg)
        time.sleep(duration)
        return True
        
    def run_sequence(self):
        """Execute a sequence of waypoints"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("Starting waypoint sequence...")
        self.get_logger().info("=" * 60)
        
        # Wait for joint states
        rate = self.create_rate(10)
        timeout = 5.0
        start_time = time.time()
        while self.current_joints is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        if self.current_joints is None:
            self.get_logger().error("Failed to receive joint states!")
            return False
        
        # Waypoint 1
        self.get_logger().info("\n[1/4] Moving to waypoint 1...")
        self.move_to_joints([0.5, 0.3, -0.5, 0.0, 0.0, 0.0], 3.0)
        
        # Waypoint 2
        self.get_logger().info("\n[2/4] Moving to waypoint 2...")
        self.move_to_joints([-0.5, 0.5, -0.3, 0.2, -0.2, 0.0], 3.0)
        
        # Waypoint 3
        self.get_logger().info("\n[3/4] Moving to waypoint 3...")
        self.move_to_joints([0.0, 0.8, -0.8, 0.0, 0.0, 0.0], 3.0)
        
        # Return to zero
        self.get_logger().info("\n[4/4] Returning to zero position...")
        self.move_to_joints([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 3.0)
        
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("✓ Sequence complete!")
        self.get_logger().info("=" * 60)
        return True


def main(args=None):
    rclpy.init(args=args)
    
    print("\n" + "=" * 60)
    print("NOTE: MTC Python bindings have compatibility issues in Jazzy")
    print("This is a simplified alternative that demonstrates waypoint motion")
    print("For full MTC functionality, use the C++ API")
    print("=" * 60 + "\n")
    
    try:
        node = SimpleMoveSequence()
        
        # Run the sequence
        time.sleep(1.0)  # Let node initialize
        node.run_sequence()
        
        # Keep node alive
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
