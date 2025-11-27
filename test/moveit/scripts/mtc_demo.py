#!/usr/bin/env python3
"""
MoveIt Task Constructor example for aar6 robot
This demonstrates a simple pick-and-place like sequence
"""

import rclpy
from rclpy.node import Node
from moveit.task_constructor import core, stages
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import PoseStamped
import sys


class MTCTaskNode(Node):
    def __init__(self):
        super().__init__('mtc_task_node')
        
        # Create the task
        self.task = core.Task("aar6_demo_task")
        
        # Define planning group
        self.group_name = "kin_group"
        self.task.setProperty("group", self.group_name)
        self.task.setProperty("eef", "gripper")  # end effector group
        self.task.setProperty("ik_frame", "ee_link")  # end effector link
        
        self.get_logger().info("MTC Task Node initialized")
        
        # Build the task
        self.setup_task()
        
    def setup_task(self):
        """Build the MTC task with stages"""
        
        # Start from current state
        current_state = stages.CurrentState("current state")
        self.task.add(current_state)
        
        # Move to a home/ready position
        move_to_home = stages.MoveTo("move to home", core.JointInterpolationPlanner())
        move_to_home.setGroup(self.group_name)
        move_to_home.setGoal("home")  # Assumes you have a "home" pose defined in SRDF
        self.task.add(move_to_home)
        
        # Create a container for the pick sequence
        pick_container = core.SerialContainer("pick sequence")
        self.task.add(pick_container)
        
        # Approach motion
        approach = stages.MoveRelative("approach object", core.CartesianPath())
        approach.setGroup(self.group_name)
        approach.setIKFrame("ee_link")
        approach.setMinMaxDistance(0.05, 0.15)
        
        # Set approach direction (move down in Z)
        approach.along("z")
        pick_container.add(approach)
        
        # Generate grasp pose
        generate_grasp = stages.GenerateGraspPose("generate grasp pose")
        generate_grasp.setEndEffector("gripper")
        generate_grasp.setObject("target_object")
        generate_grasp.setAngleDelta(0.2)
        pick_container.add(generate_grasp)
        
        # Allow collision with object
        allow_collision = stages.ModifyPlanningScene("allow collision")
        allow_collision.allowCollisions(
            "target_object",
            self.task.getRobotModel().getJointModelGroup("gripper").getLinkModelNames(),
            True
        )
        pick_container.add(allow_collision)
        
        # Close gripper
        close_gripper = stages.MoveTo("close gripper", core.JointInterpolationPlanner())
        close_gripper.setGroup("gripper")
        close_gripper.setGoal("closed")  # Assumes "closed" pose in SRDF
        pick_container.add(close_gripper)
        
        # Attach object
        attach_object = stages.ModifyPlanningScene("attach object")
        attach_object.attachObject("target_object", "ee_link")
        pick_container.add(attach_object)
        
        # Lift motion
        lift = stages.MoveRelative("lift object", core.CartesianPath())
        lift.setGroup(self.group_name)
        lift.setIKFrame("ee_link")
        lift.setMinMaxDistance(0.05, 0.15)
        lift.along("z")
        pick_container.add(lift)
        
        # Move to place location
        move_to_place = stages.MoveTo("move to place", core.JointInterpolationPlanner())
        move_to_place.setGroup(self.group_name)
        
        # Create place pose
        place_pose = PoseStamped()
        place_pose.header.frame_id = "world"
        place_pose.pose.position.x = 0.3
        place_pose.pose.position.y = 0.3
        place_pose.pose.position.z = 0.3
        place_pose.pose.orientation.w = 1.0
        
        move_to_place.setGoal(place_pose)
        self.task.add(move_to_place)
        
        # Open gripper
        open_gripper = stages.MoveTo("open gripper", core.JointInterpolationPlanner())
        open_gripper.setGroup("gripper")
        open_gripper.setGoal("open")  # Assumes "open" pose in SRDF
        self.task.add(open_gripper)
        
        # Detach object
        detach_object = stages.ModifyPlanningScene("detach object")
        detach_object.detachObject("target_object", "ee_link")
        self.task.add(detach_object)
        
        # Return to home
        return_home = stages.MoveTo("return home", core.JointInterpolationPlanner())
        return_home.setGroup(self.group_name)
        return_home.setGoal("home")
        self.task.add(return_home)
        
        self.get_logger().info("Task pipeline built successfully")
        
    def plan(self):
        """Plan the task"""
        self.get_logger().info("Planning task...")
        
        try:
            success = self.task.plan()
            if success:
                self.get_logger().info("✓ Task planning succeeded!")
                return True
            else:
                self.get_logger().error("✗ Task planning failed")
                return False
        except Exception as e:
            self.get_logger().error(f"Planning error: {str(e)}")
            return False
            
    def execute(self):
        """Execute the planned task"""
        self.get_logger().info("Executing task...")
        
        try:
            success = self.task.execute()
            if success:
                self.get_logger().info("✓ Task execution succeeded!")
                return True
            else:
                self.get_logger().error("✗ Task execution failed")
                return False
        except Exception as e:
            self.get_logger().error(f"Execution error: {str(e)}")
            return False


def main(args=None):
    rclpy.init(args=args)
    
    node = MTCTaskNode()
    
    # Plan the task
    if node.plan():
        # Execute the task if planning succeeded
        node.execute()
    
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
