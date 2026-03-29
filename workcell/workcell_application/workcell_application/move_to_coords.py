#!/usr/bin/env python3
"""
Utility script to move the robot to specific XYZ coordinates or a named pose.
Executes the movement immediately upon launch based on the provided parameters.
"""
import sys
import math

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from moveit.planning import MoveItPy


def plan_and_execute(robot, arm, logger, target):
    """
    Plans and executes a trajectory using MoveItPy.
    Target can be a string (named pose) or PoseStamped (cartesian).
    """
    arm.set_start_state_to_current_state()
    
    if isinstance(target, str):
        arm.set_goal_state(configuration_name=target)
    else:
        arm.set_goal_state(pose_stamped_msg=target, pose_link="pisoftgrip_tcp")
    
    plan_result = arm.plan()

    if plan_result:
        logger.info("Executing planned trajectory...")
        success = robot.execute(plan_result.trajectory, controllers=[])
        if not success:
            logger.error("❌ Trajectory execution failed by the controller! ❌")
            return False
        return True
    else:
        logger.error("❌ Motion planning failed! (Unreachable or collision) ❌")
        return False


class TargetNode(Node):
    """Node to handle parameter initialization and parsing."""
    def __init__(self):
        super().__init__('move_to_coords_node')
        
        # --- Declare Launch Parameters ---
        self.declare_parameter('named_pose', '')
        self.declare_parameter('coords', '')
        self.declare_parameter('yaw', 0.0) # Optional rotation around Z-axis in degrees
        
        # --- Read Parameters ---
        self.named_pose = self.get_parameter('named_pose').value
        self.coords_str = self.get_parameter('coords').value
        self.target_yaw = self.get_parameter('yaw').value

        self.valid_target = False
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0

        # --- 1. Validation: Check if ANY target is provided ---
        if not self.named_pose and not self.coords_str:
            self.get_logger().error("\n" + "="*60)
            self.get_logger().error("❌ NO TARGET SPECIFIED! ❌")
            self.get_logger().error("You must provide either a 'named_pose' or 'coords'.")
            self.get_logger().error("Examples:")
            self.get_logger().error('  ros2 launch workcell_application move_to_coords.launch.py named_pose:="ready"')
            self.get_logger().error('  ros2 launch workcell_application move_to_coords.launch.py coords:="0 0.5 0.1"')
            self.get_logger().error("="*60 + "\n")
            return

        # --- 2. Validation: Parse the coordinate string if provided ---
        if self.coords_str:
            try:
                # Remove brackets and commas, then split by spaces
                clean_str = self.coords_str.replace('[', '').replace(']', '').replace(',', ' ')
                parts = clean_str.split()
                
                if len(parts) != 3:
                    raise ValueError(f"Expected 3 values (X Y Z), got {len(parts)}")
                
                self.target_x = float(parts[0])
                self.target_y = float(parts[1])
                self.target_z = float(parts[2])
            except Exception as e:
                self.get_logger().error("\n" + "="*60)
                self.get_logger().error(f"❌ INVALID 'coords' FORMAT: {e} ❌")
                self.get_logger().error('Please use formats like: coords:="0 0.5 0.1" or coords:="[0, 0.5, 0.1]"')
                self.get_logger().error("="*60 + "\n")
                return

        # If we reach this point, the target is valid!
        self.valid_target = True


def main(args=None):
    rclpy.init(args=args)
    logger = get_logger("move_to_coords")
    
    # 1. Initialize ROS Node to get and validate parameters
    node = TargetNode()
    
    # 2. Fail Fast: If parameters are missing or broken, exit immediately!
    if not node.valid_target:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # 3. Only initialize MoveItPy if we actually have a valid target
    logger.info("="*60)
    logger.info("Valid target received. Initializing MoveItPy...")
    logger.info("="*60)
    ur5e = MoveItPy(node_name="move_to_coords_moveit")
    ur5e_arm = ur5e.get_planning_component("ur_arm")

    # --- BUILD TARGET ---
    if node.named_pose != '':
        target = node.named_pose
        logger.info(f"🎯 Target set to NAMED POSE: '{target}'")
    else:
        target = PoseStamped()
        target.header.frame_id = "ur5e_base_link"
        target.pose.position.x = node.target_x
        target.pose.position.y = node.target_y
        target.pose.position.z = node.target_z
        
        # Calculate quaternion from standard top-down grip + custom yaw
        yaw_rad = math.radians(node.target_yaw)
        q = quaternion_from_euler(math.pi, 0.0, yaw_rad)
        target.pose.orientation.x = q[0]
        target.pose.orientation.y = q[1]
        target.pose.orientation.z = q[2]
        target.pose.orientation.w = q[3]
        
        logger.info(f"🎯 Target set to CARTESIAN: X={node.target_x:.3f}, Y={node.target_y:.3f}, Z={node.target_z:.3f} (Yaw: {node.target_yaw}°)")

    try:
        success = plan_and_execute(ur5e, ur5e_arm, logger, target)
        
        if success:
            logger.info("✅ Target reached successfully!")
        else:
            logger.error("❌ Failed to reach target.")

    except KeyboardInterrupt:
        logger.info("Script aborted by user.")
    finally:
        node.destroy_node()
        if rclpy.ok(): 
            rclpy.shutdown()

if __name__ == '__main__':
    main()