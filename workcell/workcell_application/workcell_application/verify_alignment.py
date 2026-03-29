#!/usr/bin/env python3
"""
Verification script for alignment and calibration.
Moves the robot to defined test positions from a YAML file.
"""
import os
import time
import math
import yaml

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from moveit.planning import MoveItPy
from ament_index_python.packages import get_package_share_directory

def plan_and_execute(robot, arm, logger, target):
    """
    Plans and executes a trajectory to either a named pose or a PoseStamped.
    Includes validation and a physical settling delay for the simulation.
    """
    arm.set_start_state_to_current_state()
    
    if isinstance(target, str):
        # If the target value is a string, use the predefined SRDF pose
        arm.set_goal_state(configuration_name=target)
    else:
        # If the target is a coordinate, use the TCP (Tool Center Point)
        arm.set_goal_state(pose_stamped_msg=target, pose_link="pisoftgrip_tcp")
    
    plan_result = arm.plan()

    if plan_result:
        logger.info("Executing planned trajectory...")
        
        # 1. Execute the trajectory and store the success status
        success = robot.execute(plan_result.trajectory, controllers=[])
        
        # 2. If execution fails on the controller side, catch it and return False
        if not success:
            logger.error("❌ Trajectory execution failed by the controller! ❌")
            return False
            
        return True
    else:
        logger.error("❌ Motion planning failed! ❌")
        return False


class TriggerNode(Node):
    """A simple node to listen for a manual trigger message."""
    def __init__(self):
        super().__init__('alignment_trigger_node')

        # --- Trigger Setup ---
        self.triggered = False
        self.subscription = self.create_subscription(
            Empty,
            '/trigger/next_step',
            self.trigger_callback,
            10
        )

    def trigger_callback(self, msg):
        self.get_logger().info("Next step trigger received! Moving to the next pose...")
        self.triggered = True


def load_poses_from_yaml(logger):
    """Liest die Test-Posen aus der YAML-Datei."""
    try:
        # Passe hier deinen Package-Namen an, falls er anders lautet!
        pkg_share = get_package_share_directory('workcell_application')
        yaml_path = os.path.join(pkg_share, 'config', 'verify_alignment.yaml')
        
        with open(yaml_path, 'r') as file:
            data = yaml.safe_load(file)
            return data.get('test_poses', [])
    except Exception as e:
        logger.error(f"❌ Failed to load YAML file: {e} ❌")
        return []


def main(args=None):
    rclpy.init(args=args)
    logger = get_logger("verify_alignment")
    logger.info("="*50)
    logger.info("Initializing MoveItPy...")
    logger.info("="*50 + "\n")

    # Instantiate MoveItPy and get the planning component
    ur5e = MoveItPy(node_name="verify_alignment_moveit")
    ur5e_arm = ur5e.get_planning_component("ur_arm")
    
    # Standard orientation: Gripper pointing straight down
    q_down = quaternion_from_euler(math.pi, 0.0, 0.0)

    # --- Lade Posen aus YAML ---
    test_poses = load_poses_from_yaml(logger)
    if not test_poses:
        logger.error("No test poses loaded. Exiting.")
        rclpy.shutdown()
        return

    # --- MOVE TO READY POSE FIRST ---
    logger.info("="*50)
    logger.info("Initializing: Moving to 'ready' pose...")
    logger.info("="*50 + "\n")
    plan_and_execute(ur5e, ur5e_arm, logger, "ready")

    logger.info("="*50)
    logger.info(" ALIGNMENT VERIFICATION SCRIPT STARTED ")
    logger.info(" Measure from the robot base to the TCP at each")
    logger.info(" position and compare with expected values.")
    logger.info("="*50 + "\n")

    trigger_node = TriggerNode()

    try:
        # Iterate through all defined test poses
        for i, pose_config in enumerate(test_poses):
            name = pose_config.get("name", f"Pose {i+1}")
            pose_type = pose_config.get("type", "cartesian")
            
            logger.info("\n" + "="*50)
            logger.info("="*50)
            logger.info(f"🟢 Test {i+1}: {name} 🟢")
            logger.info("="*50)
            
            # --- THE TRIGGER WAIT LOOP ---
            logger.info("="*50)
            logger.info("⏯  Waiting for manual trigger. Run in a separate terminal:")
            logger.info("ros2 topic pub --once /trigger/next_step std_msgs/msg/Empty")
            logger.info("="*50 + "\n")
            
            trigger_node.triggered = False # Reset the flag
            
            # Spin the node to check for messages until the flag becomes True
            while not trigger_node.triggered and rclpy.ok():
                rclpy.spin_once(trigger_node, timeout_sec=0.1)

            # 2. Prepare the target based on its type
            if pose_type == "named":
                target = pose_config["target"]
            else:
                target = PoseStamped()
                target.header.frame_id = "ur5e_base_link"
                target.pose.position.x = float(pose_config.get("x", 0.0))
                target.pose.position.y = float(pose_config.get("y", 0.0))
                target.pose.position.z = float(pose_config.get("z", 0.0))
                
                # Wenn in der YAML ein 'q' angegeben ist, nutze dieses.
                # Ansonsten nimm automatisch q_down als Standard.
                if "q" in pose_config:
                    q = pose_config["q"]
                else:
                    q = q_down
                    
                target.pose.orientation.x = float(q[0])
                target.pose.orientation.y = float(q[1])
                target.pose.orientation.z = float(q[2])
                target.pose.orientation.w = float(q[3])

            # 3. Plan and execute the motion
            success = plan_and_execute(ur5e, ur5e_arm, logger, target)
            
            if success:
                logger.info("="*50)
                logger.info(f"✅ Position {i+1} reached:" + "\n")
                logger.info(f"{name}" + "\n")
                logger.info("Measure from the robot base to the TCP at each")
                logger.info("position and compare with expected values.")
                logger.info("="*50 + "\n")
            else:
                logger.info("="*50)
                logger.error("❌ Position could not be reached. ❌")
                logger.info("="*50 + "\n")

        logger.info("="*50)
        logger.info("All test poses have been executed.")
        logger.info("="*50 + "\n")

    except KeyboardInterrupt:
        logger.info("="*50)
        logger.info("Verification aborted by user.")
        logger.info("="*50 + "\n")
    finally:
        logger.info("="*50)
        logger.info("Shutting down verification script.")
        logger.info("="*50 + "\n")
        trigger_node.destroy_node()
        if rclpy.ok(): 
            rclpy.shutdown()

if __name__ == '__main__':
    main()