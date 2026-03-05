#!/usr/bin/env python3
"""
Verification script for alignment and calibration.
Moves the robot to defined test positions requiring user confirmation.
"""
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from moveit.planning import MoveItPy

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
        logger
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
        self.triggered = False
        self.subscription = self.create_subscription(
            Empty,
            '/next_step',
            self.trigger_callback,
            10
        )

    def trigger_callback(self, msg):
        self.get_logger().info("Next step trigger received! Moving to the next pose...")
        self.triggered = True


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

    # --- MOVE TO READY POSE FIRST ---
    logger.info("="*50)
    logger.info("Initializing: Moving to 'ready' pose...")
    logger.info("="*50 + "\n")
    plan_and_execute(ur5e, ur5e_arm, logger, "ready")

    # =========================================================================
    # CONFIGURATION OF TEST POSES
    # =========================================================================
    # TEMPLATE & EXPLANATION:
    # This list contains dictionaries. Each dictionary defines one test pose.
    # There are two types of poses you can define: "named" or "cartesian".
    #
    # 1. NAMED POSE (Predefined in your SRDF, e.g., via MoveIt Setup Assistant)
    # {
    #     "name": "Description shown in the terminal logger",
    #     "type": "named",       # Must be strictly "named"
    #     "target": "ready"      # The exact string name of the pose in the SRDF
    # }
    #
    # 2. CARTESIAN POSE (Exact X/Y/Z coordinates in the workspace)
    # {
    #     "name": "Description shown in the terminal logger",
    #     "type": "cartesian",   # Must be strictly "cartesian"
    #     "x": 0.0,              # X position in meters (relative to ur5e_base_link)
    #     "y": 0.5,              # Y position in meters
    #     "z": 0.10,             # Z position in meters
    #     "q": q_down            # Quaternion array [x, y, z, w] for the gripper orientation
    # }
    # =========================================================================
    
    test_poses = [
        {
            "name": "Ready Pose (Home)",
            "type": "named",
            "target": "ready"
        },
        {
            "name": "TCP Check: 10cm Z-Height, 50cm Y-Offset",
            "type": "cartesian",
            "x": 0.0,
            "y": 0.5,
            "z": 0.10,
            "q": q_down
        },
        {
            "name": "TCP Check: 1cm Z-Height, 60cm Y-Offset",
            "type": "cartesian",
            "x": 0.0,
            "y": 0.5,
            "z": 0.01,
            "q": q_down
        }
        # Add additional test poses here as needed
    ]

    logger.info("="*50)
    logger.info(" ALIGNMENT VERIFICATION SCRIPT STARTED ")
    logger.info(" Messure from the robot base to the TCP at each")
    logger.info(" position and compare with expected values.")
    logger.info("="*50 + "\n")


    trigger_node = TriggerNode()

    try:
        # Iterate through all defined test poses
        for i, pose_config in enumerate(test_poses):
            name = pose_config["name"]
            logger.info("\n" + "="*50)
            logger.info("="*50)
            logger.info(f"🟢 Test {i+1}: {name} 🟢")
            logger.info("="*50)
            
            # --- THE TRIGGER WAIT LOOP ---
            logger.info("="*50)
            logger.info("⏯  Waiting for manual trigger. Run in a separate terminal:")
            logger.info("ros2 topic pub --once /next_step std_msgs/msg/Empty")
            logger.info("="*50 + "\n")
            
            trigger_node.triggered = False # Reset the flag
            
            # Spin the node to check for messages until the flag becomes True
            while not trigger_node.triggered and rclpy.ok():
                rclpy.spin_once(trigger_node, timeout_sec=0.1)

            # 2. Prepare the target based on its type
            if pose_config["type"] == "named":
                target = pose_config["target"]
            else:
                target = PoseStamped()
                target.header.frame_id = "ur5e_base_link"
                target.pose.position.x = pose_config["x"]
                target.pose.position.y = pose_config["y"]
                target.pose.position.z = pose_config["z"]
                
                q = pose_config["q"]
                target.pose.orientation.x = q[0]
                target.pose.orientation.y = q[1]
                target.pose.orientation.z = q[2]
                target.pose.orientation.w = q[3]

            # 3. Plan and execute the motion
            success = plan_and_execute(ur5e, ur5e_arm, logger, target)
            
            if success:
                logger.info("="*50)
                logger.info(f"✅ Position {i+1} reached:" + "\n")
                logger.info(f"{name}" + "\n")
                logger.info("Messure from the robot base to the TCP at each")
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
        rclpy.shutdown()

if __name__ == '__main__':
    main()