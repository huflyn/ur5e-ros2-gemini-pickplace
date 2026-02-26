#!/usr/bin/env python3
"""
Verification script for alignment and calibration.
Moves the robot to defined test positions requiring user confirmation.
"""
import time
import math
import rclpy
from rclpy.logging import get_logger
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

    try:
        # Iterate through all defined test poses
        for i, pose_config in enumerate(test_poses):
            name = pose_config["name"]
            logger.info("="*50)
            logger.info(f"--- Test {i+1}: {name} ---")
            logger.info("="*50)
            
            # --- Countdown Timer ---
            wait_time = 10  # Seconds to wait for you to measure / clear the area
            logger.info(f"Waiting {wait_time} seconds before moving to the next pose...")
            
            for countdown in range(wait_time, 0, -1):
                logger.info(f"Starting in {countdown}...")
                time.sleep(1.0)

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
                logger.info("✅ Position reached. You can now take your measurements.")
                logger.info(" Messure from the robot base to the TCP at each")
                logger.info(" position and compare with expected values.")
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