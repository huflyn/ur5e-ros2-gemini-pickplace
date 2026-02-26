#!/usr/bin/env python3

import time
import copy
import math
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from tf_transformations import quaternion_from_euler

# Correct custom message package
from color_detection_msgs.msg import LegoBrick

# MoveIt 2 Python API
from moveit.planning import MoveItPy

class BrickSorterNode(Node):
    """Handles ROS 2 background subscriptions and publishers."""
    def __init__(self):
        super().__init__('brick_sorter_node')
        self.lego_brick_data = None
        self.accept_new_bricks = True # Flag to control whether we accept new brick data while processing one

        # --- Declare Parameters ---
        self.declare_parameter('hover_height', 0.30)
        self.declare_parameter('dropoff_height', 0.12)
        self.declare_parameter('grasp_z_offset', 0.0) # Optional: To ensure a secure grasp by going slightly below the detected Z

        self.declare_parameter('dropoff_yellow', [0.27, 0.558])
        self.declare_parameter('dropoff_red', [0.27, 0.438])
        self.declare_parameter('dropoff_green', [0.27, 0.318])
        self.declare_parameter('dropoff_blue', [0.27, 0.198])

        # --- Read Parameters ---
        self.hover_height = self.get_parameter('hover_height').value
        self.dropoff_height = self.get_parameter('dropoff_height').value
        self.grasp_z_offset = self.get_parameter('grasp_z_offset').value

        # Map them to a dictionary for easy access
        self.dropoffs = {
            'yellow': self.get_parameter('dropoff_yellow').value,
            'red': self.get_parameter('dropoff_red').value,
            'green': self.get_parameter('dropoff_green').value,
            'blue': self.get_parameter('dropoff_blue').value,
        }
        
        # Subscriber for brick coordinates
        self.subscription = self.create_subscription(
            LegoBrick,
            '/lego_brick_info',
            self.brick_callback,
            10
        )
        
        # Publisher for Webots Vacuum Gripper
        self.gripper_pub = self.create_publisher(
            Bool, 
            '/ur5e/vacuum_gripper/turn_on', 
            10
        )

    def brick_callback(self, msg):
        # Only accept a new brick if we are not currently processing one
        if self.accept_new_bricks and self.lego_brick_data is None:
            self.lego_brick_data = msg
            self.get_logger().info(f"New '{msg.color.data}' brick accepted at X={msg.position.point.x:.3f}, Y={msg.position.point.y:.3f}, Z={msg.position.point.z:.3f}")

    def set_gripper(self, turn_on: bool):
        """Activates or deactivates the Webots vacuum gripper via Topic."""
        msg = Bool()
        msg.data = turn_on
        self.gripper_pub.publish(msg)
        
        state_str = "ON (Suction)" if turn_on else "OFF (Release)"
        self.get_logger().info(f"Vacuum gripper set to: {state_str}")


def plan_and_execute(robot, arm, logger, target):
    """Plans and executes a trajectory to either a named pose or a PoseStamped."""
    arm.set_start_state_to_current_state()
    
    if isinstance(target, str):
        # Wenn der Target-Wert ein Text ist, nutze die SRDF-Pose
        arm.set_goal_state(configuration_name=target)
    else:
        # Wenn es eine Koordinate ist, nutze den TCP
        arm.set_goal_state(pose_stamped_msg=target, pose_link="pisoftgrip_tcp")
    
    plan_result = arm.plan()

    if plan_result:
        logger.info("Executing planned trajectory...")
        
        # 1. Wir speichern ab, ob die Bewegung ERFOLGREICH ausgeführt wurde
        success = robot.execute(plan_result.trajectory, controllers=[])
        
        # 2. Durchatmen! Wir geben den Webots-Gelenken kurz Zeit, 
        # sich physikalisch auszuschwingen und exakt zum Stehen zu kommen.
        time.sleep(0.5)
        
        # 3. Wir geben das echte Ergebnis zurück, nicht einfach immer "True"
        if not success:
            logger.error("Trajectory execution failed by the controller!")
            return False
            
        return True
    else:
        logger.error("Motion planning failed!")
        return False


def main(args=None):
    rclpy.init(args=args)
    logger = get_logger("brick_sorter_main")
    logger.info("Initializing MoveItPy and Perception Node...")

    # 1. Load MoveItPy and the robot arm component
    ur5e = MoveItPy(node_name="brick_sorter_moveit")
    ur5e_arm = ur5e.get_planning_component("ur_arm") # Make sure this matches your SRDF group name!

    # 2. Start the background node for callbacks
    brick_sorter_node = BrickSorterNode()
    executor = MultiThreadedExecutor()
    executor.add_node(brick_sorter_node)

    # 3. Gripper orientation: pointing straight down (from your old tf settings)
    # Using roll=0, pitch=0, yaw=pi as per your ROS 1 script
    q_down = quaternion_from_euler(math.pi, 0.0, 0.0)

    # --- Initialisierungs-Fahrt VOR der Schleife ---
    logger.info("Initializing: Moving to 'ready' pose before starting the detection cycle...")
    plan_and_execute(ur5e, ur5e_arm, logger, "ready")


    try:
        while rclpy.ok():
            # Spin the executor to check for new ROS messages
            executor.spin_once(timeout_sec=0.1)

            if brick_sorter_node.lego_brick_data is not None:

                brick_sorter_node.accept_new_bricks = False # Sets flag to ignore new bricks while processing the current one

                brick = brick_sorter_node.lego_brick_data
                color = brick.color.data
                
                logger.info(
                    "\n=================================================================\n"
                    f" Starting Pick & Place for: {color.upper()} BRICK\n"
                    "-----------------------------------------------------------------\n"
                    f" * Robot Target   : X = {brick.position.point.x:.3f}, Y = {brick.position.point.y:.3f}, Z = {brick.position.point.z:.3f}\n"
                    f" * Camera Distance: {brick.camera_distance_mm:.1f} mm\n"
                    "================================================================="
                )

                # Check if we have a drop-off location for this color
                if color not in brick_sorter_node.dropoffs:
                    logger.error(f"No drop-off location defined for color '{color}'. Skipping brick!")
                    brick_sorter_node.lego_brick_data = None
                    brick_sorter_node.accept_new_bricks = True
                    continue

                target_xy = brick_sorter_node.dropoffs[color]

                # 1. Hover pose above the brick
                pose_above = PoseStamped()
                pose_above.header.frame_id = "ur5e_base_link"
                pose_above.pose.orientation.x = q_down[0]
                pose_above.pose.orientation.y = q_down[1]
                pose_above.pose.orientation.z = q_down[2]
                pose_above.pose.orientation.w = q_down[3]
                
                pose_above.pose.position.x = brick.position.point.x
                pose_above.pose.position.y = brick.position.point.y
                pose_above.pose.position.z = brick_sorter_node.hover_height # Dynamic height!

                # 2. Grasp pose
                pose_grasp = copy.deepcopy(pose_above)
                #pose_grasp.pose.position.z = brick.position.point.z + brick_sorter_node.grasp_z_offset # Apply optional Z offset for better grasping
                pose_grasp.pose.position.z = 0.005 # static low Z to ensure a good grip, since the perception Z can be noisy. Adjust as needed based on your tests!

                # 3. Hover pose above the drop-off zone
                pose_drop_hover = copy.deepcopy(pose_above)
                pose_drop_hover.pose.position.x = target_xy[0] # Dynamic X from YAML
                pose_drop_hover.pose.position.y = target_xy[1] # Dynamic Y from YAML
                    
                # 4. Drop-off pose
                pose_drop = copy.deepcopy(pose_drop_hover)
                pose_drop.pose.position.z = brick_sorter_node.dropoff_height # Drop-off height from YAML

                # ---------------------------------------------------------
                # Execution Sequence (with Error Handling)
                # ---------------------------------------------------------
                try:
                    logger.info("Step 1: Moving above the brick")
                    if not plan_and_execute(ur5e, ur5e_arm, logger, pose_above):
                        logger.error("Failed to reach hover pose. Aborting cycle!")
                        plan_and_execute(ur5e, ur5e_arm, logger, "ready") # Zurück auf Start
                        continue # Schleife sofort abbrechen und von vorne beginnen
                    
                    logger.info(f"Step 2: Lowering to grasp position z={brick.position.point.z:.3f} with optional offset {brick_sorter_node.grasp_z_offset:.3f}")
                    if not plan_and_execute(ur5e, ur5e_arm, logger, pose_grasp):
                        logger.error("Failed to reach grasp pose. Aborting cycle!")
                        plan_and_execute(ur5e, ur5e_arm, logger, pose_above) # Zuerst hochziehen
                        plan_and_execute(ur5e, ur5e_arm, logger, "ready") # Dann zurück auf Start
                        continue
                    
                    logger.info("Step 3: Activating gripper")
                    brick_sorter_node.set_gripper(turn_on=True)
                    time.sleep(1.0) # Wait for vacuum to build up
                    
                    logger.info("Step 4: Lifting the brick")
                    if not plan_and_execute(ur5e, ur5e_arm, logger, pose_above):
                        logger.error("Failed to lift brick. Dropping and aborting!")
                        brick_sorter_node.set_gripper(turn_on=False) # Notabwurf!
                        plan_and_execute(ur5e, ur5e_arm, logger, "ready")
                        continue
                    
                    logger.info(f"Step 5: Moving to {color} drop-off zone")
                    if not plan_and_execute(ur5e, ur5e_arm, logger, pose_drop_hover):
                        logger.error("Failed to reach drop-off zone. Dropping and aborting!")
                        brick_sorter_node.set_gripper(turn_on=False)
                        plan_and_execute(ur5e, ur5e_arm, logger, "ready")
                        continue
                    
                    logger.info("Step 6: Lowering to drop-off height")
                    if not plan_and_execute(ur5e, ur5e_arm, logger, pose_drop):
                        logger.warn("Failed to lower completely. Dropping from current height.")
                        # Hier brechen wir NICHT ab, sondern lassen den Stein einfach fallen.
                    
                    logger.info("Step 7: Releasing gripper")
                    brick_sorter_node.set_gripper(turn_on=False)
                    time.sleep(1.0) # Wait for vacuum to release
                    
                    logger.info("Step 8: Retreating upwards")
                    plan_and_execute(ur5e, ur5e_arm, logger, pose_drop_hover)

                    logger.info("Step 9: Returning to 'ready' pose to clear the camera view")
                    plan_and_execute(ur5e, ur5e_arm, logger, "ready")

                finally:
                    logger.info("Cycle complete or aborted. Flushing stale camera data...")
                    
                    # 1. Kurz warten, damit die Kamera den nun leeren Tisch registriert
                    time.sleep(0.5)
                    
                    # 2. Alle alten "Geister-Nachrichten" aus der ROS Queue saugen und wegwerfen
                    for _ in range(15):
                        executor.spin_once(timeout_sec=0.01)
                    
                    # 3. System wieder scharfschalten für den nächsten sauberen Durchlauf
                    brick_sorter_node.lego_brick_data = None
                    brick_sorter_node.accept_new_bricks = True
                    logger.info("Waiting for the next target...")

    except KeyboardInterrupt:
        logger.info("Application manually stopped.")
    finally:
        brick_sorter_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()