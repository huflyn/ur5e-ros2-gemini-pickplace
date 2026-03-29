#!/usr/bin/env python3
"""
workcell_application/pick_and_place.py

Gemini-based Pick-and-Place Orchestrator.
Architecture mirrors the proven brick_sorter.py patterns:
  - MoveItPy for motion planning
  - Pilz LIN for linear Cartesian moves
  - OMPL for free-space transit
  - Topic-based gripper (Webots) / UR I/O (real HW)

Detection is triggered on-demand via /detect_objects service
instead of continuous topic subscription.
"""

import time
import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Empty, String
from tf_transformations import quaternion_from_euler

from object_interfaces.msg import DetectedObject
from object_interfaces.srv import DetectObjects

from moveit.planning import MoveItPy, PlanRequestParameters

from ur_msgs.srv import SetIO


class PickAndPlaceNode(Node):

    def __init__(self):
        super().__init__('pick_and_place_node')

        # Robotiq EPick with URCap
        self.script_pub = self.create_publisher(String, '/urscript_interface/script_command', 10)

        # --- Safe Workspace Boundaries ---
        # Declare the enable flag (defaults to False if missing)
        self.declare_parameter('enable_workspace_safety', False)
        self.safety_enabled = self.get_parameter('enable_workspace_safety').value

        # Only load and calculate bounds if safety is actually enabled
        if self.safety_enabled:
            self.declare_parameter('workspace_min_x', 0.0)
            self.declare_parameter('workspace_max_x', 0.0)
            self.declare_parameter('workspace_min_y', 0.0)
            self.declare_parameter('workspace_max_y', 0.0)
            self.declare_parameter('workspace_safety_tolerance', 0.0)

            ws_min_x = self.get_parameter('workspace_min_x').value
            ws_max_x = self.get_parameter('workspace_max_x').value
            ws_min_y = self.get_parameter('workspace_min_y').value
            ws_max_y = self.get_parameter('workspace_max_y').value
            tolerance = self.get_parameter('workspace_safety_tolerance').value

            self.safe_min_x = ws_min_x - tolerance
            self.safe_max_x = ws_max_x + tolerance
            self.safe_min_y = ws_min_y - tolerance
            self.safe_max_y = ws_max_y + tolerance
            
            self.workspace_bounds = f"X: [{self.safe_min_x:.2f}, {self.safe_max_x:.2f}], Y: [{self.safe_min_y:.2f}, {self.safe_max_y:.2f}]"
            self.workspace_safety_str = f"✅ Workspace safety boundaries are ENABLED: {self.workspace_bounds}"
        else:
            self.workspace_safety_str = "⚠️  Workspace safety boundaries are DISABLED."
        self.get_logger().info(self.workspace_safety_str)

        # --- Declare Parameters ---
        self.declare_parameter('base_frame', 'ur5e_base_link')
        self.declare_parameter('tcp_link', 'pisoftgrip_tcp')
        self.declare_parameter('planning_group', 'ur_arm')

        self.declare_parameter('hover_height', 0.25)
        self.declare_parameter('grasp_height', 0.01)
        self.declare_parameter('dropoff_height', 0.10)

        self.declare_parameter('use_sim_gripper', False)
        self.declare_parameter('gripper_topic', '/ur5e/vacuum_gripper/turn_on')

        self.declare_parameter('object_center_offset', 0.0) # Offset from front face to center of object in meters (for 3D grasping), adjust in pick_and_place_parameters.yaml if needed

        self.declare_parameter('dropoff_default', [0.27, 0.250]) # adjust in pick_and_place_parameters.yaml if needed
        self.declare_parameter('dropoff_red',    [0.27, 0.450])
        self.declare_parameter('dropoff_blue',   [0.27, 0.350])
        self.declare_parameter('dropoff_green',  [-0.27, 0.450])
        self.declare_parameter('dropoff_yellow', [-0.27, 0.350])

        # --- Read Parameters ---
        self.base_frame = self.get_parameter('base_frame').value
        self.tcp_link = self.get_parameter('tcp_link').value
        self.planning_group = self.get_parameter('planning_group').value

        self.hover_height = self.get_parameter('hover_height').value
        self.grasp_height = self.get_parameter('grasp_height').value
        self.dropoff_height = self.get_parameter('dropoff_height').value

        self.use_sim_gripper = self.get_parameter('use_sim_gripper').value
        gripper_topic = self.get_parameter('gripper_topic').value

        self.object_center_offset = self.get_parameter('object_center_offset').value

        self.dropoffs = {
            'default': self.get_parameter('dropoff_default').value,
            'red':     self.get_parameter('dropoff_red').value,
            'blue':    self.get_parameter('dropoff_blue').value,
            'green':   self.get_parameter('dropoff_green').value,
            'yellow':  self.get_parameter('dropoff_yellow').value,
        }

        # --- Gripper Setup ---
        if self.use_sim_gripper:
            self.gripper_pub = self.create_publisher(Bool, gripper_topic, 10)
            self.gripper_status_str = f"Webots vacuum via {gripper_topic}"
        else:
            # Real hardware: UR I/O Service (digital_out[0])
            self.io_client = self.create_client(SetIO, '/io_and_status_controller/set_io')
            self.gripper_status_str = "Real hardware mode (UR I/O pin 0 for Robotiq EPick)"

        # --- Vision Service Client ---
        self.detect_client = self.create_client(DetectObjects, '/detect_objects')

        # --- Trigger Subscriber ---
        self.start_triggered = False
        self.stop_triggered = False
        self.current_prompt = "" 
        
        self.trigger_sub = self.create_subscription(String, '/pick_and_place/scan', self.trigger_callback, 10)
        self.stop_sub = self.create_subscription(Empty, '/pick_and_place/stop', self.stop_callback, 10)

        # --- Initialized message with all parameters for easy debugging ──
        self.get_logger().info("PickAndPlaceNode initialized.")
        self.get_logger().info(f"Base frame:  {self.base_frame}")
        self.get_logger().info(f"TCP link:    {self.tcp_link}")
        self.get_logger().info(f"Drop-offs:   {list(self.dropoffs.keys())}")
        self.get_logger().info(self.workspace_safety_str)

    # --- Workspace Safety Check Function ---
    def is_target_safe(self, x, y):
        """
        Validates coordinates. If safety is disabled, always returns True.
        """
        if not self.safety_enabled:
            return True
            
        if (self.safe_min_x <= x <= self.safe_max_x) and (self.safe_min_y <= y <= self.safe_max_y):
            return True
        else:
            self.get_logger().error(
                f"🛑 Safety Abort! Target ({x:.3f}, {y:.3f}) is outside the safe workspace bounds."
            )
            return False

    # --- Gripper Control ---
    def set_gripper(self, activate: bool):
        """Activate (True) or deactivate (False) the gripper."""
        if self.use_sim_gripper:
            msg = Bool()
            msg.data = activate
            self.gripper_pub.publish(msg)
        else:
            # Real hardware I/O service
            if not self.io_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("🛑 UR I/O service not available!")
                return

            req = SetIO.Request()
            req.fun = 1  # 1 = Standard Digital Output
            req.pin = 0  # digital_out[0]
            
            if activate:
                req.state = 1.0
                self.io_client.call_async(req)
                # Wait for vacuum to build up before moving the arm
                #time.sleep(0.5) 
            else:
                req.state = 0.0
                self.io_client.call_async(req)
                # Wait for the item to drop
                #time.sleep(0.5)

        action = "GRIP (suction ON)" if activate else "RELEASE (suction OFF)"
        self.get_logger().info(f"Gripper: {action}")

    # --- Vision Service ---
    def detect_objects(self, executor, prompt: str) -> list:
        """
        Calls /detect_objects service synchronously.
        """
        if not self.detect_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("🛑 /detect_objects service not available!")
            return []

        self.get_logger().info("🟢 Calling /detect_objects service...")

        # Include the current prompt in the service request for Gemini to use in its response
        req = DetectObjects.Request()
        req.user_prompt = prompt
        future = self.detect_client.call_async(req)

        while rclpy.ok() and not future.done():
            time.sleep(0.1)

        result = future.result()
        if result is None:
            self.get_logger().error("🛑 Service call returned None!")
            return []

        if not result.success:
            self.get_logger().error(f"🛑 Detection failed: {result.error_message}")
            return []

        return list(result.bricks)

    # --- Trigger Callback ---
    def trigger_callback(self, msg):
        self.current_prompt = msg.data # Store the latest prompt for use in custom Gemini instructions
        if self.current_prompt:
            self.get_logger().info(f"▶️  Scan trigger with custom prompt received!\nPrompt: '{self.current_prompt}'")
        else:
            self.get_logger().info("▶️  Scan trigger received! Waking up from standby...")
        self.start_triggered = True
    
    def stop_callback(self, msg):
        self.get_logger().info("🛑 SOFT STOP trigger received! Will abort batch.")
        self.stop_triggered = True


# --------------------------------------------------------------
# Motion Planning Functions (identical patterns to brick_sorter)
# --------------------------------------------------------------

def plan_and_execute_ompl(robot, arm, logger, target, tcp_link="pisoftgrip_tcp"):
    """
    Plans and executes a trajectory using OMPL (free-space motion).
    Target can be a named pose (str) or a PoseStamped.
    """
    arm.set_start_state_to_current_state()

    if isinstance(target, str):
        arm.set_goal_state(configuration_name=target)
    else:
        arm.set_goal_state(pose_stamped_msg=target, pose_link=tcp_link)

    plan_params = PlanRequestParameters(robot, "ompl_rrtc")

    plan_result = arm.plan(single_plan_parameters=plan_params)

    if plan_result:
        logger.info("🟢 Executing OMPL trajectory...")
        success = robot.execute(plan_result.trajectory, controllers=[])
        time.sleep(0.1)
        if not success:
            logger.error("🛑 Trajectory execution failed!")
        return success
    else:
        logger.error("🛑 OMPL planning failed!")
        return False

def plan_and_execute_travel(robot, arm, logger, target, tcp_link="pisoftgrip_tcp"):
    """
    Plans and executes a free-space motion to the target pose, trying Pilz PTP first and falling back to OMPL if it fails.
    """
    # Try 1: Pilz PTP for direct point-to-point motion (ideal for most cases)
    try:
        arm.set_start_state_to_current_state()

        if isinstance(target, str):
            arm.set_goal_state(configuration_name=target)
        else:
            arm.set_goal_state(pose_stamped_msg=target, pose_link=tcp_link)

        plan_params = PlanRequestParameters(robot, "pilz_ptp")
        plan_result = arm.plan(single_plan_parameters=plan_params)

        if plan_result:
            logger.info("🟢 Executing Pilz PTP trajectory...")
            success = robot.execute(plan_result.trajectory, controllers=[])
            time.sleep(0.1)
            return success
    except Exception as e:
        logger.warn(f"🛑 Pilz PTP exception: {e}")

    logger.warn("🛑 Pilz PTP failed → trying OMPL fallback...")

    # Try 2: OMPL free-space as fallback if Pilz fails (e.g. due to collision in tight spaces)
    try:
        arm.set_start_state_to_current_state()
        if isinstance(target, str):
            arm.set_goal_state(configuration_name=target)
        else:
            arm.set_goal_state(pose_stamped_msg=target, pose_link=tcp_link)

        plan_params = PlanRequestParameters(robot, "ompl_rrtc")
        plan_result = arm.plan(single_plan_parameters=plan_params)

        if plan_result:
            logger.info("🟢 Executing OMPL trajectory...")
            success = robot.execute(plan_result.trajectory, controllers=[])
            time.sleep(0.1)
            return success
    except Exception as e:
        logger.warn(f"🛑 OMPL exception: {e}")

    logger.warn("🛑 Planning failed...")
    return False


def plan_and_execute_pilz(robot, arm, logger, target, tcp_link="pisoftgrip_tcp"):
    """
    Plans and executes a linear Cartesian path using Pilz LIN planner.
    Used for precise vertical and horizontal moves near objects.
    """
    # Try 1: Pilz LIN for Cartesian straight-line motion (ideal for approach and descent)
    try:
        arm.set_start_state_to_current_state()

        if isinstance(target, str):
            arm.set_goal_state(configuration_name=target)
        else:
            arm.set_goal_state(pose_stamped_msg=target, pose_link=tcp_link)

        plan_params = PlanRequestParameters(robot, "pilz_lin")
        plan_result = arm.plan(single_plan_parameters=plan_params)

        if plan_result:
            logger.info("🟢 Executing Pilz LIN trajectory...")
            success = robot.execute(plan_result.trajectory, controllers=[])
            time.sleep(0.1)
            return success
    except Exception as e:
        logger.warn(f"🛑 Pilz LIN exception: {e}")

    logger.warn("🛑 Pilz LIN failed → trying Pilz PTP fallback...")

    # Try 2: Pilz PTP as fallback if LIN fails (e.g. due to singularities or unreachable poses)
    try:
        arm.set_start_state_to_current_state()

        if isinstance(target, str):
            arm.set_goal_state(configuration_name=target)
        else:
            arm.set_goal_state(pose_stamped_msg=target, pose_link=tcp_link)

        plan_params = PlanRequestParameters(robot, "pilz_ptp")
        plan_result = arm.plan(single_plan_parameters=plan_params)

        if plan_result:
            logger.info("🟢 Executing Pilz PTP trajectory...")
            success = robot.execute(plan_result.trajectory, controllers=[])
            time.sleep(0.1)
            return success
    except Exception as e:
        logger.warn(f"🛑Pilz PTP exception: {e}")

    logger.warn("🛑 Pilz PTP failed...")
    return False


# -----------------------------------------------------------------------------------
# Main Pick and Place operation: Trigger → Scan → Loop: Brick → Pick → Place → Repeat
# -----------------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    logger = get_logger("pick_and_place")

    # --- Initialize MoveItPy ---
    logger.info("🟢 Initializing MoveItPy...")
    ur5e = MoveItPy(node_name="pick_place_moveit")

    # --- Start communication node ---
    node = PickAndPlaceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # --- Background thread for ROS callbacks ---
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    ur5e_arm = ur5e.get_planning_component(node.planning_group)
    tcp_link = node.tcp_link

    # --- Helper: Create a PoseStamped with downward orientation ---
    def make_pose(x, y, z, yaw=0.0):
        pose = PoseStamped()
        pose.header.frame_id = node.base_frame

        # Pitch offset to ensure the gripper faces downwards, can be adjusted if needed
        pitch_offset = math.radians(0.0)
        q_down = quaternion_from_euler(math.pi + pitch_offset, 0.0, yaw)

        pose.pose.orientation.x = q_down[0]
        pose.pose.orientation.y = q_down[1]
        pose.pose.orientation.z = q_down[2]
        pose.pose.orientation.w = q_down[3]
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        return pose

    # --- Initial homing ---
    logger.info("🟢 Initializing: Gripper OFF, moving to 'ready' pose")
    node.set_gripper(False)
    time.sleep(1.0)
    if not plan_and_execute_travel(ur5e, ur5e_arm, logger, "ready", tcp_link):
        logger.warn("⚠️ Homing failed. Robot might already be in 'ready' pose. Proceeding to standby...")

    try:
        while rclpy.ok():
            # -----------------------------
            # MANUAL TRIGGER (Standby Loop)
            # -----------------------------
            logger.info(
                "Status:\n" +
                "="*60 + "\n" +
                "🟢 PickAndPlaceNode (Client Node) ready. 🟢\n" +
                f"🤜 Gripper: {node.gripper_status_str}\n" +
                f"{node.workspace_safety_str}\n" +
                "⏸️  STANDBY: Waiting for SCAN trigger.\n\n" +
                "🅰️  Default Mode - 'Detects all bricks on the table':\n" +
                "   ros2 topic pub --once /pick_and_place/scan std_msgs/msg/String \"{data: ''}\"\n\n" +
                "🅱️  User Prompt Mode - add your instructions to the data field:\n" +
                "   ros2 topic pub --once /pick_and_place/scan std_msgs/msg/String \"{data: 'Pick the red object and place it somewhere safe.'}\"\n\n" +
                "⏹️  To manually STOP the Pick and Place process and return to STANDBY:\n" +
                "   ros2 topic pub --once /pick_and_place/stop std_msgs/msg/Empty\n" +
                "="*60
            )

            # Reset trigger
            node.start_triggered = False
            node.stop_triggered = False
            
            # wait for trigger to start the pick-and-place cycle
            while rclpy.ok() and not node.start_triggered:
                time.sleep(0.1)

            if not rclpy.ok():
                break # if shutdown signal received while waiting for trigger, exit main loop

            # --- PHASE 0: SCAN ---
            logger.info("=" * 60)
            logger.info("🟢 PHASE 0: Scanning table via perception service...")
            logger.info("=" * 60)

            # Call the vision service to detect bricks, passing the current prompt for Gemini to use in its response
            bricks = node.detect_objects(executor, node.current_prompt)
            
            # <--- NEU: Die magische Task-Planner Weiche
            if node.current_prompt:
                logger.info("🅱️  User Prompt Mode active. Using Gemini's selection and sequence.")
                bricks_sorted = bricks # Gemini is responsible for both selecting which bricks to pick and the order in which to pick them, based on the custom prompt provided by the user.
            else:
                logger.info("🅰️  Default Mode active. Sorting all visible bricks.")
                bricks_sorted = sorted(bricks, key=lambda b: b.camera_distance_mm)
            
            logger.info(f"🟢 Detected {len(bricks_sorted)} object(s). Starting Batch Processing!")

            # ---------------------
            # BATCH PROCESSING LOOP
            # ---------------------

            # Helper function to check for soft stop trigger at multiple points in the cycle
            def check_abort():
                if node.stop_triggered:
                    raise RuntimeError("🛑 STOP trigger received by operator!")

            for i, object in enumerate(bricks_sorted):

                # check for soft stop at the start of each cycle before processing the next object
                if not rclpy.ok() or node.stop_triggered:
                    logger.warn("⚠️  BATCH ABORTED BY OPERATOR! Skipping remaining bricks.")
                    break

                color = object.color.data
                bx = object.position.point.x
                by = object.position.point.y + node.object_center_offset
                bz = object.position.point.z
                yaw = math.radians(object.yaw_degrees)

                logger.info("-" * 60)
                logger.info(f"BATCH {i+1}/{len(bricks_sorted)} | TARGET: {color.upper()} object")
                logger.info(f"Position : X={bx:.3f}, Y={by:.3f}, Z={bz:.3f}")
                logger.info("-" * 60)

                # Determine drop-off location
                if object.has_user_dropoff:
                    # User drop-off provided by Gemini's vision analysis
                    target_xy = [object.user_dropoff_position.x, object.user_dropoff_position.y]
                    logger.info(f"USER-DEFINED Drop-off for '{color}': X={target_xy[0]:.3f}, Y={target_xy[1]:.3f}")
                elif color in node.dropoffs:
                    # Fallback to default color-coded drop-offs defined in parameters
                    target_xy = node.dropoffs[color]
                    logger.info(f"Default Drop-off for '{color}'.")
                else:
                    logger.warn(f"No drop-off for '{color}', using 'default'.")
                    target_xy = node.dropoffs['default']

                # --- WORKSPACE SAFETY SANITY CHECK ---
                logger.info("🛡️  Performing safety sanity check on coordinates...")

                # 1. Check Pick Coordinates
                is_pick_safe = node.is_target_safe(bx, by)
                if not is_pick_safe:
                    logger.warn(f"⏭️  Skipping {color} object due to unsafe PICK coordinates: X={bx:.3f}, Y={by:.3f}")
                    continue  # Skip and move to the next object
                
                # 2. Check Drop-off Coordinates (only if Pick was safe)
                is_drop_safe = node.is_target_safe(target_xy[0], target_xy[1])
                if not is_drop_safe:
                    logger.warn(f"⏭️  Skipping {color} object due to unsafe DROP-OFF coordinates: X={target_xy[0]:.3f}, Y={target_xy[1]:.3f}")
                    continue  # Skip and move to the next object

                # --- DEFINE ALL POSES FOR THIS CYCLE ---
                pose_hover_pick_straight = make_pose(bx, by, node.hover_height, 0.0)
                pose_hover_pick_oriented = make_pose(bx, by, node.hover_height, yaw)
                pose_grasp               = make_pose(bx, by, node.grasp_height, yaw)
                pose_hover_drop          = make_pose(target_xy[0], target_xy[1], node.hover_height, 0.0)
                pose_drop                = make_pose(target_xy[0], target_xy[1], node.dropoff_height, 0.0)

                escape_pose = None

                # -----------------------------------------------------------
                # MAIN PICK AND PLACE SEQUENCE WITH ERROR HANDLING & RECOVERY
                # -----------------------------------------------------------
                try:
                    check_abort()

                    # --- PHASE 1: APPROACH + ORIENT (OMPL) ---
                    # OMPL plans free-space motion including yaw rotation
                    # Rotation happens safely at hover height, not near the table
                    logger.info("🟢 PHASE 1: Approach + orient (OMPL)")
                    if not plan_and_execute_pilz(ur5e, ur5e_arm, logger, pose_hover_pick_oriented, tcp_link):
                        logger.warn("🛑 Pilz failed, trying OMPL fallback...")
                        if not plan_and_execute_ompl(ur5e, ur5e_arm, logger, pose_grasp, tcp_link):
                            raise RuntimeError("🛑 Failed to reach oriented hover pose")

                    check_abort()
                    escape_pose = pose_hover_pick_straight

                    # --- PHASE 2: DESCEND (Pilz) ---
                    # Pure vertical descent, yaw is already set
                    logger.info("🟢 PHASE 2: Descend to grasp (Pilz)")
                    if not plan_and_execute_pilz(ur5e, ur5e_arm, logger, pose_grasp, tcp_link):
                        logger.warn("🛑 Pilz failed, trying OMPL fallback...")
                        if not plan_and_execute_ompl(ur5e, ur5e_arm, logger, pose_grasp, tcp_link):
                            raise RuntimeError("🛑 Failed to descend to grasp")

                    check_abort()

                    # --- PHASE 3: GRASP ---
                    logger.info("🟢 PHASE 3: Grasping")
                    node.set_gripper(True)
                    time.sleep(0.5) # Ensure gripper has time to activate before lifting the arm

                    check_abort()

                    # --- PHASE 4: LIFT (Pilz) ---
                    # Straight up, keeping yaw to avoid rotating with object near table
                    logger.info("🟢 PHASE 4: Lift (Pilz)")
                    if not plan_and_execute_pilz(ur5e, ur5e_arm, logger, pose_hover_pick_oriented, tcp_link):
                        logger.warn("🛑 Pilz failed, trying OMPL fallback...")
                        if not plan_and_execute_ompl(ur5e, ur5e_arm, logger, pose_hover_pick_oriented, tcp_link):
                            raise RuntimeError("🛑 Failed to lift object")

                    check_abort()

                    # --- PHASE 5: TRANSPORT ---
                    logger.info(f"🟢 PHASE 5: Transport to {color} drop-off (OMPL)")
                    if not plan_and_execute_pilz(ur5e, ur5e_arm, logger, pose_hover_drop, tcp_link):
                        logger.warn("🛑 Pilz failed, trying OMPL fallback...")
                        if not plan_and_execute_ompl(ur5e, ur5e_arm, logger, pose_hover_drop, tcp_link):
                            raise RuntimeError("🛑 Failed to reach drop-off zone")

                    check_abort()
                    escape_pose = pose_hover_drop

                    # --- PHASE 6: LOWER (Pilz) ---
                    logger.info("🟢 PHASE 6: Lower to drop-off (Pilz)")
                    if not plan_and_execute_pilz(ur5e, ur5e_arm, logger, pose_drop, tcp_link):
                        logger.warn("🛑 Pilz failed, trying OMPL fallback...")
                        if not plan_and_execute_ompl(ur5e, ur5e_arm, logger, pose_drop, tcp_link):
                            logger.warn("🛑 Failed to lower completely. Releasing from current height.")

                    check_abort()

                    # ─--- PHASE 7: RELEASE ---
                    logger.info("🟢 PHASE 7: Release")
                    node.set_gripper(False)
                    time.sleep(0.5) # Ensure gripper has time to release before moving the arm

                    check_abort()

                    # --- PHASE 8: RETREAT (Pilz) ---
                    logger.info("🟢 PHASE 8: Retreat (Pilz)")
                    if not plan_and_execute_pilz(ur5e, ur5e_arm, logger, pose_hover_drop, tcp_link):
                        logger.warn("🛑 Pilz failed, trying OMPL fallback...")
                        if not plan_and_execute_ompl(ur5e, ur5e_arm, logger, pose_hover_drop, tcp_link):
                            logger.warn("🛑 Retreat failed.")

                    escape_pose = None

                except RuntimeError as e:
                    # --- ERROR RECOVERY & SOFT STOP ---
                    logger.info(
                        "Status:"
                        "\n" + "="*60 + "\n" +
                        f"🛑 CYCLE ABORTED: {e}\n" +
                        "Releasing gripper and skipping to next object in batch...\n" +
                        "="*60
                    )

                    node.set_gripper(False)
                    time.sleep(0.5)

                    # 1. Retract: only attempt to retreat if we had a defined escape pose (we were in the middle of the cycle)
                    if escape_pose is not None:
                        logger.info("Executing vertical retract & untwist to safe hover height...")
                        plan_and_execute_pilz(ur5e, ur5e_arm, logger, escape_pose, tcp_link)
                    else:
                        logger.info("Arm is already safe. Skipping retract.")
                        
                    # 2. Return to ready pose before next cycle
                    logger.info("Returning to ready pose...")
                    plan_and_execute_travel(ur5e, ur5e_arm, logger, "ready", tcp_link)
                    
                    # 3. Break out of the batch processing loop to return to standby and wait for next trigger
                    break
            
            # --- End of batch loop ---

            logger.info(
                "Status:"
                "\n" + "="*60 + "\n" +
                "🟢🏁 BATCH COMPLETE: Returning to ready pose.\n" +
                "="*60
            )


            plan_and_execute_travel(ur5e, ur5e_arm, logger, "ready", tcp_link)

    except KeyboardInterrupt:
        logger.info("Application stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()