#!/usr/bin/env python3
"""
workcell_application/pick_and_place.py

Gemini-based Pick-and-Place Orchestrator.
Architecture mirrors the proven brick_sorter.py patterns:
  - MoveItPy for motion planning
  - Pilz LIN for linear Cartesian moves
  - OMPL for free-space transit
  - Topic-based gripper (Webots) / UR I/O (real HW)

Detection is triggered on-demand via /detect_bricks service
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
from std_msgs.msg import Bool, Empty
from tf_transformations import quaternion_from_euler

from brick_interfaces.msg import LegoBrick
from brick_interfaces.srv import DetectBricks

from moveit.planning import MoveItPy, PlanRequestParameters


# ═══════════════════════════════════════════════════════════════
#  ROS 2 Node: Handles communication (services, gripper, params)
# ═══════════════════════════════════════════════════════════════

class PickAndPlaceNode(Node):
    """
    Lightweight ROS 2 node for communication.
    MoveItPy runs on its own internal node, so we keep this separate
    and spin it via MultiThreadedExecutor (same pattern as brick_sorter.py).
    """

    def __init__(self):
        super().__init__('pick_and_place_node')

        # ── Parameters ─────────────────────────────────────────
        self.declare_parameter('base_frame', 'ur5e_base_link')
        self.declare_parameter('tcp_link', 'pisoftgrip_tcp')
        self.declare_parameter('planning_group', 'ur_arm')

        self.declare_parameter('hover_height', 0.25)
        self.declare_parameter('grasp_height', 0.01)
        self.declare_parameter('dropoff_height', 0.10)

        self.declare_parameter('use_sim_gripper', True)
        self.declare_parameter('gripper_topic', '/ur5e/vacuum_gripper/turn_on')

        self.declare_parameter('dropoff_default', [0.27, 0.250])
        self.declare_parameter('dropoff_red',    [0.27, 0.450])
        self.declare_parameter('dropoff_blue',   [0.27, 0.350])
        self.declare_parameter('dropoff_green',  [-0.27, 0.450])
        self.declare_parameter('dropoff_yellow', [-0.27, 0.350])

        # ── Read Parameters ────────────────────────────────────
        self.base_frame = self.get_parameter('base_frame').value
        self.tcp_link = self.get_parameter('tcp_link').value
        self.planning_group = self.get_parameter('planning_group').value

        self.hover_height = self.get_parameter('hover_height').value
        self.grasp_height = self.get_parameter('grasp_height').value
        self.dropoff_height = self.get_parameter('dropoff_height').value

        self.use_sim_gripper = self.get_parameter('use_sim_gripper').value
        gripper_topic = self.get_parameter('gripper_topic').value

        self.dropoffs = {
            'default': self.get_parameter('dropoff_default').value,
            'red':     self.get_parameter('dropoff_red').value,
            'blue':    self.get_parameter('dropoff_blue').value,
            'green':   self.get_parameter('dropoff_green').value,
            'yellow':  self.get_parameter('dropoff_yellow').value,
        }

        # ── Gripper Setup ──────────────────────────────────────
        if self.use_sim_gripper:
            self.gripper_pub = self.create_publisher(Bool, gripper_topic, 10)
            self.get_logger().info(f"Gripper: Webots vacuum via {gripper_topic}")
        else:
            # Real hardware: placeholder for UR I/O service client
            self.get_logger().info("Gripper: Real hardware mode (UR I/O)")
            # TODO: self.io_client = self.create_client(SetIO, '/io_and_status_controller/set_io')

        # ── Vision Service Client ──────────────────────────────
        self.detect_client = self.create_client(DetectBricks, '/detect_bricks')

        # ── Trigger Subscriber ────────────────────────────
        self.start_triggered = False
        self.stop_triggered = False

        self.trigger_sub = self.create_subscription(Empty, '/pick_and_place/scan', self.trigger_callback, 10)
        self.stop_sub = self.create_subscription(Empty, '/pick_and_place/stop', self.stop_callback, 10)

        # ── Initialized message with all parameters for easy debugging ──
        self.get_logger().info("PickAndPlaceNode initialized.")
        self.get_logger().info(f"  Base frame:  {self.base_frame}")
        self.get_logger().info(f"  TCP link:    {self.tcp_link}")
        self.get_logger().info(f"  Drop-offs:   {list(self.dropoffs.keys())}")

    # ── Gripper Control ────────────────────────────────────────

    def set_gripper(self, activate: bool):
        """Activate (True) or deactivate (False) the gripper."""
        if self.use_sim_gripper:
            msg = Bool()
            msg.data = activate
            self.gripper_pub.publish(msg)
        else:
            # TODO: Implement real hardware gripper
            # io_req = SetIO.Request()
            # io_req.fun = 1; io_req.pin = 0
            # io_req.state = 1.0 if activate else 0.0
            # self.io_client.call_async(io_req)
            pass

        action = "GRIP (suction ON)" if activate else "RELEASE (suction OFF)"
        self.get_logger().info(f"Gripper: {action}")

    # ── Vision Service ─────────────────────────────────────────

    def detect_bricks(self, executor) -> list:
        """
        Calls /detect_bricks service synchronously.
        Spins the executor while waiting for the response.
        Returns a list of LegoBrick messages.
        """
        if not self.detect_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/detect_bricks service not available!")
            return []

        self.get_logger().info("Calling /detect_bricks service...")
        future = self.detect_client.call_async(DetectBricks.Request())

        while rclpy.ok() and not future.done():
            time.sleep(0.1)

        result = future.result()
        if result is None:
            self.get_logger().error("Service call returned None!")
            return []

        if not result.success:
            self.get_logger().error(f"Detection failed: {result.error_message}")
            return []

        return list(result.bricks)

    # ── Trigger Callback ──────────────────────────────────
    def trigger_callback(self, msg):
        self.get_logger().info("▶️  Scan trigger received! Waking up from standby...")
        self.start_triggered = True
    
    def stop_callback(self, msg):
        self.get_logger().info("🛑 SOFT STOP trigger received! Will abort batch.")
        self.stop_triggered = True


# ═══════════════════════════════════════════════════════════════
#  Motion Planning Functions (identical patterns to brick_sorter)
# ═══════════════════════════════════════════════════════════════

def plan_and_execute(robot, arm, logger, target, tcp_link="pisoftgrip_tcp"):
    """
    Plans and executes a trajectory using OMPL (free-space motion).
    Target can be a named pose (str) or a PoseStamped.
    """
    arm.set_start_state_to_current_state()

    if isinstance(target, str):
        arm.set_goal_state(configuration_name=target)
    else:
        arm.set_goal_state(pose_stamped_msg=target, pose_link=tcp_link)

    plan_result = arm.plan()

    if plan_result:
        logger.info("  Executing OMPL trajectory...")
        success = robot.execute(plan_result.trajectory, controllers=[])
        time.sleep(0.1)
        if not success:
            logger.error("  Trajectory execution failed!")
        return success
    else:
        logger.error("  OMPL planning failed!")
        return False


def plan_and_execute_cartesian(robot, arm, logger, target_pose, tcp_link="pisoftgrip_tcp"):
    """
    Plans and executes a linear Cartesian path using Pilz LIN planner.
    Used for precise vertical and horizontal moves near objects.
    """
    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=target_pose, pose_link=tcp_link)

    plan_params = PlanRequestParameters(robot, "pilz_lin")
    plan_params.planning_pipeline = "pilz_industrial_motion_planner"
    plan_params.planner_id = "LIN"

    plan_result = arm.plan(single_plan_parameters=plan_params)

    if plan_result:
        logger.info("  Executing Pilz LIN trajectory...")
        success = robot.execute(plan_result.trajectory, controllers=[])
        time.sleep(0.1)
        return success
    else:
        logger.error("  Pilz LIN planning failed! (singularity or unreachable?)")
        return False


# ═══════════════════════════════════════════════════════════════
#  Main Loop: Scan → Select → Pick → Place → Repeat
# ═══════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    logger = get_logger("pick_and_place")

    # ── 1. Initialize MoveItPy ─────────────────────────────────
    logger.info("Initializing MoveItPy...")
    ur5e = MoveItPy(node_name="pick_place_moveit")

    # ── 2. Start communication node ───────────────────────────
    node = PickAndPlaceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # ── Background thread for ROS callbacks
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    ur5e_arm = ur5e.get_planning_component(node.planning_group)
    tcp_link = node.tcp_link

    # ── 3. Helper: Create a PoseStamped with downward orientation ──
    def make_pose(x, y, z, yaw=0.0):
        pose = PoseStamped()
        pose.header.frame_id = node.base_frame
        q_down = quaternion_from_euler(math.pi, 0.0, yaw) # Gripper orientation: TCP pointing down + optional yaw
        pose.pose.orientation.x = q_down[0]
        pose.pose.orientation.y = q_down[1]
        pose.pose.orientation.z = q_down[2]
        pose.pose.orientation.w = q_down[3]
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        return pose

    # ── 5. Initial homing ──────────────────────────────────────
    logger.info("Initializing: Gripper OFF, moving to 'ready' pose")
    node.set_gripper(False)
    time.sleep(1.0)
    plan_and_execute(ur5e, ur5e_arm, logger, "ready", tcp_link)

    try:
        while rclpy.ok():
            # ─────────────────────────────────────────────────
            #  MANUELLER TRIGGER (Standby Loop)
            # ─────────────────────────────────────────────────
            logger.info(
                "Status:\n" +
                "="*60 + "\n" +
                "🟢 PickAndPlaceNode (Client Node) ready.\n" +
                "⏸️  STANDBY: Waiting for SCAN trigger.\n" +
                "👉 Place bricks and pub to /pick_and_place/scan:\n" +
                "ros2 topic pub --once /pick_and_place/scan std_msgs/msg/Empty\n\n" +
                "To manually STOP the Pick and Place process\n" +
                "and return to STANDBY, pub to /pick_and_place/stop:\n" +
                "ros2 topic pub --once /pick_and_place/stop std_msgs/msg/Empty\n" +
                "="*60
            )

            # Reset trigger
            node.start_triggered = False
            node.stop_triggered = False
            
            # Warten, bis der Trigger via Topic kommt
            while rclpy.ok() and not node.start_triggered:
                time.sleep(0.1)

            if not rclpy.ok():
                break # Falls du mit Strg+C abprichst, während er wartet

            # ─────────────────────────────────────────────────
            #  PHASE 0: SCAN (Nur noch 1 Call pro Tisch!)
            # ─────────────────────────────────────────────────
            logger.info("=" * 60)
            logger.info("🟢 PHASE 0: Scanning table via perception service...")
            logger.info("=" * 60)

            bricks = node.detect_bricks(executor)
            
            if not bricks:
                logger.warn("No bricks detected! Returning to standby.")
                continue # jump back to start of while loop and wait for next trigger

            # Sortiere die GANZE Liste einmalig (nächste Steine zuerst)
            bricks_sorted = sorted(bricks, key=lambda b: b.camera_distance_mm)
            logger.info(f"Detected {len(bricks_sorted)} brick(s). Starting Batch Processing!")

            # ─────────────────────────────────────────────────
            #  BATCH-SCHLEIFE über alle gefundenen Steine
            # ─────────────────────────────────────────────────

            def check_abort():
                if node.stop_triggered:
                    raise RuntimeError("STOP trigger received by operator!")

            for i, brick in enumerate(bricks_sorted):

                # Prüfen, ob vor dem nächsten Stein schon Stop gedrückt wurde
                if not rclpy.ok() or node.stop_triggered:
                    logger.warn("⚠️ BATCH ABORTED BY OPERATOR! Skipping remaining bricks.")
                    break

                color = brick.color.data
                bx = brick.position.point.x
                by = brick.position.point.y
                bz = brick.position.point.z
                yaw = math.radians(brick.yaw_degrees)

                logger.info("-" * 60)
                logger.info(f"BATCH {i+1}/{len(bricks_sorted)} | TARGET: {color.upper()} brick")
                logger.info(f"Position : X={bx:.3f}, Y={by:.3f}, Z={bz:.3f}")
                logger.info("-" * 60)

                # Determine drop-off location
                if color in node.dropoffs:
                    target_xy = node.dropoffs[color]
                else:
                    logger.warn(f"No drop-off for '{color}', using 'default'.")
                    target_xy = node.dropoffs['default']

                # ─────────────────────────────────────────────────
                #  DEFINE ALL POSES FOR THIS CYCLE
                # ─────────────────────────────────────────────────
                pose_hover_pick_straight = make_pose(bx, by, node.hover_height, 0.0)
                pose_hover_pick_oriented = make_pose(bx, by, node.hover_height, yaw)
                pose_grasp               = make_pose(bx, by, node.grasp_height, yaw)
                pose_hover_drop          = make_pose(target_xy[0], target_xy[1], node.hover_height, 0.0)
                pose_drop                = make_pose(target_xy[0], target_xy[1], node.dropoff_height, 0.0)

                # ─────────────────────────────────────────────────
                #  EXECUTE PICK-AND-PLACE CYCLE
                # ─────────────────────────────────────────────────
                escape_pose = None

                try:
                    check_abort()

                    # ── PHASE 1: APPROACH + ORIENT (OMPL) ─────────
                    # OMPL plans free-space motion including yaw rotation
                    # Rotation happens safely at hover height, not near the table
                    logger.info("🟢 PHASE 1: Approach + orient (OMPL)")
                    if not plan_and_execute(ur5e, ur5e_arm, logger, pose_hover_pick_oriented, tcp_link):
                        raise RuntimeError("Failed to reach oriented hover pose")

                    check_abort()
                    escape_pose = pose_hover_pick_straight

                    # ── PHASE 2: DESCEND (LIN) ────────────────────
                    # Pure vertical descent, yaw is already set
                    logger.info("🟢 PHASE 2: Descend to grasp (LIN)")
                    if not plan_and_execute_cartesian(ur5e, ur5e_arm, logger, pose_grasp, tcp_link):
                        raise RuntimeError("Failed to descend to grasp")

                    check_abort()

                    # ── PHASE 3: GRASP ────────────────────────────
                    logger.info("🟢 PHASE 3: Grasping")
                    node.set_gripper(True)
                    time.sleep(1.0)

                    check_abort()

                    # ── PHASE 4: LIFT (LIN) ───────────────────────
                    # Straight up, keeping yaw to avoid rotating with brick near table
                    logger.info("🟢 PHASE 4: Lift (LIN)")
                    if not plan_and_execute_cartesian(ur5e, ur5e_arm, logger, pose_hover_pick_oriented, tcp_link):
                        raise RuntimeError("Failed to lift brick")

                    check_abort()

                    """ # ── PHASE 5: UNTWIST (LIN) ────────────────────
                    # Rotate back to neutral yaw at safe hover height
                    if yaw != 0.0:
                        logger.info("🟢 PHASE 5: Untwist at hover height (LIN)")
                        if not plan_and_execute_cartesian(ur5e, ur5e_arm, logger, pose_hover_pick_straight, tcp_link):
                            raise RuntimeError("Failed to untwist at hover height")
                    else:
                        logger.info("🟢 PHASE 5: Skipped (yaw is 0°)")

                    check_abort() """
                    escape_pose = None

                    # ── PHASE 6: TRANSPORT ────────────────────────
                    logger.info(f"🟢 PHASE 6: Transport to {color} drop-off")
                    if not plan_and_execute_cartesian(ur5e, ur5e_arm, logger, pose_hover_drop, tcp_link):
                        logger.warn("  LIN failed, trying OMPL fallback...")
                        if not plan_and_execute(ur5e, ur5e_arm, logger, pose_hover_drop, tcp_link):
                            raise RuntimeError("Failed to reach drop-off zone")

                    check_abort()
                    escape_pose = pose_hover_drop

                    # ── PHASE 7: LOWER (LIN) ──────────────────────
                    logger.info("🟢 PHASE 7: Lower to drop-off (LIN)")
                    if not plan_and_execute_cartesian(ur5e, ur5e_arm, logger, pose_drop, tcp_link):
                        logger.warn("Failed to lower completely. Releasing from current height.")

                    check_abort()

                    # ── PHASE 8: RELEASE ──────────────────────────
                    logger.info("🟢 PHASE 8: Release")
                    node.set_gripper(False)
                    time.sleep(0.5)

                    check_abort()

                    # ── PHASE 9: RETREAT (LIN) ────────────────────
                    logger.info("🟢 PHASE 9: Retreat (LIN)")
                    if not plan_and_execute_cartesian(ur5e, ur5e_arm, logger, pose_hover_drop, tcp_link):
                        logger.warn("Vertical retreat failed.")

                    escape_pose = None

                except RuntimeError as e:
                    # ── ERROR RECOVERY & SOFT STOP ──
                    logger.info(
                        "Status:"
                        "\n" + "="*60 + "\n" +
                        f"🛑 CYCLE ABORTED: {e}\n" +
                        "Releasing gripper and skipping to next brick in batch...\n" +
                        "="*60
                    )

                    node.set_gripper(False)
                    time.sleep(0.5)

                    # 1. Den sicheren Rückzug ausführen (falls wir in der Gefahrenzone waren)
                    if escape_pose is not None:
                        logger.info("Executing vertical retract & untwist to safe hover height...")
                        plan_and_execute_cartesian(ur5e, ur5e_arm, logger, escape_pose, tcp_link)
                    else:
                        logger.info("Arm is already safe. Skipping retract.")
                        
                    # 2. Zurück zur Home-Pose
                    logger.info("Returning to ready pose...")
                    plan_and_execute(ur5e, ur5e_arm, logger, "ready", tcp_link)
                    
                    # 3. WICHTIG: Die Batch-Schleife für die restlichen Steine komplett abbrechen!
                    break
            
            # --- ENDE DER BATCH SCHLEIFE ---
            logger.info(
                "Status:"
                "\n" + "="*60 + "\n" +
                "🟢🏁 BATCH COMPLETE: Returning to ready pose.\n" +
                "="*60
            )


            plan_and_execute(ur5e, ur5e_arm, logger, "ready", tcp_link)

    except KeyboardInterrupt:
        logger.info("Application stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()