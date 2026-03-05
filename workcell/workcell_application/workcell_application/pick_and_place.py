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
import copy
import math
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from tf_transformations import quaternion_from_euler

from color_detection_msgs.msg import LegoBrick
from color_detection_msgs.srv import DetectBricks

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
        self.declare_parameter('grasp_height', 0.005)
        self.declare_parameter('dropoff_height', 0.10)

        self.declare_parameter('use_sim_gripper', True)
        self.declare_parameter('gripper_topic', '/ur5e/vacuum_gripper/turn_on')

        self.declare_parameter('dropoff_red',    [0.27, 0.450])
        self.declare_parameter('dropoff_blue',   [0.27, 0.350])
        self.declare_parameter('dropoff_green',  [-0.27, 0.450])
        self.declare_parameter('dropoff_yellow', [-0.27, 0.350])
        self.declare_parameter('dropoff_default', [0.0, 0.450])

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
            'red':     self.get_parameter('dropoff_red').value,
            'blue':    self.get_parameter('dropoff_blue').value,
            'green':   self.get_parameter('dropoff_green').value,
            'yellow':  self.get_parameter('dropoff_yellow').value,
            'default': self.get_parameter('dropoff_default').value,
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

        # Spin executor while waiting (non-blocking wait pattern)
        while rclpy.ok() and not future.done():
            executor.spin_once(timeout_sec=0.1)

        result = future.result()
        if result is None:
            self.get_logger().error("Service call returned None!")
            return []

        if not result.success:
            self.get_logger().error(f"Detection failed: {result.error_message}")
            return []

        return list(result.bricks)


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
        time.sleep(0.5)  # Let joints settle in physics sim
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

    ur5e_arm = ur5e.get_planning_component(node.planning_group)
    tcp_link = node.tcp_link

    # ── 3. Gripper orientation: TCP pointing straight down ────
    q_down = quaternion_from_euler(math.pi, 0.0, 0.0)

    # ── 4. Helper: Create a PoseStamped with downward orientation ──
    def make_pose(x, y, z):
        pose = PoseStamped()
        pose.header.frame_id = node.base_frame
        pose.pose.orientation.x = q_down[0]
        pose.pose.orientation.y = q_down[1]
        pose.pose.orientation.z = q_down[2]
        pose.pose.orientation.w = q_down[3]
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        return pose

    # ── 5. Initial homing ──────────────────────────────────────
    logger.info("=" * 60)
    logger.info("  Initializing: Gripper OFF, moving to 'ready' pose")
    logger.info("=" * 60)
    node.set_gripper(False)
    time.sleep(1.0)
    plan_and_execute(ur5e, ur5e_arm, logger, "ready", tcp_link)

    # ── 6. Main pick-and-place loop ────────────────────────────
    idle_logged = False

    try:
        while rclpy.ok():
            # Process pending ROS callbacks
            executor.spin_once(timeout_sec=0.1)

            # ─────────────────────────────────────────────────
            #  PHASE 0: SCAN
            # ─────────────────────────────────────────────────
            logger.info("=" * 60)
            logger.info("  PHASE 0: Scanning table for bricks...")
            logger.info("=" * 60)

            bricks = node.detect_bricks(executor)

            if not bricks:
                if not idle_logged:
                    logger.info("  No bricks detected. Waiting...")
                    idle_logged = True
                time.sleep(3.0)
                continue

            idle_logged = False
            logger.info(f"  Detected {len(bricks)} brick(s).")

            # ─────────────────────────────────────────────────
            #  PHASE 1: SELECT (closest brick first)
            # ─────────────────────────────────────────────────
            bricks_sorted = sorted(bricks, key=lambda b: b.camera_distance_mm)
            brick = bricks_sorted[0]
            color = brick.color.data
            bx = brick.position.point.x
            by = brick.position.point.y
            bz = brick.position.point.z

            logger.info("-" * 60)
            logger.info(f"  SELECTED: {color.upper()} brick")
            logger.info(f"    Position : X={bx:.3f}, Y={by:.3f}, Z={bz:.3f}")
            logger.info(f"    Distance : {brick.camera_distance_mm:.0f} mm")
            logger.info("-" * 60)

            # Determine drop-off location
            if color in node.dropoffs:
                target_xy = node.dropoffs[color]
            else:
                logger.warn(f"  No drop-off for '{color}', using 'default'.")
                target_xy = node.dropoffs['default']

            # ─────────────────────────────────────────────────
            #  DEFINE ALL POSES FOR THIS CYCLE
            # ─────────────────────────────────────────────────
            pose_hover_pick = make_pose(bx, by, node.hover_height)
            pose_grasp      = make_pose(bx, by, node.grasp_height)
            pose_hover_drop = make_pose(target_xy[0], target_xy[1], node.hover_height)
            pose_drop       = make_pose(target_xy[0], target_xy[1], node.dropoff_height)

            # ─────────────────────────────────────────────────
            #  EXECUTE PICK-AND-PLACE CYCLE
            # ─────────────────────────────────────────────────
            try:
                # ── PHASE 2: APPROACH ──────────────────────
                logger.info("=" * 60)
                logger.info("  PHASE 2: Approach (OMPL → hover above brick)")
                logger.info("=" * 60)

                if not plan_and_execute(ur5e, ur5e_arm, logger,
                                        pose_hover_pick, tcp_link):
                    raise RuntimeError("Failed to reach hover pose above brick")

                # ── PHASE 3: DESCEND (Pilz LIN) ───────────
                logger.info("=" * 60)
                logger.info("  PHASE 3: Descend to grasp (Pilz LIN)")
                logger.info("=" * 60)

                if not plan_and_execute_cartesian(ur5e, ur5e_arm, logger,
                                                   pose_grasp, tcp_link):
                    raise RuntimeError("Failed to descend to grasp pose")

                # ── PHASE 4: GRASP ────────────────────────
                logger.info("  PHASE 4: Activating gripper...")
                node.set_gripper(True)
                time.sleep(1.0)  # Wait for suction to build

                # ── PHASE 5: LIFT (Pilz LIN) ──────────────
                logger.info("=" * 60)
                logger.info("  PHASE 5: Lifting brick (Pilz LIN)")
                logger.info("=" * 60)

                if not plan_and_execute_cartesian(ur5e, ur5e_arm, logger,
                                                   pose_hover_pick, tcp_link):
                    raise RuntimeError("Failed to lift brick")

                # ── PHASE 6: TRANSPORT ─────────────────────
                logger.info("=" * 60)
                logger.info(f"  PHASE 6: Transport to {color} drop-off")
                logger.info("=" * 60)

                # Try LIN first (smooth), fall back to OMPL if singularity
                if not plan_and_execute_cartesian(ur5e, ur5e_arm, logger,
                                                   pose_hover_drop, tcp_link):
                    logger.warn("  LIN transfer failed, trying OMPL fallback...")
                    if not plan_and_execute(ur5e, ur5e_arm, logger,
                                            pose_hover_drop, tcp_link):
                        raise RuntimeError("Failed to reach drop-off zone")

                # ── PHASE 7: LOWER TO DROP-OFF (Pilz LIN) ─
                logger.info("=" * 60)
                logger.info("  PHASE 7: Lowering to drop-off (Pilz LIN)")
                logger.info("=" * 60)

                if not plan_and_execute_cartesian(ur5e, ur5e_arm, logger,
                                                   pose_drop, tcp_link):
                    logger.warn("  Failed to lower completely. Releasing from current height.")

                # ── PHASE 8: RELEASE ──────────────────────
                logger.info("  PHASE 8: Releasing brick...")
                time.sleep(0.5)
                node.set_gripper(False)
                time.sleep(1.0)  # Wait for suction to release

                # ── PHASE 9: RETREAT ──────────────────────
                logger.info("=" * 60)
                logger.info("  PHASE 9: Retreat and return to ready")
                logger.info("=" * 60)

                # Straight up from drop-off
                if not plan_and_execute_cartesian(ur5e, ur5e_arm, logger,
                                                   pose_hover_drop, tcp_link):
                    logger.warn("  Vertical retreat failed. Trying OMPL...")

                # Back to ready (OMPL, free-space)
                if not plan_and_execute(ur5e, ur5e_arm, logger,
                                        "ready", tcp_link):
                    logger.error("  Failed to return to ready pose!")

                logger.info("=" * 60)
                logger.info(f"  CYCLE COMPLETE: {color.upper()} brick sorted!")
                logger.info("=" * 60 + "\n")

            except RuntimeError as e:
                # ── ERROR RECOVERY ─────────────────────────
                logger.error(f"  CYCLE ABORTED: {e}")
                logger.info("  Releasing gripper and returning to ready...")
                node.set_gripper(False)
                time.sleep(0.5)
                plan_and_execute(ur5e, ur5e_arm, logger, "ready", tcp_link)

            # Brief pause before next scan cycle
            time.sleep(1.0)

    except KeyboardInterrupt:
        logger.info("Application stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()