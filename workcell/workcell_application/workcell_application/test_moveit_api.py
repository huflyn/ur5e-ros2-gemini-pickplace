#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""

import time
import math

# ros libraries
import rclpy
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
    MultiPipelinePlanRequestParameters,
)


def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
    ):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("*** Executing plan ***")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
        logger.info("*** Plan executed successfully ***")
    else:
        logger.error("*** Planning failed ***")

    time.sleep(sleep_time)


def main(args=None):

    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init(args=args)
    logger = rclpy.logging.get_logger("moveit_py_test_node.pose_goal")
    logger.info("Initialise MoveItPy...")

    # instantiate MoveItPy instance and get planning component
    ur5e = MoveItPy(node_name="moveit_py_test_node")
    planning_group = "ur_arm"
    ur5e_arm = ur5e.get_planning_component(planning_group)
    logger.info("MoveItPy instance created")


    ###########################################################################
    # Plan 1 - set states with predefined string
    ###########################################################################
    logger.info(
        "\n-------------------------------------------------------------\n"
        "        Plan 1 - set states with predefined string         \n"
        "-------------------------------------------------------------"
    )

    # set start state to current robot state
    ur5e_arm.set_start_state_to_current_state()
    
    # set pose goal using predefined state (defined in SRDF)
    ur5e_arm.set_goal_state(configuration_name="ready")

    # plan to goal
    plan_and_execute(ur5e, ur5e_arm, logger, sleep_time=3.0)


    ###########################################################################
    # Plan 2a - set goal state with PoseStamped message (pilz_lin)
    ###########################################################################
    logger.info(
        "\n-------------------------------------------------------------\n"
        "  Plan 2a - set goal state with PoseStamped message (pilz_lin)  \n"
        "-------------------------------------------------------------"
    )

    # set plan start state to current state
    ur5e_arm.set_start_state_to_current_state()

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "ur5e_base_link"

    pose_goal.pose.position.x = -0.03
    pose_goal.pose.position.y = 0.7
    pose_goal.pose.position.z = 0.2

    q = quaternion_from_euler(math.pi, 0.0, math.pi/2)
    pose_goal.pose.orientation.x = q[0]
    pose_goal.pose.orientation.y = q[1]
    pose_goal.pose.orientation.z = q[2]
    pose_goal.pose.orientation.w = q[3]

    # set pose goal with PoseStamped message
    ur5e_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="pisoftgrip_tcp")

    # initialize single-pipeline plan request parameters
    single_plan_parameters = PlanRequestParameters(
        ur5e,
        "pilz_lin"
    )

    # plan to goal
    plan_and_execute(ur5e, ur5e_arm, logger, sleep_time=3.0)


    ###########################################################################
    # Plan 2b - set goal state with PoseStamped message (pilz_lin)
    ###########################################################################
    logger.info(
        "\n-------------------------------------------------------------\n"
        "  Plan 2b - set goal state with PoseStamped message (pilz_lin)  \n"
        "-------------------------------------------------------------"
    )

    # set plan start state to current state
    ur5e_arm.set_start_state_to_current_state()

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "ur5e_base_link"

    pose_goal.pose.position.x = -0.03
    pose_goal.pose.position.y = 0.7
    pose_goal.pose.position.z = 0.0

    q = quaternion_from_euler(math.pi, 0.0, math.pi/2)
    pose_goal.pose.orientation.x = q[0]
    pose_goal.pose.orientation.y = q[1]
    pose_goal.pose.orientation.z = q[2]
    pose_goal.pose.orientation.w = q[3]

    # set pose goal with PoseStamped message
    ur5e_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="pisoftgrip_tcp")

    # initialize single-pipeline plan request parameters
    single_plan_parameters = PlanRequestParameters(
        ur5e,
        "pilz_lin"
    )

    # plan to goal
    plan_and_execute(ur5e, ur5e_arm, logger, sleep_time=3.0)


    ###########################################################################
    # Plan 3 - Planning with Multiple Pipelines simultaneously
    ###########################################################################
    logger.info(
        "\n------------------------------------------------------------\n"
        "   Plan 3 - Planning with Multiple Pipelines simultaneously   \n"
        "------------------------------------------------------------"
    )

    # set plan start state to current state
    ur5e_arm.set_start_state_to_current_state()

    # set pose goal with PoseStamped message
    ur5e_arm.set_goal_state(configuration_name="up")

    # initialise multi-pipeline plan request parameters
    multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
        ur5e, ["ompl_rrtc", "pilz_lin", "chomp_planner"]
    )

    # plan to goal
    plan_and_execute(
        ur5e,
        ur5e_arm,
        logger,
        multi_plan_parameters=multi_pipeline_plan_request_params,
        sleep_time=3.0,
    )


    ###########################################################################
    # Clean shutdown
    ###########################################################################
    logger.info(
        "\n------------------------------------------------------------\n"
        "           Test-Skript erfolgreich beendet.                   \n"
        "------------------------------------------------------------\n"
    )

    rclpy.shutdown()

if __name__ == '__main__':
    main()