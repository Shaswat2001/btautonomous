#!/usr/bin/env python3

import py_trees
import py_trees_ros
from behaviour import grasping
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
import sys

tau = 2*math.pi

if __name__ == "__main__":

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_automate")

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name_arm = "fr3_arm"
    move_group_arm = moveit_commander.MoveGroupCommander(group_name_arm)
    move_group_arm.set_planner_id("RRTstar")

    group_name_hand = "fr3_hand"
    move_group_hand = moveit_commander.MoveGroupCommander(group_name_hand)
    
    # joint_goal = move_group_arm.get_current_joint_values()
    # joint_goal[0] = 0
    # joint_goal[1] = -tau / 8
    # joint_goal[2] = 0
    # joint_goal[3] = -tau / 4
    # joint_goal[4] = 0
    # joint_goal[5] = tau / 6  # 1/6 of a turn
    # joint_goal[6] = 0

    # joint_goal = move_group_hand.get_current_joint_values()
    # joint_goal[0] = 0.002

    # plan = move_group_hand.plan(joint_goal)
    # move_success = "RUNNING"
    # if plan[1].joint_trajectory.points:
    #     for i in range(100):
    #         move_success = move_group_hand.execute(plan[1])

    

    # # The go command can be called with joint values, poses, or without any
    # # parameters if you have already set the pose or joint target for the group
    # move_group_arm.go(joint_goal, wait=True)

    # # Calling ``stop()`` ensures that there is no residual movement
    # move_group_arm.stop()
    
    # plan = move_group_arm.plan()

    # move_group_arm.execute(plan, wait=True)

    root = py_trees.composites.Selector(name="TOPLLayer")
    root.add_child(grasping.OpenGripper("hand",move_group_hand,0.03))

    ros_py_tree = py_trees_ros.trees.BehaviourTree(root)

    done = False
    while not rospy.is_shutdown() and not done:

        ros_py_tree.tick()

        if ros_py_tree.root.status == py_trees.common.Status.SUCCESS:
            print("Behavior tree succeeded")
            done = True
        elif ros_py_tree.root.status == py_trees.common.Status.FAILURE:
            print("Behavior tree failed.")
            done = True
        rospy.sleep(0.5)
    rospy.spin()





