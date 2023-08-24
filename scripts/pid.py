import rospy
import numpy as np
import math

from moveit_msgs.msg import RobotTrajectory
from std_msgs.msg import Float32MultiArray
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import Pose

import moveit_commander

def pid(msg, prev_dx = 0, int_dx = 0, prev_dy = 0, int_dy = 0, robot = moveit_commander.RobotCommander(), group = moveit_commander.MoveGroupCommander('arm_group')):
    x = msg.data[0]
    y = msg.data[1]

    Kp_z = 1
    Ki_z = 0.01
    Kd_z = 0.01
    Kv_z = 0.00001
    Kp_j1 = 0.75
    Ki_j1 = 0.01
    Kd_j1 = 0.1
    Kv_j1 = 0.00005

    current_joint_values = robot.get_current_state().joint_state.position
    current_pose = group.get_current_pose().pose

    error_x = x - 320
    error_y = y - 240

    dz = Kv_z*(Kp_z*error_y + Ki_z*int_dy + Kd_z*(error_y - prev_dy))
    dj1 = Kv_j1*(Kp_j1*error_x + Ki_j1*int_dx + Kd_j1*(error_x - prev_dx))

    prev_dy = error_y
    int_dy += error_y
    prev_dx = error_x
    int_dx += error_x

    j1 = current_joint_values[0] - dj1
    j2 = current_joint_values[1]
    j3 = current_joint_values[2]
    j4 = current_joint_values[3]
    j5 = current_joint_values[4]
    j6 = current_joint_values[5]

    if j1 > 1.57:
        j1 = 1.57
    elif j1 < -1.57:
        j1 = -1.57

    joint_angles = [j1, j2, j3, j4, j5, j6]

    target_pose = Pose()
    target_pose.position.z = current_pose.position.x
    target_pose.position.z = current_pose.position.y
    target_pose.position.z = current_pose.position.z - dz

    # Set the target joint angles
    group.set_start_state_to_current_state()
    group.set_joint_value_target(joint_angles)

    # # Set the target pose for the end-effector
    # group.set_pose_target(target_pose)

    # Plan and execute the motion
    plan = group.go(wait=True)

def pid2(msg, prev_dx = 0, int_dx = 0, prev_dy = 0, int_dy = 0, robot = moveit_commander.RobotCommander(), group = moveit_commander.MoveGroupCommander('arm_group')):
    x = msg.data[0]
    y = msg.data[1]

    Kp_z = 0.1
    Ki_z = 0.01
    Kd_z = 0.1
    Kv_z = 0.05
    Kp_j1 = 0.75
    Ki_j1 = 0.01
    Kd_j1 = 0.1
    Kv_j1 = 0.000075

    error_x = x - 320
    error_y = y - 240

    # Get the current pose of the end-effector
    current_pose = group.get_current_pose().pose

    dz = Kv_z*(Kp_z*error_y + Ki_z*int_dy + Kd_z*(error_y - prev_dy))
    dj1 = Kv_j1*(Kp_j1*error_x + Ki_j1*int_dx + Kd_j1*(error_x - prev_dx))

    prev_dy = error_y
    int_dy += error_y
    prev_dx = error_x
    int_dx += error_x

    target_pose = Pose()
    target_pose.position.z = current_pose.position.z - dz

    # Set the target pose for the end-effector
    group.set_pose_target(target_pose)

    # Plan and execute the motion
    plan = group.go(wait=True)

def main():
    # Initialize the ROS node and MoveIt
    print("Hey Universe!")
    rospy.init_node('motion_planning_node')
    aruco_pose_sub = rospy.Subscriber("/aruco/pose", Float32MultiArray, pid)
    rospy.spin()

if __name__ == '__main__':
    main()