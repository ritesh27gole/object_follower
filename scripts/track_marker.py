import rospy
import numpy as np
import math

from moveit_msgs.msg import RobotTrajectory
from std_msgs.msg import Float32MultiArray
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import Pose

import moveit_commander

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error_x = 0
        self.integral_x = 0
        self.prev_error_y = 0
        self.integral_y = 0

    def compute_control_signal(self, error_x, error_y):
        dt = 1.0  # Time step (you can adjust this based on your control rate)
        
        # Compute the PID control signal for the x direction
        derivative_x = (error_x - self.prev_error_x) / dt
        self.integral_x += error_x * dt
        control_signal_x = self.Kp * error_x + self.Ki * self.integral_x + self.Kd * derivative_x
        
        # Compute the PID control signal for the y direction
        derivative_y = (error_y - self.prev_error_y) / dt
        self.integral_y += error_y * dt
        control_signal_y = self.Kp * error_y + self.Ki * self.integral_y + self.Kd * derivative_y
        
        self.prev_error_x = error_x
        self.prev_error_y = error_y

        return control_signal_x, control_signal_y

def pid(msg, robot=moveit_commander.RobotCommander(),
        group=moveit_commander.MoveGroupCommander('arm_group')):
    x = msg.data[0]
    y = msg.data[1]

    current_joint_values = group.get_current_joint_values()
    print("Current joint values:", current_joint_values)

    error_x = x - 320
    error_y = y - 240

    pid_controller = PIDController(Kp=0.05, Ki=0.0002, Kd=0.002)
    control_signal_x, control_signal_y = pid_controller.compute_control_signal(error_x, error_y)

    dx = control_signal_x * (math.pi / (640 * 6))
    dy = control_signal_y * (math.pi / (480 * 6))

    print("dx:", dx)
    print("dy:", dy)

    j1 = current_joint_values[0] - dx
    j2 = current_joint_values[1] - dy
    j3 = current_joint_values[2]
    j4 = current_joint_values[3]
    j5 = current_joint_values[4]
    j6 = current_joint_values[5]

    joint_angles = [j1, j2, j3, j4, j5, j6]

    # Set the target joint angles
    group.set_joint_value_target(joint_angles)

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
