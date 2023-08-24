import rospy
import math
import moveit_commander

def initial_pose(robot = moveit_commander.RobotCommander(), group = moveit_commander.MoveGroupCommander('arm_group')):
    joint_angles = [-5*math.pi/180, -40*math.pi/180, -86*math.pi/180, 0.0, 0.0, 0.0]
    # joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # Set the target joint angles
    group.set_joint_value_target(joint_angles)

    # Plan and execute the motion
    plan = group.go(wait=True)
    print("Initial Pose achieved")

def main():
    # Initialize the ROS node and MoveIt
    print("Pose initialization started")
    rospy.init_node('pose_init_node')
    initial_pose()

if __name__ == '__main__':
    main()