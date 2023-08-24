import rospy
from geometry_msgs.msg import Twist
import time

def pattern_1():
    data1 = Twist()
    data2 = Twist()
    
    data1.linear.x = 0.01
    data1.linear.y = 0.0
    data1.linear.z = 0.0
    data1.angular.x = 0.0
    data1.angular.y = 0.0
    data1.angular.z = 0.0
    data2.linear.x = -0.01
    data2.linear.y = 0.0
    data2.linear.z = 0.0
    data2.angular.x = 0.0
    data2.angular.y = 0.0
    data2.angular.z = 0.0

    print("Going forward")
    cmd_vel_pub.publish(data1)
    time.sleep(60.0)

    print("Going backwards")
    cmd_vel_pub.publish(data2)
    time.sleep(60.0)  
    

if __name__ == '__main__':
    print("Hey Universe!")
    rospy.init_node('wheely_controller_node')
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    while not rospy.is_shutdown():
        pattern_1()
