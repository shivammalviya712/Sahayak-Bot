#!/usr/bin/env python


import math
import sys
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def pose_callback(msg):
    rospy.loginfo(msg)
    pass


def revolve(linear_x, angular_z):
    # Declaration
    rospy.init_node('node_turtle_revolve', anonymous=True)
    rate = rospy.Rate(10)
    velocity_publisher = rospy.Publisher(
            'turtle1/cmd_vel',
            Twist,
            queue_size=10,
            latch=True
    )
    rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback)
    velocity_msg = Twist()

    # Initialisation
    velocity_msg.linear.x = linear_x
    velocity_msg.linear.y = 0
    velocity_msg.linear.z = 0
    velocity_msg.angular.x = 0
    velocity_msg.angular.y = 0
    velocity_msg.angular.z = angular_z

    # Revolution
    try:
        T = abs(2*math.pi/velocity_msg.angular.z)
    except ZeroDivisionError:
        T = 0

    total_dist = abs(T * velocity_msg.linear.x)    
    current_dist = 0
    t0 = rospy.Time.now().to_sec()

    while (current_dist < total_dist):
        velocity_publisher.publish(velocity_msg)
        t1 = rospy.Time.now().to_sec()
        current_dist = velocity_msg.linear.x * (t1-t0)
        rospy.loginfo("Current distance = %s" % current_dist)
        rospy.loginfo("Total distance = %s" % total_dist)
        rate.sleep() 

    #rospy.loginfo("Delta t = %s" % (t1-t0))
    #rospy.loginfo("T = %s" % T)
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    velocity_publisher.publish(velocity_msg)
    log_msg = "Goal reached"
    rospy.loginfo(log_msg)

    rospy.spin()


if __name__ == "__main__":
    args = rospy.myargv(argv=sys.argv)
    try:
        revolve(linear_x=float(args[1]),
                angular_z=float(args[2]))
    except rospy.ROSInterruptException:
        pass