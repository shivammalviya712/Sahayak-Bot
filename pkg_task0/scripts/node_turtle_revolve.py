#!/usr/bin/env python

# Python libraries
import math
import sys

# ROS libraries
import rospy

# ROS messages
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class NodeTurtleRevolve:
    def __init__(self, vx, wz):
        # turtle1/cmd_vel publisher
        self.velocity_publisher = rospy.Publisher(
            'turtle1/cmd_vel',
            Twist,
            queue_size=10,
        )
        # turtle1/pose subscriber
        rospy.Subscriber(
            "/turtle1/pose", 
            Pose,
            callback=self.pose_callback
        )
        self.rate = rospy.Rate(10)
        self.velocity_msg = Twist()
        
        # Used to calculate the distance covered
        # in the callback
        self.theta = 0
        self.shift = 0
        
        # Intialial velocity to published
        self.velocity_msg.linear.x = vx
        self.velocity_msg.linear.y = 0
        self.velocity_msg.linear.z = 0
        self.velocity_msg.angular.x = 0
        self.velocity_msg.angular.y = 0
        self.velocity_msg.angular.z = wz
        
        # Calculation of the total distance 
        # to be covered
        self.radius = abs(
            self.velocity_msg.linear.x
            /self.velocity_msg.angular.z
        )
        self.total_distance = 2*math.pi*self.radius


    def pose_callback(self, position):
        # Convert theta into 0 to infinity format 
        dtheta = position.theta + self.shift - self.theta
        self.shift += (dtheta < -math.pi) * 2*math.pi
        self.theta = position.theta + self.shift


    def revolve(self):

        # Revolve till total distance is covered or 
        # Ctrl+C is pressed.
        while ((self.theta*self.radius < self.total_distance) 
                and not rospy.is_shutdown()):
            self.velocity_publisher.publish(self.velocity_msg)
            self.rate.sleep()
            rospy.loginfo(
                "Distance covered = %s" % (self.theta*self.radius)
            )

        # Stop the turtle and display "Goal reached!!" 
        # if the total distance is covered.
        if not rospy.is_shutdown():
            self.velocity_msg.linear.x = 0
            self.velocity_msg.angular.z = 0
            self.velocity_publisher.publish(self.velocity_msg)
            log_msg = "Goal reached!!"
            rospy.loginfo(log_msg)


if __name__ == "__main__":
    args = rospy.myargv(argv=sys.argv)
    rospy.init_node("node_turtle_revolve", anonymous=True)
    node_turtle_revolve = NodeTurtleRevolve(
        float(args[1]),
        float(args[2]))
    try:
        node_turtle_revolve.revolve()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass