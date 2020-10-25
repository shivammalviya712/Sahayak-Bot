#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

pose = [0,0,0]

flag = 0
ctr = 0
Kp = 10
Ki = 0.5
Kd = 10
esum=0
eold=ediff=0

def Waypoints(t):
	x  = t
	y  = 2*math.sin(t)*math.sin(t/2)
	return [x,y]

def control_loop():
	global pose
	global flag
	global ctr
	global Kp
	global Ki
	global Kd
	global esum
	global eold
	global ediff

	rospy.init_node('ebot_controller')
    
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
	rospy.Subscriber('/odom', Odometry, odom_callback)
    
	rate = rospy.Rate(10) 

	velocity_msg = Twist()
	velocity_msg.linear.x = 0
	velocity_msg.angular.z = 0
	pub.publish(velocity_msg)

	while not rospy.is_shutdown() and flag==0:
		thresh = 0.2
		ctr += 0.05
		[X,Y] = Waypoints(ctr)
		theta_h = math.atan2(Y-pose[1],X-pose[0])
		theta_p = math.atan2(math.sin(pose[2]),math.cos(pose[2]))
		e = theta_h - theta_p
		esum = e
		ediff = e
		print("After increasing ctr : X="+str(X)+" Y="+str(Y)+" theta_h="+str(theta_h)+" theta_p="+str(theta_p)+" e="+str(e))

		while ((pose[0]-X)**2 + (pose[1]-Y)**2)**0.5 > thresh:
			eold = e
			velocity_msg.angular.z = Kp*e + Ki*esum + Kd*ediff
			if abs(e) > 0.08:
				velocity_msg.linear.x = 0.02
				thresh = 0.02
			else:
				velocity_msg.linear.x = 0.5
				thresh = 0.2
			pub.publish(velocity_msg)
			print("Current position is : x="+str(pose[0])+" y="+str(pose[1])+" theta="+str(pose[2])+" ctr="+str(ctr)+" e="+str(e))
			theta_p = math.atan2(math.sin(pose[2]),math.cos(pose[2]))
			e = theta_h - theta_p
			ediff = e - eold
			esum += e
			rate.sleep()

		if ctr > (2*math.pi - 0.1) and ctr < (2*math.pi + 0.1):
			flag = 1
			velocity_msg.angular.z = 0		
			velocity_msg.linear.x = 0
			pub.publish(velocity_msg)
			print("Completed traversing sin path!!!")

def odom_callback(data):
	global pose
	x  = data.pose.pose.orientation.x;
	y  = data.pose.pose.orientation.y;
	z = data.pose.pose.orientation.z;
	w = data.pose.pose.orientation.w;
	pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]

def laser_callback(msg):
   # global regions
   # regions = {
   #     'bright':  	,
   #     'fright': 	,
   #     'front':  	,
   #     'fleft':  	,
   #     'bleft':   	,
   # }
	pass

if __name__ == '__main__':
	try:
		control_loop()
	except rospy.ROSInterruptException:
		pass

