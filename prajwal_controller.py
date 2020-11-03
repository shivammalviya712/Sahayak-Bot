#!/usr/bin/env python3

import rospy
import numpy as np
import math
from tf.transformations import euler_from_quaternion
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Vector3

class Vector:
    def __init__(self,x=0,y=0,z=0):
        self.x = x
        self.y  = y
        self.z = z
    
    def __add__(self, other):
        return Vector(self.x+other.x,self.y+other.y,self.z + other.z)
    
    def __sub__(self, other):
        return Vector(self.x-other.x,self.y-other.y,self.z - other.z)
    def __str__(self):
         return "x: {} y: {} z: {}".format(self.x,self.y,self.z)

    def rotate(self, theta):
        x = self.x
        y = self.y
        self.x = math.cos(theta)*x - math.sin(theta)*y
        self.y = math.cos(theta)*y + math.sin(theta)*x  
    def polar2cart(r ,theta):
        return Vector(r*math.cos(theta), r*math.sin(theta),0)
    def getr(self):
        return math.sqrt(self.x*self.x+self.y*self.y)

    def gettheta(self):
        return math.atan2(self.y,self.x)
    
    def sat_function(r):
        c1 = 4
        c2 = 0.7
        return(10*math.exp(-c1*(r-c2))/(1+math.exp(-c1*(r-c2))))

class EbotController:
    def __init__(self):
        self.laserdata = LaserScan()
        self.current_pose = Pose2D()
        self.goal_point = Vector()
        self.rate = rospy.Rate(10)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 5)
        self.command = Twist()
        self.listenOdometry = rospy.Subscriber('/odom', Odometry, self.getPosition)
        self.listenLaserData = rospy.Subscriber('/ebot/laser/scan', LaserScan, self.getLaserData)
        self.hold = Twist()
        self.obstacle_threshold = 1
        self.goalArray = []
        """
        There are total 720 laser scans from
        Taking the vision range to be 60
        and obstacle threshold distance 
        but i need samples from -30 to 30 i.e 80 samples
        so i need samples from 359-40 to 359+40
        """
    
    def getLaserData(self,data):
        self.laserdata = data
        

    def getPosition(self,data):
        self.current_pose.x = data.pose.pose.position.x
        self.current_pose.y = data.pose.pose.position.y
        x  = data.pose.pose.orientation.x
        y  = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w
        temp1 = euler_from_quaternion([x,y,z,w])

        self.current_pose.theta = temp1[2] 
        #rospy.loginfo("lin_vel %s, ang_vel %s, x=%s, y=%s", self.command.linear.x, self.command.angular.z,data.x,data.y)
    
    def clearShot(self): #No obstacle In between
        #scan threshold = 90 samples
        threshold = 2
        phi_d = math.atan2(self.goal_point.y-self.current_pose.y, self.goal_point.x - self.current_pose.x)
        phibot = phi_d - self.current_pose.theta 
        print('calculated angle is', phibot, phi_d)
        if(phibot < -3.14):
            phibot = 2*3.14 + phibot
        elif(phibot > 3.14):
            phibot = 2*3.14 - phibot
        phibot = phibot + 2.36 #135 degree
        centresampleNo = math.floor(phibot/2/2.36*720)
        print('inside the clearshot, centresampleno', centresampleNo,phibot)
        for i in range(centresampleNo-90,centresampleNo+90):
            if i <= 0:
                if i<-100:
                    return False
                else:
                    continue
            elif i >= 720:
                if i > 800:
                    return False
                else:
                    continue 
            if self.laserdata.ranges[i] <= threshold:
                return False
            
        return True

    def final_controller(self):
        #set goal
        # while u reach the end of goal array 
        #   if clearshot
        #       go2goal
        #   else
        #       untill clear shot 
        #           follow wall
        #               right or left follow wall
        self.rate.sleep()
        for i in self.goalArray:
            self.goal_point = i
            while math.sqrt(math.pow((self.current_pose.x-self.goal_point.x),2)+math.pow((self.current_pose.y-self.goal_point.y),2)) > 0.1:
                self.goal_point = i
                if(self.clearShot()):
                    print('no obstacle moving to goal')
                    self.move2goal()
                else:
                    print('found the obstacle calling the follow wall routine')
                    while(not self.clearShot()):
                        if(self.chooseFollowWall()):
                            self.follow_wall_right()
                            
                        else:
                            self.follow_wall_right()
                            
                self.rate.sleep()
        self.vel_pub.publish(Twist())


            



    def stop(self):
        self.vel_pub.publish(Twist())

    def move2goal(self):
        self.reach_theta(math.atan2(self.goal_point.y-self.current_pose.y, self.goal_point.x - self.current_pose.x))
        print('inside the move2goal')
        while(not self.check_obstacle()):
            if math.sqrt(math.pow((self.current_pose.x-self.goal_point.x),2)+math.pow((self.current_pose.y-self.goal_point.y),2)) > 0.1:
                self.reach_theta(math.atan2(self.goal_point.y-self.current_pose.y, self.goal_point.x - self.current_pose.x))
                self.move_straight()
            else:
                self.stop()
                break
            self.rate.sleep()
        print('end of move goal routine')
        return
    # these are used for follow wall
    def move_straight(self):
        self.command.linear.x = 0.3
        self.command.angular.z = 0
        self.vel_pub.publish(self.command)

    def check_obstacle(self): #check Obstacle part
        i=359-40
        #print("this is the laser data",self.laserdata.ranges)
        #print("this is the odom data", self.current_pose)
        while(i< 359+40):
            if(self.laserdata.ranges[i] < self.obstacle_threshold):
                print('found obstacle in the range')
                return True
            i = i+1
        print("no obstacle in the range")
        return False
    
    
    #Follow walls according to the georgia tech
    def follow_wall_right(self): #complete folllow wall implementation
        i = 1
        end = 240
        theta_shift = 0.006554
        theta_start = -3.1429/4
        theta = theta_start
        resultant = Vector()
        print('inside follow wall right', len(self.laserdata.ranges))
        while i<=end:
            r = self.laserdata.ranges[i]
            r = Vector.sat_function(r)
            resultant = resultant + Vector.polar2cart(r,theta)
            theta = theta + theta_shift
            i = i+1
        #resultant.rotate(3.1429/2)
        goaltheta= resultant.gettheta() + self.current_pose.theta
        #print('this is the angle made by the resultant vector', resultant.gettheta())
        self.reach_theta(goaltheta)
        self.move_straight()

    #Follow walls according to the georgia tech
    def follow_wall_left(self): #complete folllow wall implementation
        i = 719
        end = 719-240
        theta_shift = 0.006554
        theta_start = -3.1429/4
        theta = theta_start
        resultant = Vector()
        #print('inside follow wall left', len(self.laserdata.ranges))
        while i >= end:
            r = self.laserdata.ranges[i]
            r = Vector.sat_function(r)
            resultant = resultant + Vector.polar2cart(r,theta)
            theta = theta + theta_shift
            i = i-1
        #resultant.rotate(3.1429/2)

        goaltheta= -resultant.gettheta() + self.current_pose.theta
        #print('this is the angle made by the resultant vector', goaltheta, self.current_pose.theta)
        self.reach_theta(goaltheta)
        self.move_straight()
        self.rate.sleep()
    
    def reach_theta(self,goaltheta): ## This is for FollowWall aproach
        self.command.linear.x = 0
        #print('the current angle is', self.current_pose.theta)
        goaltheta_check = goaltheta
        goaltheta_follow = goaltheta
        temp = self.current_pose.theta
        if(goaltheta < 0):
            goaltheta_check = 2*3.1429 + goaltheta
        if(temp < 0):
            temp = 2*3.1429 + temp
        while mod(goaltheta_check - temp) > 0.04:
            if( self.current_pose.theta < 0):
                temp = 2*3.1429 + self.current_pose.theta
            else:
                temp = self.current_pose.theta   
            print(np.sign(goaltheta_follow - temp))
            self.command.angular.z = 0.5*np.sign(goaltheta_check - temp)
            if(mod(goaltheta_check - temp) > 3.14):
                self.command.angular.z = -self.command.angular.z
            #print("the current and goal thetas are", goaltheta_check, temp)
            self.vel_pub.publish(self.command)
            self.rate.sleep()
        self.command.angular.z = 0

    def cal_vec(self, r, theta):## Polar to cartesiann form
        temp = Vector3(r*math.cos(theta),r*math.sin(theta),0)
        return temp

    def update_twist(self): ## Update Twist according to the Goal Pose
        
        r = math.sqrt(math.pow((self.current_pose.x-self.goal_point.x)))
        return r

    def chooseFollowWall(self):
        #print('choosing the follow wall function, inside the choosefollow wall')
        sum = 0
        for i in range(360-80, 360):
            sum = sum + Vector.sat_function(self.laserdata.ranges[i])
        for i in range(361, 360+81):
            sum = sum - Vector.sat_function(self.laserdata.ranges[i])
        print(sum)
        if sum >0:
            return True
        
        return False

         
    def follow_waypoints(self):
        pass
        


def mod(data):
    if data < 0:
        return -data
    return data



def controller1():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    ebot1 = EbotController()
    ebot1.rate.sleep()
    #print("This is the first laser data", ebot1.laserdata.ranges)
    temp = np.arange(0,2*3.14, 0.2)
    for i in temp:
        ebot1.goalArray.append(Vector(i, 2*math.sin(i)*math.sin(i/2)))

    ebot1.goalArray.append(Vector(12.5,0,0))
    ebot1.rate.sleep()
    ebot1.final_controller()

def controller2():
    ebot2 = EbotController()
    ebot2.goal_point.x = 0
    ebot2.goal_point.y = 0
    ebot2.rate.sleep()
    
    ebot2.rate.sleep()
    if(ebot2.clearShot()):
        print("Way is clear, currentpose and goal point is", ebot2.current_pose,ebot2.goal_point)
    else:
        print("way isn't clear, current and goal point is ", ebot2.current_pose,ebot2.goal_point)
    #ebot2.vel_pub.publish(Twist())
    # spin() simply keeps python from exiting until this node is stopped
    
        
if __name__ == '__main__':
    rospy.init_node('ebot_controller', anonymous=True)
    print("this node is starting")
    controller1()
    rospy.spin()
