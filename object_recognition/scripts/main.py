#! /usr/bin/env python
import rospy
from manipulator import Manipulator
from pose_settings import PoseSettings
from object_recognition.msg import ObjectPose

object_centroids = {}
num_picked_objects = 0

def detection_info_callback(data):
  object_centroids[data.name] = data.pose

def main():

  # Initialise node
  node_name = 'sara'  # Super Automatic Robot Arm
  rospy.init_node(node_name, anonymous=True)
  sara = Manipulator()
  settings = PoseSettings()

  # Wait for the object_detection node to publish the data
  rospy.Subscriber('detection_info', ObjectPose, detection_info_callback)
  while len(object_centroids.keys()) < 3:
    rospy.sleep(0.5)
  
  print(object_centroids)

  # can
  # battery 
  # glue


  # biscuits = settings.biscuits
  # soap = settings.soap
  # soap2 = settings.soap2

  # sara.pick(soap)
  # rospy.sleep(1.0)
  # sara.place(soap)
  # rospy.sleep(1.0)

  # sara.pick(soap2)
  # rospy.sleep(1.0)
  # sara.place(soap2)
  # rospy.sleep(1.0)

  # sara.pick(biscuits)
  # rospy.sleep(1.0)
  # sara.place(biscuits)
  # rospy.sleep(1.0)


if __name__ == '__main__':
  main()
