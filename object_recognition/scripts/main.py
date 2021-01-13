#! /usr/bin/env python
import rospy
from manipulator import Manipulator
from pose_settings import PoseSettings
from object_recognition.msg import ObjectPose

def detection_info_callback():
  pass

def main():

  # Initialise node
  node_name = 'sara'  # Super Automatic Robot Arm
  rospy.init_node(node_name, anonymous=True)

  # Wait for the object_detection node to publish the data
  rospy.Subscriber('detection_info', ObjectPose, detection_info_callback)
  # Init Manipulator and PoseSettings

  # can
  # battery 
  # glue

  sara = Manipulator()
  settings = PoseSettings()

  biscuits = settings.biscuits
  soap = settings.soap
  soap2 = settings.soap2

  sara.pick(soap)
  rospy.sleep(1.0)
  sara.place(soap)
  rospy.sleep(1.0)

  sara.pick(soap2)
  rospy.sleep(1.0)
  sara.place(soap2)
  rospy.sleep(1.0)

  # sara.pick(biscuits)
  # rospy.sleep(1.0)
  # sara.place(biscuits)
  # rospy.sleep(1.0)


if __name__ == '__main__':
  main()
