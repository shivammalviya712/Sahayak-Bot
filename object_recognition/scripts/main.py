#! /usr/bin/env python
import rospy
from manipulator import Manipulator
from pose_settings import PoseSettings
from object_recognition.msg import ObjectPose

object_centroids = {}


def detection_info_callback(data):
  object_centroids[data.name] = data.pose


def main():

  # Initialise node
  node_name = 'sara'  # Super Automatic Robot Arm
  rospy.init_node(node_name, anonymous=True)
  sara = Manipulator()
  capture_pose = [0.12965710797413593, -0.2558361846975669, -
                  0.4842167126928663, -1.525936090630152, 5.7309490891563115, 1.807201005762695]
  sara.set_joint_angles(capture_pose)

  # Wait for the object_detection node to publish the data
  rospy.Subscriber('detection_info', ObjectPose, detection_info_callback)
  while len(object_centroids.keys()) < 3:
    rospy.sleep(0.5)

  print(object_centroids)
  # Init Manipulator and PoseSettings

  # can
  # battery
  # glue

  settings = PoseSettings(object_centroids)

  # coke = settings.coke

  # sara.pick(coke)
  # rospy.sleep(1.0)
  # sara.place(coke)
  # rospy.sleep(1.0)


if __name__ == '__main__':
  main()
