'''
Config Module 
Author: eYRC_SB_363
'''
from geometry_msgs.msg import Pose

class Config():
  """"""

  def __init__(self):
    """"""
    self.object_id = str()
    self.pick_pose = Pose()
    self.place_pose = Pose()
    self.support_surface = str()
    self.gripper_close = 0