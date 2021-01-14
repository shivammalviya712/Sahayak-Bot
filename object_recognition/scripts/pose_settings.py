'''
PoseSettings Module
Author: eYRC_SB_363
'''

from config import Config

class PoseSettings:
  """"""

  def __init__(self, centroids):
    """"""
    self.coke = Config()
    self.battery = Config()
    self.glue = Config()
    self.set_coke_config(centroids['Coke Can'].pose)
    self.set_battery_config(centroids['Battery'].pose)
    self.set_glue_config(centroids['Glue'].pose)
    
  def set_coke_config(self, centroid):
    ''''''
    self.coke.object_id = 'Coke Can'
    self.coke.support_surface = 'dropbox.dae_0'

    self.coke.pick_pose.orientation.x = 0.0032
    self.coke.pick_pose.orientation.y = 0.7071
    self.coke.pick_pose.orientation.z = -0.7071
    self.coke.pick_pose.orientation.w = 0.0034
    self.coke.pick_pose.position.x = centroid.position.x  # centroid_x
    self.coke.pick_pose.position.y = centroid.position.y  # centroid_y
    self.coke.pick_pose.position.z = 0.87

    self.coke.place_pose.position.x = 0.0350783554977
    self.coke.place_pose.position.y = -0.684259610707
    self.coke.place_pose.position.z = 0.940677042001
    self.coke.place_pose.orientation.x = -0.519362592958
    self.coke.place_pose.orientation.y = -0.459293822137
    self.coke.place_pose.orientation.z = 0.522168683518
    self.coke.place_pose.orientation.w = 0.496640260082
    self.coke.gripper_close = 0.26


  def set_battery_config(self, centroid):
    '''
    - Battery is soap2!!!
    '''
    self.battery.object_id = 'Battery'
    self.battery.support_surface = 'dropbox.dae_0'

    self.battery.pick_pose.orientation.x = 0.0032
    self.battery.pick_pose.orientation.y = 0.7071
    self.battery.pick_pose.orientation.z = -0.7071
    self.battery.pick_pose.orientation.w = 0.0034
    self.battery.pick_pose.position.x = centroid.position.x  # centroid_x
    self.battery.pick_pose.position.y = centroid.position.y  # centroid_y
    self.battery.pick_pose.position.z = 0.87

    self.battery.place_pose.position.x = 0.0350783554977
    self.battery.place_pose.position.y = -0.684259610707
    self.battery.place_pose.position.z = 0.940677042001
    self.battery.place_pose.orientation.x = -0.519362592958
    self.battery.place_pose.orientation.y = -0.459293822137
    self.battery.place_pose.orientation.z = 0.522168683518
    self.battery.place_pose.orientation.w = 0.496640260082
    self.battery.gripper_close = 0.26


  def set_glue_config(self, centroid):
    ''''''
    self.glue.object_id = 'Glue'
    self.glue.support_surface = 'dropbox.dae_0'

    self.glue.pick_pose.orientation.x = 0.0032
    self.glue.pick_pose.orientation.y = 0.7071
    self.glue.pick_pose.orientation.z = -0.7071
    self.glue.pick_pose.orientation.w = 0.0034
    self.glue.pick_pose.position.x = centroid.position.x  # centroid_x
    self.glue.pick_pose.position.y = centroid.position.y  # centroid_y
    self.glue.pick_pose.position.z = 0.87

    self.glue.place_pose.position.x = 0.0350783554977
    self.glue.place_pose.position.y = -0.684259610707
    self.glue.place_pose.position.z = 0.940677042001
    self.glue.place_pose.orientation.x = -0.519362592958
    self.glue.place_pose.orientation.y = -0.459293822137
    self.glue.place_pose.orientation.z = 0.522168683518
    self.glue.place_pose.orientation.w = 0.496640260082
    self.glue.gripper_close = 0.26


  