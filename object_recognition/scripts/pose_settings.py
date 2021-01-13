'''
PoseSettings Module
Author: eYRC_SB_363
'''

from config import Config

class PoseSettings():
  """"""

  def __init__(self, centroids):
    """"""
    self.biscuits = Config()
    self.soap = Config()
    self.soap2 = Config()

    self.set_biscuits_conifg(centroids.biscuits)
    self.set_soap_config(centroids.soap)
    self.set_soap2_config(centroids.soap2)

  def set_biscuits_conifg(self, centroid):
    """"""
    self.biscuits.object_id = 'biscuits.dae_0'
    self.biscuits.support_surface = 'dropbox.dae-4'

    self.biscuits.pick_pose.orientation.x = 0.0032
    self.biscuits.pick_pose.orientation.y = 0.7071
    self.biscuits.pick_pose.orientation.z = -0.7071
    self.biscuits.pick_pose.orientation.w = 0.0034
    # self.biscuits.pick_pose.position.x = 0.540000  # centroid_x
    # self.biscuits.pick_pose.position.y = -0.240000  # centroid_y
    self.biscuits.pick_pose.position.x = centroid.x  # centroid_x
    self.biscuits.pick_pose.position.y = centroid.y  # centroid_y
    # centroid_z=0.609995;half ht.=0.07;ascend 0.18 for pickup
    self.biscuits.pick_pose.position.z = (centroid.z + 0.07) + 0.18

    self.biscuits.place_pose.position.x = 0.0350783554977
    self.biscuits.place_pose.position.y = -0.684259610707
    self.biscuits.place_pose.position.z = 0.940677042001
    self.biscuits.place_pose.orientation.x = -0.519362592958
    self.biscuits.place_pose.orientation.y = -0.459293822137
    self.biscuits.place_pose.orientation.z = 0.522168683518
    self.biscuits.place_pose.orientation.w = 0.496640260082

    self.biscuits.gripper_close = 0.26

  def set_soap_config(self, centroid):
    """"""
    self.soap.object_id = 'soap.dae_0'
    self.soap.support_surface = 'dropbox.dae'

    self.soap.pick_pose.orientation.x = 0.2928
    self.soap.pick_pose.orientation.y = 0.6426
    self.soap.pick_pose.orientation.z = -0.6392
    self.soap.pick_pose.orientation.w = -0.3043
    self.soap.pick_pose.position.x = centroid.position.x  # centroid_x
    self.soap.pick_pose.position.y = centroid.position.y  # centroid_y
    # centroid_z=0.610000;half ht.=0.05;ascend 0.18 for pickup
    self.soap.pick_pose.position.z = (centroid.position.z + 0.05) + 0.18

    self.soap.place_pose.position.x = 0.0173790624205
    self.soap.place_pose.position.y = 0.631378173894
    self.soap.place_pose.position.z = 1.0237827587
    self.soap.place_pose.orientation.x = -0.158614946142
    self.soap.place_pose.orientation.y = 0.746094027655
    self.soap.place_pose.orientation.z = -0.641531705499
    self.soap.place_pose.orientation.w = 0.0813761119608

    self.soap.gripper_close = 0.25

  def set_soap2_config(self, centroid):
    """"""
    self.soap2.object_id = 'soap2.dae_0'
    self.soap2.support_surface = 'dropbox.dae-4'

    self.soap2.pick_pose.orientation.x = -0.2388
    self.soap2.pick_pose.orientation.y = 0.6660
    self.soap2.pick_pose.orientation.z = -0.6620
    self.soap2.pick_pose.orientation.w = 0.2471
    self.soap2.pick_pose.position.x = centroid.position.x  # centroid_x
    self.soap2.pick_pose.position.y = centroid.position.y  # centroid_y
    # centroid_z=0.612497;half ht.=0.042;ascend 0.18 for pickup
    self.soap2.pick_pose.position.z = (centroid.position.z + 0.04) + 0.18

    self.soap2.place_pose.position.x = 0.0679432142948
    self.soap2.place_pose.position.y = -0.721176519918
    self.soap2.place_pose.position.z = 1.1098643132
    self.soap2.place_pose.orientation.x = -0.562639029023
    self.soap2.place_pose.orientation.y = 0.539285210375
    self.soap2.place_pose.orientation.z = -0.404748887091
    self.soap2.place_pose.orientation.w = 0.478316969477

    self.soap2.gripper_close = 0.40

