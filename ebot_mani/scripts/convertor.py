"""
Methods to convert messages from one type to another

Author: eYRC_SB_363
"""

# Import 
import copy

from utils import PCLPointCloudXYZ, PCLPointCloudXYZRGB, ROSPointCloud 
import sensor_msgs.point_cloud2 as pc2


class Convertor(object):
    """
    Convert different messages types to one another
    """
    def __init__(self):
        pass


    def ros_to_pclxyzrgb(self, ros_pc):
        """
        Converts a ROS PointCloud2 message to a pcl PointXYZRGB
        """
        # TODO: I guess this could be improved by using np.fromstring 
        # But mostly probably this is the most efficient since everywhere
        # this way it's implemented.

        # NOTE: What if value of msg change during execution of this function
        msg = copy.copy(ros_pc)
        # construct a numpy record type equivalent to the point type of this cloud
        points_list = []
        pcl_xyzrgb = PCLPointCloudXYZRGB()
        for point in pc2.read_points(msg, skip_nans=True):
            # Print length of point - 4 - (x, y, z, rgb)
            points_list.append([point[0], point[1], point[2], point[3]])
        pcl_xyzrgb.from_list(points_list)

        return pcl_xyzrgb

    
    def ros_to_pclxyz(self, ros_pc):
        """
        Converts a ROS PointCloud2 message to a pcl PointCloud
        """
        # TODO: I guess this could be improved by using np.fromstring 
        # But mostly probably this is the most efficient since everywhere
        # this way it's implemented.

        # NOTE: What if value of msg change during execution of this function
        msg = copy.copy(ros_pc)
        # construct a numpy record type equivalent to the point type of this cloud
        points_list = []
        pcl_xyz = PCLPointCloudXYZ()
        for point in pc2.read_points(msg, skip_nans=True):
            # Print length of point - 4 - (x, y, z, rgb)
            points_list.append([point[0], point[1], point[2]])
        pcl_xyz.from_list(points_list)

        return pcl_xyz


    def xyzrgb_to_xyz(self, pcl_xyzrgb):
        """
        Convert PointCloud_PointXYZRGB_PointXYZ() to PointCloud()
        """
        points_list = []
        pcl_xyz = PCLPointCloudXYZ()
        for data in pcl_xyzrgb:
            points_list.append([data[0], data[1], data[2]])
        pcl_xyz.from_list(points_list)
        
        return pcl_xyz