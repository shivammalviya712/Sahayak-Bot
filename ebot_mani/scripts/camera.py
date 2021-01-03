"""
Camera

Author: eYRC_SB_363
"""
# Import from files
from pointcloud import PointCloud


class Camera(object):
    """
    Interface for camera
    """

    def __init__(self, settings, flags):
        self.settings = settings
        self.flags = flags
        self.pc = PointCloud(
            settings=self.settings.pointcloud,
            flags=self.flags
        )

    
    def analysis(self):
        """
        Print all the messages
        """
        self.pc.analysis()
