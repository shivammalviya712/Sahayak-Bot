"""
Data structures

Author: eYRC_SB_363
"""


# Import
import rospy
from sensor_msgs.msg import PointCloud2
from pcl import PointCloud_PointXYZRGB
from pcl import PointCloud


class ROSPointCloud(PointCloud2):
    """
    Extra functionalities to PointCloud2
    """
    def __init__(self):
        PointCloud2.__init__(self)


    def set(self, msg):
        """
        Set the value of self to msg
        """
        self.header = msg.header
        self.height = msg.height
        self.width = msg.width
        self.fields = msg.fields
        self.is_bigendian = msg.is_bigendian
        self.point_step = msg.point_step
        self.row_step = msg.row_step
        self.data = msg.data
        self.is_dense = msg.is_dense

    
    def analysis(self):
        """
        Print details of the ROS PointCloud2
        """
        # https://robotics.stackexchange.com/questions/19290/what-is-the-definition-of-the-contents-of-pointcloud2

        # point_step is the length of a point in bytes
        # row_step is the length of a row in bytes
        # The PointField message simply tells ROS how the data is formatted
        # Offset tells at which byte the related field starts
        # In big-endian the higher order bytes of a point is stored 
        # in the lower memory address
        # is_dense is true if there are no invalid points

        # height = 480
        # width = 640
        # point_step = 32
        # row_step = point_step * width = 32 * 640 = 20480
        # len(data) = row_step * height = 20480 * 480 = 9830400
        # type(data): <type 'str'>

        rospy.loginfo("############### ROS's PointCloud ###############")
        rospy.loginfo(dir(self))
        rospy.loginfo('header: ' + str(self.header))
        rospy.loginfo('height: ' + str(self.height))
        rospy.loginfo('width: ' + str(self.width))
        rospy.loginfo('fields: ' + str(self.fields))
        rospy.loginfo('is_bigendian: ' + str(self.is_bigendian))
        rospy.loginfo('point_step: ' + str(self.point_step))
        rospy.loginfo('row_step: ' + str(self.row_step))
        rospy.loginfo('len(data): ' + str(len(self.data)))
        rospy.loginfo('type(data): ' + str(type(self.data)))
        rospy.loginfo('is_dense: ' + str(self.is_dense))


class PCLPointCloudXYZ(PointCloud):
    """
    Extra functionalities to PointCloud
    """
    def __init__(self):
        PointCloud.__init__(self)


    def analysis(self):
        """
        Print details of the PointCloud
        """
        # height = 1
        # width = 265118 < 480*640 = 307200 (Due to Nan values)
        # size = 265118 < 480*640 = 307200 (Due to Nan values)
        rospy.loginfo("########## PCL's PointCloud ##########")
        rospy.loginfo(dir(self))
        rospy.loginfo('size: ' + str(self.size))
        rospy.loginfo('height: ' + str(self.height))
        rospy.loginfo('width: ' + str(self.width))
        rospy.loginfo('is_dense: ' + str(self.is_dense))


class PCLPointCloudXYZRGB(PointCloud_PointXYZRGB):
    """
    Extra functionalities to PointCloud_PointXYZRGB
    """
    def __init__(self):
        PointCloud_PointXYZRGB.__init__(self)


    def analysis(self):
        """
        Print details of the PointCloud_PointXYZRGB
        """
        # height = 1
        # width = 265118 < 480*640 = 307200 (Due to Nan values)
        # size = 265118 < 480*640 = 307200 (Due to Nan values)
        rospy.loginfo("########## PCL's PointCloud ##########")
        rospy.loginfo(dir(self))
        rospy.loginfo('size: ' + str(self.size))
        rospy.loginfo('height: ' + str(self.height))
        rospy.loginfo('width: ' + str(self.width))
        rospy.loginfo('is_dense: ' + str(self.is_dense))
