#!/usr/bin/env python


# Import
import rospy
import pcl
import numpy as np
import copy
import inspect
import sensor_msgs.point_cloud2 as pc2

from pcl import PointCloud_PointXYZRGB
from sensor_msgs.msg import PointCloud2


class Config(object):
    """
    - Store the values of the hyperparamter of a particular object
    """
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class Settings(object):
    """
    Store the values of all the hyperparamters
    """
    def __init__(self):
        self.camera = self.init_camera()
        

    def init_filter(self):
        """
        Set the filters' paramters
        """
        voxel_filter = Config(
            leaf_size=0.005,
        )
        passthrough_filter = Config(
            axis='z',
            min_range=0.1,
            max_range=1.1
        )
        ransac_filter = Config(
            threshold=0.01
        )
        statistical_outlier_filter = Config(
            mean_k=50,
            std_dev_mul_thresh=1.0
        )
        
        return Config(
            voxel_filter=voxel_filter,
            passthrough_filter=passthrough_filter,
            ransac_filter=ransac_filter,
            statistical_outlier_filter=statistical_outlier_filter
        )


    def init_pointcloud(self):
        """
        Set pointcloud's parameters
        """
        pointcloud = Config(
            filters=self.init_filter(),
            save_pc_enable=False,
            analysis_enable=False
        )

        return pointcloud


    def init_camera(self):
        """
        Set the camera's paramters
        """
        poincloud = self.init_pointcloud()
        
        return Config(
            pointcloud=poincloud
        )


class Flag(object):
    """
    - Flag class, but of course! 
    - It can be of three types 
        - Level triggered - 'lvl_trig'
        - Positive edge triggered - 'pos_edg_trig'
        - Negative edge triggered - 'neg_edg_trig'
    """
    def __init__(self, update_func, kind, service_func, enable=True):
        """
        Arguments:
            - update_func: Function which will update this flag
                - Can be None
            - kind: String which tells the kind of this flag
            - service_func: The function to be executed when value=True
                - Can be None
        """
        self.kinds = ['lvl_trig', 'pos_edg_trig', 'neg_edg_trig']
        self.kind = None
        self.update_func = None
        self.flag = None
        self.flag_lag = None
        self.service_func = None
        self.value = False
        self.enable = enable
        self.set_kind(kind)
        self.initialise()
        self.set_update_func(update_func)
        self.set_service_func(service_func)


    def set_kind(self, kind):
        """
        - Check whether the kind type given is correct
        - Raise error if not correct  
        """
        if kind in self.kinds:
            self.kind = kind 
        
        else:
            raise Exception('kind type given to the flag is invalid. \nGiven ' + kind)


    def set_update_func(self, update_func):
        """
        - Check whether the update_func is valid or not
        - Update self.update_func if valid 
        """
        if update_func!=None:
            if callable(update_func):
                self.update_func = update_func

            else:
                raise Exception("The update_func argument is invalid")


    def set_service_func(self, service_func):
        """
        - Check whether the update_func is valid or not
        - Update self.update_func if valid 
        """
        if service_func!=None:
            if callable(service_func):
                self.service_func = service_func

            else:
                raise Exception("The service_func argument is invalid")


    def initialise(self):
        """
        - Initialise the required attributes based on the class type
        """
        if self.kind == 'lvl_trig':
            self.flag = False

        elif self.kind == 'pos_edg_trig':
            self.flag_lag = False
            self.flag = False

        elif self.kind == 'neg_edg_trig':
            self.flag_lag = False
            self.flag = False   


    def update(self):
        """
        Calls the update_flags function of the holder class
        """
        if self.update_func!=None:
            self.update_func()

        if self.kind == 'lvl_trig':
            self.value = self.flag
        elif self.kind == 'pos_edg_trig':
            self.value = ((not self.flag_lag) and self.flag)
        elif self.kind == 'neg_edg_trig':
            self.value = (self.flag_lag and (not self.flag))

        if self.kind!='lvl_trig':
            self.flag_lag = self.flag


    def service(self):
        """
        Execute the service function if any
        """
        if self.service_func!=None and self.value==True:
            self.service_func()


class Flags(object):
    """
    Store all the flags
    """
    def __init__(self):
        self.list = []

    
    def update(self):
        """
        - Update all the flags
        - Execute associated service, if any
        """
        # Update all the flag values
        for flag in self.list:
            if flag.enable:
                flag.update()

        # Execution is done after the update so that
        # it won't affect the state, before all the
        # flags get updated
        for flag in self.list:
            if flag.enable:
                flag.service()


    def add(self, update_func, kind, service_func, enable):
        """
        - Create new flag object 
        - Add it to the list
        - Return the new flag object
        """
        flag = Flag(
            update_func=update_func,
            kind=kind,
            service_func=service_func,
            enable=enable
        )
        self.list.append(flag)
        
        return flag


class ROSPointCloudMsg(PointCloud2):
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


class PCLPointCloudMsg(PointCloud_PointXYZRGB):
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


class Filters(object):
    """
    Implementation of filters
    """
    def __init__(self, settings):
        self.settings = settings


    def voxel_filter(self, point_cloud):
        """
        - It creates a 3D voxel grid over the input point cloud data.
        Then, in each voxel, all the points present will be approximated 
        with their centroid.
        - The bigger the leaf size less the information retained.
        """
        settings = self.settings.voxel_filter
        voxel_filter = point_cloud.make_voxel_grid_filter()
        voxel_filter.set_leaf_size(settings.leaf_size, settings.leaf_size, settings.leaf_size) 
        return voxel_filter.filter()


    def passthrough_filter(self, point_cloud):
        """
        Filter out points which are outside a specific range of a specific axis
        """
        settings = self.settings.passthrough_filter
        pass_filter = point_cloud.make_passthrough_filter()
        pass_filter.set_filter_field_name(settings.axis)
        pass_filter.set_filter_limits(settings.min_range, settings.max_range)
        return pass_filter.filter()

        
    def ransac_filter(self, point_cloud):
        """
        RANSAC - RANdom SAmple Consensus
        Consensus - A general agreement

        - Return inliers (plane) and outliers (not plane) 
        """
        settings = self.settings.ransac_filter
        # Return a pcl.Segmentation object with this object set as the input-cloud
        segmenter = point_cloud.make_segmenter()
        segmenter.set_model_type(pcl.SACMODEL_PLANE)
        segmenter.set_method_type(pcl.SAC_RANSAC)
        segmenter.set_distance_threshold(settings.threshold)

        inlier_indices, coefficients = segmenter.segment()
        inliers = point_cloud.extract(inlier_indices, negative = False)
        outliers = point_cloud.extract(inlier_indices, negative = True)

        return inliers, outliers


    def statistical_outlier_filter(self, point_cloud):
        """
        This filters out the statistical noise of the scene
        """
        settings = self.settings.statistical_outlier_filter
        noise_filter = point_cloud.make_statistical_outlier_filter()
        
        # Set the number of points (k) to use for mean distance estimation. 
        noise_filter.set_mean_k(settings.mean_k)

        # Any point with a mean distance larger than global (mean distance+x*std_dev)
        # will be considered outlier
        noise_filter.set_std_dev_mul_thresh(settings.std_dev_mul_thresh)

        noise_filter.set_negative(False)
        inliers = noise_filter.filter()
        noise_filter.set_negative(True)
        outliers = noise_filter.filter()

        return inliers, outliers


class PointCloud(object):
    """
    Connects ROS's PointCloud2 and PCL's pointcloud
    """
    def __init__(self, settings, flags):
        self.settings = settings
        self.flags = flags
        self.analysis_flag = None
        self.save_pc_flag = None
        self.topic = 'camera2/depth/points2'
        self.pc_subscriber = rospy.Subscriber(
            name=self.topic,
            data_class=PointCloud2,
            callback=self.callback
        )
        self.ros = ROSPointCloudMsg()
        self.pcl = PCLPointCloudMsg()
        self.filters = Filters(
            settings=self.settings.filters
        )
        self.init_flags()


    def callback(self, msg):
        """
        Updates the ros_pc data
        """
        self.ros.set(msg)
        self.analysis_flag.flag = True
        self.save_pc_flag.flag = True


    def init_flags(self):
        """
        Instantiate the flags
        """
        self.analysis_flag = self.flags.add(
            update_func=None, 
            kind='pos_edg_trig',
            service_func=self.analysis,
            enable=self.settings.analysis_enable
        )

        self.save_pc_flag = self.flags.add(
            update_func=None,
            kind='pos_edg_trig',
            service_func=None,
            enable=self.settings.save_pc_enable
        )
        self.save_pc_flag.flag = True


    def analysis(self):
        """
        Print details of the Pointcloud
        """
        self.ros.analysis()
        self.pcl.analysis()


    def ros_to_pcl(self):
        """
        Converts a ROS PointCloud2 message to a pcl PointXYZRGB

        Update:
            self.pcl: PCL PointCloud_PointXYZRGB message
        """
        # TODO: I guess this could be improved by using np.fromstring 
        # But mostly probably this is the most efficient since everywhere
        # this way it's implemented.

        # NOTE: What if value of msg change during execution of this function
        msg = copy.copy(self.ros)
        # construct a numpy record type equivalent to the point type of this cloud
        points_list = []

        for point in pc2.read_points(msg, skip_nans=True):
            # Print length of point - 4 - (x, y, z, rgb)
            points_list.append([point[0], point[1], point[2], point[3]])

        self.pcl.from_list(points_list)

    
    def preprocess(self):
        """
        Apply the filters to the pointcloud data
        """
        self.ros_to_pcl()
        
        if self.save_pc_flag.value:
            pcl.save(self.pcl, 'raw.pcd')
            rospy.loginfo('Saved raw')
        
        self.pcl = self.filters.voxel_filter(
            point_cloud=self.pcl
        )
        
        if self.save_pc_flag.value:
            pcl.save(self.pcl, 'voxel.pcd')
            rospy.loginfo('Saved voxel')
        self.pcl = self.filters.passthrough_filter(
            point_cloud=self.pcl
        )
        
        if self.save_pc_flag.value:
            pcl.save(self.pcl, 'passthrough.pcd')
            rospy.loginfo('Saved passthrough')
        inlier, outlier = self.filters.ransac_filter(
            point_cloud=self.pcl
        )

        if self.save_pc_flag.value:
            pcl.save(inlier, 'ransac_inlier.pcd')
            pcl.save(outlier, 'ransac_outlier.pcd')
            rospy.loginfo('Saved ransac')


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


def main():
    rospy.init_node(name='sara')
    settings = Settings()
    flags = Flags()
    camera = Camera(
        settings=settings.camera,
        flags=flags
    )
    rate = rospy.Rate(hz=1)
    rate.sleep()
    rospy.loginfo('Start')
    while True:
        time = rospy.Time.now()
        flags.update()
        camera.pc.preprocess()
        rospy.loginfo('Time taken: ' + str((rospy.Time.now() - time)/1e6))
        rate.sleep()


if __name__=='__main__':
    main()