"""
Pointcloud

Author: eYRC_SB_363
"""


# Import
import rospy
import copy
import inspect
import pcl
import numpy as np
import tf2_ros
import tf2_geometry_msgs

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

# Import from files
from utils import PCLPointCloudXYZ, ROSPointCloud, PCLPointCloudXYZRGB
from filters import Filters
from convertor import Convertor


class PointCloud(object):
    """
    Connects ROS's PointCloud2 and PCL's pointcloud
    """
    def __init__(self, settings, flags):
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
        self.detection_info_pub = rospy.Publisher(
            'detection_info', Pose, queue_size=10)
        self.detection_info_pub_stamped = rospy.Publisher(
            'detection_info_stamped', PoseStamped, queue_size=10)
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
        self.ros = ROSPointCloud()
        self.pcl_xyzrgb = PCLPointCloudXYZRGB()
        self.pcl_xyz = PCLPointCloudXYZ()
        self.filters = Filters()
        self.convertor = Convertor()
        self.clusters = []
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
        self.pcl_xyz = self.convertor.ros_to_pclxyz(self.ros)
        self.pcl_xyz.analysis()


    def preprocess(self):
        """
        Apply the filters to the pointcloud data
        """
        self.pcl_xyz = self.convertor.ros_to_pclxyz(self.ros)
        
        # Downsample
        self.pcl_xyz = self.filters.voxel_filter(
            point_cloud=self.pcl_xyz, 
            leaf_size=self.settings.filters.voxel_filter.leaf_size,
            save_flag=self.save_pc_flag.value
        )
        
        # Allow pointcloud which are only within the range
        self.pcl_xyz = self.filters.cropbox(
            point_cloud=self.pcl_xyz,
            translation=self.settings.filters.cropbox_filter.translation,
            rotation=self.settings.filters.cropbox_filter.rotation,
            ranges=self.settings.filters.cropbox_filter.ranges,
            save_flag=self.save_pc_flag.value
        )
        
        # Remove pointcloud making plane
        self.pcl_xyz = self.filters.ransac_filter(
            point_cloud=self.pcl_xyz,
            threshold=self.settings.filters.ransac_filter.threshold,
            save_flag=self.save_pc_flag.value
        )

        # Statistical outlier filter
        self.pcl_xyz = self.filters.statistical_outlier_filter(
            point_cloud=self.pcl_xyz,
            mean_k=self.settings.filters.statistical_outlier_filter.mean_k,
            std_dev_mul_thresh=self.settings.filters.statistical_outlier_filter.std_dev_mul_thresh,
            save_flag=self.save_pc_flag.value
        )

        # Separating clusters
        self.clusters = self.filters.euclidean_cluster_extraction(
            point_cloud_xyz=self.pcl_xyz,
            cluster_tolerance=self.settings.filters.euclidean_cluster_filter.cluster_tolerance,
            max_cluster_size=self.settings.filters.euclidean_cluster_filter.max_cluster_size,
            min_cluster_size=self.settings.filters.euclidean_cluster_filter.min_cluster_size,
            save_flag=self.save_pc_flag.value
        )

    def localize_clusters(self):
        try:
            transform = self.tfBuffer.lookup_transform(
                'base_link', 'camera_depth_frame2', rospy.Time())
            print('Transform: ' + str(transform.transform.translation))
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            print('TF Error!!')

        for index, cluster in enumerate(self.clusters):
            # np.save('cluster_'+str(index)+'.npy', np.asarray(cluster))
            points = np.asarray(cluster)
            min_boundary = np.min(points, axis=0)
            max_boundary = np.max(points, axis=0)
            centroid = (min_boundary+max_boundary)/2

            cluster_pose_stamped = PoseStamped()
            cluster_pose_stamped.header.stamp = rospy.Time.now()
            cluster_pose_stamped.header.frame_id = 'camera_depth_frame2'
            cluster_pose_stamped.pose.position.x = centroid[0]
            cluster_pose_stamped.pose.position.y = centroid[1]
            cluster_pose_stamped.pose.position.z = centroid[2]
            # Transform so the pose is w.r.to the base link
            # Reference: https://piazza.com/class/kftilbrk6dm7ea?cid=1120
            cluster_pose_stamped = tf2_geometry_msgs.do_transform_pose(
                cluster_pose_stamped, transform
            )

            cluster_pose = Pose()
            cluster_pose.position = cluster_pose_stamped.pose.position
            self.detection_info_pub.publish(cluster_pose)
            self.detection_info_pub_stamped.publish(cluster_pose_stamped)