"""
Filters

Author: eYRC_SB_363
"""


# Import
import pcl
import numpy as np
import rospy

# Import from files
from utils import PCLPointCloudXYZ, ROSPointCloud, PCLPointCloudXYZRGB


class Filters(object):
    """
    Implementation of filters
    """
    def __init__(self):
        pass


    def voxel_filter(self, point_cloud, leaf_size, save_flag):
        """
        - It creates a 3D voxel grid over the input point cloud data.
        Then, in each voxel, all the points present will be approximated 
        with their centroid.
        - The bigger the leaf size less the information retained.
        """
        voxel_filter = point_cloud.make_voxel_grid_filter()
        voxel_filter.set_leaf_size(leaf_size, leaf_size, leaf_size)
        point_cloud = voxel_filter.filter()
        if save_flag:
            pcl.save(point_cloud, 'voxel.pcd')
            rospy.loginfo('Saved voxel')
            rospy.loginfo('size: ' + str(point_cloud.size))
            rospy.loginfo('height: ' + str(point_cloud.height))
            rospy.loginfo('width: ' + str(point_cloud.width))
            rospy.loginfo('')
 
        return point_cloud


    def passthrough_filter(self, point_cloud, axis, min_range, max_range, save_flag):
        """
        Filter out points which are outside a specific range of a specific axis
        """
        pass_filter = point_cloud.make_passthrough_filter()
        pass_filter.set_filter_field_name(axis)
        pass_filter.set_filter_limits(min_range, max_range)
        pass_filter.filter()
        if save_flag:
            pcl.save(point_cloud, 'passthrough.pcd')
            rospy.loginfo('Saved passthrough')
            rospy.loginfo('size: ' + str(point_cloud.size))
            rospy.loginfo('height: ' + str(point_cloud.height))
            rospy.loginfo('width: ' + str(point_cloud.width))
            rospy.loginfo('')
 
        return point_cloud

        
    def cropbox(self, point_cloud, translation, rotation, ranges, save_flag):
        """
        This method is similar to the pass-through filter, 
        but instead of using three pass-through filters in series, 
        you can use one CropBox filter.
        """
        clipper = point_cloud.make_cropbox()
        clipper.set_Translation(translation[0], translation[1], translation[2])
        clipper.set_Rotation(rotation[0], rotation[1], rotation[2])
        clipper.set_MinMax(
            ranges[0], ranges[1], ranges[2], ranges[3],
            ranges[4], ranges[5], ranges[6], ranges[7]
        )
        point_cloud = clipper.filter()
        if save_flag:
            pcl.save(point_cloud, 'cropbox.pcd')
            rospy.loginfo('Saved cropbox')
            rospy.loginfo('size: ' + str(point_cloud.size))
            rospy.loginfo('height: ' + str(point_cloud.height))
            rospy.loginfo('width: ' + str(point_cloud.width))
            rospy.loginfo('')

        return point_cloud


    def ransac_filter(self, point_cloud, threshold, save_flag):
        """
        RANSAC - RANdom SAmple Consensus
        Consensus - A general agreement

        - Return pointcloud without pointcloud making plane 
        """
        # Return a pcl.Segmentation object with this object set as the input-cloud
        segmenter = point_cloud.make_segmenter()
        segmenter.set_model_type(pcl.SACMODEL_PLANE)
        segmenter.set_method_type(pcl.SAC_RANSAC)
        segmenter.set_distance_threshold(threshold)

        inlier_indices, coefficients = segmenter.segment()
        # inliers = point_cloud.extract(inlier_indices, negative = False)
        point_cloud = point_cloud.extract(inlier_indices, negative = True)

        if save_flag:
            pcl.save(point_cloud, 'ransac.pcd')
            rospy.loginfo('Saved ransac')
            rospy.loginfo('size: ' + str(point_cloud.size))
            rospy.loginfo('height: ' + str(point_cloud.height))
            rospy.loginfo('width: ' + str(point_cloud.width))
            rospy.loginfo('')

        return point_cloud


    def statistical_outlier_filter(self, point_cloud, mean_k, std_dev_mul_thresh, save_flag):
        """
        This filters out the statistical noise of the scene

        - Return pointcloud without noise
        """
        noise_filter = point_cloud.make_statistical_outlier_filter()
        
        # Set the number of points (k) to use for mean distance estimation. 
        noise_filter.set_mean_k(mean_k)

        # Any point with a mean distance larger than global (mean distance+x*std_dev)
        # will be considered outlier
        noise_filter.set_std_dev_mul_thresh(std_dev_mul_thresh)

        noise_filter.set_negative(False)
        point_cloud = noise_filter.filter()
        # noise_filter.set_negative(True)
        # outliers = noise_filter.filter()

        if save_flag:
            pcl.save(point_cloud, 'statistical_outlier.pcd')
            rospy.loginfo('Saved statistical_outlier')
            rospy.loginfo('size: ' + str(point_cloud.size))
            rospy.loginfo('height: ' + str(point_cloud.height))
            rospy.loginfo('width: ' + str(point_cloud.width))
            rospy.loginfo('')

        return point_cloud


    def euclidean_cluster_extraction(self, point_cloud_xyz, cluster_tolerance, 
                                     max_cluster_size, min_cluster_size, save_flag):
        """
        Separate clusters from the pointcloud
        """
        tree = point_cloud_xyz.make_kdtree()
        ec = point_cloud_xyz.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(0.02)
        ec.set_MinClusterSize(min_cluster_size)
        ec.set_MaxClusterSize(max_cluster_size)
        ec.set_SearchMethod(tree)
        cluster_indices = ec.Extract()
        clusters = []

        for j, indices in enumerate(cluster_indices):
            points = [0 for i in range(len(indices))]
            for i, index in enumerate(indices):
                points[i] = point_cloud_xyz[index]

            clusters.append(PCLPointCloudXYZ())
            clusters[j].from_list(points)

            if save_flag:
                pcl.save(clusters[j], 'euclidean_cluster_' + str(j) + '.pcd')
                rospy.loginfo('Saved euclidean_cluster_' + str(j))
                rospy.loginfo('size: ' + str(clusters[j].size))
                rospy.loginfo('height: ' + str(clusters[j].height))
                rospy.loginfo('width: ' + str(clusters[j].width))
                rospy.loginfo('')

        return clusters
