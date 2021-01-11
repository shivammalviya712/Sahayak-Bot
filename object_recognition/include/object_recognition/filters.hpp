/*
Author: eYRC_SB_363
*/

#ifndef _FILTERS_H
#define _FILTERS_H

// Standard libraries
#include <iostream>
#include <vector>
#include <string>

// ROS Libraries
#include <ros/ros.h>
#include <ros/console.h>

// PCL Libraries
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>

typedef pcl::PointXYZRGB PointType;

class Filters 
{
// Attributes
public:

// Methods
public:
    Filters();
    ~Filters();
    pcl::PointCloud<PointType>::Ptr voxel_filter(
        pcl::PointCloud<PointType>::Ptr &pointcloud_ptr,
        std::vector<float> leaf_size
    );
    pcl::PointCloud<PointType>::Ptr cropbox_filter(
        pcl::PointCloud<PointType>::Ptr &pointcloud_ptr,
        Eigen::Vector4f min_range,
        Eigen::Vector4f max_range,
        Eigen::Vector3f translation,
        Eigen::Vector3f rotation
    );
    pcl::PointCloud<PointType>::Ptr ransac_filter(
        pcl::PointCloud<PointType>::Ptr &pointcloud_ptr,
        float threshold
    );
    pcl::PointCloud<PointType>::Ptr statistical_outlier_filter(
        pcl::PointCloud<PointType>::Ptr &pointcloud_ptr
    );
    std::vector<pcl::PointCloud<PointType>::Ptr> cluster_filter(
        pcl::PointCloud<PointType>::Ptr &pointcloud_ptr,
        float tolerance,
        int min_cluster_size,
        long max_cluster_size
    );
};

#endif