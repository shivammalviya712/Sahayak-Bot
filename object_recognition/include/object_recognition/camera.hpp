/*
Author: eYRC_SB_363
*/

#ifndef _CAMERA_H
#define _CAMERA_H

// Standard libraries
#include <iostream>
#include <vector>
#include <string>

// ROS Libraries
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

//PCL Libraries
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>

// My header files
#include <object_recognition/filters.hpp>

typedef pcl::PointXYZRGB PointType;

class Camera
{
    // Attributes
private:
    ros::NodeHandle _node_handle;
    std::string _pc_topic;
    ros::Subscriber _pc_sub;
    pcl::PointCloud<PointType>::Ptr _pcl_pc_ptr;
    std::vector<pcl::PointCloud<PointType>::Ptr> _ptrs_cluster;
    Filters _filters;

    // Methods
public:
    Camera(ros::NodeHandle node_handle);
    ~Camera();
    void pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg_ptr);
    void analysis();
    void preprocess();
    void save_pc(pcl::PointCloud<PointType>::Ptr &pointcloud_ptr, std::string filename);
    void save_pc(std::vector<pcl::PointCloud<PointType>::Ptr> &ptrs_cluster, std::string filename);
};

#endif