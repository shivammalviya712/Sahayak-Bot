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
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>

//PCL Libraries
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>

// My header files
#include <object_recognition/filters.hpp>
#include <object_recognition/recognition.hpp>
#include <object_recognition/ObjectPose.h>  

typedef pcl::PointXYZRGB PointType;

class Camera
{
    // Attributes
private:
    ros::NodeHandle _node_handle;
    std::string _pc_topic;
    ros::Subscriber _pc_sub;
    ros::Publisher _centroid_pub;
    ros::Publisher _centroid_pub_rviz;
    pcl::PointCloud<PointType>::Ptr _pcl_pc_ptr;
    std::vector<pcl::PointCloud<PointType>::Ptr> _ptrs_cluster;
    tf2_ros::Buffer _tfBuffer;
    tf2_ros::TransformListener* _tfListenerPtr;
    Filters _filters;
    Recognition recognizer;

    // Methods
public:
    Camera(ros::NodeHandle node_handle);
    ~Camera();
    void pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg_ptr);
    void analysis();
    void preprocess();
    void save_pc(pcl::PointCloud<PointType>::Ptr &pointcloud_ptr, std::string filename);
    void save_pc(std::vector<pcl::PointCloud<PointType>::Ptr> &ptrs_cluster, std::string filename);
    void detect();
};

#endif