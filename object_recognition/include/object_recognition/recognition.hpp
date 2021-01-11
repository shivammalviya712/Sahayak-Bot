/*
Author: eYRC_SB_363
*/

#ifndef _RECOGNITION_H
#define _RECOGNITION_H

// Standard libraries
#include <iostream>
#include <string>
#include <vector>

// ROS Libraries
#include <ros/console.h>
#include <ros/ros.h>

// PCL Libraries
#include <pcl/io/pcd_io.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>


typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;
typedef pcl::PointXYZRGB PointType;

class Recognition {
  // Attributes
private:
  float _model_ss = 0.01f;
  float _scene_ss = 0.03f;
  float _rf_rad = 0.015f;
  float _descr_rad = 0.02f;
  float _cg_size = 0.01f;
  float _cg_thresh = 5.0f;
  pcl::PointCloud<PointType>::Ptr _model;
  pcl::PointCloud<PointType>::Ptr _model_keypoints;
  pcl::PointCloud<NormalType>::Ptr _model_normals;
  pcl::PointCloud<DescriptorType>::Ptr _model_descriptors;
  pcl::PointCloud<PointType>::Ptr _scene;
  pcl::PointCloud<PointType>::Ptr _scene_keypoints;
  pcl::PointCloud<NormalType>::Ptr _scene_normals;
  pcl::PointCloud<DescriptorType>::Ptr _scene_descriptors;
// Methods
public:
  Recognition(
    std::string model_filename,
    std::string scene_filename
  );
  // ~Recognition();
  void pointcloud_to_centroid();
};
#endif