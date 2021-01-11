/*
Author: eYRC_SB_363
*/

#ifndef _RECOGNITION_H
#define _RECOGNITION_H

// Standard libraries
#include <iostream>
#include <string>
#include <vector>
#include <tuple>

// // ROS Libraries

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

class Recognition
{
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
        std::string scene_filename);
    ~Recognition();
    void pointcloud_to_centroid();
    pcl::PointCloud<NormalType>::Ptr compute_normals(
        pcl::PointCloud<PointType>::Ptr &pointcloud_ptr,
        int k_nearest_neighbour
    );
    pcl::PointCloud<PointType>::Ptr extract_keypoints(
        pcl::PointCloud<PointType>::Ptr &pointcloud_ptr,
        float sampling_size    
    );
    pcl::PointCloud<DescriptorType>::Ptr compute_descriptors(
        pcl::PointCloud<PointType>::Ptr &model,
        pcl::PointCloud<PointType>::Ptr &model_keypoints,
        pcl::PointCloud<NormalType>::Ptr &model_normals,
        float radius_search
    );
    pcl::CorrespondencesPtr find_correspondences(
        pcl::PointCloud<DescriptorType>::Ptr &model_descriptors,
        pcl::PointCloud<DescriptorType>::Ptr &scene_descriptors
    );
    pcl::PointCloud<RFType>::Ptr compute_reference_frames(
        pcl::PointCloud<PointType>::Ptr &pointcloud_ptr,
        pcl::PointCloud<PointType>::Ptr &pointcloud_keypoints_ptr,
        pcl::PointCloud<NormalType>::Ptr &pointcloud_normals_ptr,
        float rf_radius
    );
    std::tuple<
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>,
        std::vector<pcl::Correspondences>
    > hough3D(
        pcl::CorrespondencesPtr &model_scene_corrs,
        float bin_size,
        float threshold
    );
};
#endif