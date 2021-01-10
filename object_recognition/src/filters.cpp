/*
Author: eYRC_SB_363
*/

#include <object_recognition/filters.hpp>

Filters::Filters() {};

Filters::~Filters() {};

pcl::PointCloud<PointType>::Ptr Filters::voxel_filter(
    pcl::PointCloud<PointType>::Ptr &pointcloud_ptr,
    std::vector<float> leaf_size
)
{
    std::cout << "No. of points before voxel: " << pointcloud_ptr->size() << std::endl;
    pcl::PointCloud<PointType>::Ptr filtered_cloud_ptr(new pcl::PointCloud<PointType>());
    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setInputCloud(pointcloud_ptr);
    voxel_filter.setLeafSize(leaf_size[0], leaf_size[1], leaf_size[2]);
    voxel_filter.filter(*filtered_cloud_ptr);
    std::cout << "No. of points after voxel: " << filtered_cloud_ptr->size() << "\n" << std::endl;

    return filtered_cloud_ptr;
}

pcl::PointCloud<PointType>::Ptr Filters::cropbox_filter(
        pcl::PointCloud<PointType>::Ptr &pointcloud_ptr,
        Eigen::Vector4f min_range,
        Eigen::Vector4f max_range,
        Eigen::Vector3f translation,
        Eigen::Vector3f rotation
)
{
    std::cout << "No. of points before cropbox: " << pointcloud_ptr->size() << std::endl;
    pcl::PointCloud<PointType>::Ptr filtered_cloud_ptr(new pcl::PointCloud<PointType>());
    pcl::CropBox<PointType> cropbox_filter;
    cropbox_filter.setInputCloud(pointcloud_ptr);
    cropbox_filter.setMin(min_range);
    cropbox_filter.setMax(max_range);
    cropbox_filter.setTranslation(translation);
    cropbox_filter.setRotation(rotation);
    cropbox_filter.filter(*filtered_cloud_ptr);
    std::cout << "No. of points after cropbox: " << filtered_cloud_ptr->size() << "\n" << std::endl;

    return filtered_cloud_ptr;
}

