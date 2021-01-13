/*
Author: eYRC_SB_363
*/

#include <object_recognition/filters.hpp>

Filters::Filters() {};

Filters::~Filters() {};

// Voxel filter: Downsamples the pointcloud
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
    std::cout << "No. of points after voxel: " << filtered_cloud_ptr->size() << std::endl;

    return filtered_cloud_ptr;
}

// Cropbox filter: Remove the points outside the specified range
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
    std::cout << "No. of points after cropbox: " << filtered_cloud_ptr->size() << std::endl;

    return filtered_cloud_ptr;
}

// RANSAC filter: Separate the pointcloud in inliers and outliers, according to the specified model
pcl::PointCloud<PointType>::Ptr Filters::ransac_filter(
    pcl::PointCloud<PointType>::Ptr &pointcloud_ptr,
    float threshold
)
{
    std::cout << "No. of points before ransac: " << pointcloud_ptr->size() << std::endl;
    pcl::PointCloud<PointType>::Ptr filtered_cloud_ptr(new pcl::PointCloud<PointType>);
    pcl::SACSegmentation<PointType> sac;
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inlier_indices_ptr(new pcl::PointIndices);
	pcl::PointCloud<PointType>::Ptr outlier_ptr(new pcl::PointCloud<PointType>);
 
	sac.setInputCloud(pointcloud_ptr);
	sac.setMethodType(pcl::SAC_RANSAC);
	sac.setModelType(pcl::SACMODEL_PLANE);
	sac.setDistanceThreshold(threshold); 
	sac.segment(*inlier_indices_ptr, *coefficients);

    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(pointcloud_ptr);
    extract.setIndices(inlier_indices_ptr);
    extract.setNegative(true);
    extract.filter(*filtered_cloud_ptr);
    std::cout << "No. of points after ransac: " << filtered_cloud_ptr->size() << std::endl;

    return filtered_cloud_ptr;
}

// Remove the outlier points
pcl::PointCloud<PointType>::Ptr Filters::statistical_outlier_filter(
    pcl::PointCloud<PointType>::Ptr &pointcloud_ptr
)
{
    std::cout << "Statistical outlier filter isn't implemented" << "\n" << std::endl;
}

std::vector<pcl::PointCloud<PointType>::Ptr> Filters::cluster_filter(
    pcl::PointCloud<PointType>::Ptr &pointcloud_ptr,
    float tolerance,
    int min_cluster_size,
    long max_cluster_size
)
{
    std::cout << "No. of points before cluster: " << pointcloud_ptr->size() << std::endl;
    std::vector<pcl::PointCloud<PointType>::Ptr> ptrs_cluster;
    pcl::search::KdTree<PointType>::Ptr tree_ptr(new pcl::search::KdTree<PointType>);
    pcl::EuclideanClusterExtraction<PointType> ec_ext;
    std::vector<pcl::PointIndices> clusters_indices;

    tree_ptr->setInputCloud(pointcloud_ptr);
    ec_ext.setClusterTolerance(tolerance);
    ec_ext.setMinClusterSize(min_cluster_size);
    ec_ext.setMaxClusterSize(max_cluster_size);
    ec_ext.setSearchMethod(tree_ptr);
    ec_ext.setInputCloud(pointcloud_ptr);
    ec_ext.extract(clusters_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = clusters_indices.begin(); it != clusters_indices.end(); ++it)
    {   
        pcl::PointCloud<PointType>::Ptr cloud_cluster_ptr(new pcl::PointCloud<PointType>);
        for (std::vector<int>::const_iterator p_it = it->indices.begin (); p_it != it->indices.end (); ++p_it)
            cloud_cluster_ptr->push_back ((*pointcloud_ptr)[*p_it]);
        cloud_cluster_ptr->width = cloud_cluster_ptr->size();
        cloud_cluster_ptr->height = 1;
        cloud_cluster_ptr->is_dense = true;
        ptrs_cluster.push_back(cloud_cluster_ptr);
        std::cout << "No. of points in cluster " << j << ": " << ptrs_cluster[j]->size() << std::endl;
        j++;
    }

    return ptrs_cluster;

}

