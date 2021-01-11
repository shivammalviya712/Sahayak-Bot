/*
Author: eYRC_SB_363
*/

#include <object_recognition/camera.hpp>
#include <object_recognition/camera_settings.hpp>

Camera::Camera(ros::NodeHandle node_handle)
{
    _node_handle = node_handle;
    _pc_topic = "camera2/depth/points2";
    _pcl_pc_ptr.reset(new pcl::PointCloud<PointType>());
    _pc_sub = _node_handle.subscribe(_pc_topic, 10, &Camera::pc_callback, this);
}

Camera::~Camera() {}

void Camera::pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg_ptr)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg_ptr, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *_pcl_pc_ptr);
}

void Camera::preprocess()
{
    // Save
    save_pc(_pcl_pc_ptr, "raw");
    
    // Voxel Filter
    _pcl_pc_ptr = _filters.voxel_filter(
        _pcl_pc_ptr,
        CameraSettings::Filters::VoxelFilter::leaf_size
    );
    // Save
    save_pc(_pcl_pc_ptr, "after_voxel");

    // Cropbox Filter
    _pcl_pc_ptr = _filters.cropbox_filter(
        _pcl_pc_ptr,
        CameraSettings::Filters::CropboxFilter::min_range,
        CameraSettings::Filters::CropboxFilter::max_range,
        CameraSettings::Filters::CropboxFilter::translation,
        CameraSettings::Filters::CropboxFilter::rotation
    );
    // Save
    save_pc(_pcl_pc_ptr, "after_cropbox");

    // Ransac filter
    _pcl_pc_ptr = _filters.ransac_filter(
        _pcl_pc_ptr,
        CameraSettings::Filters::RansacFilter::threshold
    );
    // Save
    save_pc(_pcl_pc_ptr, "after_ransac");

    // Clustering
    _ptrs_cluster = _filters.cluster_filter(
        _pcl_pc_ptr,
        CameraSettings::Filters::ClusterFilter::tolerance,
        CameraSettings::Filters::ClusterFilter::min_cluster_size,
        CameraSettings::Filters::ClusterFilter::max_cluster_size
    );
    // Save
    save_pc(_ptrs_cluster, "after_cluster");

}

void Camera::analysis()
{    
    std::cout << "Analysis of Pointcloud\n  "
        << "Width: " << _pcl_pc_ptr->width << "\n"
        << "Height: " << _pcl_pc_ptr->height << "\n"
        << std::endl;
}

void Camera::save_pc(pcl::PointCloud<PointType>::Ptr &pointcloud_ptr, std::string filename)
{
    filename = filename + ".pcd";
    if(
        pcl::io::savePCDFileASCII (
            "/home/shivam/Projects/eYRC/catkin_ws/src/object_recognition/point_cloud/" + filename,
            *_pcl_pc_ptr
        )>=0
    )
    {
        std::cout << "Saved " << filename << "\n" << std::endl;
    }
    else
    {
        std::cout << "Not saved " << filename << "\n" << std::endl;
    }
    
}

void Camera::save_pc(std::vector<pcl::PointCloud<PointType>::Ptr> &ptrs_cluster, std::string filename)
{
    pcl::PointCloud<PointType>::Ptr cloud_cluster_ptr;
    for (int j=0; j<ptrs_cluster.size(); j++)
    {
        cloud_cluster_ptr = ptrs_cluster[j];
        std::string clustername = filename + std::to_string(j) + ".pcd";
        if(
            pcl::io::savePCDFileASCII (
                "/home/shivam/Projects/eYRC/catkin_ws/src/object_recognition/point_cloud/" + clustername,
                *cloud_cluster_ptr
            )>=0
        )
        {
            std::cout << "Saved " << clustername << std::endl;
        }
        else
        {
            std::cout << "Not saved " << clustername << std::endl;
        }
    }
    std::cout << "" << std::endl;
}