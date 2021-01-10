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

void Camera::analysis()
{
    if (_pcl_pc_ptr->size()>0)
    {
        std::cout << "Analysis of Pointcloud\n  "
            << "Width: " << _pcl_pc_ptr->width << "\n"
            << "Height: " << _pcl_pc_ptr->height << "\n"
            << std::endl;
    }
}

void Camera::preprocess()
{
    // Save
    save_pc(_pcl_pc_ptr, "raw.pcd");
    
    // Voxel Filter
    _pcl_pc_ptr = _filters.voxel_filter(
        _pcl_pc_ptr,
        CameraSettings::Filters::VoxelFilter::leaf_size
    );

    // Save
    save_pc(_pcl_pc_ptr, "after_voxel.pcd");

    // Cropbox Filter
    _pcl_pc_ptr = _filters.cropbox_filter(
        _pcl_pc_ptr,
        CameraSettings::Filters::CropboxFilter::min_range,
        CameraSettings::Filters::CropboxFilter::max_range,
        CameraSettings::Filters::CropboxFilter::translation,
        CameraSettings::Filters::CropboxFilter::rotation
    );
    
    // Save
    save_pc(_pcl_pc_ptr, "after_cropbox.pcd");
}

void Camera::save_pc(pcl::PointCloud<PointType>::Ptr &pointcloud_ptr, std::string filename)
{
    if(
        pcl::io::savePCDFileASCII (
            "/home/shivam/Projects/eYRC/catkin_ws/src/object_recognition/point_cloud/" + filename,
            *_pcl_pc_ptr
        )>=0
    )
    {
        std::cout << "Saved " << filename << std::endl;
    }
    else
    {
        std::cout << "Not saved " << filename << std::endl;
    }
    
}