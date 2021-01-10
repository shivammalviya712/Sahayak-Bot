#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

float angular_resolution_x = 0.5f,
      angular_resolution_y = angular_resolution_x;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool live_update = false;

void setViewerPose(pcl::visualization::PCLVisualizer &viewer, const Eigen::Affine3f &viewer_pose)
{
    Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
    Eigen::Vector3f up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
    viewer.setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2],
                             look_at_vector[0], look_at_vector[1], look_at_vector[2],
                             up_vector[0], up_vector[1], up_vector[2]);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lab_node");

    // Load pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> &point_cloud = *point_cloud_ptr;
    Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
    std::string filepath = "/home/shivam/Projects/eYRC/catkin_ws/src/object_recognition/point_cloud/can.pcd";

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, point_cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout << "Loaded "
              << "\n"
              << "Width: " << point_cloud.width << "\n"
              << "Height: " << point_cloud.height << "\n"
              << std::endl;

    // Convert into range image
    float angular_resolution = (float)(1.0f * (M_PI / 180.0f));
    float max_angle_width = (float)(180.0f * (M_PI / 180.0f));
    float max_angle_height = (float)(180.0f * (M_PI / 180.0f));
    Eigen::Affine3f sensor_pose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noise_level = 0.00;
    float min_range = 0.0f;
    int border_size = 1;

    pcl::RangeImage::Ptr range_image_ptr(new pcl::RangeImage);
    pcl::RangeImage range_image = *range_image_ptr;
    range_image.createFromPointCloud(
        point_cloud,
        angular_resolution,
        max_angle_width,
        max_angle_height,
        sensor_pose,
        coordinate_frame,
        noise_level,
        min_range,
        border_size);

    std::cout << range_image << "\n";

    // Visualization of range image
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(1, 1, 1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(range_image_ptr, 0, 0, 0);
    viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
    viewer.initCameraParameters();
    setViewerPose(viewer, range_image.getTransformationToWorldSystem());

    pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
    range_image_widget.showRangeImage(range_image);

    while (!viewer.wasStopped())
    {
        range_image_widget.spinOnce();
        viewer.spinOnce();
        pcl_sleep(0.01);

        if (live_update)
        {
            scene_sensor_pose = viewer.getViewerPose();
            range_image.createFromPointCloud(point_cloud, angular_resolution_x, angular_resolution_y,
                                             pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
                                             scene_sensor_pose, pcl::RangeImage::LASER_FRAME, noise_level, min_range, border_size);
            range_image_widget.showRangeImage(range_image);
        }
    }

    return (0);
}
