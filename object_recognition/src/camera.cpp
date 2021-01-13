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
    _centroid_pub = _node_handle.advertise<object_recognition::ObjectPose>("detection_info", 10);
    _centroid_pub_rviz = _node_handle.advertise<geometry_msgs::PoseStamped>("detection_info_rviz", 10);
    
    _tfListenerPtr = new tf2_ros::TransformListener(_tfBuffer);
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
    // save_pc(_pcl_pc_ptr, "raw");
    
    // Voxel Filter
    // _pcl_pc_ptr = _filters.voxel_filter(
    //     _pcl_pc_ptr,
    //     CameraSettings::Filters::VoxelFilter::leaf_size
    // );
    // Save
    // save_pc(_pcl_pc_ptr, "after_voxel");

    // Cropbox Filter
    _pcl_pc_ptr = _filters.cropbox_filter(
        _pcl_pc_ptr,
        CameraSettings::Filters::CropboxFilter::min_range,
        CameraSettings::Filters::CropboxFilter::max_range,
        CameraSettings::Filters::CropboxFilter::translation,
        CameraSettings::Filters::CropboxFilter::rotation
    );
    // Save
    // save_pc(_pcl_pc_ptr, "after_cropbox");

    // Ransac filter
    _pcl_pc_ptr = _filters.ransac_filter(
        _pcl_pc_ptr,
        CameraSettings::Filters::RansacFilter::threshold
    );
    // // Save
    // save_pc(_pcl_pc_ptr, "after_ransac");

    // Clustering
    _ptrs_cluster = _filters.cluster_filter(
        _pcl_pc_ptr,
        CameraSettings::Filters::ClusterFilter::tolerance,
        CameraSettings::Filters::ClusterFilter::min_cluster_size,
        CameraSettings::Filters::ClusterFilter::max_cluster_size
    );
    // // Save
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
            "/home/raj/catkin_ws/src/SBRepo/object_recognition/point_cloud/" + filename,
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
                "/home/raj/catkin_ws/src/SBRepo/object_recognition/point_cloud/" + clustername,
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

void Camera::detect() 
{
    bool debug = true;
    std::string target_frame = "base_link";
    std::string source_frame = "camera_depth_frame2";
    geometry_msgs::TransformStamped transformStamped = _tfBuffer.lookupTransform(
        target_frame, 
        source_frame,
        ros::Time(0)
    );
    
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> ptrs_object_cluster;
    ptrs_object_cluster = recognizer.recognize_objects(_ptrs_cluster);
    PointType min_point, max_point, centroid;
    std::vector<std::string> object_names{"Coke Can", "Battery", "Glue Box"};

    geometry_msgs::PoseStamped object_pose_stamped;
    object_pose_stamped.pose.orientation.x = 0;
    object_pose_stamped.pose.orientation.y = 0;
    object_pose_stamped.pose.orientation.z = 0;
    object_pose_stamped.pose.orientation.w = 1;
    object_recognition::ObjectPose object_pose;

    for(int i = 0; i < ptrs_object_cluster.size(); ++i){
        pcl::PointCloud<PointType> object_cloud = *ptrs_object_cluster[i];
        // Compute the mid point for the x,y surface

        // This is very inefficient!!!
        // https://stackoverflow.com/questions/35669182/this-predefined-function-slowing-down-my-programs-performance
        pcl::getMinMax3D (object_cloud, min_point, max_point);
        centroid.x = (min_point.x+max_point.x)/2.0f;
        centroid.y = (min_point.y+max_point.y)/2.0f;
        centroid.z = (min_point.z+max_point.z)/2.0f;
        object_pose_stamped.pose.position.x = centroid.x;
        object_pose_stamped.pose.position.y = centroid.y;
        object_pose_stamped.pose.position.z = centroid.z;
        object_pose_stamped.header.frame_id = source_frame;
        tf2::doTransform(
            object_pose_stamped, 
            object_pose_stamped, 
            transformStamped
        );
        object_pose.name = object_names[i];
        object_pose.pose = object_pose_stamped;
        _centroid_pub_rviz.publish(object_pose.pose);
        _centroid_pub.publish(object_pose);
        if(debug){
            char c;
            std::cout << "Enter any character to continue: ";
            std::cin >> c;
        }
    }
    
    
    // std::vector<std::vector<float>> centroids;
    // centroids = recognizer.recognize_objects(_ptrs_cluster);
    
    // // Transform such that the pose is w.r.to the base link
    // // Reference: https://piazza.com/class/kftilbrk6dm7ea?cid=1120
    // object_pose_stamped.pose.position.x = centroids[0][0];
    // object_pose_stamped.pose.position.y = centroids[0][1];
    // object_pose_stamped.pose.position.z = centroids[0][2];
    // object_pose.name = "Coke can";

    // object_pose_stamped.pose.position.x = centroids[1][0];
    // object_pose_stamped.pose.position.y = centroids[1][1];
    // object_pose_stamped.pose.position.z = centroids[1][2];
    // tf2::doTransform(
    //     object_pose_stamped, 
    //     object_pose_stamped, 
    //     transformStamped
    // );
    // object_pose.name = "Battery";
    // object_pose.pose = object_pose_stamped;
    // _centroid_pub_rviz.publish(object_pose.pose);
    // _centroid_pub.publish(object_pose);
    // if(debug){
    //     char c;
    //     cout << "Enter any character to continue: ";
    //     cin >> c;
    // }

    // object_pose_stamped.pose.position.x = centroids[2][0];
    // object_pose_stamped.pose.position.y = centroids[2][1];
    // object_pose_stamped.pose.position.z = centroids[2][2];
    // tf2::doTransform(
    //     object_pose_stamped, 
    //     object_pose_stamped, 
    //     transformStamped
    // );
    // object_pose.name = "Glue";
    // object_pose.pose = object_pose_stamped;
    // _centroid_pub_rviz.publish(object_pose.pose);
    // _centroid_pub.publish(object_pose);
    // if(debug){
    //     char c;
    //     cout << "Enter any character to continue: ";
    //     cin >> c;
    // }
}