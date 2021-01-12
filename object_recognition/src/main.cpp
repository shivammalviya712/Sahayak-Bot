/*
Author: eYRC_SB_363
*/

#include <object_recognition/main.hpp>

int main(int argc, char **argv)
{
    // ROS stuffs
    ros::init(argc, argv, "main_node");
    ros::NodeHandle node_handle;
    ros::Rate loop_rate(1);

    // Recognition recognition(
    //     "/home/shivam/Projects/eYRC/catkin_ws/src/object_recognition/point_cloud/milk.pcd", 
    //     "/home/shivam/Projects/eYRC/catkin_ws/src/object_recognition/point_cloud/milk_cartoon_all_small_clorox.pcd"
    // );
    // recognition.pointcloud_to_centroid();

    Camera camera(node_handle);
    // Buffer so that we can receive callback messages
    // Ofcourse better ways would be there to do this

    ros::Duration(4).sleep();

    while (ros::ok())
    {
        ros::spinOnce();
        
        camera.analysis();
        camera.preprocess();
        
        loop_rate.sleep();
    }

    return 0;
}
