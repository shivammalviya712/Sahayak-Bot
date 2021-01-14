/*
Author: eYRC_SB_363
*/

#include <object_recognition/main.hpp>

bool detect;

int main(int argc, char **argv)
{
    // ROS stuffs
    ros::init(argc, argv, "main_node");
    ros::NodeHandle node_handle;
    ros::Subscriber detect_sub = node_handle.subscribe("detect", 10, detect_callback);

    Camera camera(node_handle);

    while(!detect)
    {   
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    ros::spinOnce();
    camera.preprocess();
    camera.detect();

    return 0;
}

void detect_callback(std_msgs::Bool msg)
{
    std::cout << "Detect set to true" << std::endl;
    detect = true;
}
