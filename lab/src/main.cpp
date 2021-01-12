#include <recognition.hpp>

int main()
{
    Recognition recognition(
        "/home/shivam/Projects/eYRC/catkin_ws/src/object_recognition/point_cloud/after_cluster3.pcd", 
        "/home/shivam/Projects/eYRC/catkin_ws/src/object_recognition/point_cloud/after_ransac.pcd"
    );
    recognition.pointcloud_to_centroid();
}