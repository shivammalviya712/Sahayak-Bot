#include <recognition.hpp>

int main()
{
    Recognition recognition(
        "/home/shivam/Projects/eYRC/catkin_ws/src/lab/point_cloud/milk.pcd", 
        "/home/shivam/Projects/eYRC/catkin_ws/src/lab/point_cloud/milk_cartoon_all_small_clorox.pcd"
    );
    recognition.pointcloud_to_centroid();

}