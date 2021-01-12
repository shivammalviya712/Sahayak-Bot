#include <recognition.hpp>

int main()
{
    Recognition recognition( 
        "/home/shivam/Projects/eYRC/catkin_ws/src/object_recognition/point_cloud/after_ransac.pcd"
    );
    recognition.recognize_objects();

    // std::string input_filepath = "/home/shivam/Projects/eYRC/catkin_ws/src/object_recognition/point_cloud/milk.pcd";
    // std::string output_filepath = "/home/shivam/Projects/eYRC/catkin_ws/src/object_recognition/point_cloud/centered_milk.pcd";
    // recognition.center_n_save(input_filepath, output_filepath);
}