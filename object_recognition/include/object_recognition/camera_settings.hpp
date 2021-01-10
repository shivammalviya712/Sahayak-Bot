/*
Author: eYRC_SB_363
*/

#ifndef _SETTINGS_H
#define _SETTINGS_H

// Standard Libraries
#include <vector>
#include <string>

// PCL header files
#include <pcl/common/transforms.h>

namespace CameraSettings::Filters::VoxelFilter
{
    std::vector<float> leaf_size{0.02f, 0.02f, 0.02f};
}

namespace CameraSettings::Filters::CropboxFilter
{
    Eigen::Vector4f min_range = Eigen::Vector4f(-0.5, -0.5, 0.1, 1);
    Eigen::Vector4f max_range = Eigen::Vector4f(0.5, 0.5, 1, 1);
    Eigen::Vector3f translation = Eigen::Vector3f(0, 0, 0); // Replaced affine3f with vector3f  
    Eigen::Vector3f rotation = Eigen::Vector3f(0, 0, 0); // Replaced affine3f with vector3f
}


#endif