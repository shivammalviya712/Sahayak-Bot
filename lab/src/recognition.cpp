/*
Author: eYRC_SB_363
*/

#include <recognition.hpp>

Recognition::Recognition(
    std::string scene_filename
)
{
    _can_filepath = "/home/shivam/Projects/eYRC/catkin_ws/src/object_recognition/point_cloud/centered_can.pcd";
    _battery_filepath = "/home/shivam/Projects/eYRC/catkin_ws/src/object_recognition/point_cloud/centered_battery.pcd";
    _glue_filepath = "/home/shivam/Projects/eYRC/catkin_ws/src/object_recognition/point_cloud/centered_glue.pcd";

    // Temporarly loading from the file
    _scene_ptr = load_from_pcd(scene_filename);
    _scene_normals_ptr = compute_normals(_scene_ptr, 10);;
    _scene_keypoints_ptr = extract_keypoints(_scene_ptr, _scene_ss);
    _scene_descriptors_ptr = compute_descriptors(
        _scene_ptr,
        _scene_keypoints_ptr,
        _scene_normals_ptr,
        _descr_rad
    );
}

Recognition::~Recognition() {}

void Recognition::recognize_objects()
{
    pcl::PointCloud<PointType>::Ptr model_ptr;
    Eigen::Vector4f model_centroid;
    std::vector<float> detected_centroid;

    model_ptr = load_from_pcd(_can_filepath);
    detected_centroid = get_pc_centroid(model_ptr);
    // pcl::compute3DCentroid(*model_ptr, model_centroid);
    printf (
        "Centroid of can = < %0.3f, %0.3f, %0.3f >\n\n",
        detected_centroid[0],
        detected_centroid[1],
        detected_centroid[2]
    );

    model_ptr = load_from_pcd(_battery_filepath);
    // pcl::compute3DCentroid(*model_ptr, model_centroid);
    printf (
        "Centroid of battery = < %0.3f, %0.3f, %0.3f >\n\n",
        detected_centroid[0],
        detected_centroid[1],
        detected_centroid[2]
    );

    model_ptr = load_from_pcd(_glue_filepath);
    // pcl::compute3DCentroid(*model_ptr, model_centroid);
    printf (
        "Centroid of glue = < %0.3f, %0.3f, %0.3f >\n\n",
        detected_centroid[0],
        detected_centroid[1],
        detected_centroid[2]
    );
}

std::vector<float> Recognition::get_pc_centroid(
    pcl::PointCloud<PointType>::Ptr &model_ptr
)
{
    std::cout << "Total points in model: " << model_ptr->size() << std::endl;
    std::cout << "Total points in scene: " << _scene_ptr->size() << std::endl;

    // Define variables
    pcl::PointCloud<PointType>::Ptr model_keypoints_ptr;
    pcl::PointCloud<NormalType>::Ptr model_normals_ptr;
    pcl::PointCloud<DescriptorType>::Ptr model_descriptors_ptr;

    //  Compute Normals
    model_normals_ptr = compute_normals(model_ptr, 10);

    // Extract keypoints
    model_keypoints_ptr = extract_keypoints(model_ptr, _model_ss);
    std::cout << "Total points in model keypoints: " << model_keypoints_ptr->size() << std::endl;
    std::cout << "Total points in scene keypoints: " << _scene_keypoints_ptr->size() << std::endl;

    //  Compute Descriptor
    model_descriptors_ptr = compute_descriptors(
        model_ptr,
        model_keypoints_ptr,
        model_normals_ptr,
        _descr_rad
    );

    //  Find correspondence
    pcl::CorrespondencesPtr correspondences_ptr = find_correspondences(
        model_descriptors_ptr,
        _scene_descriptors_ptr
    );
    std::cout << "Correspondences found: " << correspondences_ptr->size() << std::endl;

    // Clustering
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;
    tie(rototranslations, clustered_corrs) = geometric_consistency(
        correspondences_ptr,
        model_keypoints_ptr,
        model_normals_ptr,
        _scene_keypoints_ptr,
        _scene_normals_ptr,
        _cg_size,
        _cg_thresh
    );
    std::cout << "Instances found: " << rototranslations.size() << std::endl;

    // Results
    // result_analysis(rototranslations, clustered_corrs);

    // Visualisation
    visualization(
        model_ptr,
        model_keypoints_ptr,
        _scene_ptr,
        _scene_keypoints_ptr,
        rototranslations,
        clustered_corrs
    );

    std::vector<float> centroid;
    centroid.push_back((rototranslations[0].block<3,1>(0, 3))(0));
    centroid.push_back((rototranslations[0].block<3,1>(0, 3))(1));
    centroid.push_back((rototranslations[0].block<3,1>(0, 3))(2));

    return centroid;
}

pcl::PointCloud<NormalType>::Ptr Recognition::compute_normals(
    pcl::PointCloud<PointType>::Ptr &pointcloud_ptr,
    int k_nearest_neighbour
)
{
    pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
    pcl::PointCloud<NormalType>::Ptr pointcloud_normals_ptr(new pcl::PointCloud<NormalType>);
    
    norm_est.setKSearch(k_nearest_neighbour);
    norm_est.setInputCloud(pointcloud_ptr);
    norm_est.compute(*pointcloud_normals_ptr);

    return pointcloud_normals_ptr;
}

pcl::PointCloud<PointType>::Ptr Recognition::extract_keypoints(
    pcl::PointCloud<PointType>::Ptr &pointcloud_ptr,
    float sampling_size
)
{
    pcl::UniformSampling<PointType> uniform_sampling;
    pcl::PointCloud<PointType>::Ptr pointcloud_keypoints_ptr((new pcl::PointCloud<PointType>));

    uniform_sampling.setInputCloud(pointcloud_ptr);
    uniform_sampling.setRadiusSearch(sampling_size);
    uniform_sampling.filter(*pointcloud_keypoints_ptr);

    return pointcloud_keypoints_ptr;
}

pcl::PointCloud<DescriptorType>::Ptr Recognition::compute_descriptors(
    pcl::PointCloud<PointType>::Ptr &pointcloud_ptr,
    pcl::PointCloud<PointType>::Ptr &pointcloud_keypoints_ptr,
    pcl::PointCloud<NormalType>::Ptr &pointcloud_normals_ptr,
    float radius_search  
)
{
    pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
    pcl::PointCloud<DescriptorType>::Ptr pointcloud_descriptors_ptr(new pcl::PointCloud<DescriptorType>);

    descr_est.setRadiusSearch(radius_search);
    descr_est.setInputCloud(pointcloud_keypoints_ptr);
    descr_est.setInputNormals(pointcloud_normals_ptr);
    descr_est.setSearchSurface(pointcloud_ptr);
    descr_est.compute(*pointcloud_descriptors_ptr);

    return pointcloud_descriptors_ptr;
}

pcl::CorrespondencesPtr Recognition::find_correspondences(
    pcl::PointCloud<DescriptorType>::Ptr &model_descriptors,
    pcl::PointCloud<DescriptorType>::Ptr &scene_descriptors
)
{
    pcl::CorrespondencesPtr correspondences_ptr(new pcl::Correspondences);
    pcl::KdTreeFLANN<DescriptorType> match_search;
    
    match_search.setInputCloud(model_descriptors);

    for (std::size_t i = 0; i < scene_descriptors->size(); ++i)
    {
        std::vector<int> neigh_indices(1);
        std::vector<float> neigh_sqr_dists(1);
        if (!std::isfinite(_scene_descriptors_ptr->at(i).descriptor[0])) // skipping NaNs
        {
            continue;
        }
        int found_neighs = match_search.nearestKSearch(
            _scene_descriptors_ptr->at(i), 1, neigh_indices, neigh_sqr_dists
        );
        if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f)
        {
            pcl::Correspondence corr(neigh_indices[0], static_cast<int>(i), neigh_sqr_dists[0]);
            correspondences_ptr->push_back(corr);
        }
    }

    return correspondences_ptr;
}

pcl::PointCloud<RFType>::Ptr Recognition::compute_reference_frames(
    pcl::PointCloud<PointType>::Ptr &pointcloud_ptr,
    pcl::PointCloud<PointType>::Ptr &pointcloud_keypoints_ptr,
    pcl::PointCloud<NormalType>::Ptr &pointcloud_normals_ptr,
    float rf_radius
)
{
    pcl::PointCloud<RFType>::Ptr pointcloud_rf(new pcl::PointCloud<RFType>());
    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
    
    rf_est.setFindHoles(true);
    rf_est.setRadiusSearch(rf_radius);
    rf_est.setInputCloud(pointcloud_keypoints_ptr);
    rf_est.setInputNormals(pointcloud_normals_ptr);
    rf_est.setSearchSurface(pointcloud_ptr);
    rf_est.compute(*pointcloud_rf);

    return pointcloud_rf;
}

std::tuple<
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>,
    std::vector<pcl::Correspondences>
> Recognition::hough3D(
    pcl::CorrespondencesPtr &correspondences_ptr,
    pcl::PointCloud<PointType>::Ptr &model_ptr,
    pcl::PointCloud<PointType>::Ptr &model_keypoints_ptr,
    pcl::PointCloud<NormalType>::Ptr &model_normals_ptr,
    pcl::PointCloud<PointType>::Ptr &scene_ptr,
    pcl::PointCloud<PointType>::Ptr &scene_keypoints_ptr,
    pcl::PointCloud<NormalType>::Ptr &scene_normals_ptr,
    float bin_size,
    float threshold
)
{
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;

    // Reference Frames only for Hough
    pcl::PointCloud<RFType>::Ptr model_rf = compute_reference_frames(
        model_ptr,
        model_keypoints_ptr,
        model_normals_ptr,
        0.015f
    );
    pcl::PointCloud<RFType>::Ptr scene_rf = compute_reference_frames(
        scene_ptr,
        scene_keypoints_ptr,
        scene_normals_ptr,
        0.015f
    );

    //  Clustering
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    clusterer.setHoughBinSize(bin_size);
    clusterer.setHoughThreshold(threshold);
    clusterer.setUseInterpolation(true);
    clusterer.setUseDistanceWeight(false);
    clusterer.setInputCloud(model_keypoints_ptr);
    clusterer.setInputRf(model_rf);
    clusterer.setSceneCloud(scene_keypoints_ptr);
    clusterer.setSceneRf(scene_rf);
    clusterer.setModelSceneCorrespondences(correspondences_ptr);
    clusterer.recognize(rototranslations, clustered_corrs);

    return {rototranslations, clustered_corrs};
}

std::tuple<
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>,
    std::vector<pcl::Correspondences>
> Recognition::geometric_consistency(
    pcl::CorrespondencesPtr &correspondences_ptr,
    pcl::PointCloud<PointType>::Ptr &model_keypoints_ptr,
    pcl::PointCloud<NormalType>::Ptr &model_normals_ptr,
    pcl::PointCloud<PointType>::Ptr &scene_keypoints_ptr,
    pcl::PointCloud<NormalType>::Ptr &scene_normals_ptr,
    float gc_size,
    float threshold
)
{
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;

    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
    gc_clusterer.setGCSize(gc_size);
    gc_clusterer.setGCThreshold(threshold);
    gc_clusterer.setInputCloud (model_keypoints_ptr);
    gc_clusterer.setSceneCloud (scene_keypoints_ptr);
    gc_clusterer.setModelSceneCorrespondences (correspondences_ptr);
    
    gc_clusterer.recognize (rototranslations, clustered_corrs);

    return {rototranslations, clustered_corrs};
}

void Recognition::result_analysis(
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> rototranslations,
    std::vector<pcl::Correspondences> clustered_corrs
)
{
    std::cout << "Model instances found: " << rototranslations.size () << std::endl;
    for (std::size_t i = 0; i < rototranslations.size (); ++i)
    {
        std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
        std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

        // Print the rotation matrix and translation vector
        Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
        Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

        printf ("\n");
        printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
        printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
        printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
        printf ("\n");
        printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
    }
}

void Recognition::visualization(
    pcl::PointCloud<PointType>::Ptr &model_ptr,
    pcl::PointCloud<PointType>::Ptr &model_keypoints_ptr,
    pcl::PointCloud<PointType>::Ptr &scene_ptr,
    pcl::PointCloud<PointType>::Ptr &scene_keypoints_ptr,
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> rototranslations,
    std::vector<pcl::Correspondences> clustered_corrs
)
{
    pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
    viewer.addPointCloud (scene_ptr, "scene_cloud");

    pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());
 
    //  We are translating the model so that it doesn't end in the middle of the scene representation
    pcl::transformPointCloud (*model_ptr, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
    pcl::transformPointCloud (*model_keypoints_ptr, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
    viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");

    pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints_ptr, 0, 0, 255);
    viewer.addPointCloud (scene_keypoints_ptr, scene_keypoints_color_handler, "scene_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
    viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");

    for (std::size_t i = 0; i < rototranslations.size (); ++i)
    {
        pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
        pcl::transformPointCloud (*model_ptr, *rotated_model, rototranslations[i]);

        std::stringstream ss_cloud;
        ss_cloud << "instance" << i;

        pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
        viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());
        
        for (std::size_t j = 0; j < clustered_corrs[i].size (); ++j)
        {
            std::stringstream ss_line;
            ss_line << "correspondence_line" << i << "_" << j;
            PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
            PointType& scene_point = scene_keypoints_ptr->at (clustered_corrs[i][j].index_match);

            //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
            viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
        }
    }

    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
}

pcl::PointCloud<PointType>::Ptr Recognition::load_from_pcd(std::string &filepath)
{
    pcl::PointCloud<PointType>::Ptr pointcloud_ptr(new pcl::PointCloud<PointType>); 
    if (pcl::io::loadPCDFile(filepath, *pointcloud_ptr) < 0)
    {
        std::cout << "Error loading file " + filepath <<  std::endl;
    }

    return pointcloud_ptr; 
}

void Recognition::save_to_pcd(pcl::PointCloud<PointType>::Ptr &pointcloud_ptr, std::string &filepath)
{
    if(pcl::io::savePCDFileASCII (filepath, *pointcloud_ptr)>=0)
    {
        std::cout << "Saved " << filepath << "\n" << std::endl;
    }
    else
    {
        std::cout << "Not saved " << filepath << "\n" << std::endl;
    }
}

pcl::PointCloud<PointType>::Ptr Recognition::center_pointcloud(pcl::PointCloud<PointType>::Ptr &pointcloud_ptr)
{
    pcl::PointCloud<PointType>::Ptr transformed_cloud_ptr(new pcl::PointCloud<PointType>);
    
    // Compute centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*pointcloud_ptr, centroid);
    std::cout << "Computed centroid is: "
              << centroid[0] << " "
              << centroid[1] << " "
              << centroid[2] << " "
              << centroid[3] << " "
              << std::endl;
    
    // Transform
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();;
    transform.translation() << -centroid[0], -centroid[1], -centroid[2];
    std::cout << "Transformation matrix: " << "\n"
              << transform.matrix() << "\n"
              << std::endl;

    pcl::transformPointCloud(*pointcloud_ptr, *transformed_cloud_ptr, transform);

    return transformed_cloud_ptr;
}

void Recognition::center_n_save(
    std::string &input_filepath, std::string &output_filepath
)
{
    pcl::PointCloud<PointType>::Ptr input_pc_ptr = load_from_pcd(input_filepath);
    pcl::PointCloud<PointType>::Ptr output_pc_ptr = center_pointcloud(input_pc_ptr);
    save_to_pcd(output_pc_ptr, output_filepath);
}