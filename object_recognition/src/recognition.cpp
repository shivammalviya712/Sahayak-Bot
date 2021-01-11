/*
Author: eYRC_SB_363
*/

#include <object_recognition/recognition.hpp>

Recognition::Recognition(
    std::string model_filename,
    std::string scene_filename
)
{
    _model.reset(new pcl::PointCloud<PointType>());
    _model_keypoints.reset(new pcl::PointCloud<PointType>());
    _scene.reset(new pcl::PointCloud<PointType>());
    _scene_keypoints.reset(new pcl::PointCloud<PointType>());
    _model_normals.reset(new pcl::PointCloud<NormalType>());
    _scene_normals.reset(new pcl::PointCloud<NormalType>());
    _model_descriptors.reset(new pcl::PointCloud<DescriptorType>());
    _scene_descriptors.reset(new pcl::PointCloud<DescriptorType>());
    // Load cloud (Just for initial period!)
    if (pcl::io::loadPCDFile(model_filename, *_model) < 0)
    {
        std::cout << "Error loading model cloud." << std::endl;
    }
    if (pcl::io::loadPCDFile(scene_filename, *_scene) < 0)
    {
        std::cout << "Error loading scene cloud." << std::endl;
    }
}

Recognition::~Recognition() {}

void Recognition::pointcloud_to_centroid()
{
    std::cout << "Total points in model: " << _model->size() << std::endl;
    std::cout << "Total points in scene: " << _scene->size() << std::endl;

    //  Compute Normals
    _model_normals = compute_normals(_model, 10);
    _scene_normals = compute_normals(_scene, 10);
    
    // Extract keypoints
    _model_keypoints = extract_keypoints(_model, _model_ss);
    _scene_keypoints = extract_keypoints(_scene, _scene_ss);
    std::cout << "Total points in model keypoints: " << _model_keypoints->size() << std::endl;
    std::cout << "Total points in scene keypoints: " << _scene_keypoints->size() << std::endl;

    //  Compute Descriptor
    _model_descriptors = compute_descriptors(
        _model,
        _model_keypoints,
        _model_normals,
        _descr_rad
    );
    _scene_descriptors = compute_descriptors(
        _scene,
        _scene_keypoints,
        _scene_normals,
        _descr_rad
    );

    //  Find correspondence
    pcl::CorrespondencesPtr correspondences_ptr = find_correspondences(
        _model_descriptors,
        _scene_descriptors
    );
    std::cout << "Correspondences found: " << correspondences_ptr->size() << std::endl;

    // Hough3D
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;
    tie(rototranslations, clustered_corrs) = hough3D(correspondences_ptr, _cg_size, _cg_thresh);
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
        if (!std::isfinite(_scene_descriptors->at(i).descriptor[0])) // skipping NaNs
        {
            continue;
        }
        int found_neighs = match_search.nearestKSearch(
            _scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists
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
    float bin_size,
    float threshold
)
{
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;

    // Reference Frames only for Hough
    pcl::PointCloud<RFType>::Ptr model_rf = compute_reference_frames(
        _model,
        _model_keypoints,
        _model_normals,
        0.015f
    );
    pcl::PointCloud<RFType>::Ptr scene_rf = compute_reference_frames(
        _scene,
        _scene_keypoints,
        _scene_normals,
        0.015f
    );

    //  Clustering
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    clusterer.setHoughBinSize(bin_size);
    clusterer.setHoughThreshold(threshold);
    clusterer.setUseInterpolation(true);
    clusterer.setUseDistanceWeight(false);
    clusterer.setInputCloud(_model_keypoints);
    clusterer.setInputRf(model_rf);
    clusterer.setSceneCloud(_scene_keypoints);
    clusterer.setSceneRf(scene_rf);
    clusterer.setModelSceneCorrespondences(correspondences_ptr);
    clusterer.recognize(rototranslations, clustered_corrs);

    return {rototranslations, clustered_corrs};
}