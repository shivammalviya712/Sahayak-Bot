/*
Author: eYRC_SB_363
*/

#include <object_recognition/recognition.hpp>

Recognition::Recognition(std::string model_filename,
                         std::string scene_filename) {
  _model.reset(new pcl::PointCloud<PointType>());
  _model_keypoints.reset(new pcl::PointCloud<PointType>());
  _scene.reset(new pcl::PointCloud<PointType>());
  _scene_keypoints.reset(new pcl::PointCloud<PointType>());
  _model_normals.reset(new pcl::PointCloud<NormalType>());
  _scene_normals.reset(new pcl::PointCloud<NormalType>());
  _model_descriptors.reset(new pcl::PointCloud<DescriptorType>());
  _scene_descriptors.reset(new pcl::PointCloud<DescriptorType>());
  // Load cloud (Just for initial period!)
  if (pcl::io::loadPCDFile(model_filename, *_model) < 0) {
    std::cout << "Error loading model cloud." << std::endl;
  }
  if (pcl::io::loadPCDFile(scene_filename, *_scene) < 0) {
    std::cout << "Error loading scene cloud." << std::endl;
  }
}

// pcl::PointCloud<NormalType>::Ptr Recognition::compute_normals()

void Recognition::pointcloud_to_centroid() {
  //  Compute Normals
  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
  norm_est.setKSearch(10);
  norm_est.setInputCloud(_model);
  norm_est.compute(*_model_normals);

  norm_est.setInputCloud(_scene);
  norm_est.compute(*_scene_normals);

  //  Downsample Clouds to Extract keypoints
  pcl::UniformSampling<PointType> uniform_sampling;
  uniform_sampling.setInputCloud(_model);
  uniform_sampling.setRadiusSearch(_model_ss);
  uniform_sampling.filter(*_model_keypoints);
  std::cout << "Model total points: " << _model->size()
            << "; Selected Keypoints: " << _model_keypoints->size()
            << std::endl;

  uniform_sampling.setInputCloud(_scene);
  uniform_sampling.setRadiusSearch(_scene_ss);
  uniform_sampling.filter(*_scene_keypoints);
  std::cout << "Scene total points: " << _scene->size()
            << "; Selected Keypoints: " << _scene_keypoints->size()
            << std::endl;

  //  Compute Descriptor for keypoints
  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
  descr_est.setRadiusSearch(_descr_rad);

  descr_est.setInputCloud(_model_keypoints);
  descr_est.setInputNormals(_model_normals);
  descr_est.setSearchSurface(_model);
  descr_est.compute(*_model_descriptors);

  descr_est.setInputCloud(_scene_keypoints);
  descr_est.setInputNormals(_scene_normals);
  descr_est.setSearchSurface(_scene);
  descr_est.compute(*_scene_descriptors);

  pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());

  pcl::KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud(_model_descriptors);

  //  For each scene keypoint descriptor, find nearest neighbor into the model
  //  keypoints descriptor cloud and add it to the correspondences vector.
  for (std::size_t i = 0; i < _scene_descriptors->size(); ++i) {
    std::vector<int> neigh_indices(1);
    std::vector<float> neigh_sqr_dists(1);
    if (!std::isfinite(
            _scene_descriptors->at(i).descriptor[0]))  // skipping NaNs
    {
      continue;
    }
    int found_neighs = match_search.nearestKSearch(
        _scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
    if (found_neighs == 1 &&
        neigh_sqr_dists[0] <
            0.25f)  //  add match only if the squared descriptor distance is
                    //  less than 0.25 (SHOT descriptor distances are between 0
                    //  and 1 by design)
    {
      pcl::Correspondence corr(neigh_indices[0], static_cast<int>(i),
                               neigh_sqr_dists[0]);
      model_scene_corrs->push_back(corr);
    }
  }
  std::cout << "Correspondences found: " << model_scene_corrs->size()
            << std::endl;

  //  Using Hough3D
  //  Compute (Keypoints) Reference Frames only for Hough
  pcl::PointCloud<RFType>::Ptr model_rf(new pcl::PointCloud<RFType>());
  pcl::PointCloud<RFType>::Ptr scene_rf(new pcl::PointCloud<RFType>());

  pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
  rf_est.setFindHoles(true);
  rf_est.setRadiusSearch(_rf_rad);

  rf_est.setInputCloud(_model_keypoints);
  rf_est.setInputNormals(_model_normals);
  rf_est.setSearchSurface(_model);
  rf_est.compute(*model_rf);

  rf_est.setInputCloud(_scene_keypoints);
  rf_est.setInputNormals(_scene_normals);
  rf_est.setSearchSurface(_scene);
  rf_est.compute(*scene_rf);

  //  Clustering
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
      rototranslations;
  std::vector<pcl::Correspondences> clustered_corrs;
  pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
  // clusterer.setHoughBinSize(_cg_size);
  // clusterer.setHoughThreshold(_cg_thresh);
  // clusterer.setUseInterpolation(true);
  // clusterer.setUseDistanceWeight(false);

  // clusterer.setInputCloud(_model_keypoints);
  // clusterer.setInputRf(model_rf);
  // clusterer.setSceneCloud(_scene_keypoints);
  // clusterer.setSceneRf(scene_rf);
  // clusterer.setModelSceneCorrespondences(model_scene_corrs);

  // // clusterer.cluster (clustered_corrs);
  // clusterer.recognize(rototranslations, clustered_corrs);
}
