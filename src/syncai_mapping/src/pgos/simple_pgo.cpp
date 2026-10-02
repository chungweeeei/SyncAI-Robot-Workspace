#include "syncai_mapping/pgos/simple_pgo.h"

SimplePGO::SimplePGO(const Config & config) : m_config(config)
{
  gtsam::ISAM2Params isam2_params;
  isam2_params.relinearizeThreshold = 0.01;
  isam2_params.relinearizeSkip = 1;
  m_isam2 = std::make_shared<gtsam::ISAM2>(isam2_params);
  m_initial_values.clear();
  m_graph.resize(0);
  m_r_offset.setIdentity();
  m_t_offset.setZero();

  m_icp.setMaximumIterations(50);
  m_icp.setMaxCorrespondenceDistance(10);
  m_icp.setTransformationEpsilon(1e-6);
  m_icp.setEuclideanFitnessEpsilon(1e-6);
  m_icp.setRANSACIterations(0);
}

bool SimplePGO::isKeyPose(const PoseWithTime & pose)
{
  // The first pose is accepted unconditionally
  if (m_key_poses.size() == 0) return true;

  const KeyPoseWithCloud & last_item = m_key_poses.back();

  // Compute the translation and rotation deltas
  double delta_trans = (pose.t - last_item.t_local).norm();
  double delta_deg =
    Eigen::Quaterniond(pose.r).angularDistance(Eigen::Quaterniond(last_item.r_local)) * 57.324;

  // Either one exceeding its threshold makes this a key pose
  if (delta_trans > m_config.key_pose_delta_trans || delta_deg > m_config.key_pose_delta_deg)
    return true;

  return false;
}

bool SimplePGO::addKeyPose(const CloudWithPose & cloud_with_pose)
{
  bool is_key_pose = isKeyPose(cloud_with_pose.pose);
  if (!is_key_pose) return false;

  size_t idx = m_key_poses.size();  // Index of the new node

  /**
    * m_r_offset / m_t_offset is the local -> global offset, computed after the previous round of
    * optimisation. The offset has to be applied because every other node in the graph lives in
    * the global frame; if the new node used its local pose directly as the initial value it would
    * be inconsistent with the rest of the graph, so it is moved into the global frame with the
    * last offset first.
    */
  M3D init_r = m_r_offset * cloud_with_pose.pose.r;
  V3D init_t = m_r_offset * cloud_with_pose.pose.t + m_t_offset;

  // Add the initial value
  m_initial_values.insert(idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)));

  // Add the factor
  if (idx == 0) {
    // First frame -> prior constraint (PriorFactor)
    // A PriorFactor is an "absolute constraint": it declares outright that node 0 belongs at
    // this absolute pose
    gtsam::noiseModel::Diagonal::shared_ptr noise =
      gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * 1e-12);
    m_graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)), noise));
  } else {
    // Every later frame -> odometry constraint (BetweenFactor)
    // A BetweenFactor does not say where idx is, only that the relative pose from idx-1 to idx
    // should be this one
    const KeyPoseWithCloud & last_item = m_key_poses.back();

    M3D r_between = last_item.r_local.transpose() * cloud_with_pose.pose.r;  // Relative rotation
    // Relative translation
    V3D t_between = last_item.r_local.transpose() * (cloud_with_pose.pose.t - last_item.t_local);
    gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-6).finished());
    m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
      idx - 1, idx, gtsam::Pose3(gtsam::Rot3(r_between), gtsam::Point3(t_between)), noise));
  }

  // Finally, store the keyframe
  KeyPoseWithCloud item;
  item.time = cloud_with_pose.pose.second;
  item.r_local = cloud_with_pose.pose.r;
  item.t_local = cloud_with_pose.pose.t;
  item.body_cloud = cloud_with_pose.cloud;
  item.r_global = init_r;
  item.t_global = init_t;
  m_key_poses.push_back(item);
  return true;
}

CloudType::Ptr SimplePGO::getSubMap(int idx, int half_range, double resolution)
{
  assert(idx >= 0 && idx < static_cast<int>(m_key_poses.size()));
  int min_idx = std::max(0, idx - half_range);
  int max_idx = std::min(static_cast<int>(m_key_poses.size()) - 1, idx + half_range);

  CloudType::Ptr ret(new CloudType);
  for (int i = min_idx; i <= max_idx; i++) {
    CloudType::Ptr body_cloud = m_key_poses[i].body_cloud;
    CloudType::Ptr global_cloud(new CloudType);
    pcl::transformPointCloud(
      *body_cloud, *global_cloud, m_key_poses[i].t_global,
      Eigen::Quaterniond(m_key_poses[i].r_global));
    *ret += *global_cloud;
  }
  if (resolution > 0) {
    pcl::VoxelGrid<PointType> voxel_grid;
    voxel_grid.setLeafSize(resolution, resolution, resolution);
    voxel_grid.setInputCloud(ret);
    voxel_grid.filter(*ret);
  }
  return ret;
}

void SimplePGO::searchForLoopPairs()
{
  // The trajectory is still too short to have possibly looped back on itself.
  if (m_key_poses.size() < 10) return;

  // cool down time
  if (m_config.min_loop_detect_duration > 0.0) {
    if (m_history_pairs.size() > 0) {
      double current_time = m_key_poses.back().time;
      double last_time = m_key_poses[m_history_pairs.back().second].time;
      if (current_time - last_time < m_config.min_loop_detect_duration) return;
    }
  }

  // Find candidates
  size_t cur_idx = m_key_poses.size() - 1;
  const KeyPoseWithCloud & last_item = m_key_poses.back();
  pcl::PointXYZ last_pose_pt;
  last_pose_pt.x = last_item.t_global(0);
  last_pose_pt.y = last_item.t_global(1);
  last_pose_pt.z = last_item.t_global(2);

  // Every older frame except the current one
  pcl::PointCloud<pcl::PointXYZ>::Ptr key_poses_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < m_key_poses.size() - 1; i++) {
    pcl::PointXYZ pt;
    pt.x = m_key_poses[i].t_global(0);
    pt.y = m_key_poses[i].t_global(1);
    pt.z = m_key_poses[i].t_global(2);
    key_poses_cloud->push_back(pt);
  }

  // Radius search
  // {TODO} Possible performance hotspot: every call rebuilds the whole KD-tree (all historical
  // keyframes are inserted again from scratch)
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(key_poses_cloud);
  std::vector<int> ids;
  std::vector<float> sqdists;
  int neighbors = kdtree.radiusSearch(last_pose_pt, m_config.loop_search_radius, ids, sqdists);
  if (neighbors == 0) return;

  // Among the candidates that are close in space, only one that is also more than 60 s apart in
  // time is accepted as a loop closure.
  int loop_idx = -1;
  for (size_t i = 0; i < ids.size(); i++) {
    int idx = ids[i];
    // Far enough apart in time -> take it
    if (std::abs(last_item.time - m_key_poses[idx].time) > m_config.loop_time_tresh) {
      loop_idx = idx;
      break;
    }
  }

  if (loop_idx == -1) return;

  /** Point-cloud verification: align with ICP and derive the loop-closure edge.
    * So far we only know "close in space, far apart in time", which cannot be trusted yet (two
    * different corridors can happen to be close in position too).
    * Target side: the point clouds of the 5 keyframes on each side of the old keyframe (11 in
    * total) are each transformed to the world frame with their own t_global / r_global, stacked,
    * and downsampled.
    */
  CloudType::Ptr target_cloud =
    getSubMap(loop_idx, m_config.loop_submap_half_range, m_config.submap_resolution);
  CloudType::Ptr source_cloud = getSubMap(m_key_poses.size() - 1, 0, m_config.submap_resolution);
  CloudType::Ptr align_cloud(new CloudType);

  m_icp.setInputSource(source_cloud);
  m_icp.setInputTarget(target_cloud);
  m_icp.align(*align_cloud);

  if (!m_icp.hasConverged() || m_icp.getFitnessScore() > m_config.loop_score_tresh) return;

  // Convert the ICP result into a GTSAM BetweenFactor
  // The correction transform computed by ICP (expressed in the global frame)
  M4F loop_transform = m_icp.getFinalTransformation();

  LoopPair one_pair;
  one_pair.source_id = cur_idx;
  one_pair.target_id = loop_idx;
  one_pair.score = m_icp.getFitnessScore();

  // First apply the correction to the current frame's global pose, giving the corrected current
  // pose
  M3D r_refined = loop_transform.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].r_global;
  V3D t_refined = loop_transform.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].t_global +
                  loop_transform.block<3, 1>(0, 3).cast<double>();

  // Then express it as the relative pose of source with respect to target
  one_pair.r_offset = m_key_poses[loop_idx].r_global.transpose() * r_refined;
  one_pair.t_offset =
    m_key_poses[loop_idx].r_global.transpose() * (t_refined - m_key_poses[loop_idx].t_global);

  // Cache it; smoothAndUpdate adds it to the graph
  m_cache_pairs.push_back(one_pair);

  // Used by the markers and the cool-down
  m_history_pairs.emplace_back(one_pair.target_id, one_pair.source_id);
}

void SimplePGO::smoothAndUpdate()
{
  bool has_loop = !m_cache_pairs.empty();

  // Add the loop-closure factors
  if (has_loop) {
    for (LoopPair & pair : m_cache_pairs) {
      m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
        pair.target_id, pair.source_id,
        gtsam::Pose3(gtsam::Rot3(pair.r_offset), gtsam::Point3(pair.t_offset)),
        gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * pair.score)));
    }
    // Clear cache_pairs
    std::vector<LoopPair>().swap(m_cache_pairs);
  }

  // smooth and mapping
  // iSAM2 (Incremental Smoothing and Mapping) is incremental: internally it uses a Bayes tree
  // and only relinearises and re-solves the part that changed.
  m_isam2->update(m_graph, m_initial_values);
  m_isam2->update();
  if (has_loop) {
    m_isam2->update();
    m_isam2->update();
    m_isam2->update();
    m_isam2->update();
  }
  m_graph.resize(0);
  m_initial_values.clear();

  // Write the optimised result back to the keyframes
  gtsam::Values estimate_values = m_isam2->calculateBestEstimate();  // Best estimate
  for (size_t i = 0; i < m_key_poses.size(); i++) {
    gtsam::Pose3 pose = estimate_values.at<gtsam::Pose3>(i);
    m_key_poses[i].r_global = pose.rotation().matrix().cast<double>();  // Overwrite global
    m_key_poses[i].t_global = pose.translation().matrix().cast<double>();
  }

  // Recompute the offset and store the updated value
  const KeyPoseWithCloud & last_item = m_key_poses.back();
  m_r_offset = last_item.r_global * last_item.r_local.transpose();
  m_t_offset = last_item.t_global - m_r_offset * last_item.t_local;
}