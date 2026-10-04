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
  m_icp.setMaxCorrespondenceDistance(m_config.loop_icp_max_corr_dist);
  m_icp.setTransformationEpsilon(1e-6);
  m_icp.setEuclideanFitnessEpsilon(1e-6);
  m_icp.setRANSACIterations(0);

  // Same settings as syncai_localizer's refine stage, which is the other
  // small_gicp user in the workspace. setNumThreads() has to come before the
  // first setInputTarget() (the kd-tree builder reads it).
  m_gicp.setNumThreads(m_config.loop_gicp_num_threads);
  m_gicp.setCorrespondenceRandomness(m_config.loop_gicp_num_neighbors);
  m_gicp.setRegistrationType("GICP");
  m_gicp.setMaximumIterations(50);
  m_gicp.setMaxCorrespondenceDistance(m_config.loop_icp_max_corr_dist);
  m_gicp.setTransformationEpsilon(1e-6);
}

bool SimplePGO::alignLoop(
  const CloudType::Ptr & source, const CloudType::Ptr & target, M4F & transform, double & fitness)
{
  CloudType::Ptr aligned(new CloudType);
  if (m_config.loop_registration == "icp") {
    m_icp.setInputSource(source);
    m_icp.setInputTarget(target);
    m_icp.align(*aligned);
    if (!m_icp.hasConverged()) return false;
    transform = m_icp.getFinalTransformation();
    fitness = m_icp.getFitnessScore();
    return true;
  }
  m_gicp.setInputSource(source);
  m_gicp.setInputTarget(target);
  m_gicp.align(*aligned);
  if (!m_gicp.hasConverged()) return false;
  transform = m_gicp.getFinalTransformation();
  fitness = m_gicp.getFitnessScore();
  return true;
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
  M4F loop_transform;
  double fitness = 0.0;
  if (!alignLoop(source_cloud, target_cloud, loop_transform, fitness)) return;
  if (fitness > m_config.loop_score_tresh) return;

  // The correction transform computed by the registration, expressed in the
  // global frame: it maps the source keyframe's world cloud onto the submap.
  if (m_config.loop_planar_correction) {
    // Keep only what the loop is trusted for: where the source keyframe should
    // sit in x / y and how it should be yawed. Its world z, roll and pitch
    // stay exactly as the graph has them now. Built so that the source POSE
    // is displaced by the registration's own xy displacement (not by the raw
    // translation column, which is meaningless 20 m from the origin when the
    // transform carries a rotation) and yawed about the world z axis.
    const M3D r_full = loop_transform.block<3, 3>(0, 0).cast<double>();
    const V3D t_full = loop_transform.block<3, 1>(0, 3).cast<double>();
    const V3D src_t = m_key_poses[cur_idx].t_global;
    const V3D displaced = r_full * src_t + t_full;
    const double yaw = std::atan2(r_full(1, 0), r_full(0, 0));
    const M3D r_yaw = Eigen::AngleAxisd(yaw, V3D::UnitZ()).toRotationMatrix();
    const V3D target_pos(displaced.x(), displaced.y(), src_t.z());
    const V3D t_planar = target_pos - r_yaw * src_t;
    loop_transform.setIdentity();
    loop_transform.block<3, 3>(0, 0) = r_yaw.cast<float>();
    loop_transform.block<3, 1>(0, 3) = t_planar.cast<float>();
  }

  LoopPair one_pair;
  one_pair.source_id = cur_idx;
  one_pair.target_id = loop_idx;
  one_pair.score = fitness;
  one_pair.icp_r = loop_transform.block<3, 3>(0, 0).cast<double>();
  one_pair.icp_t = loop_transform.block<3, 1>(0, 3).cast<double>();

  // First apply the correction to the current frame's global pose, giving the corrected current
  // pose
  M3D r_refined = loop_transform.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].r_global;
  V3D t_refined = loop_transform.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].t_global +
                  loop_transform.block<3, 1>(0, 3).cast<double>();

  // Then express it as the relative pose of source with respect to target
  one_pair.r_offset = m_key_poses[loop_idx].r_global.transpose() * r_refined;
  one_pair.t_offset =
    m_key_poses[loop_idx].r_global.transpose() * (t_refined - m_key_poses[loop_idx].t_global);

  // And in the world frame, for the planar factor: where the corrected source
  // sits relative to the target in x / y, and by how much it is yawed.
  {
    const V3D d = t_refined - m_key_poses[loop_idx].t_global;
    const double yaw_s = std::atan2(r_refined(1, 0), r_refined(0, 0));
    const M3D & r_t = m_key_poses[loop_idx].r_global;
    const double yaw_t = std::atan2(r_t(1, 0), r_t(0, 0));
    one_pair.planar_meas = V3D(d.x(), d.y(), PlanarLoopFactor::wrapAngle(yaw_s - yaw_t));
  }

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
      // Pose3 tangent order is (roll, pitch, yaw, x, y, z). The fork used the
      // fitness score as the variance on all six; roll / pitch / z now have
      // their own, tight, value -- a loop is trusted in the plane, and the
      // chain is far too compliant in z (see loop_planar_correction) to be
      // handed a z or tilt request with sigma ~0.3 m / 18 deg.
      if (m_config.loop_planar_correction) {
        // World-frame x / y / yaw only; z / roll / pitch are not in the error.
        m_graph.add(PlanarLoopFactor(
          pair.target_id, pair.source_id, pair.planar_meas,
          gtsam::noiseModel::Diagonal::Variances(gtsam::Vector3::Ones() * pair.score)));
        continue;
      }
      const double rpz = m_config.loop_noise_var_roll_pitch_z;
      gtsam::Vector6 var;
      var << rpz, rpz, pair.score, pair.score, pair.score, rpz;
      m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
        pair.target_id, pair.source_id,
        gtsam::Pose3(gtsam::Rot3(pair.r_offset), gtsam::Point3(pair.t_offset)),
        gtsam::noiseModel::Diagonal::Variances(var)));
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