#include "syncai_pointlio/map_builder/map_builder.h"

#include <algorithm>

MapBuilder::MapBuilder(Config & config, std::shared_ptr<PointEKF> kf) : m_config(config), m_kf(kf)
{
  m_kf->setConfig(config);
  m_imu_initializer = std::make_shared<IMUInitializer>(config, kf);
  m_lidar_processor = std::make_shared<LidarProcessor>(config, kf);
  m_status = BuilderStatus::IMU_INIT;
}

void MapBuilder::processIMU(const IMUData & imu)
{
  // The complete handling of one IMU event.
  // In the EKF, predictState uses the state's own omg/acc together with the dt passed in to
  // predict the state at that time:
  //   delta omg => m_x.omg * dt
  //   delta acc => (m_x.r_wi * m_x.acc + m_x.g) * dt;
  m_kf->predictState(imu.time - m_last_state_time);
  m_kf->predictCov(imu.time - m_last_cov_time);  // The covariance matrix likewise
  m_kf->updateIMU(imu.gyro, imu.acc);  // Correct the estimate with the IMU reading as measurement
  m_last_state_time = std::max(imu.time, m_last_state_time);
  m_last_cov_time = std::max(imu.time, m_last_cov_time);
}

void MapBuilder::predictTo(double time)
{
  m_kf->predictState(time - m_last_state_time);
  m_last_state_time = std::max(time, m_last_state_time);
}

void MapBuilder::process(SyncPackage & package)
{
  if (m_status == BuilderStatus::IMU_INIT) {
    if (m_imu_initializer->initialize(package)) {
      // The propagation clock starts at the end of this frame
      m_last_state_time = package.cloud_end_time;
      m_last_cov_time = package.cloud_end_time;
      m_status = BuilderStatus::MAP_INIT;
    }
    return;
  }

  // Rebuilt for every frame; empty for the MAP_INIT frame, whose cloud is
  // then published with the end state alone (see deskewToEndBody).
  m_group_poses.clear();

  if (m_status == BuilderStatus::MAP_INIT) {
    // This frame only advances the state and builds the initial map with the end-of-frame pose
    // (same flow as fastlio2)
    for (const auto & imu : package.imus) processIMU(imu);

    // If the currently estimated omg / acc held, where would we be, which way would we face and
    // how fast would we be moving at cloud_end_time
    predictTo(package.cloud_end_time);

    // Project the first point cloud into world coordinates, i.e. the map frame
    CloudType::Ptr cloud_world = LidarProcessor::transformCloud(
      package.cloud, m_lidar_processor->r_wl(), m_lidar_processor->t_wl());
    
    // This first point cloud becomes the seed of the map
    m_lidar_processor->initCloudMap(cloud_world->points);

    // Switch to the normal mapping state
    m_status = BuilderStatus::MAPPING;
    return;
  }

  // MAPPING: the point-by-point event loop. Both sequences, IMU and point groups, are already
  // sorted by time, and two cursors process them in time order: IMU -> predict + IMU measurement
  // update; point group -> predict + point-to-plane update (Point-LIO output model, no scan
  // undistortion).
  m_lidar_processor->preprocess(package, m_groups);
  m_lidar_processor->trimCloudMap();

  // Below is the loop that runs the two sorted sequences merged in time order.
  // The main reason pointlio needs no separate correction (undistortion) step is that it
  // corrects as it projects:
  //   group1 projected -> update -> the state gets more accurate
  //   group2 projected (with the corrected state) -> update -> more accurate still
  //   group3 projected -> ..
  // Every group's pose benefits from the corrections of all the groups before it, so error does
  // not accumulate within one frame's point cloud.
  size_t imu_idx = 0;
  for (const auto & group : m_groups) {
    // Check whether any IMU sample is earlier than the point group's time. If not, predict
    // straight to the group's time; if so, first processIMU up to each IMU time, then predict to
    // the group's time.
    while (imu_idx < package.imus.size() && package.imus[imu_idx].time <= group.time) {
      processIMU(package.imus[imu_idx]);
      imu_idx++;
    }
    predictTo(group.time);
    m_lidar_processor->processGroup(group);
    // After the update, also when it found no plane to use: this is the
    // state updateChunk projected the group into the map with.
    m_group_poses.push_back({group.time, m_kf->x().r_wi, m_kf->x().t_wi});
  }

  // After the last point group there may still be IMU samples left to consume
  while (imu_idx < package.imus.size()) {
    processIMU(package.imus[imu_idx]);
    imu_idx++;
  }
  predictTo(package.cloud_end_time);
  m_group_poses.push_back({package.cloud_end_time, m_kf->x().r_wi, m_kf->x().t_wi});

  // After downsampling, each map voxel keeps only the point closest to its centre.
  // Done incrementally: the map is not rebuilt, only the incoming points are judged for insertion
  m_lidar_processor->incrCloudMap();
}

CloudType::Ptr MapBuilder::deskewToEndBody(
  const CloudType::Ptr & cloud, double cloud_start_time) const
{
  const State & end = m_kf->x();
  const M3D r_end_t = end.r_wi.transpose();
  CloudType::Ptr out(new CloudType);
  out->resize(cloud->size());
  out->header = cloud->header;
  for (size_t i = 0; i < cloud->size(); ++i) {
    const PointType & p = cloud->points[i];
    const V3D b(p.x, p.y, p.z);
    const V3D in_imu = end.r_il * b + end.t_il;
    V3D w;
    if (m_group_poses.empty()) {
      w = end.r_wi * in_imu + end.t_wi;
    } else {
      // The group times are VoxelGrid means of the raw per-point offsets, not
      // a subset of them, so this is "the last group at or before the point"
      // rather than an exact match. Points before the first group take the
      // first group's state; the list ends with the end state.
      const double t = cloud_start_time + p.curvature / 1000.0;
      auto it = std::upper_bound(
        m_group_poses.begin(), m_group_poses.end(), t,
        [](double v, const StampedPose & s) { return v < s.t; });
      const StampedPose & s = (it == m_group_poses.begin()) ? *it : *(it - 1);
      w = s.r_wi * in_imu + s.t_wi;
    }
    const V3D e = r_end_t * (w - end.t_wi);
    PointType & q = out->points[i];
    q = p;
    q.x = static_cast<float>(e.x());
    q.y = static_cast<float>(e.y());
    q.z = static_cast<float>(e.z());
  }
  return out;
}
