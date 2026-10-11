#pragma once
#include "syncai_pointlio/map_builder/imu_initializer.h"
#include "syncai_pointlio/map_builder/lidar_processor.h"

enum BuilderStatus
{
    IMU_INIT,       // accumulating enough IMU samples
    MAP_INIT,       // initial map built
    MAPPING         
};

class MapBuilder
{
public:
    MapBuilder(Config &config, std::shared_ptr<PointEKF> kf);

    void process(SyncPackage &package);
    BuilderStatus status() { return m_status; }
    std::shared_ptr<LidarProcessor> lidar_processor() { return m_lidar_processor; }

    // Motion-compensate one scan of the last process() call into the body
    // (IMU) frame at cloud_end_time -- the frame and stamp lio_odom carries.
    // Each point is placed with the state right after the group its time
    // falls into was updated (the pose Point-LIO itself mapped that time
    // with), then brought back with the end state. Without this a published
    // cloud is the raw scan put through the extrinsic alone, and a keyframe
    // taken while turning smears every wall: on dp1f_1008_2 turning keyframes
    // had twice the wall thickness of straight ones (0.56 m vs 0.22 m).
    // Before the first MAPPING frame there are no group poses and every point
    // gets the end state, i.e. the old extrinsic-only output.
    CloudType::Ptr deskewToEndBody(const CloudType::Ptr &cloud, double cloud_start_time) const;

private:
    // One IMU event: predict to its time + update with the IMU as a measurement (output model)
    void processIMU(const IMUData &imu);
    // Push the nominal state forward to time (the covariance keeps propagating at IMU rate)
    void predictTo(double time);

    Config m_config;
    BuilderStatus m_status;
    std::shared_ptr<PointEKF> m_kf;
    std::shared_ptr<IMUInitializer> m_imu_initializer;
    std::shared_ptr<LidarProcessor> m_lidar_processor;
    Vec<PointGroup> m_groups;
    // Post-update state after each group of the last MAPPING frame, plus the
    // end-of-scan state last; times ascending. Read by deskewToEndBody.
    struct StampedPose
    {
        double t;
        M3D r_wi;
        V3D t_wi;
    };
    Vec<StampedPose> m_group_poses;
    double m_last_state_time = -1.0;
    double m_last_cov_time = -1.0;
};
