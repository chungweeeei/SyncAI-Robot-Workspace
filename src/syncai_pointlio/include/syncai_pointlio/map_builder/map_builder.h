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
    double m_last_state_time = -1.0;
    double m_last_cov_time = -1.0;
};
