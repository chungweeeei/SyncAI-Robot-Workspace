#pragma once
#include "syncai_pointlio/map_builder/commons.h"
#include "syncai_pointlio/map_builder/point_ekf.h"
#include "syncai_pointlio/map_builder/ikd_Tree.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

struct LocalMap
{
    bool initialed = false;
    BoxPointType local_map_corner;
    Vec<BoxPointType> cub_to_rm;
};

// One group of points sharing the same per-point timestamp (curvature).
// Indices are [begin, end) into the downsampled scan; time is absolute.
struct PointGroup
{
    double time;
    int begin;
    int end;
};

class LidarProcessor
{
public:
    LidarProcessor(Config &config, std::shared_ptr<PointEKF> kf);

    void trimCloudMap();

    void incrCloudMap();

    void initCloudMap(PointVec &point_vec);

    // Downsample the scan, re-sort by per-point time and split it into
    // same-timestamp groups (Point-LIO "time compressing").
    void preprocess(SyncPackage &package, Vec<PointGroup> &groups);

    // Point-to-plane update of one group at its own timestamp; groups larger
    // than batch_max_points are split into sequential chunks.
    void processGroup(const PointGroup &group);

    static CloudType::Ptr transformCloud(CloudType::Ptr inp, const M3D &r, const V3D &t);
    M3D r_wl() { return m_kf->x().r_wi * m_kf->x().r_il; }
    V3D t_wl() { return m_kf->x().t_wi + m_kf->x().r_wi * m_kf->x().t_il; }

private:
    void updateChunk(int begin, int end);

    Config m_config;
    LocalMap m_local_map;
    std::shared_ptr<PointEKF> m_kf;
    std::shared_ptr<KD_TREE<PointType>> m_ikdtree;
    CloudType::Ptr m_cloud_down_lidar;
    CloudType::Ptr m_cloud_down_world;
    std::vector<bool> m_point_selected_flag;
    CloudType::Ptr m_norm_vec;
    std::vector<PointVec> m_nearest_points;
    pcl::VoxelGrid<PointType> m_scan_filter;
    HMatX12D m_H;
    Eigen::VectorXd m_z;
};
