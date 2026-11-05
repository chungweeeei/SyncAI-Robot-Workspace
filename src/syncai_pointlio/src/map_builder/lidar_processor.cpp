#include "syncai_pointlio/map_builder/lidar_processor.h"

#include <cmath>
#ifdef MP_EN
// Came in through ikd_Tree.h's includes until the tree went (2026-11-05).
#include <omp.h>
#endif

LidarProcessor::LidarProcessor(Config &config, std::shared_ptr<PointEKF> kf) : m_config(config), m_kf(kf)
{
    m_map = std::make_shared<IVoxMap>(m_config.map_grid_resolution, m_config.map_capacity, m_config.map_nearby);
    m_cloud_down_lidar.reset(new CloudType);
    m_cloud_down_world.reset(new CloudType(10000, 1));
    m_norm_vec.reset(new CloudType(10000, 1));
    m_nearest_points.resize(10000);
    m_point_selected_flag.resize(10000, false);
    m_H.resize(m_config.batch_max_points, 12);
    m_z.resize(m_config.batch_max_points);

    if (m_config.scan_resolution > 0.0)
    {
        m_scan_filter.setLeafSize(m_config.scan_resolution, m_config.scan_resolution, m_config.scan_resolution);
    }
}

void LidarProcessor::incrCloudMap()
{
    if (m_cloud_down_lidar->empty())
        return;

    // m_cloud_down_world was already transformed inside processGroup / updateChunk with the state
    // at each point group's own time (equivalent to point-by-point undistortion), so it is used
    // as is here and not recomputed.
    //
    // A point is added only if no map point already sits in its map_resolution voxel, judged on
    // the neighbours the update just found for it: the first point in a voxel stays there for
    // good. This is upstream Point-LIO's rule (both branches; 2026-11-05). The port had FAST-LIO2's
    // instead -- add the point if it is nearer the voxel centre than its neighbours, and let the
    // ikd-tree's downsampling replace the voxel's point with it -- under which each voxel held the
    // minimum, over every scan ever inserted, of the distance to its centre. With ~4 800 scans per
    // 8-minute dock rest and a hundred laps a night, that minimum walks off the true surface by a
    // few sigma of the observation spread, in a different direction in every voxel, and the
    // planes the update fits get thicker all night (docs/pointlio-map-gravity-fixes.md §2: dock
    // match ratio 84 % -> 59 %, walking residual 2.55 -> 3.01 cm, then a divergence at P3).
    // First-wins cannot drift that way. Its own cost: whatever the first observation got wrong,
    // a person standing there included, stays until the map is reset.
    const double res = m_config.map_resolution;
    int size = m_cloud_down_lidar->size();
    PointVec point_to_add;
    point_to_add.reserve(size);
    for (int i = 0; i < size; i++)
    {
        const PointType &point_world = m_cloud_down_world->points[i];
        const PointVec &points_near = m_nearest_points[i];
        const float cx = (std::floor(point_world.x / res) + 0.5) * res;
        const float cy = (std::floor(point_world.y / res) + 0.5) * res;
        const float cz = (std::floor(point_world.z / res) + 0.5) * res;
        bool need_add = true;
        for (const PointType &q : points_near)
        {
            if (std::fabs(q.x - cx) < 0.5 * res && std::fabs(q.y - cy) < 0.5 * res && std::fabs(q.z - cz) < 0.5 * res)
            {
                need_add = false;
                break;
            }
        }
        if (need_add)
            point_to_add.push_back(point_world);
    }
    m_map->addPoints(point_to_add);
}

void LidarProcessor::initCloudMap(PointVec &point_vec)
{
    // The whole first scan, not thinned per voxel: there is nothing to match it against yet,
    // as upstream's ikdtree.Build() / ivox_->AddPoints() of the first frame.
    m_map->addPoints(point_vec);
}

void LidarProcessor::preprocess(SyncPackage &package, Vec<PointGroup> &groups)
{
    // The VoxelGrid downsampling voxel size is applied to this frame's point cloud before it
    // enters the EKF
    if (m_config.scan_resolution > 0.0)
    {
        m_scan_filter.setInputCloud(package.cloud);
        m_scan_filter.filter(*m_cloud_down_lidar);
    }
    else
    {
        pcl::copyPointCloud(*package.cloud, *m_cloud_down_lidar);
    }

    // The voxel filter does not preserve time order; re-sort by curvature (per-point time offset
    // in ms)
    std::sort(m_cloud_down_lidar->points.begin(), m_cloud_down_lidar->points.end(),
              [](const PointType &p1, const PointType &p2)
              { return p1.curvature < p2.curvature; });

    // These form five parallel arrays, all addressed by the same index
    int size = m_cloud_down_lidar->size();
    m_stats = LidarUpdateStats();
    m_stats.points = size;
    if (static_cast<int>(m_cloud_down_world->size()) < size)
    {
        m_cloud_down_world->resize(size);
        m_norm_vec->resize(size);
        m_nearest_points.resize(size);
        m_point_selected_flag.resize(size, false);
    }

    // time compressing: points sharing one timestamp form a group
    groups.clear();
    int begin = 0;
    for (int i = 1; i <= size; i++)
    {
        if (i == size || m_cloud_down_lidar->points[i].curvature != m_cloud_down_lidar->points[begin].curvature)
        {
            double group_time = package.cloud_start_time + m_cloud_down_lidar->points[begin].curvature / 1000.0;
            groups.push_back({group_time, begin, i});
            begin = i;
        }
    }
}

void LidarProcessor::processGroup(const PointGroup &group)
{
    for (int chunk_begin = group.begin; chunk_begin < group.end; chunk_begin += m_config.batch_max_points)
    {
        int chunk_end = std::min(chunk_begin + m_config.batch_max_points, group.end);
        updateChunk(chunk_begin, chunk_end);
    }
}

void LidarProcessor::updateChunk(int begin, int end)
{
    const State &state = m_kf->x();
    ++m_stats.chunks;
    // A reduction, not ++m_stats.near in the loop: the loop may run on
    // MP_PROC_NUM threads, and a shared counter would lose increments the way
    // the vector<bool> flags lost writes (see m_point_selected_flag).
    int near_num = 0;
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for reduction(+ : near_num)
#endif
    for (int i = begin; i < end; i++)
    {
        PointType &point_body = m_cloud_down_lidar->points[i];
        PointType &point_world = m_cloud_down_world->points[i];
        Eigen::Vector3d point_body_vec(point_body.x, point_body.y, point_body.z);
        Eigen::Vector3d point_world_vec = state.r_wi * (state.r_il * point_body_vec + state.t_il) + state.t_wi;
        point_world.x = point_world_vec(0);
        point_world.y = point_world_vec(1);
        point_world.z = point_world_vec(2);
        point_world.intensity = point_body.intensity;
        std::vector<float> point_sq_dist;
        auto &points_near = m_nearest_points[i];
        // Searched out to upstream's 5 m, the gate below keeps sqrt(5) m. The candidates past the
        // gate still matter: incrCloudMap judges a point's voxel by this list.
        m_map->nearestSearch(point_world, m_config.near_search_num, 5.0, points_near, point_sq_dist);
        if (points_near.size() >= static_cast<size_t>(m_config.near_search_num) && point_sq_dist[m_config.near_search_num - 1] <= 5)
            m_point_selected_flag[i] = true;
        else
            m_point_selected_flag[i] = false;
        if (!m_point_selected_flag[i])
            continue;
        ++near_num;

        Eigen::Vector4d pabcd;
        m_point_selected_flag[i] = false;
        if (esti_plane(points_near, m_config.plane_thr, pabcd))
        {
            double pd2 = pabcd(0) * point_world_vec(0) + pabcd(1) * point_world_vec(1) + pabcd(2) * point_world_vec(2) + pabcd(3);
            double s = 1 - 0.9 * std::fabs(pd2) / std::sqrt(point_body_vec.norm());
            if (s > 0.9)
            {
                m_point_selected_flag[i] = true;
                m_norm_vec->points[i].x = pabcd(0);
                m_norm_vec->points[i].y = pabcd(1);
                m_norm_vec->points[i].z = pabcd(2);
                m_norm_vec->points[i].intensity = pd2;
            }
        }
    }

    int effect_num = 0;
    for (int i = begin; i < end; i++)
    {
        if (!m_point_selected_flag[i])
            continue;
        const PointType &laser_p = m_cloud_down_lidar->points[i];
        const PointType &norm_p = m_norm_vec->points[i];
        Eigen::Vector3d laser_p_vec(laser_p.x, laser_p.y, laser_p.z);
        Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z);
        m_H.row(effect_num).setZero();
        m_H.block<1, 3>(effect_num, 0) = -norm_vec.transpose() * state.r_wi * Sophus::SO3d::hat(state.r_il * laser_p_vec + state.t_il);
        m_H.block<1, 3>(effect_num, 3) = norm_vec.transpose();
        if (m_config.esti_il)
        {
            m_H.block<1, 3>(effect_num, 6) = -norm_vec.transpose() * state.r_wi * state.r_il * Sophus::SO3d::hat(laser_p_vec);
            m_H.block<1, 3>(effect_num, 9) = norm_vec.transpose() * state.r_wi;
        }
        m_z(effect_num) = norm_p.intensity;
        m_stats.residual_sum += std::fabs(norm_p.intensity);
        effect_num++;
    }
    m_stats.near += near_num;
    m_stats.effective += effect_num;
    if (effect_num < 1)
        return;
    ++m_stats.chunks_updated;

    m_kf->updateLidar(m_H, m_z, effect_num);

    // Re-transform this chunk to world coordinates with the updated state, for incrCloudMap to
    // insert into the map
    const State &post = m_kf->x();
    for (int i = begin; i < end; i++)
    {
        const PointType &point_body = m_cloud_down_lidar->points[i];
        Eigen::Vector3d point_body_vec(point_body.x, point_body.y, point_body.z);
        Eigen::Vector3d point_world_vec = post.r_wi * (post.r_il * point_body_vec + post.t_il) + post.t_wi;
        m_cloud_down_world->points[i].x = point_world_vec(0);
        m_cloud_down_world->points[i].y = point_world_vec(1);
        m_cloud_down_world->points[i].z = point_world_vec(2);
    }
}

CloudType::Ptr LidarProcessor::transformCloud(CloudType::Ptr inp, const M3D &r, const V3D &t)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = r.cast<float>();
    transform.block<3, 1>(0, 3) = t.cast<float>();
    CloudType::Ptr ret(new CloudType);
    pcl::transformPointCloud(*inp, *ret, transform);
    return ret;
}
