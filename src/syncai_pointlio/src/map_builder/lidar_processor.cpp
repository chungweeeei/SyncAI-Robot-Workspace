#include "syncai_pointlio/map_builder/lidar_processor.h"

LidarProcessor::LidarProcessor(Config &config, std::shared_ptr<PointEKF> kf) : m_config(config), m_kf(kf)
{
    m_ikdtree = std::make_shared<KD_TREE<PointType>>();
    m_ikdtree->set_downsample_param(m_config.map_resolution);
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

void LidarProcessor::trimCloudMap()
{
    // Sliding local map: keeps the ikd-Tree from growing without bound as the robot travels
    // further. This logic was carried over from fastlio2.
    // Core idea: maintain a cube of side cube_len (300 m) and push it forward when the robot is
    // about to reach its boundary.
    m_local_map.cub_to_rm.clear();
    const State &state = m_kf->x();
    Eigen::Vector3d pos_lidar = state.t_wi + state.r_wi * state.t_il;

    if (!m_local_map.initialed)
    {
        for (int i = 0; i < 3; i++)
        {
            m_local_map.local_map_corner.vertex_min[i] = pos_lidar[i] - m_config.cube_len / 2.0;
            m_local_map.local_map_corner.vertex_max[i] = pos_lidar[i] + m_config.cube_len / 2.0;
        }
        m_local_map.initialed = true;
        return;
    }
    
    // Check whether the cube needs to be pushed forward
    float dist_to_map_edge[3][2];
    bool need_move = false;
    double det_thresh = m_config.move_thresh * m_config.det_range;
    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_lidar(i) - m_local_map.local_map_corner.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_lidar(i) - m_local_map.local_map_corner.vertex_max[i]);

        if (dist_to_map_edge[i][0] <= det_thresh || dist_to_map_edge[i][1] <= det_thresh)
            need_move = true;
    }
    if (!need_move)
        return;
    
    // Push the cube and mark the boxes to delete
    BoxPointType new_corner, temp_corner;
    new_corner = m_local_map.local_map_corner;
    float mov_dist = std::max((m_config.cube_len - 2.0 * m_config.move_thresh * m_config.det_range) * 0.5 * 0.9, double(m_config.det_range * (m_config.move_thresh - 1)));

    for (int i = 0; i < 3; i++)
    {
        temp_corner = m_local_map.local_map_corner;
        if (dist_to_map_edge[i][0] <= det_thresh)
        {
            new_corner.vertex_max[i] -= mov_dist;
            new_corner.vertex_min[i] -= mov_dist;
            temp_corner.vertex_min[i] = m_local_map.local_map_corner.vertex_max[i] - mov_dist;
            m_local_map.cub_to_rm.push_back(temp_corner);
        }
        else if (dist_to_map_edge[i][1] <= det_thresh)
        {
            new_corner.vertex_max[i] += mov_dist;
            new_corner.vertex_min[i] += mov_dist;
            temp_corner.vertex_max[i] = m_local_map.local_map_corner.vertex_min[i] + mov_dist;
            m_local_map.cub_to_rm.push_back(temp_corner);
        }
    }
    m_local_map.local_map_corner = new_corner;

    PointVec points_history;
    m_ikdtree->acquire_removed_points(points_history);

    // Delete the points outside the local map
    if (m_local_map.cub_to_rm.size() > 0)
        m_ikdtree->Delete_Point_Boxes(m_local_map.cub_to_rm);
    return;
}

void LidarProcessor::incrCloudMap()
{
    if (m_cloud_down_lidar->empty())
        return;
        
    // m_cloud_down_world was already transformed inside processGroup / updateChunk with the state
    // at each point group's own time (equivalent to point-by-point undistortion), so it is used
    // as is here and not recomputed.
    int size = m_cloud_down_lidar->size();
    PointVec point_to_add;
    PointVec point_no_need_downsample;
    for (int i = 0; i < size; i++)
    {
        // A point with no neighbours nearby must be added to the map
        if (m_nearest_points[i].empty())
        {
            point_to_add.push_back(m_cloud_down_world->points[i]);
            continue;
        }

        const PointVec &points_near = m_nearest_points[i];
        bool need_add = true;
        PointType mid_point;
        mid_point.x = std::floor(m_cloud_down_world->points[i].x / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;
        mid_point.y = std::floor(m_cloud_down_world->points[i].y / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;
        mid_point.z = std::floor(m_cloud_down_world->points[i].z / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;

        // If the voxel this point falls in holds no point yet, add it directly without downsampling
        if (fabs(points_near[0].x - mid_point.x) > 0.5 * m_config.map_resolution && fabs(points_near[0].y - mid_point.y) > 0.5 * m_config.map_resolution && fabs(points_near[0].z - mid_point.z) > 0.5 * m_config.map_resolution)
        {
            point_no_need_downsample.push_back(m_cloud_down_world->points[i]);
            continue;
        }
        float dist = sq_dist(m_cloud_down_world->points[i], mid_point);

        for (int readd_i = 0; readd_i < m_config.near_search_num; readd_i++)
        {
            // A point with too few neighbours must be added to the map
            if (points_near.size() < static_cast<size_t>(m_config.near_search_num))
                break;
            // If a neighbour lies closer to the voxel centre than this point does, this point
            // need not be added
            if (sq_dist(points_near[readd_i], mid_point) < dist)
            {
                need_add = false;
                break;
            }
        }
        if (need_add)
            point_to_add.push_back(m_cloud_down_world->points[i]);
    }
    m_ikdtree->Add_Points(point_to_add, true);
    m_ikdtree->Add_Points(point_no_need_downsample, false);
}

void LidarProcessor::initCloudMap(PointVec &point_vec)
{
    m_ikdtree->Build(point_vec);
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
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
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
        std::vector<float> point_sq_dist(m_config.near_search_num);
        auto &points_near = m_nearest_points[i];
        m_ikdtree->Nearest_Search(point_world, m_config.near_search_num, points_near, point_sq_dist);
        if (points_near.size() >= static_cast<size_t>(m_config.near_search_num) && point_sq_dist[m_config.near_search_num - 1] <= 5)
            m_point_selected_flag[i] = true;
        else
            m_point_selected_flag[i] = false;
        if (!m_point_selected_flag[i])
            continue;

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
        effect_num++;
    }
    if (effect_num < 1)
        return;

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
