#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Eigen>
#include <vector>

// Shared aliases of the localizer's maths (ported upstream code, MIT). Kept
// global rather than in the syncai_localizer namespace because icp_localizer.*
// is upstream-derived and reads them unqualified -- the same treatment
// syncai_mapping gives pgos/commons.h and hba/commons.h. Nothing else in the
// workspace includes this header, so the global names cannot collide; they
// would if this tree ever shared a translation unit with one of those two.

using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<PointType>;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

using M3D = Eigen::Matrix3d;
using V3D = Eigen::Vector3d;
using M3F = Eigen::Matrix3f;
using V3F = Eigen::Vector3f;
using M4F = Eigen::Matrix4f;
using V4F = Eigen::Vector4f;
