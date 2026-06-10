#ifndef SYNCAI_COSTMAP_2D__OBSERVATION_HPP_
#define SYNCAI_COSTMAP_2D__OBSERVATION_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace syncai_costmap_2d
{
/**
 * @brief Stores an observation in terms of a point cloud and the origin of the source
 * @note Tried to make members and constructor arguments const but the compiler would not accept
 * the default assignment operator for vector insertion!
 */
class Observation
{
public:
  /**
   * @brief  Creates an empty observation
   */
  Observation()
  : cloud_(new sensor_msgs::msg::PointCloud2()), obstacle_max_range_(0.0), obstacle_min_range_(0.0),
    raytrace_max_range_(0.0), raytrace_min_range_(0.0)
  {
  }
  /**
   * @brief A destructor
   */
  virtual ~Observation() { delete cloud_; }

  /**
   * @brief  Copy assignment operator
   * @param obs The observation to copy
   */
  Observation & operator=(const Observation & obs)
  {
    origin_ = obs.origin_;
    cloud_ = new sensor_msgs::msg::PointCloud2(*(obs.cloud_));
    obstacle_max_range_ = obs.obstacle_max_range_;
    obstacle_min_range_ = obs.obstacle_min_range_;
    raytrace_max_range_ = obs.raytrace_max_range_;
    raytrace_min_range_ = obs.raytrace_min_range_;

    return *this;
  }

  /**
   * @brief  Creates an observation from an origin point and a point cloud
   * @param origin The origin point of the observation
   * @param cloud The point cloud of the observation
   * @param obstacle_max_range The range out to which an observation should be able to insert
   * obstacles
   * @param obstacle_min_range The range from which an observation should be able to insert
   * obstacles
   * @param raytrace_max_range The range out to which an observation should be able to clear via
   * raytracing
   * @param raytrace_min_range The range from which an observation should be able to clear via
   * raytracing
   */
  Observation(
    geometry_msgs::msg::Point & origin, const sensor_msgs::msg::PointCloud2 & cloud,
    double obstacle_max_range, double obstacle_min_range, double raytrace_max_range,
    double raytrace_min_range)
  : origin_(origin), cloud_(new sensor_msgs::msg::PointCloud2(cloud)),
    obstacle_max_range_(obstacle_max_range), obstacle_min_range_(obstacle_min_range),
    raytrace_max_range_(raytrace_max_range), raytrace_min_range_(raytrace_min_range)
  {
  }

  /**
   * @brief  Copy constructor
   * @param obs The observation to copy
   */
  Observation(const Observation & obs)
  : origin_(obs.origin_), cloud_(new sensor_msgs::msg::PointCloud2(*(obs.cloud_))),
    obstacle_max_range_(obs.obstacle_max_range_), obstacle_min_range_(obs.obstacle_min_range_),
    raytrace_max_range_(obs.raytrace_max_range_), raytrace_min_range_(obs.raytrace_min_range_)
  {
  }

  /**
   * @brief  Creates an observation from a point cloud
   * @param cloud The point cloud of the observation
   * @param obstacle_max_range The range out to which an observation should be able to insert
   * obstacles
   * @param obstacle_min_range The range from which an observation should be able to insert
   * obstacles
   */
  Observation(
    const sensor_msgs::msg::PointCloud2 & cloud, double obstacle_max_range,
    double obstacle_min_range)
  : cloud_(new sensor_msgs::msg::PointCloud2(cloud)), obstacle_max_range_(obstacle_max_range),
    obstacle_min_range_(obstacle_min_range), raytrace_max_range_(0.0), raytrace_min_range_(0.0)
  {
  }

  geometry_msgs::msg::Point origin_;
  sensor_msgs::msg::PointCloud2 * cloud_;
  double obstacle_max_range_, obstacle_min_range_, raytrace_max_range_, raytrace_min_range_;
};

}  // namespace syncai_costmap_2d
#endif  // SYNCAI_COSTMAP_2D__OBSERVATION_HPP_
