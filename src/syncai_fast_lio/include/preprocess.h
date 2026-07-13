// #include <ros/ros.h>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

/**
 * pcl::PointXYZINormal 
 * x, y, z： 座標
 * intensity： 反射強度
 * normal_x/y/z： 法向量（特徵萃取時取用）
 * curvature：被挪用來存「每點相對時間偏移」
 */

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

enum LID_TYPE { AVIA = 1, VELO16, OUST64, MID360 };  // {1, 2, 3}
enum TIME_UNIT {
  SEC = 0,
  MS = 1,
  US = 2,
  NS = 3
};  // 不同 LiDAR 的時間欄位單位不同（秒 / 毫秒 / 微秒 / 奈秒）
enum Feature { Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint };  // 特徵萃取
enum Surround { Prev, Next };  // 檢查某點時看「前一點」還是「後一點」
enum E_jump { Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind };

struct orgtype
{
  double range;
  double dista;
  double angle[2];
  double intersect;
  E_jump edj[2];
  Feature ftype;
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

namespace velodyne_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(
  velodyne_ros::Point, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                         float, time, time)(uint16_t, ring, ring))

namespace ouster_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t;
  uint16_t reflectivity;
  uint8_t ring;
  uint16_t ambient;
  uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

namespace livox_ros
{
typedef struct {
  float x;            /**< X axis, Unit:m */
  float y;            /**< Y axis, Unit:m */
  float z;            /**< Z axis, Unit:m */
  float reflectivity; /**< Reflectivity   */
  uint8_t tag;        /**< Livox point tag   */
  uint8_t line;       /**< Laser line id     */
} LivoxPointXyzrtl;
}
POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::LivoxPointXyzrtl,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, reflectivity, reflectivity)
    (uint8_t, tag, tag)
    (uint8_t, line, line)
)

class Preprocess
{
  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();
  
  void process(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void process(const sensor_msgs::msg::PointCloud2::UniquePtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  /**
   * 全部點 / 邊角點 / 平面點
   * 1. pl_surf 是主要輸出。 pl_corn（邊角）、pl_full（全部）只有特徵萃取才有意義
   **/
  PointCloudXYZI pl_full, pl_corn, pl_surf;  
  PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
  vector<orgtype> typess[128]; //maximum 128 line lidar

  float time_unit_scale;
  int lidar_type, point_filter_num, N_SCANS, SCAN_RATE, time_unit;
  double blind;
  bool feature_enabled, given_offset_time;

private:
  // 各廠牌 handler：process() 依 lidar_type 分派到這些
  void avia_handler(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg);
  void oust64_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);
  void velodyne_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);
  void mid360_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);
  void default_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);
  
  // 特徵萃取 (feature_enabled 才會被呼叫)
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);
  void pub_func(PointCloudXYZI &pl, const rclcpp::Time &ct);

  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};
