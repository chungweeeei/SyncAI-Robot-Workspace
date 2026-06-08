#include "syncai_costmap_2d/costmap_2d_ros.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace syncai_costmap_2d
{
Costmap2DROS::Costmap2DROS(const std::string & name) : Costmap2DROS(name, "/", name)
{
}

Costmap2DROS::Costmap2DROS(const rclcpp::NodeOptions & options) : Costmap2DROS("costmap", options)
{
}
}  // namespace syncai_costmap_2d