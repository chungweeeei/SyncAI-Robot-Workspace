#ifndef SYNCAI_NAV2_CORE_GLOBAL_PLANNER_HPP_
#define SYNCAI_NAV2_CORE_GLOBAL_PLANNER_HPP_

#include <memory>
#include <string>

#include "tf2_ros/buffer.h"

namespace syncai_nav2_core
{

class GlobalPlanner
{
public:
  using Ptr = std::shared_ptr<GlobalPlanner>;

  virtual ~GlobalPlanner() {}

  virtual void initialize(std::string name, std::shared_ptr<tf2_ros::Buffer> tf_buffer) = 0;
};

}  // namespace syncai_nav2_core

#endif  // SYNCAI_NAV2_CORE_GLOBAL_PLANNER_HPP_