#ifndef SYNCAI_NAV_CORE__EXCEPTIONS_HPP_
#define SYNCAI_NAV_CORE__EXCEPTIONS_HPP_

#include <memory>
#include <stdexcept>
#include <string>

namespace syncai_nav_core
{

class PlannerException : public std::runtime_error
{
public:
  explicit PlannerException(const std::string description)
  : std::runtime_error(description) {}
  using Ptr = std::shared_ptr<PlannerException>;
};

}  // namespace syncai_nav_core

#endif  // SYNCAI_NAV_CORE__EXCEPTIONS_HPP_
