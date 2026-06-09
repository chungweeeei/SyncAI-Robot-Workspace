#ifndef SYNCAI_NAV2_COSTMAP_2D__EXCEPTIONS_HPP_
#define SYNCAI_NAV2_COSTMAP_2D__EXCEPTIONS_HPP_

#include <memory>
#include <stdexcept>
#include <string>

namespace syncai_costmap_2d
{
/**
 * @class CollisionCheckerException
 * @brief Exceptions thrown if collision checker determines a pose is in
 * collision with the environment costmap
 */

class CollisionCheckerException : public std::runtime_error
{
  /**
 * @class CollisionCheckerException
 * @brief Exceptions thrown if collision checker determines a pose is in
 * collision with the environment costmap
 */
public:
  explicit CollisionCheckerException(const std::string description)
  : std::runtime_error(description)
  {
  }
};

/**
 * @class IllegalPoseException
 * @brief Thrown when CollisionChecker encounters a fatal error
 */
class IllegalPoseException : public CollisionCheckerException
{
public:
  IllegalPoseException(const std::string name, const std::string description)
  : CollisionCheckerException(description), name_(name)
  {
  }
  std::string getCriticName() const { return name_; }

protected:
  std::string name_;
};
}  // namespace syncai_costmap_2d

#endif  // SYNCAI_NAV2_COSTMAP_2D__EXCEPTIONS_HPP_