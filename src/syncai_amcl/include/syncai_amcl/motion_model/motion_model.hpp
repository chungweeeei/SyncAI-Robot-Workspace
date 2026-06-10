#ifndef SYNCAI_AMCL__MOTION_MODEL__MOTION_MODEL_HPP_
#define SYNCAI_AMCL__MOTION_MODEL__MOTION_MODEL_HPP_

#include <memory>
#include <string>

#include "syncai_amcl/pf/pf.hpp"
#include "syncai_amcl/pf/pf_pdf.hpp"

namespace syncai_amcl
{

/**
 * @class syncai_amcl::MotionModel
 * @brief An abstract motion model class
 */
class MotionModel
{
public:
  virtual ~MotionModel() = default;

  /**
   * @brief An factory to create motion models
   * @param type Type of motion model to create in factory
   * @param alpha1 error parameters, see documentation
   * @param alpha2 error parameters, see documentation
   * @param alpha3 error parameters, see documentation
   * @param alpha4 error parameters, see documentation
   * @param alpha5 error parameters, see documentation
   * @return MotionModel A pointer to the motion model it created
   */
  virtual void initialize(
    double alpha1, double alpha2, double alpha3, double alpha4, double alpha5) = 0;

  /**
   * @brief Update on new odometry data
   * @param pf The particle filter to update
   * @param pose pose of robot in odometry update
   * @param delta change in pose in odometry update
   */
  virtual void odometryUpdate(pf_t * pf, const pf_vector_t & pose, const pf_vector_t & delta) = 0;
};
}  // namespace syncai_amcl

#endif  // SYNCAI_AMCL__MOTION_MODEL__MOTION_MODEL_HPP_
