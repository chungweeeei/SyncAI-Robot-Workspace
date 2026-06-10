#ifndef SYNCAI_AMCL__MOTION_MODEL__OMNI_MOTION_MODEL_HPP_
#define SYNCAI_AMCL__MOTION_MODEL__OMNI_MOTION_MODEL_HPP_

#include <sys/types.h>
#include <math.h>
#include <algorithm>
#include "syncai_amcl/motion_model/motion_model.hpp"
#include "syncai_amcl/angleutils.hpp"


namespace syncai_amcl
{

class OmniMotionModel : public syncai_amcl::MotionModel
{
public:
  virtual void initialize(
    double alpha1, double alpha2, double alpha3, double alpha4,
    double alpha5);
  virtual void odometryUpdate(pf_t * pf, const pf_vector_t & pose, const pf_vector_t & delta);

private:
  double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
};
}  // namespace syncai_amcl
#endif  // SYNCAI_AMCL__MOTION_MODEL__OMNI_MOTION_MODEL_HPP_
