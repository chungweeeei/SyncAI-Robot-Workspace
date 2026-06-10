#ifndef SYNCAI_AMCL__SENSORS__LASER__LASER_HPP_
#define SYNCAI_AMCL__SENSORS__LASER__LASER_HPP_

#include <string>

#include "syncai_amcl/map/map.hpp"
#include "syncai_amcl/pf/pf.hpp"
#include "syncai_amcl/pf/pf_pdf.hpp"

namespace syncai_amcl
{

// Forward declarations
class LaserData;

/*
 * @class Laser
 * @brief Base class for laser sensor models
 */
class Laser
{
public:
  /**
   * @brief A Laser constructor
   * @param max_beams number of beams to use
   * @param map Map pointer to use
   */
  Laser(size_t max_beams, map_t * map);

  /*
   * @brief Laser destructor
   */
  virtual ~Laser();

  /*
   * @brief Run a sensor update on laser
   * @param pf Particle filter to use
   * @param data Laser data to use
   * @return if it was succesful
   */
  virtual bool sensorUpdate(pf_t * pf, LaserData * data) = 0;

  /*
   * @brief Set the laser pose from an update
   * @param laser_pose Pose of the laser
   */
  void SetLaserPose(pf_vector_t & laser_pose);

protected:
  double z_hit_;
  double z_rand_;
  double sigma_hit_;

  /*
   * @brief Reallocate weights
   * @param max_samples Max number of samples
   * @param max_obs number of observations
   */
  void reallocTempData(int max_samples, int max_obs);
  map_t * map_;
  pf_vector_t laser_pose_;
  int max_beams_;
  int max_samples_;
  int max_obs_;
  double ** temp_obs_;
};

/*
 * @class LaserData
 * @brief Class of laser data to process
 */
class LaserData
{
public:
  Laser * laser;

  /*
   * @brief LaserData constructor
   */
  LaserData() { ranges = NULL; }
  /*
   * @brief LaserData destructor
   */
  virtual ~LaserData() { delete[] ranges; }

public:
  int range_count;
  double range_max;
  double (*ranges)[2];
};

/*
 * @class BeamModel
 * @brief Beam model laser sensor
 */
class BeamModel : public Laser
{
public:
  /*
   * @brief BeamModel constructor
   */
  BeamModel(
    double z_hit, double z_short, double z_max, double z_rand, double sigma_hit,
    double lambda_short, double chi_outlier, size_t max_beams, map_t * map);

  /*
   * @brief Run a sensor update on laser
   * @param pf Particle filter to use
   * @param data Laser data to use
   * @return if it was succesful
   */
  bool sensorUpdate(pf_t * pf, LaserData * data);

private:
  static double sensorFunction(LaserData * data, pf_sample_set_t * set);
  double z_short_;
  double z_max_;
  double lambda_short_;
  double chi_outlier_;
};

/*
 * @class LikelihoodFieldModel
 * @brief likelihood field model laser sensor
 */
class LikelihoodFieldModel : public Laser
{
public:
  /*
   * @brief BeamModel constructor
   */
  LikelihoodFieldModel(
    double z_hit, double z_rand, double sigma_hit, double max_occ_dist, size_t max_beams,
    map_t * map);

  /*
   * @brief Run a sensor update on laser
   * @param pf Particle filter to use
   * @param data Laser data to use
   * @return if it was succesful
   */
  bool sensorUpdate(pf_t * pf, LaserData * data);

private:
  /*
   * @brief Perform the update function
   * @param data Laser data to use
   * @param pf Particle filter to use
   * @return if it was succesful
   */
  static double sensorFunction(LaserData * data, pf_sample_set_t * set);
};

/*
 * @class LikelihoodFieldModelProb
 * @brief likelihood prob model laser sensor
 */
class LikelihoodFieldModelProb : public Laser
{
public:
  /*
   * @brief BeamModel constructor
   */
  LikelihoodFieldModelProb(
    double z_hit, double z_rand, double sigma_hit, double max_occ_dist, bool do_beamskip,
    double beam_skip_distance, double beam_skip_threshold, double beam_skip_error_threshold,
    size_t max_beams, map_t * map);

  /*
   * @brief Run a sensor update on laser
   * @param pf Particle filter to use
   * @param data Laser data to use
   * @return if it was succesful
   */
  bool sensorUpdate(pf_t * pf, LaserData * data);

private:
  /*
   * @brief Perform the update function
   * @param data Laser data to use
   * @param pf Particle filter to use
   * @return if it was succesful
   */
  static double sensorFunction(LaserData * data, pf_sample_set_t * set);
  bool do_beamskip_;
  double beam_skip_distance_;
  double beam_skip_threshold_;
  double beam_skip_error_threshold_;
};

}  // namespace syncai_amcl

#endif  // SYNCAI_AMCL__SENSORS__LASER__LASER_HPP_
