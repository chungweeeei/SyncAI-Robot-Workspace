// Time-correlated action sampling for MPPI (TC-MPPI). Not upstream.
//
// A port of M. Lee and D. Lee, "Time-Correlated Model Predictive Path
// Integral: Smooth Action Generation for Sampling-Based Control", ICRA 2025
// (Algorithm 1, eqs. 13-23). The paper's code is MATLAB and unpublished; this
// is written from the equations. ROS-free and xtensor-free on purpose, like
// syncai_util's StairMode and syncai_costmap's TerrainGrid, so
// test/test_time_correlated.cpp exercises it without a node or a costmap.
//
// One axis (vx or wz) at a time: the weights are per axis and the axes are
// sampled independently, so the joint precision is block diagonal and never
// needs building.
//
// Glossary (the paper's symbols, our names):
//   T        horizon length, OptimizerSettings::time_steps
//   d        correlation depth: the highest finite-difference order penalised,
//            and the number of already-sent commands the samples are
//            conditioned on (the "head", times -d..-1)
//   h        step size, model_dt (= the control period here, so the paper's
//            two step sizes are one)
//   R_i      the weight on the i-th difference of the action, i = 0..d
//   H        sum_i D_i^T R_i D_i, the (d+T)x(d+T) precision over head + tail
//   Htt/Hht  its tail block (TxT) and head-tail block (dxT)
//   B        L^-T where Htt = L L^T: the noise basis; V - U_hat = B E has
//            covariance Htt^-1
//   U_hat    the sampling mean (eq. 19): the warm start, low-passed and joined
//            to the head
//   U_bar    the prior mean (eq. 18, reference 0 at every order)
//
// With d = 0 it is vanilla MPPI: H = R0 I, U_hat = the warm start, B = I /
// sqrt(R0), and the weight term is the optimizer's gamma term with R0 =
// 1 / std^2 and gamma = temperature (up to a per-cycle constant, which the
// softmax drops). The tests pin that.

#ifndef SYNCAI_MPPI_CONTROLLER__TOOLS__TIME_CORRELATED_HPP_
#define SYNCAI_MPPI_CONTROLLER__TOOLS__TIME_CORRELATED_HPP_

#include <cstddef>
#include <optional>
#include <vector>

namespace mppi::tc
{

// Built once by buildAxisPrior() (on reset and on a parameter change) and
// never mutated: the noise thread and the optimizer read it concurrently.
struct AxisPrior
{
  size_t T{0};
  size_t d{0};
  float r0{0.0f};
  std::vector<double> Htt;   // T x T, row-major
  std::vector<double> HhtT;  // T x d, row-major (Hht transposed)
  std::vector<double> L;     // T x T lower Cholesky factor of Htt, row-major
  std::vector<float> B;      // T x T upper, L^-T, row-major
};

// The boundary: T >= 1, h > 0, R.size() == d + 1, every R_i >= 0 and R0 > 0
// (the last makes Htt positive definite whatever the rest are). Anything else
// is nullopt, and the caller refuses the parameters.
std::optional<AxisPrior> buildAxisPrior(
  size_t T, size_t d, float h, const std::vector<float> & R);

// U_hat = Htt^-1 (R0 u_warm - Hht^T u_head). u_warm has T entries, u_head d,
// oldest first (u_head[d-1] is the command sent last).
std::vector<float> samplingMean(
  const AxisPrior & p, const float * u_warm, const float * u_head);

// U_bar = -Htt^-1 Hht^T u_head.
std::vector<float> priorMean(const AxisPrior & p, const float * u_head);

// out[k] = B E[k] for each of `batch` rows of T standard normals.
void correlate(const AxisPrior & p, const float * E, float * out, size_t batch);

// costs[k] += lambda (U_hat - U_bar)^T Htt V[k], V row-major batch x T: the
// importance-sampling term of eq. 23.
void addWeightTerm(
  const AxisPrior & p, const float * u_hat, const float * u_bar, const float * V,
  float lambda, float * costs, size_t batch);

// sqrt(diag(Htt^-1)): each step's marginal standard deviation, for tuning the
// weights against the iid sampler's vx_std / wz_std.
std::vector<float> marginalStd(const AxisPrior & p);

}  // namespace mppi::tc

#endif  // SYNCAI_MPPI_CONTROLLER__TOOLS__TIME_CORRELATED_HPP_
