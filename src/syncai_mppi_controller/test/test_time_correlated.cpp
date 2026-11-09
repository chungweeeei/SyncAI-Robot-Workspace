// ROS-free tests of the TC-MPPI sampling core (tools/time_correlated.hpp).

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "syncai_mppi_controller/tools/time_correlated.hpp"

using mppi::tc::AxisPrior;
using mppi::tc::buildAxisPrior;

namespace
{

std::vector<float> normals(size_t n, unsigned seed)
{
  std::mt19937 rng(seed);
  std::normal_distribution<float> nd(0.0f, 1.0f);
  std::vector<float> v(n);
  for (auto & x : v) {
    x = nd(rng);
  }
  return v;
}

}  // namespace

// The one unit worked by hand: T = 3, d = 1, h = 1, R = [1, 1]. Over the
// four steps -1..2, H = I + D1^T D1 = tridiag(-1; 2, 3, 3, 2; -1). Its head
// row is (2, -1, 0, 0), so Hht = (-1, 0, 0) and Htt is the lower 3x3.
TEST(TimeCorrelated, HandWorkedPrecision)
{
  const auto p = buildAxisPrior(3, 1, 1.0f, {1.0f, 1.0f});
  ASSERT_TRUE(p);
  const std::vector<double> htt{3, -1, 0, -1, 3, -1, 0, -1, 2};
  for (size_t i = 0; i < 9; ++i) {
    EXPECT_NEAR(p->Htt[i], htt[i], 1e-9) << i;
  }
  const std::vector<double> hht_t{-1, 0, 0};
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(p->HhtT[i], hht_t[i], 1e-9) << i;
  }
}

// The step size divides each difference: R1 weighs (v_t - v_t-1) / h, so
// halving h quadruples R1's share.
TEST(TimeCorrelated, StepSizeScalesDifferences)
{
  const auto p = buildAxisPrior(3, 1, 0.5f, {1.0f, 1.0f});
  ASSERT_TRUE(p);
  // H = I + 4 D1^T D1: tail diag 1 + 8, 1 + 8, 1 + 4; off-diagonal -4.
  EXPECT_NEAR(p->Htt[0], 9.0, 1e-9);
  EXPECT_NEAR(p->Htt[8], 5.0, 1e-9);
  EXPECT_NEAR(p->Htt[1], -4.0, 1e-9);
  EXPECT_NEAR(p->HhtT[0], -4.0, 1e-9);
}

TEST(TimeCorrelated, RejectsIllegalParameters)
{
  EXPECT_FALSE(buildAxisPrior(0, 0, 0.05f, {1.0f}));             // no horizon
  EXPECT_FALSE(buildAxisPrior(8, 1, 0.0f, {1.0f, 1.0f}));        // h = 0
  EXPECT_FALSE(buildAxisPrior(8, 1, -0.05f, {1.0f, 1.0f}));      // h < 0
  EXPECT_FALSE(buildAxisPrior(8, 2, 0.05f, {1.0f, 1.0f}));       // R too short
  EXPECT_FALSE(buildAxisPrior(8, 1, 0.05f, {0.0f, 1.0f}));       // R0 = 0
  EXPECT_FALSE(buildAxisPrior(8, 1, 0.05f, {1.0f, -1.0f}));      // R1 < 0
  EXPECT_FALSE(buildAxisPrior(8, 1, 0.05f, {NAN, 1.0f}));        // not a number
  EXPECT_TRUE(buildAxisPrior(8, 1, 0.05f, {1.0f, 0.0f}));        // R1 = 0 is fine
}

// d = 0 is vanilla MPPI: the mean is the warm start, the samples are iid with
// std 1 / sqrt(R0), and the weight term differs from the optimizer's gamma
// term only by a constant per cycle, which the softmax drops.
TEST(TimeCorrelated, DepthZeroIsVanillaMppi)
{
  const size_t T = 6;
  const float std_dev = 0.3f;
  const float r0 = 1.0f / (std_dev * std_dev);
  const auto p = buildAxisPrior(T, 0, 0.05f, {r0});
  ASSERT_TRUE(p);

  const std::vector<float> u_warm{0.1f, 0.2f, 0.4f, 0.5f, 0.5f, 0.3f};
  const auto u_hat = mppi::tc::samplingMean(*p, u_warm.data(), nullptr);
  const auto u_bar = mppi::tc::priorMean(*p, nullptr);
  for (size_t t = 0; t < T; ++t) {
    EXPECT_NEAR(u_hat[t], u_warm[t], 1e-6);
    EXPECT_NEAR(u_bar[t], 0.0f, 1e-6);
    for (size_t s = 0; s < T; ++s) {
      EXPECT_NEAR(p->B[t * T + s], t == s ? std_dev : 0.0f, 1e-6);
    }
  }

  // The optimizer's term (optimizer.cpp, updateControlSequence):
  // gamma / std^2 * sum_t u_t (V_t - u_t), with gamma = temperature.
  const float temperature = 0.3f;
  const size_t batch = 5;
  const auto V = normals(batch * T, 7);
  std::vector<float> tc(batch, 0.0f);
  mppi::tc::addWeightTerm(*p, u_hat.data(), u_bar.data(), V.data(), temperature, tc.data(), batch);
  std::vector<float> diff(batch);
  for (size_t k = 0; k < batch; ++k) {
    float vanilla = 0.0f;
    for (size_t t = 0; t < T; ++t) {
      vanilla += temperature / (std_dev * std_dev) * u_warm[t] * (V[k * T + t] - u_warm[t]);
    }
    diff[k] = tc[k] - vanilla;
  }
  for (size_t k = 1; k < batch; ++k) {
    EXPECT_NEAR(diff[k], diff[0], 1e-4);
  }
}

// The samples' covariance is Htt^-1.
TEST(TimeCorrelated, SampleCovarianceIsInversePrecision)
{
  const size_t T = 8;
  const auto p = buildAxisPrior(T, 2, 0.05f, {10.0f, 2.0f, 0.01f});
  ASSERT_TRUE(p);
  const size_t batch = 40000;
  const auto E = normals(batch * T, 11);
  std::vector<float> out(batch * T);
  mppi::tc::correlate(*p, E.data(), out.data(), batch);

  // Htt^-1 column by column: the prior mean of a unit "warm start" with R0
  // stripped is not available, so solve directly through B: Htt^-1 = B B^T.
  const auto sd = mppi::tc::marginalStd(*p);
  double max_var = 0.0;
  for (auto s : sd) {
    max_var = std::max(max_var, static_cast<double>(s) * s);
  }
  for (size_t i = 0; i < T; ++i) {
    for (size_t j = 0; j < T; ++j) {
      double expected = 0.0;
      for (size_t s = 0; s < T; ++s) {
        expected += static_cast<double>(p->B[i * T + s]) * p->B[j * T + s];
      }
      double sample = 0.0;
      for (size_t k = 0; k < batch; ++k) {
        sample += static_cast<double>(out[k * T + i]) * out[k * T + j];
      }
      sample /= batch;
      EXPECT_NEAR(sample, expected, 0.05 * max_var) << i << "," << j;
    }
  }
  // And B B^T really is Htt^-1: Htt (B B^T) = I.
  for (size_t i = 0; i < T; ++i) {
    for (size_t j = 0; j < T; ++j) {
      double prod = 0.0;
      for (size_t m = 0; m < T; ++m) {
        double bbt = 0.0;
        for (size_t s = 0; s < T; ++s) {
          bbt += static_cast<double>(p->B[m * T + s]) * p->B[j * T + s];
        }
        prod += p->Htt[i * T + m] * bbt;
      }
      EXPECT_NEAR(prod, i == j ? 1.0 : 0.0, 1e-3) << i << "," << j;
    }
  }
}

// A constant is in the null space of every difference, so a constant head
// and warm start give back that constant.
TEST(TimeCorrelated, ConstantsPassThrough)
{
  const size_t T = 32;
  const auto p = buildAxisPrior(T, 2, 0.05f, {11.0f, 1.0f, 0.02f});
  ASSERT_TRUE(p);
  const std::vector<float> u_warm(T, 0.6f);
  const std::vector<float> u_head(2, 0.6f);
  const auto u_hat = mppi::tc::samplingMean(*p, u_warm.data(), u_head.data());
  for (size_t t = 0; t < T; ++t) {
    EXPECT_NEAR(u_hat[t], 0.6f, 1e-4) << t;
  }
}

// The mean starts where the last command left off, not where the warm start
// jumps to, when the difference weight dominates.
TEST(TimeCorrelated, MeanJoinsTheSentCommands)
{
  const size_t T = 32;
  const auto p = buildAxisPrior(T, 1, 0.05f, {1.0f, 1.0f});
  ASSERT_TRUE(p);
  const std::vector<float> u_warm(T, 0.8f);
  const std::vector<float> u_head{0.0f};
  const auto u_hat = mppi::tc::samplingMean(*p, u_warm.data(), u_head.data());
  EXPECT_LT(std::fabs(u_hat[0] - u_head[0]), 0.1f);
  // ... and still rises toward the warm start along the horizon.
  EXPECT_GT(u_hat[T - 1], u_hat[0]);
  for (size_t t = 1; t < T; ++t) {
    EXPECT_GE(u_hat[t], u_hat[t - 1] - 1e-6f) << t;
  }
}

// At equal marginal spread, TC samples have far less second-difference
// energy than iid ones (6 sigma^2 for iid).
TEST(TimeCorrelated, SamplesAreSmootherThanIid)
{
  const size_t T = 32;
  const auto p = buildAxisPrior(T, 2, 0.05f, {11.0f, 1.0f, 0.02f});
  ASSERT_TRUE(p);
  const size_t batch = 2000;
  const auto E = normals(batch * T, 3);
  std::vector<float> out(batch * T);
  mppi::tc::correlate(*p, E.data(), out.data(), batch);

  const auto sd = mppi::tc::marginalStd(*p);
  double var = 0.0;
  for (auto s : sd) {
    var += static_cast<double>(s) * s;
  }
  var /= T;
  double tc_d2 = 0.0;
  double iid_d2 = 0.0;
  for (size_t k = 0; k < batch; ++k) {
    for (size_t t = 1; t + 1 < T; ++t) {
      const double a = out[k * T + t + 1] - 2.0 * out[k * T + t] + out[k * T + t - 1];
      const double b = E[k * T + t + 1] - 2.0 * E[k * T + t] + E[k * T + t - 1];
      tc_d2 += a * a / var;
      iid_d2 += b * b;  // unit variance
    }
  }
  EXPECT_LT(tc_d2, iid_d2 / 5.0);
}

TEST(TimeCorrelated, WeightTermVanishesWhenMeansAgree)
{
  const size_t T = 8;
  const auto p = buildAxisPrior(T, 2, 0.05f, {10.0f, 1.0f, 0.1f});
  ASSERT_TRUE(p);
  const std::vector<float> u(T, 0.3f);
  const auto V = normals(4 * T, 5);
  std::vector<float> costs{1.0f, 2.0f, 3.0f, 4.0f};
  mppi::tc::addWeightTerm(*p, u.data(), u.data(), V.data(), 0.3f, costs.data(), 4);
  EXPECT_FLOAT_EQ(costs[0], 1.0f);
  EXPECT_FLOAT_EQ(costs[3], 4.0f);
}

// Both axes or neither, with the axis named on refusal.
TEST(TimeCorrelated, PriorsAreBuiltForBothAxesOrNeither)
{
  std::string error;
  EXPECT_TRUE(mppi::tc::buildPriors(32, 2, 0.05f, {11.0, 1.0, 0.02}, {6.0, 0.5, 0.01}, error));
  EXPECT_TRUE(error.empty());
  EXPECT_FALSE(mppi::tc::buildPriors(32, 2, 0.05f, {11.0, 1.0, 0.02}, {6.0, 0.5}, error));
  EXPECT_NE(error.find("tc_wz_weights"), std::string::npos);
  EXPECT_FALSE(mppi::tc::buildPriors(32, 2, 0.05f, {0.0, 1.0, 0.02}, {6.0, 0.5, 0.01}, error));
  EXPECT_NE(error.find("tc_vx_weights"), std::string::npos);
}
