// TC-MPPI sampling core; see tools/time_correlated.hpp for the maths and the
// glossary. Matrices are at most (d + T)^2 = 34^2 here, so plain loops in
// double: a dense Cholesky of a 32x32 is microseconds, runs only on reset and
// on a parameter change, and needs no linear-algebra dependency (the image has
// no xtensor-blas).

#include "syncai_mppi_controller/tools/time_correlated.hpp"

#include <cmath>

namespace mppi::tc
{

namespace
{

// The i-th forward difference's coefficients: (-1)^(i-j) C(i, j), j = 0..i.
std::vector<double> differenceStencil(size_t order)
{
  std::vector<double> c{1.0};
  for (size_t k = 0; k < order; ++k) {
    std::vector<double> next(c.size() + 1, 0.0);
    for (size_t j = 0; j < c.size(); ++j) {
      next[j] -= c[j];
      next[j + 1] += c[j];
    }
    c = next;
  }
  return c;
}

// H = sum_i R_i / h^(2i) D_i^T D_i over n = d + T steps, row-major n x n.
std::vector<double> precision(size_t n, size_t d, double h, const std::vector<float> & R)
{
  std::vector<double> H(n * n, 0.0);
  for (size_t i = 0; i <= d; ++i) {
    const double w = R[i] / std::pow(h, 2.0 * static_cast<double>(i));
    const auto c = differenceStencil(i);
    for (size_t r = 0; r + i < n; ++r) {
      for (size_t a = 0; a <= i; ++a) {
        for (size_t b = 0; b <= i; ++b) {
          H[(r + a) * n + (r + b)] += w * c[a] * c[b];
        }
      }
    }
  }
  return H;
}

// Lower Cholesky factor of the T x T matrix A, or empty if A is not
// positive definite.
std::vector<double> cholesky(const std::vector<double> & A, size_t T)
{
  std::vector<double> L(T * T, 0.0);
  for (size_t j = 0; j < T; ++j) {
    double diag = A[j * T + j];
    for (size_t k = 0; k < j; ++k) {
      diag -= L[j * T + k] * L[j * T + k];
    }
    if (!(diag > 0.0)) {
      return {};
    }
    L[j * T + j] = std::sqrt(diag);
    for (size_t i = j + 1; i < T; ++i) {
      double s = A[i * T + j];
      for (size_t k = 0; k < j; ++k) {
        s -= L[i * T + k] * L[j * T + k];
      }
      L[i * T + j] = s / L[j * T + j];
    }
  }
  return L;
}

// x = Htt^-1 rhs through L: L y = rhs, then L^T x = y.
std::vector<float> solveHtt(const AxisPrior & p, std::vector<double> rhs)
{
  const size_t T = p.T;
  for (size_t i = 0; i < T; ++i) {
    for (size_t k = 0; k < i; ++k) {
      rhs[i] -= p.L[i * T + k] * rhs[k];
    }
    rhs[i] /= p.L[i * T + i];
  }
  for (size_t i = T; i-- > 0;) {
    for (size_t k = i + 1; k < T; ++k) {
      rhs[i] -= p.L[k * T + i] * rhs[k];
    }
    rhs[i] /= p.L[i * T + i];
  }
  return std::vector<float>(rhs.begin(), rhs.end());
}

// -Hht^T u_head, the head's pull on the tail (zeros when d = 0).
std::vector<double> headTerm(const AxisPrior & p, const float * u_head)
{
  std::vector<double> v(p.T, 0.0);
  for (size_t t = 0; t < p.T; ++t) {
    for (size_t j = 0; j < p.d; ++j) {
      v[t] -= p.HhtT[t * p.d + j] * u_head[j];
    }
  }
  return v;
}

bool validParameters(size_t T, size_t d, float h, const std::vector<float> & R)
{
  if (T == 0 || !(h > 0.0f) || !std::isfinite(h) || R.size() != d + 1 || !(R[0] > 0.0f)) {
    return false;
  }
  for (float r : R) {
    if (!(r >= 0.0f) || !std::isfinite(r)) {
      return false;
    }
  }
  return true;
}

}  // namespace

std::optional<AxisPrior> buildAxisPrior(size_t T, size_t d, float h, const std::vector<float> & R)
{
  if (!validParameters(T, d, h, R)) {
    return std::nullopt;
  }
  const size_t n = d + T;
  const auto H = precision(n, d, h, R);

  AxisPrior p;
  p.T = T;
  p.d = d;
  p.r0 = R[0];
  p.Htt.resize(T * T);
  p.HhtT.resize(T * d);
  for (size_t t = 0; t < T; ++t) {
    for (size_t s = 0; s < T; ++s) {
      p.Htt[t * T + s] = H[(d + t) * n + (d + s)];
    }
    for (size_t j = 0; j < d; ++j) {
      p.HhtT[t * d + j] = H[(d + t) * n + j];
    }
  }
  p.L = cholesky(p.Htt, T);
  if (p.L.empty()) {
    return std::nullopt;
  }
  // B = L^-T. Column j of L^-1 solves L x = e_j; B's row j is that column.
  p.B.assign(T * T, 0.0f);
  for (size_t j = 0; j < T; ++j) {
    std::vector<double> x(T, 0.0);
    for (size_t i = j; i < T; ++i) {
      double s = i == j ? 1.0 : 0.0;
      for (size_t k = j; k < i; ++k) {
        s -= p.L[i * T + k] * x[k];
      }
      x[i] = s / p.L[i * T + i];
    }
    for (size_t i = j; i < T; ++i) {
      p.B[j * T + i] = static_cast<float>(x[i]);
    }
  }
  return p;
}

std::vector<float> samplingMean(const AxisPrior & p, const float * u_warm, const float * u_head)
{
  auto rhs = headTerm(p, u_head);
  for (size_t t = 0; t < p.T; ++t) {
    rhs[t] += static_cast<double>(p.r0) * u_warm[t];
  }
  return solveHtt(p, std::move(rhs));
}

std::vector<float> priorMean(const AxisPrior & p, const float * u_head)
{
  return solveHtt(p, headTerm(p, u_head));
}

void correlate(const AxisPrior & p, const float * E, float * out, size_t batch)
{
  const size_t T = p.T;
  for (size_t k = 0; k < batch; ++k) {
    const float * e = E + k * T;
    float * o = out + k * T;
    for (size_t t = 0; t < T; ++t) {
      float s = 0.0f;
      for (size_t m = t; m < T; ++m) {  // B is upper triangular
        s += p.B[t * T + m] * e[m];
      }
      o[t] = s;
    }
  }
}

void addWeightTerm(
  const AxisPrior & p, const float * u_hat, const float * u_bar, const float * V, float lambda,
  float * costs, size_t batch)
{
  const size_t T = p.T;
  std::vector<float> w(T, 0.0f);  // lambda Htt (U_hat - U_bar)
  for (size_t t = 0; t < T; ++t) {
    double s = 0.0;
    for (size_t m = 0; m < T; ++m) {
      s += p.Htt[t * T + m] * (static_cast<double>(u_hat[m]) - u_bar[m]);
    }
    w[t] = static_cast<float>(lambda * s);
  }
  for (size_t k = 0; k < batch; ++k) {
    float s = 0.0f;
    for (size_t t = 0; t < T; ++t) {
      s += w[t] * V[k * T + t];
    }
    costs[k] += s;
  }
}

std::shared_ptr<const Priors> buildPriors(
  size_t T, size_t d, float h, const std::vector<double> & vx_weights,
  const std::vector<double> & wz_weights, std::string & error)
{
  auto vx = buildAxisPrior(T, d, h, std::vector<float>(vx_weights.begin(), vx_weights.end()));
  auto wz = buildAxisPrior(T, d, h, std::vector<float>(wz_weights.begin(), wz_weights.end()));
  if (!vx || !wz) {
    error = std::string(!vx ? "tc_vx_weights" : "tc_wz_weights") +
            " needs tc_depth + 1 = " + std::to_string(d + 1) +
            " finite weights, the first > 0 and the rest >= 0";
    return nullptr;
  }
  return std::make_shared<const Priors>(Priors{std::move(*vx), std::move(*wz)});
}

std::vector<float> marginalStd(const AxisPrior & p)
{
  std::vector<float> sd(p.T);
  for (size_t t = 0; t < p.T; ++t) {
    double s = 0.0;
    for (size_t m = 0; m < p.T; ++m) {
      s += static_cast<double>(p.B[t * p.T + m]) * p.B[t * p.T + m];
    }
    sd[t] = static_cast<float>(std::sqrt(s));
  }
  return sd;
}

}  // namespace mppi::tc
