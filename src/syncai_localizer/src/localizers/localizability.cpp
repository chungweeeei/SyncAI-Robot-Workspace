#include "syncai_localizer/localizers/localizability.h"

#include <Eigen/Eigenvalues>
#include <cmath>

namespace
{
using M6D = Eigen::Matrix<double, 6, 6>;

// Pseudo-inverse of a symmetric PSD 3x3 through its eigen decomposition:
// eigenvalues below rel_tol x the largest are treated as zero. A plain inverse
// of a block that is itself degenerate (a corridor's translation block, say)
// would turn the other block's Schur complement into noise.
M3D pinvSym3(const M3D & A, double rel_tol = 1e-6)
{
  Eigen::SelfAdjointEigenSolver<M3D> es(A);
  if (es.info() != Eigen::Success) return M3D::Zero();
  const V3D ev = es.eigenvalues();
  const double ev_max = ev.maxCoeff();
  if (!(ev_max > 0.0)) return M3D::Zero();
  V3D inv = V3D::Zero();
  for (int i = 0; i < 3; ++i) {
    if (ev(i) > rel_tol * ev_max) inv(i) = 1.0 / ev(i);
  }
  return es.eigenvectors() * inv.asDiagonal() * es.eigenvectors().transpose();
}

// Eigen decomposition of a symmetric 3x3, eigenvalues ascending and clamped at
// zero (the Schur complement of a PSD matrix is PSD, but the truncated
// pseudo-inverse can leave it indefinite by rounding).
void decompose(const M3D & S_in, V3D & eig, M3D & vec)
{
  const M3D S = 0.5 * (S_in + S_in.transpose());
  Eigen::SelfAdjointEigenSolver<M3D> es(S);
  if (es.info() != Eigen::Success) {
    eig.setZero();
    vec.setIdentity();
    return;
  }
  eig = es.eigenvalues().cwiseMax(0.0);
  vec = es.eigenvectors();
}

void classify(const V3D & eig, const LocalizabilityConfig & cfg, std::array<Localizable, 3> & out)
{
  const double eig_max = eig.maxCoeff();
  for (int i = 0; i < 3; ++i) {
    Localizable level = Localizable::kNone;
    if (eig_max > 0.0) {
      const double ratio = eig(i) / eig_max;
      if (ratio >= cfg.ratio_partial) {
        level = Localizable::kFull;
      } else if (ratio >= cfg.ratio_none) {
        level = Localizable::kPartial;
      }
      if (cfg.min_eig_per_inlier > 0.0 && eig(i) < cfg.min_eig_per_inlier) {
        level = Localizable::kNone;
      }
    }
    out[static_cast<std::size_t>(i)] = level;
  }
}

double gain(Localizable level, const LocalizabilityConfig & cfg)
{
  switch (level) {
    case Localizable::kFull:
      return 1.0;
    case Localizable::kPartial:
      return cfg.partial_gain;
    case Localizable::kNone:
    default:
      return 0.0;
  }
}

V3D project(
  const M3D & vec, const std::array<Localizable, 3> & level, const V3D & d,
  const LocalizabilityConfig & cfg)
{
  V3D out = V3D::Zero();
  for (int i = 0; i < 3; ++i) {
    const V3D v = vec.col(i);
    out += gain(level[static_cast<std::size_t>(i)], cfg) * v.dot(d) * v;
  }
  return out;
}

char levelChar(Localizable level)
{
  switch (level) {
    case Localizable::kFull:
      return 'F';
    case Localizable::kPartial:
      return 'P';
    case Localizable::kNone:
    default:
      return 'N';
  }
}
}  // namespace

bool Localizability::transFullyDegenerate() const
{
  return !valid || (trans_level[0] == Localizable::kNone && trans_level[1] == Localizable::kNone &&
                    trans_level[2] == Localizable::kNone);
}

std::string Localizability::levelString() const
{
  std::string s;
  s.reserve(7);
  for (const Localizable l : trans_level) s.push_back(levelChar(l));
  s.push_back('/');
  for (const Localizable l : rot_level) s.push_back(levelChar(l));
  return s;
}

double Localizability::weakestTransHeading() const
{
  const V3D v = trans_vec.col(0);
  double heading = std::atan2(v.y(), v.x());
  if (heading < 0.0) heading += M_PI;
  return heading;
}

Localizability analyzeLocalizability(
  const M6D & H_body, const M3D & R_map_body, std::size_t num_inliers,
  const LocalizabilityConfig & cfg)
{
  Localizability out;
  if (num_inliers == 0 || !H_body.allFinite() || H_body.isZero(0.0)) return out;

  // Body-frame right perturbation -> map-frame left perturbation, per block
  // (see the header), then per inlier so the thresholds do not move with the
  // scan's density.
  M6D Rb = M6D::Zero();
  Rb.block<3, 3>(0, 0) = R_map_body;
  Rb.block<3, 3>(3, 3) = R_map_body;
  const M6D L = (Rb * H_body * Rb.transpose()) / static_cast<double>(num_inliers);
  const M3D Lrr = L.block<3, 3>(0, 0);
  const M3D Ltt = L.block<3, 3>(3, 3);
  const M3D Ltr = L.block<3, 3>(3, 0);
  const M3D Lrt = L.block<3, 3>(0, 3);

  // Marginal information of each block (Schur complement): what is left of a
  // translation direction once the rotation is free to absorb part of it.
  const M3D St = Ltt - Ltr * pinvSym3(Lrr) * Lrt;
  const M3D Sr = Lrr - Lrt * pinvSym3(Ltt) * Ltr;
  decompose(St, out.trans_eig, out.trans_vec);
  decompose(Sr, out.rot_eig, out.rot_vec);
  classify(out.trans_eig, cfg, out.trans_level);
  classify(out.rot_eig, cfg, out.rot_level);
  out.valid = true;
  return out;
}

V3D projectTranslation(
  const Localizability & loc, const V3D & dt_map, const LocalizabilityConfig & cfg)
{
  if (!cfg.enabled || !loc.valid) return dt_map;
  return project(loc.trans_vec, loc.trans_level, dt_map, cfg);
}

V3D projectRotation(
  const Localizability & loc, const V3D & dtheta_map, const LocalizabilityConfig & cfg)
{
  if (!cfg.enabled || !loc.valid) return dtheta_map;
  return project(loc.rot_vec, loc.rot_level, dtheta_map, cfg);
}
