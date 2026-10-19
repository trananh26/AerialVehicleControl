// Copyright 2026.
//
// SPDX-License-Identifier: MIT

#include "uwb_gz_plugin/UwbRanging.hh"

#include <algorithm>
#include <cmath>

namespace uwb_gz_plugin
{
UwbRangingModel::UwbRangingModel(std::vector<Anchor> anchors)
: anchors_(std::move(anchors)),
  rng_(std::random_device{}())
{
}

void
UwbRangingModel::UpdateAnchorPositions(
  const std::vector<Eigen::Vector3d> & positions)
{
  const std::size_t n = std::min(anchors_.size(), positions.size());
  for (std::size_t i = 0; i < n; ++i) {
    anchors_[i].position = positions[i];
  }
}

int
UwbRangingModel::CountObstacles(
  const Eigen::Vector3d & /*tag*/,
  const Eigen::Vector3d & /*anchor*/) const
{
  // Default: no obstacles. The Harmonic plugin overrides this using
  // a real ray-cast through the world physics.
  return 0;
}

static LosType
ClassifyLos(int n_obstacles, double distance, double max_range)
{
  if (distance > max_range) {
    return LosType::NLOS;        // out of range
  }
  if (n_obstacles == 0) {
    return LosType::LOS;
  }
  if (n_obstacles == 1) {
    return LosType::NLOS_S;
  }
  if (n_obstacles == 2) {
    return LosType::NLOS_H;
  }
  return LosType::NLOS;
}

std::vector<RangingResult>
UwbRangingModel::Compute(
  const Eigen::Vector3d & tag,
  double /*dt*/)
{
  std::vector<RangingResult> out;
  out.reserve(anchors_.size());

  for (const auto & a : anchors_) {
    RangingResult r;
    r.anchor_name = a.name;
    r.anchor_id   = a.id;

    const Eigen::Vector3d diff = a.position - tag;
    const double distance = diff.norm();

    const int obstacles = CountObstacles(tag, a.position);
    r.los = ClassifyLos(obstacles, distance, a.max_range);

    if (r.los == LosType::NLOS) {
      r.range = 0.0;       // no measurement available
      r.rss   = -120.0;
      out.push_back(std::move(r));
      continue;
    }

    // Select noise parameters based on LOS class.
    double bias = 0.0;
    double sigma = a.los_std;
    switch (r.los) {
      case LosType::LOS:
        sigma = a.los_std;
        bias  = 0.0;
        break;
      case LosType::NLOS_S:
        sigma = a.nlos_s_std;
        bias  = a.nlos_s_bias;
        break;
      case LosType::NLOS_H:
        sigma = a.nlos_h_std;
        bias  = a.nlos_h_bias;
        break;
      default:
        break;
    }

    std::normal_distribution<double> noise(0.0, sigma);
    r.range = std::max(0.0, distance + bias + noise(rng_));

    // Very rough path-loss model: RSS = -50 - 20*log10(d)
    // plus a class-dependent penalty.
    double rss = -50.0 - 20.0 * std::log10(std::max(distance, 0.1));
    if (r.los == LosType::NLOS_S) rss -= 5.0;
    if (r.los == LosType::NLOS_H) rss -= 12.0;
    r.rss = rss;

    out.push_back(std::move(r));
  }
  return out;
}

}  // namespace uwb_gz_plugin
