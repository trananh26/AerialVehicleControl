// Copyright 2026.
//
// SPDX-License-Identifier: MIT
//
// Pure-C++ UWB ranging model. Independent from Gazebo. Provides
//  * LOS detection via ray cast (caller supplies an injection point).
//  * Ranging noise model (Gaussian sigma + LOS/NLOS offset).
//  * Anchor storage with static positions.

#ifndef UWB_GZ_PLUGIN__UWB_RANGING_HH_
#define UWB_GZ_PLUGIN__UWB_RANGING_HH_

#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace uwb_gz_plugin
{
enum class LosType : std::uint8_t
{
  LOS      = 0,   // Line of sight
  NLOS_S   = 1,   // NLOS soft (thin obstacle)
  NLOS_H   = 2,   // NLOS hard (wall reflection)
  NLOS     = 3    // No link (out of range or blocked)
};

struct Anchor
{
  std::string name;
  int id {-1};
  Eigen::Vector3d position;            // world (ENU) anchor position
  double max_range {50.0};              // [m]
  double los_std  {0.05};               // [m] 1-sigma noise (LOS)
  double nlos_s_bias {0.10};            // [m] extra positive bias
  double nlos_h_bias {0.50};            // [m] extra positive bias
  double nlos_s_std  {0.10};            // [m] noise
  double nlos_h_std  {0.20};            // [m] noise
};

struct RangingResult
{
  std::string anchor_name;
  int anchor_id {-1};
  LosType los {LosType::NLOS};
  double range {0.0};                   // [m]
  double rss   {-100.0};                // [dBm]
};

class UwbRangingModel
{
public:
  /// Construct from a set of anchor positions (world frame).
  explicit UwbRangingModel(std::vector<Anchor> anchors);

  /// Update anchor world positions. Call once per simulation step.
  void UpdateAnchorPositions(const std::vector<Eigen::Vector3d> & positions);

  /// Inject a ray-cast LOS test: given a world-space ray from `tag` to
  /// `anchor`, returns the number of obstacles intersected.
  /// The default implementation assumes no obstacles (perfect LOS) so the
  /// plugin can be compiled without dependency on a particular collider
  /// representation. Re-override / subclass for NLOS modeling.
  virtual int CountObstacles(
    const Eigen::Vector3d & tag,
    const Eigen::Vector3d & anchor) const;

  /// Compute one ranging measurement for every anchor.
  /// `tag` is the world-frame position of the UWB tag.
  /// `dt` is elapsed time since last update in seconds (for noise seeding).
  std::vector<RangingResult> Compute(
    const Eigen::Vector3d & tag,
    double dt);

  /// Deterministic test accessors.
  const std::vector<Anchor> & anchors() const { return anchors_; }

private:
  std::vector<Anchor> anchors_;
  std::mt19937 rng_;
};

}  // namespace uwb_gz_plugin

#endif  // UWB_GZ_PLUGIN__UWB_RANGING_HH_
