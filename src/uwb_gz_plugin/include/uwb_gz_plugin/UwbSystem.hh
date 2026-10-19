// Copyright 2026.
//
// SPDX-License-Identifier: MIT
//
// Gazebo Harmonic System plugin that publishes simulated UWB ranging
// measurements.
//
// SDF parameters
// --------------
//   <update_rate>           (double, default 20.0)  Hz
//   <tag_id>                (string, required)
//   <tag_link>              (string, required, e.g. "base_link")
//   <anchor_prefix>         (string, default "uwb_anchor")
//   <tag_z_offset>          (double, default 0.0)   m
//   <all_los>               (bool,   default false)
//   <nlos_soft_wall_width>  (double, default 0.25)   m
//   <max_range>             (double, default 50.0)   m
//   <los_std>               (double, default 0.05)   m
//
// Published gz-transport topics
// -----------------------------
//   /uwb/ranging/<tag_id>   gz::msgs::Marker
//       One Marker per anchor (TEXT_VIEW_FACING) holding the range & id.
//       Useful for ros_gz_bridge -> visualization_msgs/MarkerArray.
//
//   /uwb/anchors/<tag_id>   gz::msgs::MarkerArray
//       Visualization of anchor positions, color depends on LOS class.
//
//   /uwb/tag_pose/<tag_id>  gz::msgs::Pose
//       Ground-truth tag pose in world (for debugging).

#ifndef UWB_GZ_PLUGIN__UWB_SYSTEM_HH_
#define UWB_GZ_PLUGIN__UWB_SYSTEM_HH_

#include <gz/sim/System.hh>
#include <gz/sim/config.hh>

#include <memory>
#include <string>
#include <vector>

#include <gz/transport/Node.hh>

#include "uwb_gz_plugin/UwbRanging.hh"

namespace uwb_gz_plugin
{
namespace systems
{
class UwbSystem :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate
{
public:
  UwbSystem();
  ~UwbSystem() override = default;

  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & _eventMgr) override;

  void PostUpdate(
    const gz::sim::UpdateInfo & _info,
    const gz::sim::EntityComponentManager & _ecm) override;

private:
  struct AnchorState
  {
    std::string name;
    int id {-1};
    gz::sim::Entity entity {gz::sim::kNullEntity};
    Eigen::Vector3d world_position {Eigen::Vector3d::Zero()};
  };

  // Parameters parsed from SDF.
  double update_rate_ {20.0};
  std::string tag_id_;
  std::string tag_model_name_;
  std::string tag_link_name_;
  std::string anchor_prefix_ {"uwb_anchor"};
  double tag_z_offset_ {0.0};
  bool all_los_ {false};
  double nlos_soft_wall_width_ {0.25};
  double max_range_ {50.0};
  double los_std_  {0.05};

  // Tag link entity (the UWB "tag" mounted on the vehicle).
  gz::sim::Entity tag_link_entity_ {gz::sim::kNullEntity};
  gz::sim::Entity model_entity_ {gz::sim::kNullEntity};
  bool is_world_plugin_ {false};

  // Anchors discovered by name prefix in the world.
  std::vector<AnchorState> anchors_;

  // Pure-C++ ranging model.
  std::unique_ptr<UwbRangingModel> ranging_;

  // gz-transport node for publishing.
  std::shared_ptr<gz::transport::Node> gz_node_;
  gz::transport::Node::Publisher       pub_ranging_;
  gz::transport::Node::Publisher       pub_anchors_;
  gz::transport::Node::Publisher       pub_tag_pose_;

  // Publish helpers.
  void PublishRanging(const std::vector<RangingResult> & results,
                      const Eigen::Vector3d & tag_world);
  void PublishAnchorMarkers(const std::vector<RangingResult> & results);
  void PublishTagPose(const Eigen::Vector3d & tag_world);

  // Update cached anchor world positions from ECM.
  void RefreshAnchorPositions(const gz::sim::EntityComponentManager & _ecm);

  // Look up the entity of a link by name within a model.
  gz::sim::Entity FindLinkEntity(
    const gz::sim::EntityComponentManager & _ecm,
    const gz::sim::Entity & _model,
    const std::string & _link_name) const;

  std::chrono::steady_clock::time_point last_update_;
  std::chrono::steady_clock::duration   update_period_{0};
};

}  // namespace systems
}  // namespace uwb_gz_plugin

#endif  // UWB_GZ_PLUGIN__UWB_SYSTEM_HH_
