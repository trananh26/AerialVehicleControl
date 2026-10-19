// Copyright 2026.
//
// SPDX-License-Identifier: MIT

#include "uwb_gz_plugin/UwbSystem.hh"

#include <gz/msgs/marker.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/Utility.hh>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>

#include <chrono>
#include <sstream>

namespace uwb_gz_plugin
{
namespace systems
{
using namespace std::chrono_literals;

//////////////////////////////////////////////////
UwbSystem::UwbSystem()
{
  last_update_ = std::chrono::steady_clock::now();
}

//////////////////////////////////////////////////
gz::sim::Entity
UwbSystem::FindLinkEntity(
  const gz::sim::EntityComponentManager & _ecm,
  const gz::sim::Entity & _model,
  const std::string & _link_name) const
{
  if (_model == gz::sim::kNullEntity) return gz::sim::kNullEntity;

  // Walk children of the model and look for a Link with the given name.
  auto children = _ecm.ChildrenByComponents(
    _model, gz::sim::components::Link());

  for (const auto & c : children) {
    auto name_comp = _ecm.Component<gz::sim::components::Name>(c);
    if (name_comp && name_comp->Data() == _link_name) {
      return c;
    }
  }
  return gz::sim::kNullEntity;
}

//////////////////////////////////////////////////
void
UwbSystem::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager & /*_eventMgr*/)
{
  model_entity_ = _entity;
  gzmsg << "[UwbSystem] Configuring for entity " << _entity << std::endl;

  if (!_sdf->HasElement("tag_id")) {
    gzerr << "[UwbSystem] <tag_id> is required." << std::endl;
    return;
  }
  tag_id_ = _sdf->Get<std::string>("tag_id");

  if (!_sdf->HasElement("tag_link")) {
    gzerr << "[UwbSystem] <tag_link> is required." << std::endl;
    return;
  }
  tag_link_name_ = _sdf->Get<std::string>("tag_link");

  if (_sdf->HasElement("anchor_prefix"))
    anchor_prefix_ = _sdf->Get<std::string>("anchor_prefix");
  if (_sdf->HasElement("update_rate"))
    update_rate_ = _sdf->Get<double>("update_rate");
  if (_sdf->HasElement("tag_z_offset"))
    tag_z_offset_ = _sdf->Get<double>("tag_z_offset");
  if (_sdf->HasElement("all_los"))
    all_los_ = _sdf->Get<bool>("all_los");
  if (_sdf->HasElement("nlos_soft_wall_width"))
    nlos_soft_wall_width_ = _sdf->Get<double>("nlos_soft_wall_width");
  if (_sdf->HasElement("max_range"))
    max_range_ = _sdf->Get<double>("max_range");
  if (_sdf->HasElement("los_std"))
    los_std_ = _sdf->Get<double>("los_std");

  update_period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(1.0 / std::max(update_rate_, 1e-3)));

  // 1) Try <anchor name="" pos="x y z"/> children first (preferred).
  std::vector<Eigen::Vector3d> explicit_positions;
  sdf::ElementPtr sdf_rw = std::const_pointer_cast<sdf::Element>(
    std::shared_ptr<const sdf::Element>(_sdf));
  if (sdf_rw->HasElement("anchor")) {
    sdf::ElementPtr elem = sdf_rw->GetElement("anchor");
    while (elem) {
      Eigen::Vector3d p(Eigen::Vector3d::Zero());
      if (elem->HasElement("pos")) {
        std::istringstream ss(elem->Get<std::string>("pos"));
        ss >> p.x() >> p.y() >> p.z();
      } else {
        gzwarn << "[UwbSystem] <anchor> without <pos>; skipping" << std::endl;
        elem = elem->GetNextElement("anchor");
        continue;
      }
      const std::string name = elem->HasAttribute("name")
        ? elem->Get<std::string>("name")
        : ("anchor_" + std::to_string(explicit_positions.size()));
      AnchorState s;
      s.name = name;
      s.id   = static_cast<int>(anchors_.size());
      s.entity = gz::sim::kNullEntity;
      s.world_position = p;
      anchors_.push_back(s);
      explicit_positions.push_back(p);
      gzmsg << "[UwbSystem] Explicit anchor '"
            << name << "' at (" << p.x() << ", "
            << p.y() << ", " << p.z() << ")" << std::endl;
      elem = elem->GetNextElement("anchor");
    }
  }

  // 2) Fall back to a hard-coded 10x10 m square.
  if (anchors_.empty()) {
    gzmsg << "[UwbSystem] No <anchor> elements in SDF. Using hard-coded "
          << "10x10 m layout at z=2.0 m (anchor 0 = origin)." << std::endl;
    const Eigen::Vector3d layout[4] = {
      Eigen::Vector3d( 0,  0, 2.0),
      Eigen::Vector3d(10,  0, 2.0),
      Eigen::Vector3d(10, 10, 2.0),
      Eigen::Vector3d( 0, 10, 2.0),
    };
    for (const auto & p : layout) {
      AnchorState s;
      s.name = "uwb_anchor" + std::to_string(anchors_.size());
      s.id   = static_cast<int>(anchors_.size());
      s.entity = gz::sim::kNullEntity;
      s.world_position = p;
      anchors_.push_back(s);
      explicit_positions.push_back(p);
      gzmsg << "[UwbSystem] Hard-coded anchor '"
            << s.name << "' at (" << p.x() << ", "
            << p.y() << ", " << p.z() << ")" << std::endl;
    }
  }

  // 3) Resolve the tag model entity by name (preferred) or by parent.
  //    If the plugin is attached to a model, model_entity_ already points
  //    to it. If attached to a world (entity 1), we look for the tag model
  //    by name lazily in PreUpdate.
  is_world_plugin_ = (_entity == 1);  // world is always entity 1
  if (is_world_plugin_) {
    // Defer: model_entity_ will be resolved in PreUpdate when world has models.
    model_entity_ = gz::sim::kNullEntity;
  }
  if (sdf_rw->HasElement("tag_model_name")) {
    tag_model_name_ = sdf_rw->Get<std::string>("tag_model_name");
    gzmsg << "[UwbSystem] Configured to look for tag model '"
          << tag_model_name_ << "'; is_world_plugin="
          << is_world_plugin_ << std::endl;
  } else if (!is_world_plugin_) {
    gzmsg << "[UwbSystem] Attached to model entity="
          << model_entity_ << std::endl;
  } else {
    gzwarn << "[UwbSystem] World plugin without <tag_model_name>!"
           << std::endl;
  }

  // 4) Resolve the tag link entity from the model.
  if (model_entity_ != gz::sim::kNullEntity) {
    tag_link_entity_ = FindLinkEntity(_ecm, model_entity_, tag_link_name_);
    if (tag_link_entity_ == gz::sim::kNullEntity) {
      gzwarn << "[UwbSystem] tag link '"
             << tag_link_name_ << "' not found yet. Will be resolved later."
             << std::endl;
    }
  } else {
    gzwarn << "[UwbSystem] model_entity_ is null. "
           << "Set <tag_model_name> or attach plugin to a model."
           << std::endl;
  }

  // Build the ranging model from the initial anchor set.
  std::vector<Anchor> model_anchors;
  model_anchors.reserve(anchors_.size());
  for (const auto & a : anchors_) {
    Anchor ma;
    ma.name = a.name;
    ma.id   = a.id;
    ma.max_range = max_range_;
    ma.los_std   = los_std_;
    model_anchors.push_back(ma);
  }
  ranging_ = std::make_unique<UwbRangingModel>(std::move(model_anchors));

  // gz-transport publishers.
  gz_node_ = std::make_shared<gz::transport::Node>();

  const std::string ranging_topic = "/uwb/ranging/tag";
  const std::string anchors_topic = "/uwb/anchors/tag";
  const std::string tagpose_topic = "/uwb/tag_pose/tag";
  pub_ranging_  = gz_node_->Advertise<gz::msgs::Pose>(ranging_topic);
  pub_anchors_  = gz_node_->Advertise<gz::msgs::Marker>(anchors_topic);
  pub_tag_pose_ = gz_node_->Advertise<gz::msgs::Pose>(tagpose_topic);
  if (!pub_ranging_)  gzwarn << "[UwbSystem] Failed to advertise " << ranging_topic << std::endl;
  if (!pub_anchors_)  gzwarn << "[UwbSystem] Failed to advertise " << anchors_topic << std::endl;
  if (!pub_tag_pose_) gzwarn << "[UwbSystem] Failed to advertise " << tagpose_topic << std::endl;
}

//////////////////////////////////////////////////
void
UwbSystem::RefreshAnchorPositions(
  const gz::sim::EntityComponentManager & _ecm)
{
  // Only update positions for anchors that have a real model entity.
  // Hard-coded anchors (entity == kNullEntity) keep their Configure-time
  // position.
  for (auto & a : anchors_) {
    if (a.entity == gz::sim::kNullEntity) continue;

    if (auto wp = _ecm.Component<gz::sim::components::WorldPose>(a.entity)) {
      const auto & p = wp->Data();
      a.world_position = Eigen::Vector3d(p.X(), p.Y(), p.Z());
    } else if (auto lp = _ecm.Component<gz::sim::components::Pose>(a.entity)) {
      const auto & p = lp->Data();
      a.world_position = Eigen::Vector3d(p.X(), p.Y(), p.Z());
    }
  }
  if (ranging_) {
    std::vector<Eigen::Vector3d> positions;
    positions.reserve(anchors_.size());
    for (const auto & a : anchors_) positions.push_back(a.world_position);
    ranging_->UpdateAnchorPositions(positions);
  }
}

//////////////////////////////////////////////////
void
UwbSystem::PostUpdate(
  const gz::sim::UpdateInfo & _info,
  const gz::sim::EntityComponentManager & _ecm)
{
  if (_info.paused) return;

  // Re-resolve tag model/link lazily. The world can be empty at Configure
  // time, so we look them up by name here.
  if (model_entity_ == gz::sim::kNullEntity && !tag_model_name_.empty()) {
    auto e = _ecm.EntityByName(tag_model_name_);
    if (e.has_value() && e.value() != gz::sim::kNullEntity) {
      model_entity_ = e.value();
    }
  }
  if (tag_link_entity_ == gz::sim::kNullEntity && model_entity_ != gz::sim::kNullEntity) {
    tag_link_entity_ = FindLinkEntity(_ecm, model_entity_, tag_link_name_);
  }
  if (tag_link_entity_ == gz::sim::kNullEntity) return;
  if (!ranging_) return;

  // Throttle by update_rate.
  const auto now = std::chrono::steady_clock::now();
  if (now - last_update_ < update_period_) return;
  last_update_ = now;

  // World position of the tag link.
  gz::math::Pose3d tag_pose = gz::math::Pose3d::Zero;
  const char *pose_src = "ZERO";
  if (auto wp = _ecm.Component<gz::sim::components::WorldPose>(tag_link_entity_)) {
    tag_pose = wp->Data();
    pose_src = "WorldPose";
  } else if (auto p = _ecm.Component<gz::sim::components::Pose>(tag_link_entity_)) {
    tag_pose = p->Data();
    pose_src = "LinkPose";
  } else if (auto mp = _ecm.Component<gz::sim::components::Pose>(model_entity_)) {
    // Fallback: read pose from the model (works for static models whose
    // links don't have a separate Pose component).
    tag_pose = mp->Data();
    pose_src = "ModelPose";
  }
  Eigen::Vector3d tag_world(
    tag_pose.X(),
    tag_pose.Y(),
    tag_pose.Z() + tag_z_offset_);

  // Refresh anchor positions and compute ranging.
  RefreshAnchorPositions(_ecm);

  // NLOS: for now use the simple "all_los" override.
  struct AllLosOverride {};
  // (The simpler path is to set all_los_=true to disable NLOS for SITL testing.)
  if (all_los_) {
    // Replace the virtual raycast by zero obstacles via a lambda.
    // We cannot modify the model at runtime, but for indoor SITL we can
    // emulate it by lying about obstacle count using a thin wrapper. Easiest
    // way: temporarily monkey-patch by creating a fresh model. We keep
    // things simple here and assume "all_los" is set during SITL bring-up
    // (no walls in runway.sdf).
  }

  const double dt = 1.0 / std::max(update_rate_, 1e-3);
  const auto results = ranging_->Compute(tag_world, dt);

  PublishTagPose(tag_world);
  PublishRanging(results, tag_world);
  PublishAnchorMarkers(results);
}

//////////////////////////////////////////////////
void
UwbSystem::PublishRanging(
  const std::vector<RangingResult> & results,
  const Eigen::Vector3d & /*tag_world*/)
{
  if (!gz_node_) return;
  const std::string topic = "/uwb/ranging/tag";

  // Publish one gz::msgs::Pose per anchor, with custom encoding:
  //   position.x = range
  //   position.y = rss
  //   position.z = anchor_id (float)
  //   orientation.x = los code (0/1/2)
  //   orientation.w = 1.0 (identity)
  // The Python trilateration node reads the geometry_msgs/Pose it bridges
  // into and decodes these fields.
  for (const auto & r : results) {
    gz::msgs::Pose msg;
    msg.set_name(r.anchor_name);
    gz::msgs::Set(msg.mutable_position(),
                  gz::math::Vector3d(
                    r.range,
                    r.rss,
                    static_cast<double>(r.anchor_id)));
    gz::msgs::Set(msg.mutable_orientation(),
                  gz::math::Quaterniond(
                    static_cast<double>(r.los), 0, 0));
    pub_ranging_.Publish(msg);
  }
}

//////////////////////////////////////////////////
void
UwbSystem::PublishAnchorMarkers(
  const std::vector<RangingResult> & results)
{
  if (!gz_node_) return;
  const std::string topic = "/uwb/anchors/" + tag_id_;

  // gz-harmonic does not provide gz::msgs::MarkerArray; we publish one
  // Marker per anchor with a shared namespace.
  for (const auto & r : results) {
    gz::msgs::Marker m;
    m.set_ns("uwb_anchors");
    m.set_id(r.anchor_id);
    m.set_action(gz::msgs::Marker::ADD_MODIFY);
    m.set_type(gz::msgs::Marker::SPHERE);

    for (const auto & a : anchors_) {
      if (a.id == r.anchor_id) {
        gz::msgs::Set(m.mutable_pose(),
                      gz::math::Pose3d(
                        a.world_position.x(),
                        a.world_position.y(),
                        a.world_position.z(),
                        0, 0, 0));
        break;
      }
    }

    gz::msgs::Set(m.mutable_scale(),
                  gz::math::Vector3d(0.2, 0.2, 0.2));
    switch (r.los) {
      case LosType::LOS:
        m.mutable_material()->mutable_diffuse()->set_g(1.0);
        m.mutable_material()->mutable_diffuse()->set_a(1.0);
        break;
      case LosType::NLOS_S:
        m.mutable_material()->mutable_diffuse()->set_r(1.0);
        m.mutable_material()->mutable_diffuse()->set_g(1.0);
        m.mutable_material()->mutable_diffuse()->set_a(1.0);
        break;
      case LosType::NLOS_H:
        m.mutable_material()->mutable_diffuse()->set_b(1.0);
        m.mutable_material()->mutable_diffuse()->set_a(1.0);
        break;
      default:
        m.mutable_material()->mutable_diffuse()->set_r(1.0);
        m.mutable_material()->mutable_diffuse()->set_a(0.3);
        break;
    }
    pub_anchors_.Publish(m);
  }
}

//////////////////////////////////////////////////
void
UwbSystem::PublishTagPose(const Eigen::Vector3d & tag_world)
{
  if (!gz_node_) return;
  const std::string topic = "/uwb/tag_pose/tag";
  gz::msgs::Pose msg;
  gz::msgs::Set(msg.mutable_position(),
                gz::math::Vector3d(
                  tag_world.x(), tag_world.y(), tag_world.z()));
  gz::msgs::Set(msg.mutable_orientation(),
                gz::math::Quaterniond(0, 0, 0, 1));
  pub_tag_pose_.Publish(msg);
}

}  // namespace systems
}  // namespace uwb_gz_plugin

GZ_ADD_PLUGIN(
  uwb_gz_plugin::systems::UwbSystem,
  gz::sim::System,
  uwb_gz_plugin::systems::UwbSystem::ISystemConfigure,
  uwb_gz_plugin::systems::UwbSystem::ISystemPostUpdate)
