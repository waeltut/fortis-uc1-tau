#pragma once
#include <moveit/planning_scene/planning_scene.h>
#include <array>
#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

namespace mr {
inline constexpr const char* collision_policy = "self_body_ignore_other_arm_v1";

// Start at the first moving joint, not a name prefix or the torso/mount.
// All descendants count, including fixed TCP/tool links and articulated fingers.
inline std::vector<std::string> arm_branch(const moveit::core::RobotModel& model,
                                         const std::string& group_name) {
  const auto* group = model.getJointModelGroup(group_name);
  if (!group || group->getActiveJointModels().empty())
    throw std::runtime_error("Cannot identify moving arm branch: " + group_name);
  const auto* root = group->getActiveJointModels().front()->getChildLinkModel();
  std::vector<std::string> result;
  for (const auto* link : model.getLinkModels()) {
    const auto* ancestor = link;
    while (ancestor && ancestor != root) {
      const auto* parent = ancestor->getParentJointModel();
      ancestor = parent ? parent->getParentLinkModel() : nullptr;
    }
    if (ancestor == root) result.push_back(link->getName());
  }
  return result;
}

inline collision_detection::AllowedCollisionMatrix arm_collision_matrix(
    const planning_scene::PlanningScene& scene,
    const std::vector<std::string>& ignored_branch) {
  auto acm = scene.getAllowedCollisionMatrix(); // Preserve SRDF adjacency/disabled pairs.
  const auto& links = scene.getRobotModel()->getLinkModelNames();
  for (const auto& ignored : ignored_branch)
    for (const auto& other : links) acm.setEntry(ignored, other, true);
  return acm;
}
} // namespace mr
