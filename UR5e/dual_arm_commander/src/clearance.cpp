// Cached, endpoint-only cross-arm distance query. No ROS subscriptions or services.
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <moveit/collision_detection_fcl/collision_env_fcl.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <map>
#include <algorithm>

class ClearanceChecker
{
public:
  explicit ClearanceChecker(const std::string& description)
  {
    auto urdf = urdf::parseURDF(description);
    if (!urdf)
      throw std::runtime_error("Cannot parse clearance URDF");
    auto srdf = std::make_shared<srdf::Model>();
    // No semantic exclusions: every left/right collision-link pair is checked.
    model_ = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    std::vector<std::string> left, right;
    for (const auto* link : model_->getLinkModels())
    {
      const auto source = urdf->getLink(link->getName());
      if (source && source->collision_array.size() != link->getShapes().size())
        throw std::runtime_error("Incomplete collision geometry: " + link->getName());
      if (link->getShapes().empty())
        continue;
      if (link->getName().rfind("left_", 0) == 0) left.push_back(link->getName());
      if (link->getName().rfind("right_", 0) == 0) right.push_back(link->getName());
    }
    if (left.empty() || right.empty())
      throw std::runtime_error("Missing left/right collision geometry");
    acm_ = std::make_unique<collision_detection::AllowedCollisionMatrix>(model_->getLinkModelNames(), true);
    for (const auto& a : left)
      for (const auto& b : right)
        acm_->setEntry(a, b, false);
    environment_ = std::make_unique<collision_detection::CollisionEnvFCL>(model_);
  }

  bool has_clearance(const std::map<std::string, double>& positions, double clearance) const
  {
    if (!std::isfinite(clearance) || clearance <= 0)
      throw std::invalid_argument("Clearance must be finite and positive");
    moveit::core::RobotState state(model_);
    state.setToDefaultValues();
    for (const auto* joint : model_->getActiveJointModels())
      for (const auto& variable : joint->getVariableNames())
        if (positions.count(variable) == 0)
          throw std::runtime_error("Missing clearance joint state: " + variable);
    for (const auto& item : positions)
    {
      if (!std::isfinite(item.second))
        throw std::invalid_argument("Non-finite joint position");
      const auto& names = model_->getVariableNames();
      if (std::find(names.begin(), names.end(), item.first) != names.end())
        state.setVariablePosition(item.first, item.second);
    }
    state.update();
    collision_detection::DistanceRequest request;
    request.acm = acm_.get();
    request.type = collision_detection::DistanceRequestType::GLOBAL;
    request.distance_threshold = clearance;
    request.enable_nearest_points = false;
    request.enable_signed_distance = false;
    collision_detection::DistanceResult result;
    environment_->distanceSelf(request, result, state);
    return !result.collision && result.minimum_distance.distance >= clearance;
  }

private:
  moveit::core::RobotModelPtr model_;
  std::unique_ptr<collision_detection::AllowedCollisionMatrix> acm_;
  std::unique_ptr<collision_detection::CollisionEnvFCL> environment_;
};

PYBIND11_MODULE(_clearance, module)
{
  pybind11::class_<ClearanceChecker>(module, "ClearanceChecker")
    .def(pybind11::init<const std::string&>())
    .def("has_clearance", &ClearanceChecker::has_clearance);
}
