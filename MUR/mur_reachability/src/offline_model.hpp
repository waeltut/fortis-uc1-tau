#pragma once
#include "arm_collision_policy.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
using Clock=std::chrono::steady_clock;
using namespace std::chrono_literals;
static const std::array<std::string,2> sides{"left","right"};
namespace mr {
inline std::string fingerprint(const std::string& text) {
  uint64_t hash=14695981039346656037ULL;
  for(unsigned char byte:text){hash^=byte;hash*=1099511628211ULL;}
  std::ostringstream s;s<<std::hex<<hash;return s.str();
}
}
class EmbeddedEngine;
class OfflineModel:public rclcpp::Node {
  friend class EmbeddedEngine;
public:
  explicit OfflineModel(const rclcpp::NodeOptions& options):Node("mur_kinematics",options) {
    source_=param<std::string>("source_node","/move_group");
    frame_=param<std::string>("reference_frame","mur");base_=param<std::string>("base_frame","base_link");
    groups_={param<std::string>("left_group","left_ur_manipulator"),param<std::string>("right_group","right_ur_manipulator")};
    tips_={param<std::string>("left_tip","left_tcp"),param<std::string>("right_tip","right_tcp")};
    joint_margin_=param<double>("joint_limit_margin",.02);
    pos_tol_=param<double>("verification_position_tolerance",.001);
    rot_tol_=param<double>("verification_orientation_tolerance",.01);
    for(double v:{joint_margin_,pos_tol_,rot_tol_})
      if(!std::isfinite(v)||v<0)throw std::runtime_error("Invalid offline model tolerance");
    if(pos_tol_<=0||rot_tol_<=0)throw std::runtime_error("Verification tolerances must be positive");
  }
  void initialize() {
    auto remote=std::make_shared<rclcpp::SyncParametersClient>(shared_from_this(),source_);
    if(!remote->wait_for_service(15s))throw std::runtime_error("Start MoveIt: missing parameter service "+source_);
    auto listed=remote->list_parameters({"robot_description","robot_description_semantic","robot_description_kinematics","robot_description_planning"},0,15s);
    std::vector<std::string> names;
    for(const auto& n:listed.names)if(n=="robot_description"||n=="robot_description_semantic"||n.rfind("robot_description_kinematics.",0)==0||n.rfind("robot_description_planning.",0)==0)names.push_back(n);
    std::sort(names.begin(),names.end());std::ostringstream provenance;
    for(const auto& p:remote->get_parameters(names,15s)) {
      if(p.get_type()==rclcpp::ParameterType::PARAMETER_NOT_SET)continue;
      provenance<<p.get_name()<<'='<<p.value_to_string()<<'\n';
      if(!has_parameter(p.get_name()))declare_parameter(p.get_name(),p.get_parameter_value());
      else if(!set_parameter(p).successful)throw std::runtime_error("Cannot copy "+p.get_name());
    }
    if(!has_parameter("robot_description")||!has_parameter("robot_description_semantic"))throw std::runtime_error("Source node lacks URDF/SRDF parameters");
    loader_=std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(),"robot_description",true);
    model_=loader_->getModel();if(!model_||!model_->hasLinkModel(frame_))throw std::runtime_error("reference_frame must exist in MoveIt URDF");
    for(unsigned a=0;a<2;++a) {
      const auto* g=model_->getJointModelGroup(groups_[a]);
      if(!g||!model_->hasLinkModel(tips_[a])||g->getVariableCount()==0||g->getVariableCount()>32)throw std::runtime_error("Invalid arm group/TCP");
      if(!g->getSolverInstance())throw std::runtime_error("Missing IK plugin for "+groups_[a]);
      // Verify that frame is rigidly upstream; this makes cached coordinates reusable under base motion.
      auto active=g->getActiveJointModels();if(active.empty())throw std::runtime_error("Empty arm chain");
      const auto* link=active.front()->getParentLinkModel();
      while(link&&link->getName()!=frame_) {
        auto* j=link->getParentJointModel();if(!j||j->getType()!=moveit::core::JointModel::FIXED)throw std::runtime_error("reference_frame must be rigidly upstream of arms");
        link=j->getParentLinkModel();
      }
      if(!link)throw std::runtime_error("reference_frame is not an ancestor; use mur for your combined model");
    }
    collision_scene_=std::make_shared<planning_scene::PlanningScene>(model_);
    for(unsigned a=0;a<2;++a) arm_links_[a]=mr::arm_branch(*model_,groups_[a]);
    for(const auto& link:arm_links_[0])
      if(std::find(arm_links_[1].begin(),arm_links_[1].end(),link)!=arm_links_[1].end())
        throw std::runtime_error("Arm collision branches overlap: "+link);
    for(unsigned a=0;a<2;++a) {
      if(std::find(arm_links_[a].begin(),arm_links_[a].end(),tips_[a])==arm_links_[a].end())
        throw std::runtime_error("TCP is outside moving arm branch: "+tips_[a]);
      arm_acm_[a]=mr::arm_collision_matrix(*collision_scene_,arm_links_[1-a]);
      RCLCPP_INFO(get_logger(), "%s checks self/body collisions; ignoring %zu opposite-arm links",
                  sides[a].c_str(),arm_links_[1-a].size());
    }
    model_record_=provenance.str();provenance<<std::setprecision(17)<<frame_<<'\n'<<joint_margin_<<'\n'<<pos_tol_<<'\n'<<rot_tol_<<'\n'<<"self_body_ignore_other_arm_v1\n";
    for(unsigned a=0;a<2;++a)provenance<<groups_[a]<<'\n'<<tips_[a]<<'\n';
    fingerprint_=mr::fingerprint(provenance.str());
  }
private:
  template<class T>T param(const std::string& n,const T& v){if(!has_parameter(n))declare_parameter<T>(n,v);return get_parameter(n).get_value<T>();}
  bool valid_state(moveit::core::RobotState& s,const moveit::core::JointModelGroup* group,unsigned arm)const {
    s.update();if(!s.satisfiesBounds(group))return false;
    for(const auto& name:group->getVariableNames()) {
      const auto& b=model_->getVariableBounds(name);double q=s.getVariablePosition(name);
      if(!std::isfinite(q)||(b.position_bounded_&&(q<b.min_position_+joint_margin_||q>b.max_position_-joint_margin_)))return false;
    }
    collision_detection::CollisionRequest request;
    collision_detection::CollisionResult result;
    // Whole-model check includes hand/tool descendants outside the IK group.
    // Only opposite-arm pairs are added to the SRDF's existing allowed pairs.
    collision_scene_->checkSelfCollision(request,result,s,arm_acm_[arm]);
    return !result.collision;
  }
  std::string source_,frame_,base_,fingerprint_,model_record_;
  std::array<std::string,2> groups_,tips_;
  double joint_margin_,pos_tol_,rot_tol_;
  std::shared_ptr<robot_model_loader::RobotModelLoader> loader_;
  moveit::core::RobotModelPtr model_;
  std::shared_ptr<planning_scene::PlanningScene> collision_scene_;
  std::array<std::vector<std::string>,2> arm_links_;
  std::array<collision_detection::AllowedCollisionMatrix,2> arm_acm_;
};
