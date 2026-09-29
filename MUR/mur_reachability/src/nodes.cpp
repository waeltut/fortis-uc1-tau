#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/serialization.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.h>
#include <mur_reachability/srv/find_base_candidates.hpp>
#include <yaml-cpp/yaml.h>
#include "mur_reachability/core.hpp"
#include <Eigen/Geometry>
#include <atomic>
#include <cstdlib>
#include <filesystem>
#include <future>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <random>
#include <set>

namespace fs=std::filesystem;
using Clock=std::chrono::steady_clock;
using namespace std::chrono_literals;
using Trigger=std_srvs::srv::Trigger;
using Find=mur_reachability::srv::FindBaseCandidates;
using Marker=visualization_msgs::msg::Marker;
using Markers=visualization_msgs::msg::MarkerArray;
static const std::array<std::string,2> sides{"left","right"};

mr::Transform plain(const Eigen::Isometry3d& t) {
  Eigen::Quaterniond q(t.linear());q.normalize();return {{t.translation().x(),t.translation().y(),t.translation().z()},{q.x(),q.y(),q.z(),q.w()}};
}
Eigen::Isometry3d eigen(const mr::Transform& t) {
  Eigen::Isometry3d r=Eigen::Isometry3d::Identity();r.translation()=Eigen::Vector3d(t.p[0],t.p[1],t.p[2]);
  const auto q=mr::normalize(t.q);r.linear()=Eigen::Quaterniond(q[3],q[0],q[1],q[2]).toRotationMatrix();return r;
}
Eigen::Isometry3d from_pose(const geometry_msgs::msg::Pose& p) {
  mr::Transform t{{p.position.x,p.position.y,p.position.z},{p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w}};
  for(double v:t.p)if(!std::isfinite(v)||std::abs(v)>1e8)throw std::runtime_error("Invalid goal coordinates");
  return eigen(t);
}
geometry_msgs::msg::Pose to_pose(const Eigen::Isometry3d& t) {
  auto a=plain(t);geometry_msgs::msg::Pose p;p.position.x=a.p[0];p.position.y=a.p[1];p.position.z=a.p[2];
  p.orientation.x=a.q[0];p.orientation.y=a.q[1];p.orientation.z=a.q[2];p.orientation.w=a.q[3];return p;
}
Eigen::Isometry3d planar(const mr::Candidate& c,double z) {
  Eigen::Isometry3d t=Eigen::Isometry3d::Identity();t.translation()=Eigen::Vector3d(c.x,c.y,z);
  t.linear()=Eigen::AngleAxisd(c.yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();return t;
}

class Reachability:public rclcpp::Node {
public:
  Reachability(bool builder,const rclcpp::NodeOptions& options):Node(builder?"reachability_cache":"base_candidates",options),builder_(builder) {
    source_=param<std::string>("source_node","/move_group");frame_=param<std::string>("reference_frame","mur");
    base_=param<std::string>("base_frame","base_link");fixed_=param<std::string>("fixed_frame","odom");
    scene_name_=param<std::string>("scene_service","/get_planning_scene");
    groups_={param<std::string>("left_group","left_ur_manipulator"),param<std::string>("right_group","right_ur_manipulator")};
    tips_={param<std::string>("left_tip","left_tcp"),param<std::string>("right_tip","right_tcp")};
    path_=param<std::string>("cache_file","~/.ros/mur_reachability/reachability.bin");
    if(path_.rfind("~/",0)==0){auto home=std::getenv("HOME");if(!home)throw std::runtime_error("Set absolute cache_file");path_=std::string(home)+path_.substr(1);}
    path_=fs::absolute(path_).string();
    margin_=param<double>("cross_margin",.20);joint_margin_=param<double>("joint_limit_margin",.02);
    samples_=param<int>("samples_per_arm",300000);seed_=param<int>("random_seed",42);
    voxel_=param<double>("display_voxel_size",.05);
    opt_.resolution=param<double>("grid_resolution",.05);
    opt_.height_tolerance=param<double>("height_tolerance",.04);
    opt_.angle_tolerance=param<double>("orientation_tolerance_deg",20)*3.141592653589793/180;
    int yaw_bins=param<int>("yaw_bins",24),max_candidates=param<int>("max_candidates_per_arm",2000),seed_count=param<int>("seeds_per_cell",3);
    budget_=param<double>("query_budget",3.0);max_verified_=param<int>("max_verified_per_arm",40);
    ik_timeout_=param<double>("ik_timeout",.008);
    pos_tol_=param<double>("verification_position_tolerance",.001);rot_tol_=param<double>("verification_orientation_tolerance",.01);
    floor_=param<double>("floor_z",0.0);
    if(samples_<1||samples_>5000000||yaw_bins<1||yaw_bins>360||max_candidates<1||max_candidates>100000||seed_count<1||seed_count>20||max_verified_<1||max_verified_>1000)
      throw std::runtime_error("Invalid count parameter");
    for(double v:{margin_,joint_margin_,voxel_,opt_.resolution,opt_.height_tolerance,opt_.angle_tolerance,budget_,ik_timeout_,pos_tol_,rot_tol_,floor_})
      if(!std::isfinite(v))throw std::runtime_error("Nonfinite parameter");
    if(margin_<0||joint_margin_<0||voxel_<.005||opt_.resolution<.005||opt_.height_tolerance<=0||opt_.angle_tolerance<.001||opt_.angle_tolerance>1.57||budget_<=0||budget_>60||ik_timeout_<=0||pos_tol_<=0||rot_tol_<=0)
      throw std::runtime_error("Invalid tolerance/budget parameter");
    opt_.yaw_bins=yaw_bins;opt_.max_candidates=max_candidates;opt_.seeds_per_cell=seed_count;
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
    loader_=std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(),"robot_description",!builder_);
    model_=loader_->getModel();if(!model_||!model_->hasLinkModel(frame_))throw std::runtime_error("reference_frame must exist in MoveIt URDF");
    for(unsigned a=0;a<2;++a) {
      const auto* g=model_->getJointModelGroup(groups_[a]);
      if(!g||!model_->hasLinkModel(tips_[a])||g->getVariableCount()==0||g->getVariableCount()>32)throw std::runtime_error("Invalid arm group/TCP");
      if(!builder_&&!g->getSolverInstance())throw std::runtime_error("Missing IK plugin for "+groups_[a]);
      // Verify that frame is rigidly upstream; this makes cached coordinates reusable under base motion.
      auto active=g->getActiveJointModels();if(active.empty())throw std::runtime_error("Empty arm chain");
      const auto* link=active.front()->getParentLinkModel();
      while(link&&link->getName()!=frame_) {
        auto* j=link->getParentJointModel();if(!j||j->getType()!=moveit::core::JointModel::FIXED)throw std::runtime_error("reference_frame must be rigidly upstream of arms");
        link=j->getParentLinkModel();
      }
      if(!link)throw std::runtime_error("reference_frame is not an ancestor; use mur for your combined model");
    }
    model_record_=provenance.str();provenance<<std::setprecision(17)<<frame_<<'\n'<<margin_<<'\n'<<joint_margin_<<'\n';
    for(unsigned a=0;a<2;++a)provenance<<groups_[a]<<'\n'<<tips_[a]<<'\n';
    fingerprint_=mr::fingerprint(provenance.str());
    scene_client_=create_client<moveit_msgs::srv::GetPlanningScene>(scene_name_);
    work_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto qos=rclcpp::QoS(1).reliable().transient_local();
    marker_pub_=create_publisher<Markers>(builder_?"/reachability/cache_markers":"/reachability/candidate_markers",qos);
    if(builder_) {
      generation_srv_=create_service<Trigger>("/generate_reachability_cache",[this](const std::shared_ptr<Trigger::Request>,std::shared_ptr<Trigger::Response> r){
        cancel_=false;try{generate();r->success=true;r->message="Saved "+path_+". Call /reload_reachability_cache on the query node.";}
        catch(const std::exception& e){r->message=e.what();RCLCPP_ERROR(get_logger(),"%s",e.what());}
      },rmw_qos_profile_services_default,work_group_);
      cancel_srv_=create_service<Trigger>("/cancel_reachability_cache",[this](const std::shared_ptr<Trigger::Request>,std::shared_ptr<Trigger::Response> r){cancel_=true;r->success=true;r->message="Cancellation requested";});
    } else {
      tf_=std::make_unique<tf2_ros::Buffer>(get_clock());listener_=std::make_shared<tf2_ros::TransformListener>(*tf_);
      reload_srv_=create_service<Trigger>("/reload_reachability_cache",[this](const std::shared_ptr<Trigger::Request>,std::shared_ptr<Trigger::Response> r){
        try{reload();clear_candidates();r->success=true;r->message="Cache ready: "+cache_id_;}catch(const std::exception& e){r->message=e.what();}
      },rmw_qos_profile_services_default,work_group_);
      find_srv_=create_service<Find>("/find_base_candidates",[this](const std::shared_ptr<Find::Request> q,std::shared_ptr<Find::Response> r){find(*q,*r);},rmw_qos_profile_services_default,work_group_);
      rclcpp::SubscriptionOptions options;options.callback_group=work_group_;
      goal_sub_=create_subscription<geometry_msgs::msg::PoseStamped>("/reachability/goal_pose",rclcpp::QoS(1),[this](const geometry_msgs::msg::PoseStamped::SharedPtr goal){
        Find::Request q;q.target=*goal;Find::Response r;find(q,r);RCLCPP_INFO(get_logger(),"%s",r.message.c_str());
      },options);
      for(unsigned a=0;a<2;++a) {
        candidate_pubs_[a]=create_publisher<geometry_msgs::msg::PoseArray>("/reachability/"+sides[a]+"_candidates",qos);
        verified_pubs_[a]=create_publisher<geometry_msgs::msg::PoseArray>("/reachability/"+sides[a]+"_verified",qos);
      }
    }
    timer_=create_wall_timer(2s,[this]{republish();});
    try{reload();if(builder_)cache_markers();}catch(const std::exception& e){RCLCPP_WARN(get_logger(),"Cache not loaded: %s",e.what());}
    RCLCPP_INFO(get_logger(),"Ready: %s",builder_?"/generate_reachability_cache":"/find_base_candidates and /reachability/goal_pose");
  }
private:
  template<class T>T param(const std::string& n,const T& v){if(!has_parameter(n))declare_parameter<T>(n,v);return get_parameter(n).get_value<T>();}
  std::shared_ptr<planning_scene::PlanningScene> snapshot(double seconds,moveit_msgs::msg::PlanningScene* message=nullptr) {
    if(seconds<=0)throw std::runtime_error("No time remaining for planning-scene snapshot");
    const auto end=Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
    if(!scene_client_->wait_for_service(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(seconds/2))))throw std::runtime_error("Planning-scene service unavailable");
    auto q=std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();using P=moveit_msgs::msg::PlanningSceneComponents;
    q->components.components=P::ROBOT_STATE|P::ROBOT_STATE_ATTACHED_OBJECTS|P::ALLOWED_COLLISION_MATRIX|P::LINK_PADDING_AND_SCALING|P::TRANSFORMS;
    auto f=scene_client_->async_send_request(q);
    if(f.wait_until(end)!=std::future_status::ready)throw std::runtime_error("Planning-scene snapshot timed out");
    const auto msg=f.get()->scene;
    for(const auto& g:groups_)for(const auto& name:model_->getJointModelGroup(g)->getVariableNames())
      if(std::find(msg.robot_state.joint_state.name.begin(),msg.robot_state.joint_state.name.end(),name)==msg.robot_state.joint_state.name.end())throw std::runtime_error("Snapshot missing "+name);
    auto scene=std::make_shared<planning_scene::PlanningScene>(model_);
    if(!scene->setPlanningSceneMsg(msg))throw std::runtime_error("Invalid planning-scene snapshot");
    scene->getCurrentStateNonConst().update();if(message)*message=msg;return scene;
  }
  bool valid_state(moveit::core::RobotState& s,const moveit::core::JointModelGroup* group,planning_scene::PlanningScene& scene)const {
    s.update();if(!s.satisfiesBounds(group))return false;
    for(const auto& name:group->getVariableNames()) {
      const auto& b=model_->getVariableBounds(name);double q=s.getVariablePosition(name);
      if(!std::isfinite(q)||(b.position_bounded_&&(q<b.min_position_+joint_margin_||q>b.max_position_-joint_margin_)))return false;
    }
    collision_detection::CollisionRequest q;collision_detection::CollisionResult r;scene.checkSelfCollision(q,r,s);return !r.collision;
  }
  void generate() {
    moveit_msgs::msg::PlanningScene message;auto scene=snapshot(10,&message);
    moveit::core::RobotState parked=scene->getCurrentState();parked.update();
    const Eigen::Isometry3d reference_from_model=parked.getGlobalLinkTransform(frame_).inverse();
    mr::Cache fresh;YAML::Node info;info["schema_version"]=1;info["fingerprint"]=fingerprint_;info["reference_frame"]=frame_;
    info["model_parameters"]=model_record_;info["cross_margin"]=margin_;info["joint_limit_margin"]=joint_margin_;
    info["samples_attempted_per_arm"]=samples_;info["random_seed"]=seed_;
    info["created_unix_ns"]=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    info["collision_scope"]="self only; snapshot ACM and other-arm/hand/attachments; environment excluded";
    for(const auto& name:model_->getVariableNames())info["snapshot_positions"][name]=parked.getVariablePosition(name);
    rclcpp::SerializedMessage serialized;rclcpp::Serialization<moveit_msgs::msg::PlanningScene> serializer;serializer.serialize_message(&message,&serialized);
    std::ostringstream hex;hex<<std::hex<<std::setfill('0');const auto& b=serialized.get_rcl_serialized_message();for(size_t i=0;i<b.buffer_length;++i)hex<<std::setw(2)<<unsigned(b.buffer[i]);
    info["scene_cdr_hex"]=hex.str();
    const auto start=Clock::now();std::mt19937 rng(static_cast<unsigned>(seed_));std::uniform_real_distribution<double> uniform(0,1);
    for(unsigned a=0;a<2;++a) {
      auto* group=model_->getJointModelGroup(groups_[a]);const auto names=group->getVariableNames();
      fresh.dofs[a]=names.size();info["joint_names"][sides[a]]=names;
      info["groups"][sides[a]]=groups_[a];info["tips"][sides[a]]=tips_[a];
      // Only scalar revolute/prismatic joints are supported, but dimensions/limits come from the model.
      for(auto* joint:group->getActiveJointModels())if(joint->getVariableCount()!=1)throw std::runtime_error("Unsupported multi-DOF arm joint");
      std::vector<std::pair<double,double>> bounds;
      for(const auto& name:names) {
        const auto& limit=model_->getVariableBounds(name);
        double lo=limit.position_bounded_?limit.min_position_+joint_margin_:-3.141592653589793;
        double hi=limit.position_bounded_?limit.max_position_-joint_margin_:3.141592653589793;
        if(!std::isfinite(lo)||!std::isfinite(hi)||lo>=hi)throw std::runtime_error("Invalid sampling limits for "+name);
        bounds.emplace_back(lo,hi);
      }
      moveit::core::RobotState state=parked;std::vector<double> q(names.size());auto last=Clock::now();
      for(int i=0;i<samples_;++i) {
        if(cancel_||!rclcpp::ok())throw std::runtime_error("Generation cancelled; previous cache retained");
        for(size_t j=0;j<q.size();++j)q[j]=bounds[j].first+uniform(rng)*(bounds[j].second-bounds[j].first);
        state.setJointGroupPositions(group,q);state.update();
        Eigen::Isometry3d tcp=reference_from_model*state.getGlobalLinkTransform(tips_[a]);
        bool side=a==0?tcp.translation().y()>=-margin_:tcp.translation().y()<=margin_;
        if(side&&valid_state(state,group,*scene)) {
          auto pose=plain(tcp);fresh.samples[a].push_back({pose.p,pose.q,q});
        }
        if(Clock::now()-last>3s){RCLCPP_INFO(get_logger(),"%s: %d/%d samples tried; %zu retained",sides[a].c_str(),i+1,samples_,fresh.samples[a].size());last=Clock::now();}
      }
      if(fresh.samples[a].empty())throw std::runtime_error("Zero valid samples for "+sides[a]+". Check parked state, ACM and model; cache not replaced.");
      info["retained"][sides[a]]=fresh.samples[a].size();
    }
    info["generation_seconds"]=std::chrono::duration<double>(Clock::now()-start).count();
    info["cache_id"]=mr::fingerprint(fingerprint_+info["created_unix_ns"].as<std::string>());
    fresh.metadata=YAML::Dump(info);
    fs::create_directories(fs::path(path_).parent_path());mr::save(fresh,path_+".tmp");
    if(fs::exists(path_))fs::copy_file(path_,path_+".previous",fs::copy_options::overwrite_existing);
    fs::rename(path_+".tmp",path_);
    cache_=std::move(fresh);cache_id_=info["cache_id"].as<std::string>();ready_=true;cache_markers();
    RCLCPP_INFO(get_logger(),"Saved %zu left / %zu right samples in %.1f s",cache_.samples[0].size(),cache_.samples[1].size(),std::chrono::duration<double>(Clock::now()-start).count());
  }
  void reload() {
    auto loaded=mr::load(path_);auto info=YAML::Load(loaded.metadata);
    if(info["schema_version"].as<int>()!=1||info["fingerprint"].as<std::string>()!=fingerprint_||info["reference_frame"].as<std::string>()!=frame_)
      throw std::runtime_error("Cache differs from model/settings. Generate a new cache with matching config.");
    for(unsigned a=0;a<2;++a)if(info["joint_names"][sides[a]].as<std::vector<std::string>>()!=model_->getJointModelGroup(groups_[a])->getVariableNames()||loaded.dofs[a]!=model_->getJointModelGroup(groups_[a])->getVariableCount())throw std::runtime_error("Cache joint order mismatch");
    cache_=std::move(loaded);cache_id_=info["cache_id"].as<std::string>();ready_=true;
    RCLCPP_INFO(get_logger(),"Loaded %zu left / %zu right cached poses",cache_.samples[0].size(),cache_.samples[1].size());
  }
  Eigen::Isometry3d transform(const std::string& to,const std::string& from,const rclcpp::Time& time) {
    if(to==from)return Eigen::Isometry3d::Identity();
    const auto t=tf_->lookupTransform(to,from,time,rclcpp::Duration::from_seconds(0)).transform;
    return eigen({{t.translation.x,t.translation.y,t.translation.z},{t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w}});
  }
  bool exact(unsigned arm,const mr::Candidate& c,const Eigen::Isometry3d& goal,
             const Eigen::Isometry3d& base_from_reference,double base_z,
             planning_scene::PlanningScene& scene,const moveit::core::RobotState& parked,
             Clock::time_point deadline,sensor_msgs::msg::JointState& solution) {
    const Eigen::Isometry3d world_base=planar(c,base_z);
    const Eigen::Isometry3d relative=(world_base*base_from_reference).inverse()*goal;
    if(arm==0?relative.translation().y()<-margin_:relative.translation().y()>margin_)return false;
    const Eigen::Isometry3d target=parked.getGlobalLinkTransform(frame_)*relative;
    auto* group=model_->getJointModelGroup(groups_[arm]);
    for(size_t seed:c.seeds) {
      double remaining=std::chrono::duration<double>(deadline-Clock::now()).count();if(remaining<=0)return false;
      moveit::core::RobotState state=parked;state.setJointGroupPositions(group,cache_.samples[arm][seed].joints);
      moveit::core::GroupStateValidityCallbackFn callback=[&](moveit::core::RobotState* s,const moveit::core::JointModelGroup* g,const double* q){
        s->setJointGroupPositions(g,q);s->update();
        const auto& actual=s->getGlobalLinkTransform(tips_[arm]);
        if((actual.translation()-target.translation()).norm()>pos_tol_||Eigen::AngleAxisd(target.linear().transpose()*actual.linear()).angle()>rot_tol_)return false;
        return valid_state(*s,g,scene);
      };
      bool ok=state.setFromIK(group,target,tips_[arm],std::min(ik_timeout_,remaining),callback);
      if(ok) {
        std::vector<double> values;state.copyJointGroupPositions(group,values);
        if(callback(&state,group,values.data())){solution.name=group->getVariableNames();solution.position=values;solution.header.stamp=now();return true;}
      }
    }
    return false;
  }
  void find(const Find::Request& request,Find::Response& response) {
    const auto start=Clock::now();clear_candidates();
    try {
      if(!ready_)throw std::runtime_error("No cache loaded. Generate then call /reload_reachability_cache.");
      double budget=request.time_budget==0?budget_:request.time_budget;
      unsigned max_verified=request.max_verified_per_arm==0?static_cast<unsigned>(max_verified_):request.max_verified_per_arm;
      if(!std::isfinite(budget)||budget<=0||budget>60||max_verified>1000)throw std::runtime_error("Invalid request budget/count");
      const auto deadline=start+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(budget));
      if(request.target.header.frame_id.empty())throw std::runtime_error("Target requires frame_id");
      // Respect the target's timestamp. Base/mount and current scene are a new snapshot.
      Eigen::Isometry3d goal=transform(fixed_,request.target.header.frame_id,rclcpp::Time(request.target.header.stamp))*from_pose(request.target.pose);
      Eigen::Isometry3d world_base=transform(fixed_,base_,rclcpp::Time(0,0,get_clock()->get_clock_type()));
      Eigen::Isometry3d mount=transform(base_,frame_,rclcpp::Time(0,0,get_clock()->get_clock_type()));
      if((world_base.linear().col(2)-Eigen::Vector3d::UnitZ()).norm()>.02)throw std::runtime_error("This query requires a level MiR base and level fixed frame");
      double base_z=world_base.translation().z();
      auto scene=snapshot(std::max(0.0,std::min(.75,std::chrono::duration<double>(deadline-Clock::now()).count())));
      moveit::core::RobotState parked=scene->getCurrentState();parked.update();
      std::array<mr::SearchResult,2> found;
      // Each arm gets a fair half of the remaining lookup allocation.
      const auto after_scene=Clock::now();double remaining=std::max(0.0,std::chrono::duration<double>(deadline-after_scene).count());
      for(unsigned a=0;a<2;++a) {
        auto end=after_scene+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(remaining*.2*(a+1)));
        found[a]=mr::search(cache_.samples[a],plain(mount),plain(goal),base_z,opt_,end);
        response.search_truncated=response.search_truncated||found[a].truncated;
      }
      auto stamp=now();response.cache_id=cache_id_;
      std::array<geometry_msgs::msg::PoseArray*,2> candidates{&response.left_candidates,&response.right_candidates};
      std::array<geometry_msgs::msg::PoseArray*,2> verified{&response.left_verified,&response.right_verified};
      std::array<std::vector<sensor_msgs::msg::JointState>*,2> solutions{&response.left_solutions,&response.right_solutions};
      for(unsigned a=0;a<2;++a) {
        candidates[a]->header.frame_id=verified[a]->header.frame_id=fixed_;
        candidates[a]->header.stamp=verified[a]->header.stamp=stamp;
        for(const auto& c:found[a].candidates)candidates[a]->poses.push_back(to_pose(planar(c,base_z)));
      }
      std::array<size_t,2> next{};
      while(Clock::now()<deadline) {
        bool progressed=false;
        for(unsigned a=0;a<2;++a) {
          if(Clock::now()>=deadline)break;
          if(next[a]>=found[a].candidates.size()||verified[a]->poses.size()>=max_verified)continue;
          progressed=true;const auto& c=found[a].candidates[next[a]++];sensor_msgs::msg::JointState joints;
          if(exact(a,c,goal,mount,base_z,*scene,parked,deadline,joints)) {
            verified[a]->poses.push_back(to_pose(planar(c,base_z)));solutions[a]->push_back(joints);
          }
        }
        if(!progressed)break;
      }
      response.budget_exhausted=Clock::now()>=deadline;
      response.success=true;
      std::ostringstream msg;msg<<"Candidates L/R="<<response.left_candidates.poses.size()<<"/"<<response.right_candidates.poses.size()
        <<"; exact IK+self-collision verified="<<response.left_verified.poses.size()<<"/"<<response.right_verified.poses.size()
        <<". Environment/navigation/path checks NOT performed. Results are a sampled, limited shortlist, not exhaustive.";
      response.message=msg.str();
      publish_result(response,goal);
    } catch(const std::exception& e){response.success=false;response.message=e.what();RCLCPP_ERROR(get_logger(),"%s",e.what());}
    response.elapsed_seconds=std::chrono::duration<double>(Clock::now()-start).count();
    RCLCPP_INFO(get_logger(),"Query %.3f s: %s",response.elapsed_seconds,response.message.c_str());
  }
  Marker marker(const std::string& frame,const std::string& ns,int type) {
    Marker m;m.header.frame_id=frame;m.header.stamp=now();m.ns=ns;m.id=0;m.type=type;m.action=Marker::ADD;m.pose.orientation.w=1;return m;
  }
  void cache_markers() {
    Markers all;
    for(unsigned a=0;a<2;++a) {
      auto m=marker(frame_,sides[a]+"_reachability",Marker::CUBE_LIST);m.frame_locked=true;
      m.scale.x=m.scale.y=m.scale.z=voxel_;m.color.r=a==0?.1f:1.f;m.color.g=.5;m.color.b=a==0?1.f:.05f;m.color.a=.45;
      std::set<std::tuple<long long,long long,long long>> occupied;
      for(const auto& s:cache_.samples[a])occupied.emplace(std::llround(std::floor(s.p[0]/voxel_)),std::llround(std::floor(s.p[1]/voxel_)),std::llround(std::floor(s.p[2]/voxel_)));
      for(const auto& cell:occupied) {geometry_msgs::msg::Point p;p.x=(std::get<0>(cell)+.5)*voxel_;p.y=(std::get<1>(cell)+.5)*voxel_;p.z=(std::get<2>(cell)+.5)*voxel_;m.points.push_back(p);}
      all.markers.push_back(m);
    }
    {std::lock_guard<std::mutex> guard(display_mutex_);display_=all;}marker_pub_->publish(all);
  }
  void publish_result(const Find::Response& r,const Eigen::Isometry3d& goal) {
    Markers all;
    std::array<const geometry_msgs::msg::PoseArray*,2> c{&r.left_candidates,&r.right_candidates},v{&r.left_verified,&r.right_verified};
    for(unsigned a=0;a<2;++a) {
      candidate_pubs_[a]->publish(*c[a]);verified_pubs_[a]->publish(*v[a]);
      for(unsigned status=0;status<2;++status) {
        auto m=marker(fixed_,sides[a]+(status?"_verified":"_candidates"),Marker::CUBE_LIST);
        m.scale.x=m.scale.y=opt_.resolution*.94;m.scale.z=.003;
        m.color.r=a==0?.1f:1.f;m.color.g=.5;m.color.b=a==0?1.f:.05f;m.color.a=status?.95f:.18f;
        std::set<std::pair<long long,long long>> cells;
        const auto& array=status?*v[a]:*c[a];
        for(const auto& pose:array.poses) {
          auto key=std::make_pair(std::llround(pose.position.x/opt_.resolution-.5),std::llround(pose.position.y/opt_.resolution-.5));
          if(cells.insert(key).second){auto p=pose.position;p.z=floor_+.006+.004*a+.012*status;m.points.push_back(p);}
        }
        all.markers.push_back(m);
      }
    }
    auto target=marker(fixed_,"target_tcp",Marker::ARROW);target.pose=to_pose(goal);target.scale.x=.15;target.scale.y=.025;target.scale.z=.025;
    target.color.r=1;target.color.g=1;target.color.a=1;all.markers.push_back(target);
    {std::lock_guard<std::mutex> guard(display_mutex_);display_=all;}marker_pub_->publish(all);
  }
  void clear_candidates() {
    Markers empty;auto m=marker(fixed_,"",Marker::CUBE_LIST);m.action=Marker::DELETEALL;empty.markers.push_back(m);
    {std::lock_guard<std::mutex> guard(display_mutex_);display_=empty;}marker_pub_->publish(empty);
    for(unsigned a=0;a<2;++a)if(candidate_pubs_[a]){geometry_msgs::msg::PoseArray p;p.header.frame_id=fixed_;p.header.stamp=now();candidate_pubs_[a]->publish(p);verified_pubs_[a]->publish(p);}
  }
  void republish(){std::lock_guard<std::mutex> guard(display_mutex_);marker_pub_->publish(display_);}
  bool builder_,ready_=false;
  std::string source_,frame_,base_,fixed_,scene_name_,path_,fingerprint_,cache_id_,model_record_;
  std::array<std::string,2> groups_,tips_;
  double margin_,joint_margin_,voxel_,budget_,ik_timeout_,pos_tol_,rot_tol_,floor_;
  int samples_,seed_,max_verified_;mr::SearchOptions opt_;mr::Cache cache_;
  std::shared_ptr<robot_model_loader::RobotModelLoader> loader_;moveit::core::RobotModelPtr model_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_client_;
  rclcpp::CallbackGroup::SharedPtr work_group_;
  rclcpp::Service<Trigger>::SharedPtr generation_srv_,cancel_srv_,reload_srv_;
  rclcpp::Service<Find>::SharedPtr find_srv_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_;std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<Markers>::SharedPtr marker_pub_;
  std::array<rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr,2> candidate_pubs_,verified_pubs_;
  rclcpp::TimerBase::SharedPtr timer_;std::mutex display_mutex_;Markers display_;std::atomic<bool> cancel_{false};
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {
#ifdef MR_CACHE_MODE
    constexpr bool builder=true;
#else
    constexpr bool builder=false;
#endif
    auto node=std::make_shared<Reachability>(builder,rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    node->initialize();rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),3);executor.add_node(node);executor.spin();
  }catch(const std::exception& e){std::cerr<<"mur_reachability: "<<e.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
