// Offline volume generation only; no services, legacy cache or online IK API.
#include "offline_model.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <thread>
namespace py=pybind11;
class EmbeddedEngine {
  std::shared_ptr<OfflineModel> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread thread_;
  bool owns_context_=false;
  static Eigen::Isometry3d unpack(const std::array<double,7>& v){
    for(auto x:v)if(!std::isfinite(x))throw std::runtime_error("Nonfinite transform");
    Eigen::Quaterniond q(v[6],v[3],v[4],v[5]);
    if(q.squaredNorm()<1e-16)throw std::runtime_error("Invalid quaternion");
    q.normalize();Eigen::Isometry3d t=Eigen::Isometry3d::Identity();
    t.translation()=Eigen::Vector3d(v[0],v[1],v[2]);t.linear()=q.toRotationMatrix();return t;
  }
public:
  EmbeddedEngine(const std::string& config,bool sim_time,const std::string& node_name="mur_kinematics"){
    auto context=rclcpp::contexts::get_global_default_context();
    if(!context->is_valid()){context->init(0,nullptr);owns_context_=true;}
    try{
      rclcpp::NodeOptions options;
      options.use_global_arguments(false).arguments({"--ros-args","--params-file",config,"-r","__node:="+node_name})
        .automatically_declare_parameters_from_overrides(true)
        .parameter_overrides({rclcpp::Parameter("use_sim_time",sim_time)});
      node_=std::make_shared<OfflineModel>(options);node_->initialize();
      executor_=std::make_unique<rclcpp::executors::SingleThreadedExecutor>();executor_->add_node(node_);
      thread_=std::thread([this]{executor_->spin();});
    }catch(...){close();throw;}
  }
  ~EmbeddedEngine(){close();}
  void close(){
    if(executor_)executor_->cancel();
    if(thread_.joinable())thread_.join();
    if(executor_&&node_)executor_->remove_node(node_);
    executor_.reset();node_.reset();
    if(owns_context_){rclcpp::contexts::get_global_default_context()->shutdown("Embedded engine closed");owns_context_=false;}
  }
  std::string model_info(){
    YAML::Node meta;meta["model_fingerprint"]=node_->fingerprint_;
    meta["reference_frame"]=node_->frame_;meta["base_frame"]=node_->base_;
    meta["model_parameters"]=node_->model_record_;
    meta["robot_description"]=node_->get_parameter("robot_description").as_string();
    meta["robot_description_semantic"]=node_->get_parameter("robot_description_semantic").as_string();
    meta["source_node"]=node_->source_;
    for(unsigned a=0;a<2;++a){
      meta["ignored_collision_links"][sides[a]]=node_->arm_links_[1-a];
      meta["groups"][sides[a]]=node_->groups_[a];meta["tips"][sides[a]]=node_->tips_[a];
      meta["arm_joints"][sides[a]]=node_->model_->getJointModelGroup(node_->groups_[a])->getVariableNames();
    }
    meta["reachability_policy"]="self_body_ignore_other_arm_v1";
    return YAML::Dump(meta);
  }
  std::vector<bool> evaluate_offline(unsigned arm,const std::vector<std::array<double,7>>& poses,
      const std::array<double,7>& mount,double seconds,unsigned attempts){
    if(!node_||arm>1||seconds<=0||!std::isfinite(seconds)||attempts<1||attempts>100)
      throw std::runtime_error("Invalid offline request");
    auto group=node_->model_->getJointModelGroup(node_->groups_[arm]);
    moveit::core::RobotState baseline(node_->model_);baseline.setToDefaultValues();baseline.update();const Eigen::Isometry3d inv=unpack(mount).inverse();
    auto model_reference=baseline.getGlobalLinkTransform(node_->frame_);
    std::vector<bool> result;result.reserve(poses.size());
    std::vector<double> warm;baseline.copyJointGroupPositions(group,warm);
    for(const auto& pose:poses){
      const Eigen::Isometry3d relative=inv*unpack(pose);
      const Eigen::Isometry3d target=model_reference*relative;
      bool success=false;
      for(unsigned i=0;i<attempts&&!success;++i){
        if(!rclcpp::ok())throw std::runtime_error("Offline generation interrupted");
        moveit::core::RobotState state=baseline;
        if(i)state.setToRandomPositions(group);else state.setJointGroupPositions(group,warm);
        moveit::core::GroupStateValidityCallbackFn check=[&](moveit::core::RobotState* s,const moveit::core::JointModelGroup* g,const double* q){
          s->setJointGroupPositions(g,q);s->update();const auto& actual=s->getGlobalLinkTransform(node_->tips_[arm]);
          return (actual.translation()-target.translation()).norm()<=node_->pos_tol_ &&
            Eigen::AngleAxisd(target.linear().transpose()*actual.linear()).angle()<=node_->rot_tol_ && node_->valid_state(*s,g,arm);
        };
        success=state.setFromIK(group,target,node_->tips_[arm],seconds,check);
        if(success){std::vector<double> values;state.copyJointGroupPositions(group,values);success=check(&state,group,values.data());if(success)warm=values;}
      }
      result.push_back(success);
    }
    return result;
  }
};
PYBIND11_MODULE(mur_reachability_engine,m){
  py::class_<EmbeddedEngine>(m,"Engine")
    .def(py::init<const std::string&,bool,const std::string&>(),py::arg("config"),py::arg("sim_time"),py::arg("node_name")="mur_kinematics",py::call_guard<py::gil_scoped_release>())
    .def("model_info",&EmbeddedEngine::model_info,py::call_guard<py::gil_scoped_release>())
    .def("evaluate_offline",&EmbeddedEngine::evaluate_offline,py::call_guard<py::gil_scoped_release>())
    .def("close",&EmbeddedEngine::close,py::call_guard<py::gil_scoped_release>());
}
