#include <memory>
#include <thread>
#include <string>

// MoveIt
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// tf
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

//cartesian path
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

// actions
#include <interfaces_pkg/action/move_to_pose_in_frame.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
using MoveToPoseInFrame = interfaces_pkg::action::MoveToPoseInFrame;
using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPoseInFrame>;

const double acceleration_scaling_factor = 0.02;
const double eef_step = 0.002; // 0.01
const double jump_threshold = 2.8; // 1.5


moveit_msgs::msg::RobotTrajectory apply_time_parameterization(const moveit_msgs::msg::RobotTrajectory& input_traj, const moveit::planning_interface::MoveGroupInterface& move_group, double max_vel_scaling = 0.05, double max_acc_scaling = 0.05) {
  robot_trajectory::RobotTrajectory traj(move_group.getRobotModel(), move_group.getName());
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  traj.setRobotTrajectoryMsg(*current_state, input_traj);

  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  iptp.computeTimeStamps(traj, max_vel_scaling, max_acc_scaling);

  moveit_msgs::msg::RobotTrajectory output_traj;
  traj.getRobotTrajectoryMsg(output_traj);
  return output_traj;
}


void set_robot_link_padding(moveit::planning_interface::PlanningSceneInterface& psi, const moveit::planning_interface::MoveGroupInterface& mgi, double padding_m) {
  auto logger = rclcpp::get_logger("set_robot_link_padding");

  moveit_msgs::msg::PlanningScene ps;
  ps.is_diff = true;

  const auto& robot_model = mgi.getRobotModel();
  const auto& links = robot_model->getLinkModelNames();

  for (const auto& link_name : links)
  {
    moveit_msgs::msg::LinkPadding lp;
    lp.link_name = link_name;
    lp.padding   = padding_m;
    ps.link_padding.push_back(lp);
  }
  psi.applyPlanningScene(ps);
}


bool move_to_named_target(moveit::planning_interface::MoveGroupInterface& mgi, const std::string& named_target) {
  mgi.setNamedTarget(named_target);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (mgi.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    if (mgi.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      return true;
    }
  }
  return false;
}


bool cartesian_move_to_pose_in_frame(moveit::planning_interface::MoveGroupInterface& mgi, tf2_ros::Buffer& tf_buffer, const std::string& frame, double vel_scaling, bool allow_exec_partial_path, double x, double y, double z, double rx, double ry, double rz, double rw) {
  auto logger = rclcpp::get_logger("cartesian_move_to_pose_in_frame");

  // target pose in the given frame
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = frame;
  target_pose.pose.position.x = x;
  target_pose.pose.position.y = y;
  target_pose.pose.position.z = z;
  target_pose.pose.orientation.x = rx;
  target_pose.pose.orientation.y = ry;
  target_pose.pose.orientation.z = rz;
  target_pose.pose.orientation.w = rw;

  // transform target pose to planning frame
  const std::string planning_frame = mgi.getPlanningFrame();
  geometry_msgs::msg::PoseStamped target_pose_in_planning_frame;
  try {
    auto tf = tf_buffer.lookupTransform(planning_frame, frame, tf2::TimePointZero);
    tf2::doTransform(target_pose, target_pose_in_planning_frame, tf);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(logger, ex.what());
    return false;
  }

  // add to waypoints and compute cartesian path
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target_pose_in_planning_frame.pose);
  mgi.setStartStateToCurrentState();
  moveit_msgs::msg::RobotTrajectory traj;
  double fraction = mgi.computeCartesianPath(
      waypoints,
      eef_step,
      jump_threshold,
      traj,
      true
  );

  if (fraction < 1.0) {
    RCLCPP_ERROR(logger, "cartesian path computation failed (fraction=%.2f)", fraction);
    if (!allow_exec_partial_path) {
      return false;
    }
  }

  // time-parametrisize the trajectory
  traj = apply_time_parameterization(traj, mgi, vel_scaling, acceleration_scaling_factor);

  // execute
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = traj;
  return (mgi.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
}


bool move_to_pose_in_frame(moveit::planning_interface::MoveGroupInterface& mgi, const std::string& frame, double vel_scaling, double x, double y, double z, double rx, double ry, double rz, double rw) {
  mgi.setStartStateToCurrentState();
  mgi.setMaxVelocityScalingFactor(vel_scaling);
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = frame;
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = z;
  goal.pose.orientation.x = rx;
  goal.pose.orientation.y = ry;
  goal.pose.orientation.z = rz;
  goal.pose.orientation.w = rw;

  mgi.setPoseTarget(goal);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (mgi.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    if (mgi.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      return true;
    }
  }
  return false;
}


class MoveItInterfaceNode : public rclcpp::Node {
  public:
    using ActionT = MoveToPoseInFrame;
    using GoalHandle = GoalHandleMoveToPose;
  
    explicit MoveItInterfaceNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : rclcpp::Node("moveit_interface_node", options),
      logger_(this->get_logger()),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(*tf_buffer_) {

      init_timer_ = this->create_wall_timer(std::chrono::milliseconds(0), [this]() {
        movegroupinterface_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(), "left_ur_manipulator");

        movegroupinterface_->setMaxAccelerationScalingFactor(acceleration_scaling_factor);

        // wait for tf buffer to fill
        rclcpp::sleep_for(std::chrono::seconds(1));

        set_robot_link_padding(psi_, *movegroupinterface_, 0.018);

        using namespace std::placeholders;
        action_server_ = rclcpp_action::create_server<ActionT>(
          this,
          "move_to_pose_in_frame",
          std::bind(&MoveItInterfaceNode::handle_goal, this, _1, _2),
          std::bind(&MoveItInterfaceNode::handle_cancel, this, _1),
          std::bind(&MoveItInterfaceNode::handle_accepted, this, _1));

        ready_ = true;
        RCLCPP_INFO(logger_, "Action server ready on /move_to_pose_in_frame");
        init_timer_->cancel();
      });
    }

  private:
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const ActionT::Goal> goal) {
      if (goal->frame.empty()) {
        RCLCPP_WARN(logger_, "Rejecting goal: frame must be non-empty");
        return rclcpp_action::GoalResponse::REJECT;
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
  
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>) {
      RCLCPP_INFO(logger_, "Cancel request accepted");
      return rclcpp_action::CancelResponse::ACCEPT;
    }
  
    void handle_accepted(const std::shared_ptr<GoalHandle> gh) {
      std::thread{&MoveItInterfaceNode::execute, this, gh}.detach();
    }
  
    void execute(const std::shared_ptr<GoalHandle> gh) {
      const auto goal = gh->get_goal();
      auto result = std::make_shared<ActionT::Result>();
  
      // TODO handle canceling

      RCLCPP_INFO(logger_, "rcv velocity_scaling_factor = %f", goal->velocity_scaling_factor);

      double velocity = goal->velocity_scaling_factor;
      if (velocity <= 0.0) {
        velocity = 0.01; // default to velocity 0.01 
      }
      if (goal->use_cartesian_path) {
        result->success = cartesian_move_to_pose_in_frame(*movegroupinterface_, *tf_buffer_, goal->frame, velocity, goal->allow_exec_partial_path,
          goal->x, goal->y, goal->z,
          goal->rx, goal->ry, goal->rz, goal->rw);
      } else {
        result->success = move_to_pose_in_frame(*movegroupinterface_, goal->frame, velocity,
          goal->x, goal->y, goal->z,
          goal->rx, goal->ry, goal->rz, goal->rw);
      }
  
      if (result->success) {
        gh->succeed(result);
        RCLCPP_INFO(logger_, "Goal succeeded");
      } else {
        gh->abort(result);
        RCLCPP_ERROR(logger_, "Goal aborted (planning/execution failed)");
      }
    }
  
    rclcpp::Logger logger_;
    moveit::planning_interface::PlanningSceneInterface psi_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> movegroupinterface_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    bool ready_{false};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp_action::Server<ActionT>::SharedPtr action_server_;
  };
  
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveItInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}