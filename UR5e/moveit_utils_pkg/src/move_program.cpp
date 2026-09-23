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

const bool ALLOW_EXECUTION_OF_PARTIAL_PATH = true;

const double velocity_scaling_factor = 0.05;
const double acceleration_scaling_factor = 0.05;
const double eef_step = 0.01;
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


bool cartesian_move_to_pose_in_frame(moveit::planning_interface::MoveGroupInterface& mgi, tf2_ros::Buffer& tf_buffer, const std::string& frame, double x, double y, double z, double rx, double ry, double rz, double rw) {
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
    if (!ALLOW_EXECUTION_OF_PARTIAL_PATH) {
      return false;
    }
  }

  // time-parametrisize the trajectory
  traj = apply_time_parameterization(traj, mgi, velocity_scaling_factor, acceleration_scaling_factor);

  // exexute
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = traj;
  return (mgi.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
}


bool move_to_pose_in_frame(moveit::planning_interface::MoveGroupInterface& mgi, const std::string& frame, double x, double y, double z, double rx, double ry, double rz, double rw) {
  mgi.setStartStateToCurrentState();
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



int main(int argc, char** argv) {
  // init node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("move_program", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto const logger = rclcpp::get_logger("move_program");

  // start spinning
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  // init movegroupinterface
  moveit::planning_interface::MoveGroupInterface movegroupinterface(node, "left_ur_manipulator");
  movegroupinterface.setMaxVelocityScalingFactor(velocity_scaling_factor);  // % of max joint speed
  movegroupinterface.setMaxAccelerationScalingFactor(acceleration_scaling_factor);  // % of max joint accel

  // init tf buffer and wait for fill
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf2_ros::TransformListener tf_listener(*tf_buffer);
  rclcpp::sleep_for(std::chrono::seconds(1));

  moveit::planning_interface::PlanningSceneInterface psi;
  set_robot_link_padding(psi, movegroupinterface, 0.01); // add padding to links
  // --------------------------------------- init complete ---------------------------------------------



  // Move to named pose "retreated"
  move_to_named_target(movegroupinterface, "retreated");

  bool success;
  do {
    success = cartesian_move_to_pose_in_frame(movegroupinterface, *tf_buffer, "table_corner", -0.35, 0.05, 0.20, // x,y,z
      0.70710678, 0.0, 0.0, 0.70710678); // rx,ry,rz,rw
    if (!success) break;

    success = cartesian_move_to_pose_in_frame(movegroupinterface, *tf_buffer, "table_corner", 0.0, 0.05, 0.09, // x,y,z
      0.70710678, 0.0, 0.0, 0.70710678); // rx,ry,rz,rw
    if (!success) break;

    success = cartesian_move_to_pose_in_frame(movegroupinterface, *tf_buffer, "table_corner", 0.29, 0.05, 0.09, // x,y,z
      0.70710678, 0.0, 0.0, 0.70710678); // rx,ry,rz,rw
    if (!success) break;

    success = cartesian_move_to_pose_in_frame(movegroupinterface, *tf_buffer, "table_corner", 0.29, 0.05, 0.3, // x,y,z
      0.70710678, 0.0, 0.0, 0.70710678); // rx,ry,rz,rw
    if (!success) break;

    success = cartesian_move_to_pose_in_frame(movegroupinterface, *tf_buffer, "table_corner", 0.29, 0.35, 0.3, // x,y,z
      0.70710678, 0.0, 0.0, 0.70710678); // rx,ry,rz,rw
    if (!success) break;

    // pour
    success = cartesian_move_to_pose_in_frame(movegroupinterface, *tf_buffer, "table_corner", 0.29, 0.35, 0.3, // x,y,z
      -0.573576436, 0.0, 0.0, 0.819152044); // rx,ry,rz,rw
    if (!success) break;

    success = cartesian_move_to_pose_in_frame(movegroupinterface, *tf_buffer, "table_corner", 0.29, 0.35, 0.3, // x,y,z
      0.70710678, 0.0, 0.0, 0.70710678); // rx,ry,rz,rw
    if (!success) break;

    success = cartesian_move_to_pose_in_frame(movegroupinterface, *tf_buffer, "table_corner", 0.29, 0.05, 0.3, // x,y,z
      0.70710678, 0.0, 0.0, 0.70710678); // rx,ry,rz,rw
    if (!success) break;

    success = cartesian_move_to_pose_in_frame(movegroupinterface, *tf_buffer, "table_corner", 0.29, 0.05, 0.09, // x,y,z
      0.70710678, 0.0, 0.0, 0.70710678); // rx,ry,rz,rw
    if (!success) break;

    success = cartesian_move_to_pose_in_frame(movegroupinterface, *tf_buffer, "table_corner", 0.0, 0.05, 0.09, // x,y,z
      0.70710678, 0.0, 0.0, 0.70710678); // rx,ry,rz,rw
    if (!success) break;

    success = cartesian_move_to_pose_in_frame(movegroupinterface, *tf_buffer, "table_corner", -0.35, 0.05, 0.20, // x,y,z
      0.70710678, 0.0, 0.0, 0.70710678); // rx,ry,rz,rw
    if (!success) break;
  } while(false);

  // Move to named pose "retreated"
  //move_to_named_target(movegroupinterface, "retreated");


  exec.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}