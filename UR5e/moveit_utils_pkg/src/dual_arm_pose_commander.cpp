#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

using std::placeholders::_1;

class DualArmPoseCommander : public rclcpp::Node
{
public:
  DualArmPoseCommander()
  : Node("dual_arm_moveit_commander")
  {
    declare_parameter<std::string>(
      "left_group", "left_ur_manipulator");

    declare_parameter<std::string>(
      "right_group", "right_ur_manipulator");

    declare_parameter<std::string>(
      "left_end_effector_link", "left_tcp");

    declare_parameter<std::string>(
      "right_end_effector_link", "right_tcp");

    declare_parameter<double>(
      "velocity_scaling", 0.10);

    declare_parameter<double>(
      "acceleration_scaling", 0.10);

    declare_parameter<double>(
      "planning_time", 5.0);

    declare_parameter<int>(
      "planning_attempts", 5);

    declare_parameter<bool>(
      "execute", false);

    left_group_ =
      get_parameter("left_group").as_string();

    right_group_ =
      get_parameter("right_group").as_string();

    left_ee_ =
      get_parameter("left_end_effector_link").as_string();

    right_ee_ =
      get_parameter("right_end_effector_link").as_string();

    velocity_scaling_ =
      get_parameter("velocity_scaling").as_double();

    acceleration_scaling_ =
      get_parameter("acceleration_scaling").as_double();

    planning_time_ =
      get_parameter("planning_time").as_double();

    planning_attempts_ =
      get_parameter("planning_attempts").as_int();

    execute_ =
      get_parameter("execute").as_bool();

    left_plan_pub_ =
      create_publisher<moveit_msgs::msg::RobotTrajectory>(
        "/left_arm/planned_trajectory", 10);

    right_plan_pub_ =
      create_publisher<moveit_msgs::msg::RobotTrajectory>(
        "/right_arm/planned_trajectory", 10);

    left_goal_sub_ =
      create_subscription<geometry_msgs::msg::PoseStamped>(
        "/left_arm/goal_pose",
        10,
        std::bind(
          &DualArmPoseCommander::left_goal_callback,
          this,
          _1));

    right_goal_sub_ =
      create_subscription<geometry_msgs::msg::PoseStamped>(
        "/right_arm/goal_pose",
        10,
        std::bind(
          &DualArmPoseCommander::right_goal_callback,
          this,
          _1));

    RCLCPP_INFO(
      get_logger(),
      "Dual-arm MoveIt commander created");

    RCLCPP_INFO(
      get_logger(),
      "Left group: %s, EE: %s",
      left_group_.c_str(),
      left_ee_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Right group: %s, EE: %s",
      right_group_.c_str(),
      right_ee_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Execute: %s",
      execute_ ? "true" : "false");
  }

  void initialize()
  {
    /*
     * MoveGroupInterface needs shared_from_this(), so it cannot safely
     * be constructed in the Node constructor.
     */
    left_move_group_ =
      std::make_unique<
        moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(),
          left_group_);

    right_move_group_ =
      std::make_unique<
        moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(),
          right_group_);

    configure_group(
      *left_move_group_,
      left_ee_);

    configure_group(
      *right_move_group_,
      right_ee_);

    left_move_group_->startStateMonitor(2.0);
    right_move_group_->startStateMonitor(2.0);

    RCLCPP_INFO(
      get_logger(),
      "MoveGroupInterfaces initialized");

    RCLCPP_INFO(
      get_logger(),
      "Left planning frame: %s",
      left_move_group_->getPlanningFrame().c_str());

    RCLCPP_INFO(
      get_logger(),
      "Right planning frame: %s",
      right_move_group_->getPlanningFrame().c_str());
  }

private:
  void configure_group(
    moveit::planning_interface::MoveGroupInterface & group,
    const std::string & end_effector)
  {
    group.setEndEffectorLink(end_effector);

    group.setMaxVelocityScalingFactor(
      velocity_scaling_);

    group.setMaxAccelerationScalingFactor(
      acceleration_scaling_);

    group.setPlanningTime(
      planning_time_);

    group.setNumPlanningAttempts(
      planning_attempts_);
  }

  void left_goal_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const auto target = *msg;

    std::thread(
      [this, target]()
      {
        std::lock_guard<std::mutex> lock(left_mutex_);

        plan_pose(
          "left",
          *left_move_group_,
          left_ee_,
          target,
          left_plan_pub_);
      }).detach();
  }

  void right_goal_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const auto target = *msg;

    std::thread(
      [this, target]()
      {
        std::lock_guard<std::mutex> lock(right_mutex_);

        plan_pose(
          "right",
          *right_move_group_,
          right_ee_,
          target,
          right_plan_pub_);
      }).detach();
  }

  void plan_pose(
    const std::string & arm_name,
    moveit::planning_interface::MoveGroupInterface & move_group,
    const std::string & ee_link,
    const geometry_msgs::msg::PoseStamped & target,
    const rclcpp::Publisher<
      moveit_msgs::msg::RobotTrajectory>::SharedPtr & plan_pub)
  {
    if (target.header.frame_id.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "%s target has empty frame_id",
        arm_name.c_str());
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Received %s target",
      arm_name.c_str());

    RCLCPP_INFO(
      get_logger(),
      "  frame: %s",
      target.header.frame_id.c_str());

    RCLCPP_INFO(
      get_logger(),
      "  position: %.4f %.4f %.4f",
      target.pose.position.x,
      target.pose.position.y,
      target.pose.position.z);

    RCLCPP_INFO(
      get_logger(),
      "  EE link: %s",
      ee_link.c_str());

    // Require a current robot state before solving IK.

    auto current_state = move_group.getCurrentState(2.0);

    if (!current_state) {
      RCLCPP_ERROR(
        get_logger(),
        "%s: no complete current robot state available; refusing IK",
        arm_name.c_str());
      return;
    }

    move_group.setStartStateToCurrentState();
    move_group.clearPoseTargets();

    const bool target_ok =
      move_group.setJointValueTarget(
        target,
        ee_link);

    if (!target_ok) {
      RCLCPP_ERROR(
        get_logger(),
        "%s: IK failed for target pose",
        arm_name.c_str());

      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    RCLCPP_INFO(
      get_logger(),
      "%s: planning...",
      arm_name.c_str());

    const auto result =
      move_group.plan(plan);

    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(
        get_logger(),
        "%s: planning failed, MoveIt error code %d",
        arm_name.c_str(),
        result.val);

      move_group.clearPoseTargets();
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "%s: planning succeeded with %zu trajectory points",
      arm_name.c_str(),
      plan.trajectory_.joint_trajectory.points.size());

    /*
     * Publish the exact trajectory that would be executed.
     * This means we can inspect it before enabling execution.
     */
    plan_pub->publish(plan.trajectory_);

    if (!execute_) {
      RCLCPP_INFO(
        get_logger(),
        "%s: execution disabled -- plan only",
        arm_name.c_str());

      move_group.clearPoseTargets();
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "%s: EXECUTING TRAJECTORY",
      arm_name.c_str());

    const auto exec_result =
      move_group.execute(plan);

    if (exec_result ==
        moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(
        get_logger(),
        "%s: execution succeeded",
        arm_name.c_str());
    }
    else
    {
      RCLCPP_ERROR(
        get_logger(),
        "%s: execution failed, MoveIt error code %d",
        arm_name.c_str(),
        exec_result.val);
    }

    move_group.clearPoseTargets();
  }

  std::string left_group_;
  std::string right_group_;

  std::string left_ee_;
  std::string right_ee_;

  double velocity_scaling_;
  double acceleration_scaling_;
  double planning_time_;

  int planning_attempts_;

  bool execute_;

  std::unique_ptr<
    moveit::planning_interface::MoveGroupInterface>
    left_move_group_;

  std::unique_ptr<
    moveit::planning_interface::MoveGroupInterface>
    right_move_group_;

  rclcpp::Subscription<
    geometry_msgs::msg::PoseStamped>::SharedPtr
    left_goal_sub_;

  rclcpp::Subscription<
    geometry_msgs::msg::PoseStamped>::SharedPtr
    right_goal_sub_;

  rclcpp::Publisher<
    moveit_msgs::msg::RobotTrajectory>::SharedPtr
    left_plan_pub_;

  rclcpp::Publisher<
    moveit_msgs::msg::RobotTrajectory>::SharedPtr
    right_plan_pub_;

  std::mutex left_mutex_;
  std::mutex right_mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<DualArmPoseCommander>();

  /*
   * Spin ROS callbacks on a dedicated thread.
   * MoveGroupInterface's CurrentStateMonitor needs this so
   * /joint_states can continue being processed while planning/IK
   * is running.
   */
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  /*
   * Initialize MoveGroupInterface after spinning has already started.
   */
  node->initialize();

  spinner.join();

  rclcpp::shutdown();
  return 0;
}
