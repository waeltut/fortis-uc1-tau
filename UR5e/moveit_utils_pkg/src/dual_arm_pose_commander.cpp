#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <exception>
#include <future>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

using MoveGroup = moveit::planning_interface::MoveGroupInterface;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using RobotTrajectory = moveit_msgs::msg::RobotTrajectory;
using Clock = std::chrono::steady_clock;

class DualArmPoseCommander : public rclcpp::Node
{
public:
  DualArmPoseCommander()
  : Node("dual_arm_moveit_commander")
  {
    declare_parameter<std::string>("left_group", "left_ur_manipulator");
    declare_parameter<std::string>("right_group", "right_ur_manipulator");
    declare_parameter<std::string>("dual_group", "both_ur_manipulator");
    declare_parameter<std::string>("left_end_effector_link", "left_tcp");
    declare_parameter<std::string>("right_end_effector_link", "right_tcp");
    declare_parameter<double>("velocity_scaling", 0.10);
    declare_parameter<double>("acceleration_scaling", 0.10);
    declare_parameter<double>("planning_time", 5.0);
    declare_parameter<int>("planning_attempts", 5);
    declare_parameter<double>("pairing_window_sec", 0.15);
    declare_parameter<bool>("execute", true);
    // 0 means unlimited. Nonzero rejects arrivals once that arm's queue is full.
    declare_parameter<int>("max_queue_per_arm", 0);

    // Configurable service home: poses expressed in the same reference frame.
    declare_parameter<std::string>("home_frame", "mur");
    declare_parameter<std::vector<double>>("home_left_position", {0.6, 0.15, 1.2});
    declare_parameter<std::vector<double>>("home_right_position", {0.6, -0.15, 1.2});
    declare_parameter<std::vector<double>>(
      "home_finger_positions", {0.9, 0.0, 1.0, 1.0, 1.0, 1.0});

    left_group_ = get_parameter("left_group").as_string();
    right_group_ = get_parameter("right_group").as_string();
    dual_group_ = get_parameter("dual_group").as_string();
    left_ee_ = get_parameter("left_end_effector_link").as_string();
    right_ee_ = get_parameter("right_end_effector_link").as_string();
    velocity_scaling_ = get_parameter("velocity_scaling").as_double();
    acceleration_scaling_ = get_parameter("acceleration_scaling").as_double();
    planning_time_ = get_parameter("planning_time").as_double();
    planning_attempts_ = get_parameter("planning_attempts").as_int();
    pairing_window_sec_ = get_parameter("pairing_window_sec").as_double();
    execute_ = get_parameter("execute").as_bool();
    max_queue_per_arm_ = get_parameter("max_queue_per_arm").as_int();
    home_frame_ = get_parameter("home_frame").as_string();
    home_left_position_ = get_parameter("home_left_position").as_double_array();
    home_right_position_ = get_parameter("home_right_position").as_double_array();
    home_finger_positions_ = get_parameter("home_finger_positions").as_double_array();

    if (!std::isfinite(pairing_window_sec_) || pairing_window_sec_ < 0.0 ||
        pairing_window_sec_ > 5.0) {
      throw std::invalid_argument("pairing_window_sec must be between 0 and 5 seconds");
    }
    if (max_queue_per_arm_ < 0) {
      throw std::invalid_argument("max_queue_per_arm must be >= 0");
    }
    if (!std::isfinite(velocity_scaling_) || velocity_scaling_ <= 0.0 ||
        velocity_scaling_ > 1.0 || !std::isfinite(acceleration_scaling_) ||
        acceleration_scaling_ <= 0.0 || acceleration_scaling_ > 1.0 ||
        !std::isfinite(planning_time_) || planning_time_ <= 0.0 || planning_attempts_ < 1) {
      throw std::invalid_argument("Invalid planning/scaling parameters");
    }

    if (home_frame_.empty() || home_left_position_.size() != 3 ||
        home_right_position_.size() != 3 || home_finger_positions_.size() != 6 ||
        !all_finite(home_left_position_) || !all_finite(home_right_position_) ||
        !all_finite(home_finger_positions_)) {
      throw std::invalid_argument("Invalid home pose/finger parameters");
    }

    home_left_fingers_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      "/hand_L/finger_positions", 10);
    home_right_fingers_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      "/hand_R/finger_positions", 10);
    home_status_pub_ = create_publisher<std_msgs::msg::String>(
      "/dual_arm/home_status", rclcpp::QoS(10).reliable().transient_local());
    home_service_ = create_service<std_srvs::srv::Trigger>(
      "/dual_arm/go_home",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        queue_home(std::move(response));
      });

    left_plan_pub_ = create_publisher<RobotTrajectory>("/left_arm/planned_trajectory", 10);
    right_plan_pub_ = create_publisher<RobotTrajectory>("/right_arm/planned_trajectory", 10);
    dual_plan_pub_ = create_publisher<RobotTrajectory>("/dual_arm/planned_trajectory", 10);

    left_goal_sub_ = create_subscription<PoseStamped>(
      "/left_arm/goal_pose", rclcpp::QoS(100),
      [this](const PoseStamped::SharedPtr msg) { enqueue(Arm::Left, *msg); });
    right_goal_sub_ = create_subscription<PoseStamped>(
      "/right_arm/goal_pose", rclcpp::QoS(100),
      [this](const PoseStamped::SharedPtr msg) { enqueue(Arm::Right, *msg); });

    RCLCPP_INFO(get_logger(), "Automatic dual-arm pairing, window=%.0f ms, execute=%s",
      1000.0 * pairing_window_sec_, execute_ ? "true" : "false");
  }

  ~DualArmPoseCommander() override { stop(); }

  // Call only after the node is held by shared_ptr and a ROS executor is spinning.
  void initialize()
  {
    left_move_group_ = std::make_unique<MoveGroup>(shared_from_this(), left_group_);
    right_move_group_ = std::make_unique<MoveGroup>(shared_from_this(), right_group_);
    dual_move_group_ = std::make_unique<MoveGroup>(shared_from_this(), dual_group_);

    configure_arm(*left_move_group_, left_ee_);
    configure_arm(*right_move_group_, right_ee_);
    configure_motion(*dual_move_group_);

    left_move_group_->startStateMonitor(2.0);
    right_move_group_->startStateMonitor(2.0);
    dual_move_group_->startStateMonitor(2.0);
    validate_combined_group();

    RCLCPP_INFO(get_logger(), "Single groups: %s / %s; combined group: %s",
      left_group_.c_str(), right_group_.c_str(), dual_group_.c_str());
    RCLCPP_INFO(get_logger(), "Planning frames: L=%s R=%s dual=%s",
      left_move_group_->getPlanningFrame().c_str(),
      right_move_group_->getPlanningFrame().c_str(),
      dual_move_group_->getPlanningFrame().c_str());

    worker_ = std::thread([this]() { worker_loop(); });
  }

  void stop()
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      stopping_ = true;
    }
    queue_cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();  // Wait for any in-flight plan/execute call to finish.
    }
  }

private:
  enum class Arm { Left, Right };

  struct Goal
  {
    PoseStamped pose;
    Clock::time_point received_at;
    std::uint64_t sequence;
    std::uint64_t home_id = 0;  // Nonzero only for a protected, atomic service pair.
  };

  struct PlanResult
  {
    bool success = false;
    MoveGroup::Plan plan;
  };

  static bool all_finite(const std::vector<double>& values)
  {
    return std::all_of(values.begin(), values.end(),
      [](double value) { return std::isfinite(value); });
  }

  PoseStamped home_pose(const std::vector<double>& position) const
  {
    PoseStamped result;
    result.header.frame_id = home_frame_;
    // Stamp 0 intentionally asks TF for the latest transform when this queued
    // request actually runs, rather than using a timestamp that may go stale.
    result.pose.position.x = position[0];
    result.pose.position.y = position[1];
    result.pose.position.z = position[2];
    result.pose.orientation.x = 0.0;
    result.pose.orientation.y = 0.0;
    result.pose.orientation.z = 0.0;
    result.pose.orientation.w = 1.0;
    return result;
  }

  void home_status(const std::string& status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    home_status_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "%s", status.c_str());
  }

  // Insert left+right atomically into the *same* queues used by normal topics.
  // They must never be paired with unrelated topic goals.
  void queue_home(std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (home_left_fingers_pub_->get_subscription_count() == 0 ||
        home_right_fingers_pub_->get_subscription_count() == 0) {
      response->success = false;
      response->message = "Hand command subscriber missing; home not queued";
      return;
    }
    const PoseStamped left = home_pose(home_left_position_);
    const PoseStamped right = home_pose(home_right_position_);
    std::uint64_t id = 0;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (stopping_) {
        response->success = false;
        response->message = "Commander stopped; inspect hardware and restart";
        return;
      }
      if (max_queue_per_arm_ > 0 &&
          (left_queue_.size() >= static_cast<std::size_t>(max_queue_per_arm_) ||
           right_queue_.size() >= static_cast<std::size_t>(max_queue_per_arm_))) {
        response->success = false;
        response->message = "At least one arm queue is full; neither home goal queued";
        return;
      }
      id = next_home_id_++;
      const auto arrival = Clock::now();
      left_queue_.push_back(Goal{left, arrival, next_sequence_++, id});
      right_queue_.push_back(Goal{right, arrival, next_sequence_++, id});
    }
    response->success = true;
    response->message = "Home #" + std::to_string(id) +
      " queued; success means accepted, not motion completed. See /dual_arm/home_status";
    home_status("Home #" + std::to_string(id) + ": queued");
    queue_cv_.notify_one();
  }

  // Called by the queue worker, only after successful MoveIt execution.
  void publish_home_fingers(std::uint64_t id)
  {
    std_msgs::msg::Float32MultiArray fingers;
    fingers.data.reserve(home_finger_positions_.size());
    for (const double value : home_finger_positions_) {
      fingers.data.push_back(static_cast<float>(value));
    }
    const bool hand_subscribers_ready =
      home_left_fingers_pub_->get_subscription_count() > 0 &&
      home_right_fingers_pub_->get_subscription_count() > 0;
    if (!hand_subscribers_ready) {
      home_status("Home #" + std::to_string(id) +
        ": arm trajectory succeeded, but hand command subscribers missing");
      return;
    }
    home_left_fingers_pub_->publish(fingers);
    home_right_fingers_pub_->publish(fingers);
    home_status("Home #" + std::to_string(id) +
      ": arm trajectory succeeded; finger commands published (not feedback-verified)");
  }

  static const char* arm_name(Arm arm)
  {
    return arm == Arm::Left ? "left" : "right";
  }

  static bool valid_pose(const PoseStamped& goal)
  {
    if (goal.header.frame_id.empty()) return false;
    const auto& p = goal.pose.position;
    const auto& q = goal.pose.orientation;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(q.x) || !std::isfinite(q.y) ||
        !std::isfinite(q.z) || !std::isfinite(q.w)) return false;
    const double q_norm_squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    // Reject missing/non-unit orientations rather than sending unsafe targets.
    return std::abs(q_norm_squared - 1.0) < 0.02;
  }

  void enqueue(Arm arm, const PoseStamped& pose)
  {
    if (!valid_pose(pose)) {
      RCLCPP_ERROR(get_logger(), "Rejecting %s goal: invalid frame/position/quaternion",
        arm_name(arm));
      return;
    }

    std::size_t left_count = 0;
    std::size_t right_count = 0;
    std::uint64_t sequence = 0;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (stopping_) {
        RCLCPP_ERROR(get_logger(), "Commander halted; rejecting %s goal (restart after inspection)",
          arm_name(arm));
        return;
      }
      auto& queue = (arm == Arm::Left) ? left_queue_ : right_queue_;
      if (max_queue_per_arm_ > 0 && queue.size() >= static_cast<std::size_t>(max_queue_per_arm_)) {
        RCLCPP_ERROR(get_logger(), "%s queue full (%d); goal rejected",
          arm_name(arm), max_queue_per_arm_);
        return;
      }
      sequence = next_sequence_++;
      queue.push_back(Goal{pose, Clock::now(), sequence});
      left_count = left_queue_.size();
      right_count = right_queue_.size();
    }
    RCLCPP_INFO(get_logger(), "Queued %s goal #%llu (waiting L=%zu R=%zu)",
      arm_name(arm), static_cast<unsigned long long>(sequence), left_count, right_count);
    queue_cv_.notify_one();
  }

  void configure_motion(MoveGroup& group)
  {
    group.setMaxVelocityScalingFactor(velocity_scaling_);
    group.setMaxAccelerationScalingFactor(acceleration_scaling_);
    group.setPlanningTime(planning_time_);
    group.setNumPlanningAttempts(planning_attempts_);
  }

  void configure_arm(MoveGroup& group, const std::string& ee)
  {
    if (!group.setEndEffectorLink(ee)) {
      throw std::runtime_error("Invalid end-effector link for MoveIt group: " + ee);
    }
    configure_motion(group);
  }

  void validate_combined_group()
  {
    const auto model = dual_move_group_->getRobotModel();
    if (!model) throw std::runtime_error("MoveIt robot model unavailable");
    const auto* left = model->getJointModelGroup(left_group_);
    const auto* right = model->getJointModelGroup(right_group_);
    const auto* both = model->getJointModelGroup(dual_group_);
    if (!left || !right || !both) {
      throw std::runtime_error(
        "Missing left, right, or configured combined SRDF group. Check dual_group.");
    }
    const std::set<std::string> left_vars(
      left->getVariableNames().begin(), left->getVariableNames().end());
    const std::set<std::string> right_vars(
      right->getVariableNames().begin(), right->getVariableNames().end());
    const std::set<std::string> dual_vars(
      both->getVariableNames().begin(), both->getVariableNames().end());
    std::set<std::string> expected = left_vars;
    expected.insert(right_vars.begin(), right_vars.end());
    if (expected.size() != left_vars.size() + right_vars.size() || expected != dual_vars) {
      throw std::runtime_error(
        "Combined SRDF group must contain exactly the disjoint left+right joint variables");
    }
    RCLCPP_INFO(get_logger(), "Verified combined planning group (%zu joint variables)",
      dual_vars.size());
  }

  // Only this single worker executes trajectories. Planning is pipelined with
  // the short pairing window via std::async; ROS subscription callbacks never
  // block on planning/execution, and never drop a goal because MoveIt is busy.
  void worker_loop()
  {
    while (true) {
      std::uint64_t active_home_id = 0;
      try {
        Goal first;
        std::optional<Goal> second;
        Arm first_arm = Arm::Left;

        {
          std::unique_lock<std::mutex> lock(queue_mutex_);
          queue_cv_.wait(lock, [this]() {
            return stopping_ || !left_queue_.empty() || !right_queue_.empty();
          });
          if (stopping_) return;

          const bool has_left = !left_queue_.empty();
          const bool has_right = !right_queue_.empty();
          const bool compatible_pair = has_left && has_right &&
            ((left_queue_.front().home_id == 0 && right_queue_.front().home_id == 0) ||
             (left_queue_.front().home_id != 0 &&
              left_queue_.front().home_id == right_queue_.front().home_id));
          if (compatible_pair) {
            first_arm = Arm::Left;
            first = left_queue_.front();
            second = right_queue_.front();
            left_queue_.pop_front();
            right_queue_.pop_front();
          } else if (has_left && left_queue_.front().home_id == 0) {
            first_arm = Arm::Left;
            first = left_queue_.front();
            left_queue_.pop_front();
          } else if (has_right && right_queue_.front().home_id == 0) {
            first_arm = Arm::Right;
            first = right_queue_.front();
            right_queue_.pop_front();
          } else {
            // A home request enters both queues in one critical section.
            // Encountering an unmatched home at the front is a broken invariant.
            RCLCPP_FATAL(get_logger(), "Protected home queue pairing invariant broken");
            stopping_ = true;
            return;
          }
        }

        if (second) {
          active_home_id = first.home_id;
          RCLCPP_INFO(get_logger(), "Both goals already queued; plan combined immediately");
          const bool succeeded = run_plan_and_execute(Arm::Left, first, second);
          if (first.home_id != 0) {
            if (succeeded) {
              publish_home_fingers(first.home_id);
            } else {
              home_status("Home #" + std::to_string(first.home_id) +
                ": arm planning/execution failed or execute=false; fingers unchanged");
            }
          }
          continue;
        }

        // Begin provisional single-arm planning IMMEDIATELY (no fixed sleep).
        const auto deadline = Clock::now() +
          std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(pairing_window_sec_));
        RCLCPP_INFO(get_logger(), "%s #%llu: starting provisional plan; pairing window %.0f ms",
          arm_name(first_arm), static_cast<unsigned long long>(first.sequence),
          1000.0 * pairing_window_sec_);
        auto provisional = std::async(std::launch::async, [this, first_arm, first]() {
          return plan_single(first_arm, first.pose);
        });

        {
          std::unique_lock<std::mutex> lock(queue_mutex_);
          const auto counterpart_available = [this, first_arm, deadline]() {
            if (stopping_) return true;
            const auto& other = first_arm == Arm::Left ? right_queue_ : left_queue_;
            return !other.empty() && other.front().home_id == 0 &&
              other.front().received_at <= deadline;
          };
          queue_cv_.wait_until(lock, deadline, counterpart_available);
          if (!stopping_) {
            auto& other = first_arm == Arm::Left ? right_queue_ : left_queue_;
            if (!other.empty() && other.front().home_id == 0 &&
                other.front().received_at <= deadline) {
              second = other.front();
              other.pop_front();
            }
          }
        }

        // The provisional planner owns the single-arm MoveGroupInterface.
        // Join it before using those interfaces for combined IK. If a second
        // goal arrived, its result is NEVER published or executed.
        const PlanResult provisional_result = provisional.get();
        if (is_stopping()) return;

        if (second) {
          RCLCPP_INFO(get_logger(),
            "Opposite arm arrived during pairing window; discard provisional plan and replan 12 joints");
          if (first_arm == Arm::Left) {
            run_plan_and_execute(Arm::Left, first, second);
          } else {
            run_plan_and_execute(Arm::Right, first, second);
          }
        } else {
          finish_plan_and_execute(first_arm == Arm::Left ? "left" : "right",
            first_arm == Arm::Left ? *left_move_group_ : *right_move_group_,
            first_arm == Arm::Left ? left_plan_pub_ : right_plan_pub_,
            provisional_result);
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Queue worker exception: %s; halting for inspection", e.what());
        if (active_home_id != 0) {
          home_status("Home #" + std::to_string(active_home_id) +
            ": failed with exception; finger commands not sent");
        }
        {
          std::lock_guard<std::mutex> lock(queue_mutex_);
          stopping_ = true;
        }
        queue_cv_.notify_all();
        return;
      } catch (...) {
        RCLCPP_ERROR(get_logger(), "Unknown queue worker exception; halting for inspection");
        if (active_home_id != 0) {
          home_status("Home #" + std::to_string(active_home_id) +
            ": failed with exception; finger commands not sent");
        }
        {
          std::lock_guard<std::mutex> lock(queue_mutex_);
          stopping_ = true;
        }
        queue_cv_.notify_all();
        return;
      }
    }
  }

  bool is_stopping()
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return stopping_;
  }

  // If first_arm=Left, first is L and second is R; otherwise vice versa.
  bool run_plan_and_execute(Arm first_arm, const Goal& first, const std::optional<Goal>& second)
  {
    if (!second || is_stopping()) return false;
    const auto& left = first_arm == Arm::Left ? first.pose : second->pose;
    const auto& right = first_arm == Arm::Right ? first.pose : second->pose;
    RCLCPP_INFO(get_logger(), "Planning coordinated group %s (goals #%llu + #%llu)",
      dual_group_.c_str(),
      static_cast<unsigned long long>(first.sequence),
      static_cast<unsigned long long>(second->sequence));
    const auto result = plan_dual(left, right);
    if (!is_stopping()) {
      return finish_plan_and_execute("dual_arm", *dual_move_group_, dual_plan_pub_, result);
    }
    return false;
  }

  PlanResult plan_single(Arm arm, const PoseStamped& target)
  {
    PlanResult result;
    MoveGroup& group = arm == Arm::Left ? *left_move_group_ : *right_move_group_;
    const auto& ee = arm == Arm::Left ? left_ee_ : right_ee_;
    const char* label = arm_name(arm);

    auto state = group.getCurrentState(2.0);
    if (!state) {
      RCLCPP_ERROR(get_logger(), "%s: no current robot state", label);
      return result;
    }
    group.setStartState(*state);
    group.clearPoseTargets();
    if (!group.setJointValueTarget(*state) || !group.setJointValueTarget(target, ee)) {
      RCLCPP_ERROR(get_logger(), "%s: IK failed", label);
      return result;
    }
    const auto code = group.plan(result.plan);
    result.success = (code == moveit::core::MoveItErrorCode::SUCCESS);
    if (!result.success) {
      RCLCPP_ERROR(get_logger(), "%s: planning failed (%d)", label, code.val);
    } else {
      RCLCPP_INFO(get_logger(), "%s: provisional plan ready (%zu points)", label,
        result.plan.trajectory_.joint_trajectory.points.size());
    }
    return result;
  }

  PlanResult plan_dual(const PoseStamped& left_target, const PoseStamped& right_target)
  {
    PlanResult result;
    // Snapshot robot state NOW, after the previous trajectory completes.
    auto state = dual_move_group_->getCurrentState(2.0);
    if (!state) {
      RCLCPP_ERROR(get_logger(), "%s: no current robot state", dual_group_.c_str());
      return result;
    }
    const auto model = dual_move_group_->getRobotModel();
    const auto* left_jmg = model->getJointModelGroup(left_group_);
    const auto* right_jmg = model->getJointModelGroup(right_group_);

    // Each ordinary 6-DOF IK solver uses this *same* state snapshot as seed.
    left_move_group_->setStartState(*state);
    left_move_group_->clearPoseTargets();
    if (!left_move_group_->setJointValueTarget(*state) ||
        !left_move_group_->setJointValueTarget(left_target, left_ee_)) {
      RCLCPP_ERROR(get_logger(), "dual_arm: left IK failed; nothing executed");
      return result;
    }
    right_move_group_->setStartState(*state);
    right_move_group_->clearPoseTargets();
    if (!right_move_group_->setJointValueTarget(*state) ||
        !right_move_group_->setJointValueTarget(right_target, right_ee_)) {
      RCLCPP_ERROR(get_logger(), "dual_arm: right IK failed; nothing executed");
      return result;
    }

    moveit::core::RobotState target(*state);
    const auto& left_ik = left_move_group_->getJointValueTarget();
    const auto& right_ik = right_move_group_->getJointValueTarget();
    for (const auto& joint : left_jmg->getVariableNames()) {
      target.setVariablePosition(joint, left_ik.getVariablePosition(joint));
    }
    for (const auto& joint : right_jmg->getVariableNames()) {
      target.setVariablePosition(joint, right_ik.getVariablePosition(joint));
    }
    target.update();

    dual_move_group_->setStartState(*state);
    dual_move_group_->clearPoseTargets();
    if (!dual_move_group_->setJointValueTarget(target)) {
      RCLCPP_ERROR(get_logger(), "dual_arm: joint targets out of bounds");
      return result;
    }
    const auto code = dual_move_group_->plan(result.plan);
    result.success = (code == moveit::core::MoveItErrorCode::SUCCESS);
    if (!result.success) {
      RCLCPP_ERROR(get_logger(), "dual_arm: planning failed (%d); nothing executed", code.val);
    } else {
      RCLCPP_INFO(get_logger(), "dual_arm: planned %zu joint names, %zu points",
        result.plan.trajectory_.joint_trajectory.joint_names.size(),
        result.plan.trajectory_.joint_trajectory.points.size());
    }
    return result;
  }

  bool finish_plan_and_execute(
    const char* label, MoveGroup& group,
    const rclcpp::Publisher<RobotTrajectory>::SharedPtr& pub,
    const PlanResult& result)
  {
    if (!result.success) {
      RCLCPP_WARN(get_logger(), "%s: skipping failed request; moving to next queued goal", label);
      return false;
    }
    pub->publish(result.plan.trajectory_);
    if (!execute_) {
      RCLCPP_INFO(get_logger(), "%s: plan published; execute=false", label);
      return false;
    }
    RCLCPP_WARN(get_logger(), "%s: EXECUTING; subsequent goals stay queued", label);
    const auto code = group.execute(result.plan);  // Blocking for execution result.
    if (code == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(get_logger(), "%s: execution succeeded", label);
      return true;
    } else {
      RCLCPP_ERROR(get_logger(), "%s: execution failed (%d)", label, code.val);
      RCLCPP_WARN(get_logger(), "Execution failure: verify robot state and safety before next queued goal");
      // Do not blindly execute any additional queued goals after hardware error.
      // Remain stopped until node restart/operator inspection.
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        stopping_ = true;
      }
      queue_cv_.notify_all();
      return false;
    }
  }

  std::string left_group_;
  std::string right_group_;
  std::string dual_group_;
  std::string left_ee_;
  std::string right_ee_;
  double velocity_scaling_ = 0.10;
  double acceleration_scaling_ = 0.10;
  double planning_time_ = 5.0;
  double pairing_window_sec_ = 0.15;
  int planning_attempts_ = 5;
  int max_queue_per_arm_ = 0;
  bool execute_ = true;
  std::string home_frame_;
  std::vector<double> home_left_position_;
  std::vector<double> home_right_position_;
  std::vector<double> home_finger_positions_;

  std::unique_ptr<MoveGroup> left_move_group_;
  std::unique_ptr<MoveGroup> right_move_group_;
  std::unique_ptr<MoveGroup> dual_move_group_;
  rclcpp::Subscription<PoseStamped>::SharedPtr left_goal_sub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr right_goal_sub_;
  rclcpp::Publisher<RobotTrajectory>::SharedPtr left_plan_pub_;
  rclcpp::Publisher<RobotTrajectory>::SharedPtr right_plan_pub_;
  rclcpp::Publisher<RobotTrajectory>::SharedPtr dual_plan_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr home_left_fingers_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr home_right_fingers_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr home_status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_service_;

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<Goal> left_queue_;
  std::deque<Goal> right_queue_;
  std::uint64_t next_sequence_ = 1;
  std::uint64_t next_home_id_ = 1;
  bool stopping_ = false;
  std::thread worker_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<DualArmPoseCommander>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });
    try {
      node->initialize();
      spinner.join();
      node->stop();
    } catch (...) {
      node->stop();
      executor.cancel();
      spinner.join();
      throw;
    }
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("dual_arm_moveit_commander"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
