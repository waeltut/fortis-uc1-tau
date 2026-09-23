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
#include <tf2_ros/static_transform_broadcaster.h>

//cartesian path
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <geometry_msgs/msg/pose_stamped.hpp>


const double table_size_x = 0.5;
const double table_size_y = 0.7;
const double table_size_z = 0.91;

const double table_corner_pos_x = 0.569;
const double table_corner_pos_y = -0.72;
const double table_corner_pos_z = 0;

const double table_center_pos_x = table_corner_pos_x + table_size_x / 2;
const double table_center_pos_y = table_corner_pos_y + table_size_y / 2;
const double table_center_pos_z = table_corner_pos_z + table_size_z / 2.0;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("table_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto const logger = rclcpp::get_logger("table_node");

  //rclcpp::executors::SingleThreadedExecutor exec;
  //exec.add_node(node);
  //std::thread spinner([&exec]() { exec.spin(); });

  // init movegroupinterface
  moveit::planning_interface::MoveGroupInterface movegroupinterface(node, "left_ur_manipulator");

  // init tf buffer and wait for fill
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf2_ros::TransformListener tf_listener(*tf_buffer);
  rclcpp::sleep_for(std::chrono::seconds(1));
  // --------------------------------------- init complete ---------------------------------------------

  moveit::planning_interface::PlanningSceneInterface planningsceneinterface;

  auto static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

  auto existing = planningsceneinterface.getObjects({"table"});
  if (existing.count("table") == 0) {
    const std::string planning_frame = movegroupinterface.getPlanningFrame();

    // define the table, dimensions, pose
    shape_msgs::msg::SolidPrimitive table;
    table.type = shape_msgs::msg::SolidPrimitive::BOX;
    table.dimensions = {table_size_x, table_size_y, table_size_z};
    geometry_msgs::msg::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = table_center_pos_x;
    table_pose.position.y = table_center_pos_y;
    table_pose.position.z = table_center_pos_z;

    // collisionobject of table
    moveit_msgs::msg::CollisionObject co;
    co.header.frame_id = planning_frame;
    co.id = "table";
    co.primitives.push_back(table);
    co.primitive_poses.push_back(table_pose);
    co.operation = co.ADD;

    // color for the table
    moveit_msgs::msg::ObjectColor color;
    color.id = co.id;
    color.color.r = 1.0f;
    color.color.g = 1.0f;
    color.color.b = 1.0f;
    color.color.a = 1.0f;

    moveit_msgs::msg::PlanningScene ps_msg;
    ps_msg.is_diff = true;
    ps_msg.world.collision_objects.push_back(co);
    ps_msg.object_colors.push_back(color);

    // Apply via PlanningSceneInterface
    planningsceneinterface.applyPlanningScene(ps_msg);

    RCLCPP_INFO(logger, "added table to the scene");

    ///////

    // building table_corner static transform
    geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp = node->get_clock()->now();
    ts.header.frame_id = planning_frame;
    ts.child_frame_id  = "table_corner";

    // its translation
    ts.transform.translation.x = table_corner_pos_x;
    ts.transform.translation.y = table_corner_pos_y;
    ts.transform.translation.z = table_corner_pos_z + table_size_z;
    
    // its rotation (identity)
    tf2::Quaternion q;
    q.setRPY(0,0,0);
    ts.transform.rotation.x = q.x();
    ts.transform.rotation.y = q.y();
    ts.transform.rotation.z = q.z();
    ts.transform.rotation.w = q.w();

    // send the transform
    static_broadcaster->sendTransform(ts);
    RCLCPP_INFO(logger, "published table_corner static transofrm");
  } else {
    RCLCPP_INFO(logger, "table already exists in the scene");
  }

  rclcpp::spin(node);
  //rclcpp::sleep_for(std::chrono::milliseconds(1500));
  //exec.cancel();
  //spinner.join();
  rclcpp::shutdown();
  return 0;
}