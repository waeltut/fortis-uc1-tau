#include <memory>
#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class RequestAssistance : public BT::SyncActionNode
{
public:
  RequestAssistance(const std::string & name, const BT::NodeConfiguration & config) : BT::SyncActionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    publisher_ = node_->create_publisher<std_msgs::msg::String>("/professional_namespace/assistance_required", 10);
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    std_msgs::msg::String msg;
    msg.data = "Assistance Required";
    publisher_->publish(msg);

    RCLCPP_INFO(node_->get_logger(), "request");
    return BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<RequestAssistance>("RequestAssistance");
}