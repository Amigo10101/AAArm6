#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class CartesianControlNode : public rclcpp::Node
{
public:
  CartesianControlNode() : Node("cartesian_control_aar6")
  {
    // Subscribe to Twist commands
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cartesian_cmd_vel", 10, std::bind(&CartesianControlNode::cmdCallback, this, std::placeholders::_1));
    
    // Initialize MoveGroup in a separate thread/init function if possible, 
    // but here we will do it lazily or use a shared pointer.
    // Note: MoveGroupInterface requires a node to spin. We are inside the node.
    // We can't pass 'this' directly if we are spinning 'this'. 
    // Usually we pass a separate node or use the shared_from_this() pattern carefully.
  }

  void initMoveGroup()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "kin_group");
    
    RCLCPP_INFO(this->get_logger(), "Reference frame: %s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());
  }

private:
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!move_group_) return;

    geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();
    geometry_msgs::msg::Pose target_pose = current_pose.pose;
    
    double step_dist = 0.02; // 2cm
    double step_angle = 0.1; // ~5.7 degrees
    bool update = false;

    // Apply linear changes
    if (msg->linear.x != 0) { target_pose.position.x += msg->linear.x * step_dist; update = true; }
    if (msg->linear.y != 0) { target_pose.position.y += msg->linear.y * step_dist; update = true; }
    if (msg->linear.z != 0) { target_pose.position.z += msg->linear.z * step_dist; update = true; }

    // Apply angular changes
    if (msg->angular.x != 0 || msg->angular.y != 0 || msg->angular.z != 0) {
        tf2::Quaternion q_orig, q_rot, q_new;
        tf2::fromMsg(target_pose.orientation, q_orig);
        q_rot.setRPY(msg->angular.x * step_angle, msg->angular.y * step_angle, msg->angular.z * step_angle);
        q_new = q_orig * q_rot;
        q_new.normalize();
        target_pose.orientation = tf2::toMsg(q_new);
        update = true;
    }

    if (update) {
      RCLCPP_INFO(this->get_logger(), "\nTarget Pose -> Pos: [%.3f, %.3f, %.3f],\n Orient: [%.3f, %.3f, %.3f, %.3f]", 
        target_pose.position.x, target_pose.position.y, target_pose.position.z,
        target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);
      
      // Use standard OMPL planning (PTP)
      // This plans a collision-free path to the target, but not necessarily a straight line.
      move_group_->setPlanningPipelineId("ompl");
      move_group_->setPlannerId("RRTConnectkConfigDefault");
      move_group_->setPoseTarget(target_pose);
      
      // Reset scaling to default or safe values
      move_group_->setMaxVelocityScalingFactor(0.5);
      move_group_->setMaxAccelerationScalingFactor(0.5);

      // Plan and execute
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (success) {
           RCLCPP_INFO(this->get_logger(), "Planning successful, executing...");
           move_group_->execute(my_plan);
      } else {
           RCLCPP_ERROR(this->get_logger(), "Planning failed!");
      }
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CartesianControlNode>();
  
  // We need to spin the node so MoveGroup can work, but we also need to initialize MoveGroup.
  // MoveGroupInterface needs the node to be spinning to get robot state.
  // So we spin in a background thread.
  
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  node->initMoveGroup();

  // Keep main thread alive
  while (rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();
  return 0;
}
