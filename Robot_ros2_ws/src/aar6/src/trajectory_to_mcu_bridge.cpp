#include "rclcpp/rclcpp.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class TrajectoryToMCUBridge : public rclcpp::Node
{
public:
    TrajectoryToMCUBridge() : Node("trajectory_to_mcu_bridge"), num_joints_(6), trajectory_active_(false)
    {
        // Subscribe to action feedback (published during trajectory execution)
        subscription_ = this->create_subscription<control_msgs::action::FollowJointTrajectory::Impl::FeedbackMessage>(
            "/kin_group_controller/follow_joint_trajectory/_action/feedback",
            10,
            std::bind(&TrajectoryToMCUBridge::feedback_callback, this, std::placeholders::_1));

        // Publisher for MCU commands (Float64MultiArray)
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/joint_commands",
            10);

        // Timer to check if trajectory has ended (checks every 100ms)
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TrajectoryToMCUBridge::watchdog_callback, this));

        RCLCPP_INFO(this->get_logger(), "Trajectory to MCU bridge started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /kin_group_controller/follow_joint_trajectory/_action/feedback");
        RCLCPP_INFO(this->get_logger(), "Publishing to: /joint_commands (Float64MultiArray)");
    }

private:
    void feedback_callback(const control_msgs::action::FollowJointTrajectory::Impl::FeedbackMessage::SharedPtr msg)
    {
        // Create Float64MultiArray message for MCU
        auto joint_cmd = std_msgs::msg::Float64MultiArray();
        
        // Pack desired velocities into the array
        joint_cmd.data = msg->feedback.desired.velocities;
        
        // Update the number of joints from the message
        if (!joint_cmd.data.empty()) {
            num_joints_ = joint_cmd.data.size();
        }

        // Publish to MCU
        publisher_->publish(joint_cmd);
        
        // Update last message time and set trajectory as active
        last_message_time_ = this->now();
        trajectory_active_ = true;
    }

    void watchdog_callback()
    {
        // If we haven't received a message in the last 200ms and trajectory was active
        if (trajectory_active_) {
            auto time_since_last_msg = this->now() - last_message_time_;
            if (time_since_last_msg.seconds() > 0.2) {
                // Trajectory has ended, publish zeros
                auto zero_cmd = std_msgs::msg::Float64MultiArray();
                zero_cmd.data.resize(num_joints_, 0.0);
                publisher_->publish(zero_cmd);
                
                RCLCPP_INFO(this->get_logger(), "Trajectory ended - publishing zero velocities");
                trajectory_active_ = false;
            }
        }
    }

    rclcpp::Subscription<control_msgs::action::FollowJointTrajectory::Impl::FeedbackMessage>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::Time last_message_time_;
    size_t num_joints_;
    bool trajectory_active_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryToMCUBridge>());
    rclcpp::shutdown();
    return 0;
}
