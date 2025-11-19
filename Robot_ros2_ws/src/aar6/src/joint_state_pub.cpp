#include <chrono>
#include <memory>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class JointStateWatchdog : public rclcpp::Node
{
public:
    JointStateWatchdog()
        : Node("joint_state_watchdog")
    {
        // QoS for reliable publishing/subscribing
        rclcpp::QoS qos(10);

        // Publisher
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos);

        // Subscriber to MCU joint states
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", qos,
            std::bind(&JointStateWatchdog::jointCallback, this, std::placeholders::_1));

        // Timer to check for missing updates (~1/40s)
        timer_ = this->create_wall_timer(
            25ms, std::bind(&JointStateWatchdog::timerCallback, this));

        last_received_ = this->now(); // initialize to current time

        RCLCPP_INFO(this->get_logger(), "JointStateWatchdog started.");
    }

private:
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_msg_ = *msg;
        last_received_ = this->now();
    }

    void timerCallback()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (last_msg_.name.empty())
            return; // no message received yet

        auto elapsed = this->now() - last_received_;
        if (elapsed.nanoseconds() > 25'000'000) // 25ms
        {
            // Re-publish last known joint state
            sensor_msgs::msg::JointState msg = last_msg_;
            msg.header.stamp = this->now();
            joint_pub_->publish(msg);

            // Optional: reset last_received_ to avoid flooding
            last_received_ = this->now();
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time last_received_;
    sensor_msgs::msg::JointState last_msg_;
    std::mutex mutex_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointStateWatchdog>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
