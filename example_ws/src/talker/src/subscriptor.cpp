#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <std_msgs/msg/string.hpp>

class MinimalSubscription : public rclcpp::Node {
public:
  MinimalSubscription() : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "alliance", 10,
        [this](const std_msgs::msg::String msg) { this->topic_callback(msg); });
  }

private:
  void topic_callback(const std_msgs::msg::String msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscription>());
  rclcpp::shutdown();
  return 0;
}