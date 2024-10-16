#include <memory>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <std_msgs/msg/int32.hpp>

#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/imu.hpp"
#include "hardware/endian_promise.hpp"
#include "hardware/forwarder/cboard.hpp"

namespace rmcs_core::hardware {

class Sentry : public rmcs_executor::Component,
               public rclcpp::Node,
               private forwarder::CBoard {
public:
  Sentry()
      : Node{get_component_name(),
             rclcpp::NodeOptions{}
                 .automatically_declare_parameters_from_overrides(true)},
        forwarder::CBoard{get_logger(), 0x93ac, 0xa11c}, logger_(get_logger()),
        sentry_above_command_(create_partner_component<SentryCommand>(
            get_component_name() + "_command", *this)),
        transmit_buffer_(*this, 16) {
    RCLCPP_INFO(logger_, "%s", get_parameter("data").as_string().c_str());
  }

  void update() override {
    // do recieve data
  }

  void command_update() {
    uint16_t can_commands[4];
    be_uint16_t I = 100;
    can_commands[0] = I;
    can_commands[1] = 0;
    can_commands[2] = 0;
    can_commands[3] = 0;
    transmit_buffer_.add_can2_transmission(
        0x200, std::bit_cast<uint64_t>(can_commands));

    transmit_buffer_.trigger_transmission();
  }

private:
protected:
  void can1_receive_callback(uint32_t, uint64_t, bool, bool, uint8_t) override {
    
  }

  void can2_receive_callback(uint32_t, uint64_t, bool, bool, uint8_t) override {
  }

  void dbus_receive_callback(const std::byte *, uint8_t) override {}

  void accelerometer_receive_callback(int16_t, int16_t, int16_t) override {}

private:
  rclcpp::Logger logger_;

  class SentryCommand : public rmcs_executor::Component {
  public:
    explicit SentryCommand(Sentry &sentry_above)
        : sentry_above_(sentry_above) {}

    void update() override { sentry_above_.command_update(); }

    Sentry &sentry_above_;
  };
  std::shared_ptr<SentryCommand> sentry_above_command_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr
      gimbal_calibrate_subscription_;

  // device::DjiMotor gimbal_pitch_motor_{*this, *sentry_above_command_,
  //  "/gimbal/pitch"};

  forwarder::CBoard::TransmitBuffer transmit_buffer_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Sentry, rmcs_executor::Component)