#include <memory>

#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <std_msgs/msg/int32.hpp>

#include "hardware/device/dji_motor.hpp"
#include "hardware/forwarder/cboard.hpp"

namespace rmcs_core::hardware {

class SentryBelow : public rmcs_executor::Component,
                    public rclcpp::Node,
                    private forwarder::CBoard {
public:
  SentryBelow()
      : Node{get_component_name(),
             rclcpp::NodeOptions{}
                 .automatically_declare_parameters_from_overrides(true)},
        forwarder::CBoard{get_logger(), 0xeb37, 0xa11c}, logger_(get_logger()),
        sentry_below_command_(create_partner_component<SentryBelowCommand>(
            get_component_name() + "_command", *this)),
        transmit_buffer_(*this, 16) {
    using namespace device;

    for (auto &motor : chassis_wheel_motors_)
      motor.configure(DjiMotorConfig{DjiMotorType::M3508}
                          .reverse()
                          .set_reduction_ratio(13.)
                          .enable_multi_turn_angle());

    chassis_steering_motors_[0].configure(
        DjiMotorConfig{DjiMotorType::GM6020}.set_encoder_zero_point(
            static_cast<int>(
                get_parameter("left_front_motor_zero_point").as_int())));
    chassis_steering_motors_[1].configure(
        DjiMotorConfig{DjiMotorType::GM6020}.set_encoder_zero_point(
            static_cast<int>(
                get_parameter("left_back_motor_zero_point").as_int())));
    chassis_steering_motors_[2].configure(
        DjiMotorConfig{DjiMotorType::GM6020}.set_encoder_zero_point(
            static_cast<int>(
                get_parameter("right_back_motor_zero_point").as_int())));
    chassis_steering_motors_[3].configure(
        DjiMotorConfig{DjiMotorType::GM6020}.set_encoder_zero_point(
            static_cast<int>(
                get_parameter("right_front_motor_zero_point").as_int())));

    gimbal_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
        "/gimbal/calibrate", rclcpp::QoS{0},
        [this](std_msgs::msg::Int32::UniquePtr &&msg) {
          gimbal_calibrate_subscription_callback(std::move(msg));
        });

    gimbal_yaw_motor_.configure(
        DjiMotorConfig{DjiMotorType::GM6020}.set_encoder_zero_point(
            static_cast<int>(get_parameter("yaw_motor_zero_point").as_int())));

    using namespace rmcs_description;

    tf_->set_transform<PitchLink, ImuLink>(
        Eigen::AngleAxisd{std::numbers::pi / 2, Eigen::Vector3d::UnitZ()});

    register_output("/referee/serial", referee_serial_);
    referee_serial_->read = [this](std::byte *buffer, size_t size) {
      return referee_ring_buffer_receive_.pop_front_multi(
          [&buffer](std::byte byte) { *buffer++ = byte; }, size);
    };
    referee_serial_->write = [this](const std::byte *buffer, size_t size) {
      while (uint8_t transmit_length =
                 size > 15ul ? (uint8_t)15 : (uint8_t)size) {
        if (!transmit_buffer_.add_uart1_transmission(buffer, transmit_length))
          break;
        buffer += transmit_length;
        size -= transmit_length;
      }
      return size; // TODO
    };
  }

  void update() override { update_motors(); }

  void command_update() {
    uint16_t can_commands[4];

    can_commands[0] = chassis_wheel_motors_[0].generate_command();
    can_commands[1] = chassis_wheel_motors_[1].generate_command();
    can_commands[2] = chassis_wheel_motors_[2].generate_command();
    can_commands[3] = chassis_wheel_motors_[3].generate_command();
    transmit_buffer_.add_can1_transmission(
        0x200, std::bit_cast<uint64_t>(can_commands));

    can_commands[0] = chassis_steering_motors_[0].generate_command();
    can_commands[1] = chassis_steering_motors_[1].generate_command();
    can_commands[2] = chassis_steering_motors_[2].generate_command();
    can_commands[3] = chassis_steering_motors_[3].generate_command();
    transmit_buffer_.add_can2_transmission(
        0x1FE, std::bit_cast<uint64_t>(can_commands));

    can_commands[0] = gimbal_yaw_motor_.generate_command();
    can_commands[1] = 0;
    can_commands[2] = 0;
    can_commands[3] = 0;
    transmit_buffer_.add_can1_transmission(
        0x1FE, std::bit_cast<uint64_t>(can_commands));

    transmit_buffer_.trigger_transmission();
  }

private:
  void update_motors() {
    using namespace rmcs_description;
    for (auto &motor : chassis_wheel_motors_)
      motor.update();

    for (auto &motor : chassis_steering_motors_)
      motor.update();

    gimbal_yaw_motor_.update();
  }

protected:
  void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
    RCLCPP_INFO(logger_, "[gimbal calibration] New yaw offset: %d",
                gimbal_yaw_motor_.calibrate_zero_point());
  }
  void can1_receive_callback(uint32_t can_id, uint64_t can_data,
                             bool is_extended_can_id,
                             bool is_remote_transmission,
                             uint8_t can_data_length) override {
    if (is_extended_can_id || is_remote_transmission || can_data_length < 8)
        [[unlikely]]
      return;

    if (can_id == 0x201) {
      auto &motor = chassis_wheel_motors_[0];
      motor.store_status(can_data);
    } else if (can_id == 0x202) {
      auto &motor = chassis_wheel_motors_[1];
      motor.store_status(can_data);

    } else if (can_id == 0x203) {
      auto &motor = chassis_wheel_motors_[2];
      motor.store_status(can_data);

    } else if (can_id == 0x204) {
      auto &motor = chassis_wheel_motors_[3];
      motor.store_status(can_data);

    } else if (can_id == 0x205) {
      auto &motor = gimbal_yaw_motor_;
      motor.store_status(can_data);
    }
  }

  void can2_receive_callback(uint32_t can_id, uint64_t can_data,
                             bool is_extended_can_id,
                             bool is_remote_transmission,
                             uint8_t can_data_length) override {
    if (is_extended_can_id || is_remote_transmission || can_data_length < 8)
        [[unlikely]]
      return;

    // RCLCPP_INFO(logger_, "id= %d",can_id);
    if (can_id == 0x205) {
      auto &motor = chassis_steering_motors_[0];
      motor.store_status(can_data);

    } else if (can_id == 0x206) {
      auto &motor = chassis_steering_motors_[1];
      motor.store_status(can_data);

    } else if (can_id == 0x207) {
      auto &motor = chassis_steering_motors_[2];
      motor.store_status(can_data);

    } else if (can_id == 0x208) {
      auto &motor = chassis_steering_motors_[3];
      motor.store_status(can_data);
    }
  }

  void uart1_receive_callback(const std::byte *uart_data,
                              uint8_t uart_data_length) override {
    referee_ring_buffer_receive_.emplace_back_multi(
        [&uart_data](std::byte *storage) { *storage = *uart_data++; },
        uart_data_length);
  }

private:
  rclcpp::Logger logger_;

  class SentryBelowCommand : public rmcs_executor::Component {
  public:
    explicit SentryBelowCommand(SentryBelow &sentry_below)
        : sentry_below_(sentry_below) {}

    void update() override { sentry_below_.command_update(); }

    SentryBelow &sentry_below_;
  };
  std::shared_ptr<SentryBelowCommand> sentry_below_command_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr
      gimbal_calibrate_subscription_;

  device::DjiMotor gimbal_yaw_motor_{*this, *sentry_below_command_,
                                     "/gimbal/yaw"};

  device::DjiMotor chassis_wheel_motors_[4] = {
      {*this, *sentry_below_command_, "/chassis/left_front_wheel"},
      {*this, *sentry_below_command_, "/chassis/left_back_wheel"},
      {*this, *sentry_below_command_, "/chassis/right_back_wheel"},
      {*this, *sentry_below_command_, "/chassis/right_front_wheel"}};
  device::DjiMotor chassis_steering_motors_[4] = {
      {*this, *sentry_below_command_, "/chassis/left_front_steering"},
      {*this, *sentry_below_command_, "/chassis/left_back_steering"},
      {*this, *sentry_below_command_, "/chassis/right_back_steering"},
      {*this, *sentry_below_command_, "/chassis/right_front_steering"}};

  OutputInterface<rmcs_description::Tf> tf_;

  RingBuffer<std::byte> referee_ring_buffer_receive_{256};
  OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

  forwarder::CBoard::TransmitBuffer transmit_buffer_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::SentryBelow,
                       rmcs_executor::Component)