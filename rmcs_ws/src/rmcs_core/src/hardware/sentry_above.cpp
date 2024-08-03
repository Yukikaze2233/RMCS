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
#include "hardware/forwarder/cboard.hpp"

namespace rmcs_core::hardware {

class SentryAbove : public rmcs_executor::Component,
                    public rclcpp::Node,
                    private forwarder::CBoard {
public:
  SentryAbove()
      : Node{get_component_name(),
             rclcpp::NodeOptions{}
                 .automatically_declare_parameters_from_overrides(true)},
        forwarder::CBoard{get_logger(), 0x93ac, 0xa11c}, logger_(get_logger()),
        sentry_above_command_(create_partner_component<SentryAboveCommand>(
            get_component_name() + "_command", *this)),
        transmit_buffer_(*this, 16) {
    using namespace device;

    gimbal_pitch_motor_.configure(
        DjiMotorConfig{DjiMotorType::GM6020}.set_encoder_zero_point(
            static_cast<int>(
                get_parameter("pitch_motor_zero_point").as_int())));

    gimbal_left_friction_.configure(
        DjiMotorConfig{DjiMotorType::M3508}.set_reduction_ratio(1.));
    gimbal_right_friction_.configure(
        DjiMotorConfig{DjiMotorType::M3508}.reverse().set_reduction_ratio(1.));
    gimbal_bullet_feeder_.configure(
        DjiMotorConfig{DjiMotorType::M2006}.enable_multi_turn_angle());

    register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);
    register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_imu_);
    register_output("/tf", tf_);

    using namespace rmcs_description;

    tf_->set_transform<PitchLink, ImuLink>(
        Eigen::AngleAxisd{std::numbers::pi / 2, Eigen::Vector3d::UnitZ()});

    constexpr double gimbal_center_height = 0.32059;

    tf_->set_transform<BaseLink, GimbalCenterLink>(
        Eigen::Translation3d{0, 0, gimbal_center_height});

    gimbal_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
        "/gimbal/calibrate", rclcpp::QoS{0},
        [this](std_msgs::msg::Int32::UniquePtr &&msg) {
          gimbal_calibrate_subscription_callback(std::move(msg));
        });
  }

  void update() override {
    update_motors();
    update_imu();
    dr16_.update();
  }

  void command_update() {
    uint16_t can_commands[4];

    can_commands[0] = 0;
    can_commands[1] = gimbal_bullet_feeder_.generate_command();
    can_commands[2] = gimbal_left_friction_.generate_command();
    can_commands[3] = gimbal_right_friction_.generate_command();
    transmit_buffer_.add_can2_transmission(
        0x200, std::bit_cast<uint64_t>(can_commands));

    can_commands[0] = 0;
    can_commands[1] = gimbal_pitch_motor_.generate_command();
    can_commands[2] = 0;
    can_commands[3] = 0;
    transmit_buffer_.add_can2_transmission(
        0x1FE, std::bit_cast<uint64_t>(can_commands));

    transmit_buffer_.trigger_transmission();
  }

private:
  void update_motors() {
    using namespace rmcs_description;

    gimbal_pitch_motor_.update();
    tf_->set_state<YawLink, PitchLink>(gimbal_pitch_motor_.get_angle());

    gimbal_bullet_feeder_.update();
    gimbal_left_friction_.update();
    gimbal_right_friction_.update();
  }

  void update_imu() {
    auto acc = accelerometer_data_.load(std::memory_order::relaxed);
    auto gyro = gyroscope_data_.load(std::memory_order::relaxed);

    auto solve_acc = [](int16_t value) { return value / 32767.0 * 6.0; };
    auto solve_gyro = [](int16_t value) {
      return value / 32767.0 * 2000.0 / 180.0 * std::numbers::pi;
    };

    double gx = solve_gyro(gyro.x), gy = solve_gyro(gyro.y),
           gz = solve_gyro(gyro.z);
    double ax = solve_acc(acc.x), ay = solve_acc(acc.y), az = solve_acc(acc.z);

    *gimbal_yaw_velocity_imu_ = gz;
    *gimbal_pitch_velocity_imu_ = gx;

    auto gimbal_imu_pose = imu_.update(ax, ay, az, gx, gy, gz);
    tf_->set_transform<rmcs_description::ImuLink, rmcs_description::OdomImu>(
        gimbal_imu_pose.conjugate());
  }

  void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
    RCLCPP_INFO(logger_, "[gimbal calibration] New pitch offset: %d",
                gimbal_pitch_motor_.calibrate_zero_point());
  }

protected:
  void can1_receive_callback(uint32_t, uint64_t, bool is_extended_can_id,
                             bool is_remote_transmission,
                             uint8_t can_data_length) override {
    if (is_extended_can_id || is_remote_transmission || can_data_length < 8)
        [[unlikely]]
      return;
  }

  void can2_receive_callback(uint32_t can_id, uint64_t can_data,
                             bool is_extended_can_id,
                             bool is_remote_transmission,
                             uint8_t can_data_length) override {
    if (is_extended_can_id || is_remote_transmission || can_data_length < 8)
        [[unlikely]]
      return;
    if (can_id == 0x202) {
      gimbal_bullet_feeder_.store_status(can_data);
    } else if (can_id == 0x203) {
      gimbal_left_friction_.store_status(can_data);
    } else if (can_id == 0x204) {
      gimbal_right_friction_.store_status(can_data);
    } else if (can_id == 0x206) {
      auto &motor = gimbal_pitch_motor_;
      motor.store_status(can_data);
    }
  }

  void dbus_receive_callback(const std::byte *uart_data,
                             uint8_t uart_data_length) override {
    dr16_.store_status(uart_data, uart_data_length);
  }

  void accelerometer_receive_callback(int16_t x, int16_t y,
                                      int16_t z) override {
    accelerometer_data_.store({x, y, z}, std::memory_order::relaxed);
  }

  void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
    gyroscope_data_.store({x, y, z}, std::memory_order::relaxed);
  }

private:
  rclcpp::Logger logger_;

  class SentryAboveCommand : public rmcs_executor::Component {
  public:
    explicit SentryAboveCommand(SentryAbove &sentry_above)
        : sentry_above_(sentry_above) {}

    void update() override { sentry_above_.command_update(); }

    SentryAbove &sentry_above_;
  };
  std::shared_ptr<SentryAboveCommand> sentry_above_command_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr
      gimbal_calibrate_subscription_;

  device::DjiMotor gimbal_pitch_motor_{*this, *sentry_above_command_,
                                       "/gimbal/pitch"};

  device::DjiMotor gimbal_left_friction_{*this, *sentry_above_command_,
                                         "/gimbal/left_friction"};
  device::DjiMotor gimbal_right_friction_{*this, *sentry_above_command_,
                                          "/gimbal/right_friction"};
  device::DjiMotor gimbal_bullet_feeder_{*this, *sentry_above_command_,
                                         "/gimbal/bullet_feeder"};

  device::Dr16 dr16_{*this};

  struct alignas(8) ImuData {
    int16_t x, y, z;
  };
  std::atomic<ImuData> accelerometer_data_, gyroscope_data_;
  static_assert(std::atomic<ImuData>::is_always_lock_free);
  device::Imu imu_;
  // TODO: double imu_gx_bias_, imu_gy_bias_, imu_gz_bias_;
  OutputInterface<double> gimbal_yaw_velocity_imu_;
  OutputInterface<double> gimbal_pitch_velocity_imu_;

  OutputInterface<rmcs_description::Tf> tf_;

  forwarder::CBoard::TransmitBuffer transmit_buffer_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::SentryAbove,
                       rmcs_executor::Component)