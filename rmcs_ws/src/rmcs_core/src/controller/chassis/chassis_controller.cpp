#include <limits>
#include <memory>
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/int32.hpp>

#include "rmcs_core/msgs.hpp"

namespace rmcs_core::controller::chassis {

class ChassisController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    enum class DecisionMode { INVINCIBLE = 0, BAD_HEALTH, HIDDEN, CRUISE };

    ChassisController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);

        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_);

        register_input("/referee/game_stage", game_stage_);

        register_output(
            "/chassis/left_front_wheel/control_velocity", left_front_control_velocity_, nan);
        register_output(
            "/chassis/left_back_wheel/control_velocity", left_back_control_velocity_, nan);
        register_output(
            "/chassis/right_back_wheel/control_velocity", right_back_control_velocity_, nan);
        register_output(
            "/chassis/right_front_wheel/control_velocity", right_front_control_velocity_, nan);

        decision_control_subscription_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "/sentry/control/forward/velocity", 10,
            [this](const geometry_msgs::msg::Pose2D::UniquePtr& msg) -> void {
                decision_control_velocity_.x() = msg->x;
                decision_control_velocity_.y() = msg->y;
            });

        decision_mode_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "/sentry/control/status", 10, [this](std::unique_ptr<std_msgs::msg::Int32> msg) {
                if (auto_mode_ && msg->data != static_cast<int>(DecisionMode::INVINCIBLE)) {
                    spinning_mode_ |= true;
                }
            });
    }

    void update() override {
        using namespace rmcs_core::msgs;

        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto keyboard     = *keyboard_;

        do {
            if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                reset_all_controls();
                break;
            }

            if (switch_left != Switch::DOWN) {
                auto new_spinning_mode = spinning_mode_;
                if (switch_right == Switch::MIDDLE) {
                    new_spinning_mode |= keyboard.c;
                    new_spinning_mode &= !(keyboard.ctrl && keyboard.c);
                } else if (last_switch_right_ == Switch::MIDDLE && switch_right == Switch::DOWN) {
                    new_spinning_mode = !new_spinning_mode;
                }
                if (spinning_mode_ == false && new_spinning_mode == true)
                    spinning_clockwise_ = !spinning_clockwise_;
                spinning_mode_ = new_spinning_mode;
            }

            auto move = Eigen::Vector2d{0, 0};

            auto_mode_ = *game_stage_ == GameStage::STARTED;
            auto_mode_ |= switch_left != Switch::DOWN && switch_right == Switch::UP;
            if (auto_mode_) {
                move = Eigen::Rotation2Dd{*gimbal_yaw_angle_}
                     * Eigen::Vector2d{
                         decision_control_velocity_.x(), decision_control_velocity_.y()};
            } else {
                auto keyboard_move = Eigen::Vector2d{
                    0.5 * (keyboard.w - keyboard.s), 0.5 * (keyboard.a - keyboard.d)};
                move = Eigen::Rotation2Dd{*gimbal_yaw_angle_} * (*joystick_right_ + keyboard_move);
            }

            update_wheel_velocities(move);

        } while (false);

        last_switch_right_ = switch_right;
        last_switch_left_  = switch_left;
    }

    void reset_all_controls() {
        spinning_mode_ = false;

        *left_front_control_velocity_  = nan;
        *left_back_control_velocity_   = nan;
        *right_back_control_velocity_  = nan;
        *right_front_control_velocity_ = nan;
    }

    void update_wheel_velocities(Eigen::Vector2d move) {
        constexpr double speed_limit = 80;

        if (move.norm() > 1) {
            move.normalize();
        }

        double right_oblique = speed_limit * (-move.y() * cos_45 + move.x() * sin_45);
        double left_oblique  = speed_limit * (move.x() * cos_45 + move.y() * sin_45);

        double velocities[4] = {right_oblique, left_oblique, -right_oblique, -left_oblique};

        if (spinning_mode_) {
            if (!spinning_clockwise_)
                for (auto& velocity : velocities)
                    velocity = -velocity;

            double max_speed = 0;
            for (auto& velocity : velocities) {
                max_speed = std::max(std::abs(velocity), max_speed);
            }

            auto spinning_speed = speed_limit - max_speed;
            if (spinning_speed < spinning_min_ * speed_limit) {
                spinning_speed = spinning_min_ * speed_limit;

                auto scale = (speed_limit - spinning_speed) / max_speed;
                for (auto& velocity : velocities) {
                    velocity *= scale;
                }
            } else if (spinning_speed > spinning_max_ * speed_limit) {
                spinning_speed = spinning_max_ * speed_limit;
            }

            for (auto& velocity : velocities) {
                velocity += spinning_speed;
            }

            if (!spinning_clockwise_)
                for (auto& velocity : velocities)
                    velocity = -velocity;
        }

        *left_front_control_velocity_  = velocities[0];
        *left_back_control_velocity_   = velocities[1];
        *right_back_control_velocity_  = velocities[2];
        *right_front_control_velocity_ = velocities[3];
    }

private:
    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    // Since sine and cosine function are not constexpr, we calculate once and cache them.
    static inline const double sin_45 = std::sin(std::numbers::pi / 4.0);
    static inline const double cos_45 = std::cos(std::numbers::pi / 4.0);

    // Velocity scale in spinning mode
    static inline const double spinning_max_ = 0.4;
    static inline const double spinning_min_ = 0.2;

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_core::msgs::Switch> switch_right_;
    InputInterface<rmcs_core::msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_core::msgs::Mouse> mouse_;
    InputInterface<rmcs_core::msgs::Keyboard> keyboard_;

    InputInterface<double> gimbal_yaw_angle_;

    rmcs_core::msgs::Switch last_switch_right_ = rmcs_core::msgs::Switch::UNKNOWN;
    rmcs_core::msgs::Switch last_switch_left_  = rmcs_core::msgs::Switch::UNKNOWN;

    bool spinning_mode_ = false, spinning_clockwise_ = false;

    InputInterface<rmcs_core::msgs::GameStage> game_stage_;
    bool auto_mode_ = false;

    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int32>> decision_mode_subscription_;

    OutputInterface<double> left_front_control_velocity_;
    OutputInterface<double> left_back_control_velocity_;
    OutputInterface<double> right_back_control_velocity_;
    OutputInterface<double> right_front_control_velocity_;

    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Pose2D>>
        decision_control_subscription_;
    Eigen::Vector2d decision_control_velocity_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::ChassisController, rmcs_executor::Component)