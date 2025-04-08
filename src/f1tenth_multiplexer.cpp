#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <numbers>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace ackermann_msgs::msg;
using namespace std_msgs::msg;

class F1TenthMultiplexer : public rclcpp::Node {
  rclcpp::Subscription<AckermannDriveStamped>::SharedPtr
      m_joystick_ackermann_sub;
  rclcpp::Subscription<AckermannDriveStamped>::SharedPtr
      m_gap_follow_ackermann_sub;

  rclcpp::Subscription<Bool>::SharedPtr m_gap_follow_enable_sub;
  rclcpp::Time m_gap_follow_enabled_time;
  bool m_gap_follow_enabled = false;

  rclcpp::Publisher<AckermannDriveStamped>::SharedPtr m_drive_pub;

  rclcpp::TimerBase::SharedPtr m_timer;

  std::optional<AckermannDriveStamped> m_joystick_ackermann_msg;
  std::optional<AckermannDriveStamped> m_gap_follow_ackermann_msg;

 public:
  F1TenthMultiplexer() : Node("f1tenth_multiplexer") {
    RCLCPP_INFO(this->get_logger(), "Initialized, starting now");

    m_joystick_ackermann_sub = this->create_subscription<AckermannDriveStamped>(
        "joystick_ackermann_cmd", 10,
        std::bind(&F1TenthMultiplexer::joystick_callback, this, _1));

    m_gap_follow_ackermann_sub =
        this->create_subscription<AckermannDriveStamped>(
            "gap_follow_ackermann_cmd", 10,
            std::bind(&F1TenthMultiplexer::gap_follow_callback, this, _1));

    m_gap_follow_enable_sub = this->create_subscription<Bool>(
        "gap_follow_enable", 10,
        [this](const Bool& msg) {
          m_gap_follow_enabled_time = this->get_clock()->now();
          m_gap_follow_enabled = msg.data;
        });

    m_drive_pub =
        this->create_publisher<AckermannDriveStamped>("ackermann_cmd", 10);

    m_timer = this->create_wall_timer(
        20ms, std::bind(&F1TenthMultiplexer::timer_callback, this));
  }

 private:
  void joystick_callback(const AckermannDriveStamped& msg) {
    RCLCPP_INFO(this->get_logger(), "Joystick callback");
    m_joystick_ackermann_msg = msg;
  }

  void gap_follow_callback(const AckermannDriveStamped& msg) {
    RCLCPP_INFO(this->get_logger(), "Gap follow callback");
    m_gap_follow_ackermann_msg = msg;
  }

  void validate_ackermann_msg(std::optional<AckermannDriveStamped>& msg) {
    if (!msg.has_value()) {
      return;
    }

    auto current_time = this->get_clock()->now();
    auto time_diff = current_time - msg->header.stamp;

    if (time_diff > 1s) {
      msg = {};
    }
  }

  void timer_callback() {
    AckermannDrive output;
    output.steering_angle = 0.0;
    output.speed = 0.0;

    auto current_time = this->get_clock()->now();

    // Make sure gap follow enable message is refreshed at least every second.
    if (m_gap_follow_enabled) {
      auto time_diff = current_time - m_gap_follow_enabled_time;
      if (time_diff > 1s) {
        m_gap_follow_enabled = false;
      }
    }

    // Erase old messages.
    validate_ackermann_msg(m_joystick_ackermann_msg);
    validate_ackermann_msg(m_gap_follow_ackermann_msg);

    // First use joystick control.
    if (m_joystick_ackermann_msg.has_value()) {
      output = m_joystick_ackermann_msg.value().drive;
      goto PUBLISH_OUTPUT;
    }

    if (m_gap_follow_ackermann_msg.has_value() && m_gap_follow_enabled) {
      output = m_gap_follow_ackermann_msg.value().drive;
      goto PUBLISH_OUTPUT;
    }

  PUBLISH_OUTPUT:
    AckermannDriveStamped output_msg;
    output_msg.drive = output;
    output_msg.header.stamp = current_time;
    m_drive_pub->publish(output_msg);
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<F1TenthMultiplexer>());
  rclcpp::shutdown();
  return 0;
}

