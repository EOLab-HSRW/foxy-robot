#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#define FOXY_IMPLEMENTATION
#include "foxy/foxy.h"

using namespace std::chrono_literals;

class ButtonState : public rclcpp::Node {
public:
  ButtonState() : Node("button_state") {

    const bool fail = this->declare_parameter<bool>("fail_in_constructor", true);
    const bool active_low = this->declare_parameter<bool>("active_low", true);

    robot_t robot = robot_init();

    if (!robot_ok(&robot) && fail) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
         "Fail to init robot description");
      throw std::runtime_error("Fail to init robot description");
    }

    button = gpio_init_name(&robot, "top_button");
    if (!button.ctx && fail) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
         "Fail to init button driver.");  
      throw std::runtime_error("Fail to init button");
    }

    gpio_set_as_input(button);
    (active_low) ? gpio_set_active_low(button) : gpio_set_active_high(button);

     rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1))
                 .transient_local()
                 .reliable();

    publisher_ =
        this->create_publisher<std_msgs::msg::Bool>("button_state", qos);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&ButtonState::timer_callback, this));


  }

private:
  void timer_callback() {
    auto message = std_msgs::msg::Bool();
    int v = gpio_read(button);
    message.data = (v == 1) ? true : false;
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
  gpio_t button;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ButtonState>());
  rclcpp::shutdown();
  return 0;
}

