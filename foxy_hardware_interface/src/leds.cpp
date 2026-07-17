#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"

#define FOXY_IMPLEMENTATION
#include "foxy/foxy.h"

class LedController : public rclcpp::Node
{
public:
  LedController()
  : Node("leds"), robot_(robot_init())
  {
    fail_in_constructor_ =
      declare_parameter<bool>("fail_in_constructor", true);

    for (std::size_t i = 0; i < led_names_.size(); ++i) {
      const std::string parameter_name =
        "led_indices." + led_names_[i];

      const int configured_index = declare_parameter<int>(
        parameter_name,
        static_cast<int>(default_led_indices_[i]));

      if (configured_index < 0 ||
        configured_index >= static_cast<int>(led_names_.size()))
      {
        throw std::invalid_argument(
                parameter_name + " must be in the range [0, 4]");
      }

      led_indices_[i] =
        static_cast<std::uint8_t>(configured_index);
    }

    validate_unique_led_indices();

    for (auto & state : states_) {
      state = percentage_state(0.0F, 0.0F, 0.0F);
    }

    initialize_hardware();
    create_ros_interfaces();
    publish_all_states();
  }

  ~LedController() override
  {
    std::lock_guard<std::mutex> lock(hardware_mutex_);

    if (leds_.ctx != nullptr) {
      if (hardware_available_) {
        for (const auto index : led_indices_) {
          (void)led_set_rgb(leds_, index, 0, 0, 0);
        }
      }

      led_deinit(&robot_, &leds_);
      hardware_available_ = false;
    }

    robot_deinit(&robot_);
  }

private:
  using Color = std_msgs::msg::ColorRGBA;
  using Publisher = rclcpp::Publisher<Color>;
  using Subscription = rclcpp::Subscription<Color>;

  static constexpr float minimum_percentage_ = 0.0F;
  static constexpr float maximum_percentage_ = 100.0F;

  static float clamp_percentage(float value)
  {
    if (!std::isfinite(value)) {
      return minimum_percentage_;
    }

    return std::clamp(
      value,
      minimum_percentage_,
      maximum_percentage_);
  }

  /*
   * Convert the ROS-facing percentage into the native value
   * expected by the Foxy LED API.
   *
   * The native representation remains private to this node.
   */
  static std::uint8_t percent_to_driver_value(float percentage)
  {
    constexpr float driver_maximum =
      static_cast<float>(
      std::numeric_limits<std::uint8_t>::max());

    const float normalized =
      clamp_percentage(percentage) / maximum_percentage_;

    return static_cast<std::uint8_t>(
      std::lround(normalized * driver_maximum));
  }

  static Color percentage_state(
    float red_percentage,
    float green_percentage,
    float blue_percentage)
  {
    Color state;

    state.r = clamp_percentage(red_percentage);
    state.g = clamp_percentage(green_percentage);
    state.b = clamp_percentage(blue_percentage);
    state.a = maximum_percentage_;

    return state;
  }

  void validate_unique_led_indices() const
  {
    for (std::size_t i = 0; i < led_indices_.size(); ++i) {
      for (std::size_t j = i + 1; j < led_indices_.size(); ++j) {
        if (led_indices_[i] == led_indices_[j]) {
          throw std::invalid_argument(
                  "led_indices parameters must map to five "
                  "distinct LED indices");
        }
      }
    }
  }

  void initialize_hardware()
  {
    if (!robot_ok(&robot_)) {
      handle_initialization_error(
        "Failed to initialize the Foxy robot description");
      return;
    }

    leds_ = led_init_name(
      &robot_,
      "leds_front_and_rear");

    if (leds_.ctx == nullptr) {
      handle_initialization_error(
        "Failed to initialize 'leds_front_and_rear'");
      return;
    }

    for (std::size_t i = 0; i < led_indices_.size(); ++i) {
      const int result = led_set_rgb(
        leds_,
        led_indices_[i],
        0,
        0,
        0);

      if (result != 0) {
        const std::string message =
          "Failed to switch off LED '" +
          led_names_[i] +
          "' at controller index " +
          std::to_string(
          static_cast<unsigned>(led_indices_[i])) +
          " (foxy-drivers error " +
          std::to_string(result) +
          ")";

        led_deinit(&robot_, &leds_);
        hardware_available_ = false;

        handle_initialization_error(message);
        return;
      }
    }

    hardware_available_ = true;
  }

  void handle_initialization_error(
    const std::string & message)
  {
    RCLCPP_ERROR(
      get_logger(),
      "%s",
      message.c_str());

    if (fail_in_constructor_) {
      cleanup_after_initialization_failure();
      throw std::runtime_error(message);
    }
  }

  void cleanup_after_initialization_failure()
  {
    if (leds_.ctx != nullptr) {
      led_deinit(&robot_, &leds_);
      hardware_available_ = false;
    }

    robot_deinit(&robot_);
  }

  void create_ros_interfaces()
  {
    const auto command_qos =
      rclcpp::QoS(rclcpp::KeepLast(1))
      .reliable();

    const auto state_qos =
      rclcpp::QoS(rclcpp::KeepLast(1))
      .reliable()
      .transient_local();

    for (std::size_t i = 0; i < led_names_.size(); ++i) {
      const std::string topic_base =
        "led/" + led_names_[i];

      publishers_[i] = create_publisher<Color>(
        topic_base + "/get",
        state_qos);

      subscriptions_[i] = create_subscription<Color>(
        topic_base + "/set",
        command_qos,
        [this, i](const Color::SharedPtr message) {
          set_led(i, *message);
        });
    }
  }

  void set_led(
    std::size_t led_number,
    const Color & command)
  {
    const float red_percentage =
      clamp_percentage(command.r);

    const float green_percentage =
      clamp_percentage(command.g);

    const float blue_percentage =
      clamp_percentage(command.b);

    const std::uint8_t red =
      percent_to_driver_value(red_percentage);

    const std::uint8_t green =
      percent_to_driver_value(green_percentage);

    const std::uint8_t blue =
      percent_to_driver_value(blue_percentage);

    Color state;

    {
      std::lock_guard<std::mutex> lock(hardware_mutex_);

      if (!hardware_available_) {
        RCLCPP_ERROR(
          get_logger(),
          "Cannot set '%s': LED controller is unavailable",
          led_names_[led_number].c_str());
        return;
      }

      const int result = led_set_rgb(
        leds_,
        led_indices_[led_number],
        red,
        green,
        blue);

      if (result != 0) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to set '%s' at controller index %u "
          "(foxy-drivers error %d)",
          led_names_[led_number].c_str(),
          static_cast<unsigned>(
            led_indices_[led_number]),
          result);
        return;
      }

      states_[led_number] = percentage_state(
        red_percentage,
        green_percentage,
        blue_percentage);

      state = states_[led_number];
    }

    publishers_[led_number]->publish(state);
  }

  void publish_all_states()
  {
    if (!hardware_available_) {
      return;
    }

    for (std::size_t i = 0; i < states_.size(); ++i) {
      publishers_[i]->publish(states_[i]);
    }
  }

  /*
   * ROS-facing LED ordering.
   */
  const std::array<std::string, 5> led_names_{
    "front_right",
    "front_left",
    "rear_left",
    "rear_right",
    "top"
  };

  /*
   * Logical indices exposed by leds_controller:
   *
   *   0 = front-left
   *   1 = rear-left
   *   2 = top
   *   3 = rear-right
   *   4 = front-right
   *
   * This array follows led_names_ ordering.
   */
  const std::array<std::uint8_t, 5> default_led_indices_{
    4U,
    0U,
    1U,
    3U,
    2U
  };

  robot_t robot_{};
  led_t leds_{};

  bool fail_in_constructor_{true};
  bool hardware_available_{false};

  std::array<std::uint8_t, 5> led_indices_{
    4U,
    0U,
    1U,
    3U,
    2U
  };

  std::array<Color, 5> states_{};
  std::array<Publisher::SharedPtr, 5> publishers_{};
  std::array<Subscription::SharedPtr, 5> subscriptions_{};

  std::mutex hardware_mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<LedController>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("leds"),
      "%s",
      error.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
