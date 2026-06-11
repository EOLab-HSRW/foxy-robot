#ifndef __MOTORS_HARDWARE_HPP__
#define __MOTORS_HARDWARE_HPP__

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/macros.hpp>

#include <string>
#include <vector>

#include "foxy_hardware_interface/quadrature_encoder.hpp"

#define FOXY_IMPLEMENTATION
#include "foxy/foxy.h"

namespace foxy_hardware_interface
{

class FoxyMotorsHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FoxyMotorsHardware);

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  static double clamp(double v, double lo, double hi)
  {
    return (v < lo) ? lo : (v > hi) ? hi : v;
  }

  static double apply_dead_zone(double x, double dead_zone)
  {
    // x is normalized command in [-1, 1]
    // dead_zone in [0, 1)
    if (x == 0.0 || dead_zone <= 0.0) {
      return x;
    }

    const double s = (x > 0.0) ? 1.0 : -1.0;
    const double mag = (x >= 0.0) ? x : -x;  // |x| in (0,1]

    // Remap (0,1] -> [dead_zone,1]
    const double mapped = dead_zone + (1.0 - dead_zone) * mag;
    return s * mapped;
  }

  robot_t robot_;

  // motor handles
  motor_t m1_;
  motor_t m2_;

  // joint state/command
  std::vector<double> hw_pos_{0.0, 0.0};      // rad
  std::vector<double> hw_vel_{0.0, 0.0};      // rad/s
  std::vector<double> hw_cmd_vel_{0.0, 0.0};  // rad/s

  // encoders
  std::string gpiochip_dev_{"/dev/gpiochip0"};
  QuadratureEncoder enc_left_;
  QuadratureEncoder enc_right_;
  int64_t prev_count_left_{0};
  int64_t prev_count_right_{0};

  // parameters
  unsigned int left_a_{0}, left_b_{1};
  unsigned int right_a_{2}, right_b_{3};
  bool invert_left_enc_{false};
  bool invert_right_enc_{false};
  int debounce_us_{0};

  double dead_zone_{0.0};

  double ticks_per_rev_{2048.0};    // effective counts per wheel revolution (after quad & gear ratio)
  double max_wheel_rad_s_{10.0};    // wheel rad/s corresponding to motor_set(..., 1.0)
  bool invert_left_motor_{false};
  bool invert_right_motor_{false};

  // optional (not needed for joint velocity; useful if you want to compute m/s elsewhere)
  double wheel_radius_{0.05};
};

}  // namespace

#endif
