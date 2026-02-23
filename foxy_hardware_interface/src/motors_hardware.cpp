#include "foxy_hardware_interface/motors_hardware.hpp"
#include "foxy_hardware_interface/quadrature_encoder.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>

namespace foxy_hardware_interface
{

hardware_interface::CallbackReturn FoxyMotorsHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("FoxyMotorsHardware"), "Expected exactly 2 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints) {
    bool has_vel_cmd = false, has_pos = false, has_vel = false;
    for (const auto & ci : joint.command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_VELOCITY) has_vel_cmd = true;
    }
    for (const auto & si : joint.state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) has_pos = true;
      if (si.name == hardware_interface::HW_IF_VELOCITY) has_vel = true;
    }
    if (!has_vel_cmd || !has_pos || !has_vel) {
      RCLCPP_ERROR(rclcpp::get_logger("FoxyMotorsHardware"),
                   "Joint '%s' must have cmd:velocity and state:position+velocity",
                   joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  auto getp = [&](const std::string & k, const std::string & def) -> std::string {
    auto it = info_.hardware_parameters.find(k);
    return (it == info_.hardware_parameters.end()) ? def : it->second;
  };

  // Use a device path (no libgpiod name lookup)
  gpiochip_dev_ = getp("gpiochip_dev", "/dev/gpiochip0");

  left_a_ = static_cast<unsigned int>(std::stoul(getp("left_enc_a", "0")));
  left_b_ = static_cast<unsigned int>(std::stoul(getp("left_enc_b", "1")));
  right_a_ = static_cast<unsigned int>(std::stoul(getp("right_enc_a", "2")));
  right_b_ = static_cast<unsigned int>(std::stoul(getp("right_enc_b", "3")));

  invert_left_enc_ = (getp("invert_left_enc", "false") == "true");
  invert_right_enc_ = (getp("invert_right_enc", "false") == "true");
  debounce_us_ = static_cast<uint32_t>(std::stoul(getp("debounce_us", "0")));
  
  ticks_per_rev_ = std::stod(getp("ticks_per_rev", "2048"));
  max_wheel_rad_s_ = std::stod(getp("max_wheel_rad_s", "10.0"));

  invert_left_motor_ = (getp("invert_left_motor", "false") == "true");
  invert_right_motor_ = (getp("invert_right_motor", "false") == "true");
  
  dead_zone_ = std::stod(getp("dead_zone", "0.0"));
  if (dead_zone_ < 0.0 || dead_zone_ >= 1.0) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffbotHardware"),
                 "dead_zone must be in [0.0, 1.0), got %f", dead_zone_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  wheel_radius_ = std::stod(getp("wheel_radius", "0.05")); // optional

  if (ticks_per_rev_ <= 0.0) {
    RCLCPP_ERROR(rclcpp::get_logger("FoxyMotorsHardware"), "ticks_per_rev must be > 0");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (max_wheel_rad_s_ <= 0.0) {
    RCLCPP_ERROR(rclcpp::get_logger("FoxyMotorsHardware"), "max_wheel_rad_s must be > 0");
    return hardware_interface::CallbackReturn::ERROR;
  }

  robot_ = robot_init();
  if (!robot_ok(&robot_)) {
    RCLCPP_ERROR(rclcpp::get_logger("FoxyMotorsHardware"), "Failed to init robot drivers");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FoxyMotorsHardware::on_configure(const rclcpp_lifecycle::State &)
{

  m1_ = motor_init_name(&robot_, "motor1");
  m2_ = motor_init_name(&robot_, "motor2");

  if (!m1_.ctx) {
    RCLCPP_ERROR(rclcpp::get_logger("FoxyMotorsHardware"), "motor_init for motor1 failed.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  // if (!m2_.ctx) {
  //   RCLCPP_ERROR(rclcpp::get_logger("FoxyMotorsHardware"), "motor_init for motor1 failed.");
  //   return hardware_interface::CallbackReturn::ERROR;
  // }
  motor_set(m1_, 0.0f);
  motor_set(m2_, 0.0f);

  if (!enc_left_.open(gpiochip_dev_, left_a_, left_b_, invert_left_enc_, debounce_us_)) {
    RCLCPP_ERROR(rclcpp::get_logger("FoxyMotorsHardware"),
                 "Failed to open left encoder on %s lines %u/%u",
                 gpiochip_dev_.c_str(), left_a_, left_b_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!enc_right_.open(gpiochip_dev_, right_a_, right_b_, invert_right_enc_, debounce_us_)) {
    RCLCPP_ERROR(rclcpp::get_logger("FoxyMotorsHardware"),
                 "Failed to open right encoder on %s lines %u/%u",
                 gpiochip_dev_.c_str(), right_a_, right_b_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FoxyMotorsHardware::on_activate(const rclcpp_lifecycle::State &)
{
  hw_pos_[0] = hw_pos_[1] = 0.0;
  hw_vel_[0] = hw_vel_[1] = 0.0;
  hw_cmd_vel_[0] = hw_cmd_vel_[1] = 0.0;

  enc_left_.start();
  enc_right_.start();
  prev_count_left_ = enc_left_.count();
  prev_count_right_ = enc_right_.count();

  motor_set(m1_, 0.0f);
  motor_set(m2_, 0.0f);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FoxyMotorsHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  motor_set(m1_, 0.0f);
  motor_set(m2_, 0.0f);

  enc_left_.stop();
  enc_right_.stop();

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FoxyMotorsHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> states;
  states.reserve(4);

  states.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_pos_[0]);
  states.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_vel_[0]);
  states.emplace_back(info_.joints[1].name, hardware_interface::HW_IF_POSITION, &hw_pos_[1]);
  states.emplace_back(info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &hw_vel_[1]);

  return states;
}

std::vector<hardware_interface::CommandInterface> FoxyMotorsHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> cmds;
  cmds.reserve(2);

  cmds.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_vel_[0]);
  cmds.emplace_back(info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_vel_[1]);

  return cmds;
}

hardware_interface::return_type FoxyMotorsHardware::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  if (dt <= 0.0) return hardware_interface::return_type::OK;

  const int64_t cL = enc_left_.count();
  const int64_t cR = enc_right_.count();

  const int64_t dL = cL - prev_count_left_;
  const int64_t dR = cR - prev_count_right_;

  prev_count_left_ = cL;
  prev_count_right_ = cR;

  const double rad_per_tick = 2.0 * M_PI / ticks_per_rev_;

  // Joint position (wheel angle) in rad
  hw_pos_[0] = static_cast<double>(cL) * rad_per_tick;
  hw_pos_[1] = static_cast<double>(cR) * rad_per_tick;

  // Joint velocity (wheel angular speed) in rad/s
  hw_vel_[0] = static_cast<double>(dL) * rad_per_tick / dt;
  hw_vel_[1] = static_cast<double>(dR) * rad_per_tick / dt;

  // If you need linear wheel speed (m/s): v = wheel_radius_ * hw_vel_[i]

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FoxyMotorsHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{

  // Command is wheel angular velocity (rad/s). Convert to normalized [-1, 1].
  double left = clamp(hw_cmd_vel_[0] / max_wheel_rad_s_, -1.0, 1.0);
  double right = clamp(hw_cmd_vel_[1] / max_wheel_rad_s_, -1.0, 1.0);

  // Apply dead-zone compensation
  left = apply_dead_zone(left, dead_zone_);
  right = apply_dead_zone(right, dead_zone_);

  if (invert_left_motor_) left = -left;
  if (invert_right_motor_) right = -right;

  motor_set(m1_, static_cast<float>(left));
  motor_set(m2_, static_cast<float>(right));

  return hardware_interface::return_type::OK;
}

}  // namespace foxy_hardware_interface

PLUGINLIB_EXPORT_CLASS(foxy_hardware_interface::FoxyMotorsHardware, hardware_interface::SystemInterface)

