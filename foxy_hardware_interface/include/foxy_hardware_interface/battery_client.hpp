#ifndef BATTERY_HARDWARE_HPP__
#define BATTERY_HARDWARE_HPP__

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/macros.hpp>

#include <string>
#include <vector>

#include "foxy_hardware_interface/batteryd.h"

namespace foxy_hardware_interface
{

class BatteryClientInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(BatteryClientInterface);

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  double * state_value_ptr(const std::string & name);
  double * command_value_ptr(const std::string & name);

  bool is_known_state_interface(const std::string & name) const;
  bool is_known_command_interface(const std::string & name) const;
  bool rising_edge(double command, double previous) const;

  void reset_states();
  void reset_commands();
  void apply_status(const struct batteryd_status & status);

  int read_status_file();
  int read_status_socket();
  int control_command_ok(const char * command);
  int control_version();
  void record_command_result(int rc);

  std::string gpio_name_{"battery"};
  std::string status_path_{"/run/batteryd/status"};
  std::string socket_path_{"/run/batteryd/control.sock"};
  int timeout_ms_{1000};
  bool read_via_socket_{false};
  bool fail_on_read_error_{false};
  bool ping_on_activate_{false};

  // State interfaces. Values mirror batteryd_status fields where possible.
  double online_{0.0};
  double stale_{1.0};
  double percent_{-1.0};
  double voltage_mv_{0.0};
  double current_ma_{0.0};
  double temperature_mc_{0.0};
  double time_to_empty_min_{-1.0};
  double cycle_count_{-1.0};
  double usb_out_1_mv_{0.0};
  double usb_out_2_mv_{0.0};
  double charger_voltage_mv_{0.0};
  double age_ms_{-1.0};
  double packet_count_{0.0};

  // Driver/meta state interfaces.
  double last_read_result_{0.0};
  double last_command_result_{0.0};
  double last_command_ok_{1.0};
  double protocol_version_{0.0};

  // One-shot command interfaces. Trigger on transition <= 0.5 -> > 0.5.
  double ping_command_{0.0};
  double get_command_{0.0};
  double version_command_{0.0};
  double shutdown_command_{0.0};

  double previous_ping_command_{0.0};
  double previous_get_command_{0.0};
  double previous_version_command_{0.0};
  double previous_shutdown_command_{0.0};

};

}  // namespace

#endif
