#include <cstdlib>
#include <cstring>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_set>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "foxy_hardware_interface/battery_client.hpp"

#define FOXY_BATTERY_IMPLEMENTATION
#include "foxy/battery.h"

namespace foxy_hardware_interface {

  namespace {
    constexpr double kCommandTriggerThreshold = 0.5;

    bool parse_bool_param(
        const std::unordered_map<std::string, std::string> & params,
        const std::string & key,
        bool default_value)
    {
      const auto it = params.find(key);
      if (it == params.end()) {
        return default_value;
      }
      const std::string & value = it->second;
      return value == "1" || value == "true" || value == "True" || value == "TRUE" || value == "yes";
    }

    int parse_int_param(
        const std::unordered_map<std::string, std::string> & params,
        const std::string & key,
        int default_value)
    {
      const auto it = params.find(key);
      if (it == params.end()) {
        return default_value;
      }
      try {
        return std::stoi(it->second);
      } catch (const std::exception &) {
        return default_value;
      }
    }

    std::string parse_string_param(
        const std::unordered_map<std::string, std::string> & params,
        const std::string & key,
        const std::string & default_value)
    {
      const auto it = params.find(key);
      return it == params.end() ? default_value : it->second;
    }

    bool reply_is_ok(const char * response)
    {
      return response != nullptr && response[0] == 'O' && response[1] == 'K' &&
        (response[2] == '\0' || response[2] == ' ' || response[2] == '\t' ||
         response[2] == '\r' || response[2] == '\n');
    }

    int parse_protocol_version(const char * response)
    {
      if (response == nullptr) {
        return 0;
      }

      const char * protocol = std::strstr(response, "protocol=");
      if (protocol == nullptr) {
        return 0;
      }

      protocol += std::strlen("protocol=");
      return std::atoi(protocol);
    }
  } // namespace <empty>

hardware_interface::CallbackReturn BatteryClientInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.gpios.empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("BatteryClientInterface"),
      "Expected one <gpio> block for the battery hardware component.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.gpios.size() != 1U) {
    RCLCPP_ERROR(
      rclcpp::get_logger("BatteryClientInterface"),
      "Expected exactly one <gpio> block, got %zu.", info_.gpios.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  gpio_name_ = info_.gpios.front().name;
  status_path_ = parse_string_param(info_.hardware_parameters, "status_path", FOXY_BATTERY_STATUS_PATH);
  socket_path_ = parse_string_param(info_.hardware_parameters, "socket_path", FOXY_BATTERY_CONTROL_SOCKET_PATH);
  timeout_ms_ = parse_int_param(info_.hardware_parameters, "timeout_ms", FOXY_BATTERY_DEFAULT_TIMEOUT_MS);
  read_via_socket_ = parse_bool_param(info_.hardware_parameters, "read_via_socket", false);
  fail_on_read_error_ = parse_bool_param(info_.hardware_parameters, "fail_on_read_error", false);
  ping_on_activate_ = parse_bool_param(info_.hardware_parameters, "ping_on_activate", false);

  for (const auto & state_interface : info_.gpios.front().state_interfaces) {
    if (!is_known_state_interface(state_interface.name)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("BatteryClientInterface"),
        "Unknown battery state interface '%s'.", state_interface.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  for (const auto & command_interface : info_.gpios.front().command_interfaces) {
    if (!is_known_command_interface(command_interface.name)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("BatteryClientInterface"),
        "Unknown battery command interface '%s'.", command_interface.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  reset_states();
  reset_commands();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
BatteryClientInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.gpios.front().state_interfaces.size());

  for (const auto & interface : info_.gpios.front().state_interfaces) {
    double * value = state_value_ptr(interface.name);
    if (value != nullptr) {
      state_interfaces.emplace_back(gpio_name_, interface.name, value);
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
BatteryClientInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.gpios.front().command_interfaces.size());

  for (const auto & interface : info_.gpios.front().command_interfaces) {
    double * value = command_value_ptr(interface.name);
    if (value != nullptr) {
      command_interfaces.emplace_back(gpio_name_, interface.name, value);
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn BatteryClientInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_states();
  reset_commands();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BatteryClientInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (ping_on_activate_) {
    const int rc = control_command_ok("PING");
    RCLCPP_INFO(
        rclcpp::get_logger("BatteryClientInterface"),
        "foxy_battery PING result is: %d", rc);

    record_command_result(rc);
    if (rc != FOXY_BATTERY_OK) {
      RCLCPP_ERROR(
        rclcpp::get_logger("BatteryClientInterface"),
        "foxy_battery PING failed during activation: %s", foxy_battery_strerror(rc));
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  const int rc = read_via_socket_ ? read_status_socket() : read_status_file();
  last_read_result_ = static_cast<double>(rc);

  if (rc != FOXY_BATTERY_OK && fail_on_read_error_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("BatteryClientInterface"),
      "Initial foxy_battery status read failed: %s", foxy_battery_strerror(rc));
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BatteryClientInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_commands();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type BatteryClientInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  const int rc = read_via_socket_ ? read_status_socket() : read_status_file();
  last_read_result_ = static_cast<double>(rc);

  if (rc != FOXY_BATTERY_OK) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("BatteryClientInterface"),
      *rclcpp::Clock::make_shared(),
      2000,
      "foxy_battery status read failed: %s", foxy_battery_strerror(rc));

    return fail_on_read_error_ ?
           hardware_interface::return_type::ERROR : hardware_interface::return_type::OK;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BatteryClientInterface::write(

  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (rising_edge(ping_command_, previous_ping_command_)) {
    record_command_result(control_command_ok("PING"));
  }

  if (rising_edge(get_command_, previous_get_command_)) {
    const int rc = read_status_socket();
    last_read_result_ = static_cast<double>(rc);
    record_command_result(rc);
  }

  if (rising_edge(version_command_, previous_version_command_)) {
    record_command_result(control_version());
  }

  if (rising_edge(shutdown_command_, previous_shutdown_command_)) {
    record_command_result(control_command_ok("SHUTDOWN"));
  }

  previous_ping_command_ = ping_command_;
  previous_get_command_ = get_command_;
  previous_version_command_ = version_command_;
  previous_shutdown_command_ = shutdown_command_;

  return hardware_interface::return_type::OK;
}

double * BatteryClientInterface::state_value_ptr(const std::string & name)
{
  if (name == "online") {return &online_;}
  if (name == "stale") {return &stale_;}
  if (name == "percent") {return &percent_;}
  if (name == "voltage_mv") {return &voltage_mv_;}
  if (name == "current_ma") {return &current_ma_;}
  if (name == "temperature_mc") {return &temperature_mc_;}
  if (name == "time_to_empty_min") {return &time_to_empty_min_;}
  if (name == "cycle_count") {return &cycle_count_;}
  if (name == "usb_out_1_mv") {return &usb_out_1_mv_;}
  if (name == "usb_out_2_mv") {return &usb_out_2_mv_;}
  if (name == "charger_voltage_mv") {return &charger_voltage_mv_;}
  if (name == "age_ms") {return &age_ms_;}
  if (name == "packet_count") {return &packet_count_;}
  if (name == "last_read_result") {return &last_read_result_;}
  if (name == "last_command_result") {return &last_command_result_;}
  if (name == "last_command_ok") {return &last_command_ok_;}
  if (name == "protocol_version") {return &protocol_version_;}
  return nullptr;
}

double * BatteryClientInterface::command_value_ptr(const std::string & name)
{
  if (name == "ping") {return &ping_command_;}
  if (name == "get") {return &get_command_;}
  if (name == "version") {return &version_command_;}
  if (name == "shutdown") {return &shutdown_command_;}
  return nullptr;
}

bool BatteryClientInterface::is_known_state_interface(const std::string & name) const
{
  static const std::unordered_set<std::string> known = {
    "online", "stale", "percent", "voltage_mv", "current_ma", "temperature_mc",
    "time_to_empty_min", "cycle_count", "usb_out_1_mv", "usb_out_2_mv",
    "charger_voltage_mv", "age_ms", "packet_count", "last_read_result",
    "last_command_result", "last_command_ok", "protocol_version"};
  return known.find(name) != known.end();
}

bool BatteryClientInterface::is_known_command_interface(const std::string & name) const
{
  static const std::unordered_set<std::string> known = {"ping", "get", "version", "shutdown"};
  return known.find(name) != known.end();
}

bool BatteryClientInterface::rising_edge(double command, double previous) const
{
  return command > kCommandTriggerThreshold && previous <= kCommandTriggerThreshold;
}

void BatteryClientInterface::reset_states()
{
  online_ = 0.0;
  stale_ = 1.0;
  percent_ = -1.0;
  voltage_mv_ = 0.0;
  current_ma_ = 0.0;
  temperature_mc_ = 0.0;
  time_to_empty_min_ = -1.0;
  cycle_count_ = -1.0;
  usb_out_1_mv_ = 0.0;
  usb_out_2_mv_ = 0.0;
  charger_voltage_mv_ = 0.0;
  age_ms_ = -1.0;
  packet_count_ = 0.0;
  last_read_result_ = 0.0;
  last_command_result_ = 0.0;
  last_command_ok_ = 1.0;
  protocol_version_ = 0.0;
}

void BatteryClientInterface::reset_commands()
{
  ping_command_ = 0.0;
  get_command_ = 0.0;
  version_command_ = 0.0;
  shutdown_command_ = 0.0;
  previous_ping_command_ = 0.0;
  previous_get_command_ = 0.0;
  previous_version_command_ = 0.0;
  previous_shutdown_command_ = 0.0;
}

void BatteryClientInterface::apply_status(const foxy_battery_status & status)
{
  online_ = static_cast<double>(status.online);
  stale_ = static_cast<double>(status.stale);
  percent_ = static_cast<double>(status.percent);
  voltage_mv_ = static_cast<double>(status.voltage_mv);
  current_ma_ = static_cast<double>(status.current_ma);
  temperature_mc_ = static_cast<double>(status.temperature_mc);
  time_to_empty_min_ = static_cast<double>(status.time_to_empty_min);
  cycle_count_ = static_cast<double>(status.cycle_count);
  usb_out_1_mv_ = static_cast<double>(status.usb_out_1_mv);
  usb_out_2_mv_ = static_cast<double>(status.usb_out_2_mv);
  charger_voltage_mv_ = static_cast<double>(status.charger_voltage_mv);
  age_ms_ = static_cast<double>(status.age_ms);
  packet_count_ = static_cast<double>(status.packet_count);
}

int BatteryClientInterface::read_status_file()
{
  foxy_battery_status status;
  const int rc = foxy_battery_read_status_file(status_path_.c_str(), &status);
  if (rc == FOXY_BATTERY_OK) {
    apply_status(status);
  }
  return rc;
}

int BatteryClientInterface::read_status_socket()
{
  char response[FOXY_BATTERY_RESPONSE_MAX];
  foxy_battery_status status;

  const int rc = foxy_battery_control_command_timeout(
    socket_path_.c_str(), "GET", response, sizeof(response), timeout_ms_);
  if (rc != FOXY_BATTERY_OK) {
    return rc;
  }

  if (!reply_is_ok(response)) {
    return FOXY_BATTERY_EPROTO;
  }

  const int parse_rc = foxy_battery_parse_status(response, &status);
  if (parse_rc == FOXY_BATTERY_OK) {
    apply_status(status);
  }
  return parse_rc;
}

int BatteryClientInterface::control_command_ok(const char * command)
{
  char response[FOXY_BATTERY_RESPONSE_MAX];
  const int rc = foxy_battery_control_command_timeout(
    socket_path_.c_str(), command, response, sizeof(response), timeout_ms_);
  if (rc != FOXY_BATTERY_OK) {
    return rc;
  }
  return reply_is_ok(response) ? FOXY_BATTERY_OK : foxy_battery_EPROTO;
}

int BatteryClientInterface::control_version()
{
  char response[FOXY_BATTERY_RESPONSE_MAX];
  const int rc = foxy_battery_control_command_timeout(
    socket_path_.c_str(), "VERSION", response, sizeof(response), timeout_ms_);
  if (rc != FOXY_BATTERY_OK) {
    return rc;
  }
  if (!reply_is_ok(response)) {
    return FOXY_BATTERY_EPROTO;
  }

  protocol_version_ = static_cast<double>(parse_protocol_version(response));
  return FOXY_BATTERY_OK;
}

void BatteryClientInterface::record_command_result(int rc)
{
  last_command_result_ = static_cast<double>(rc);
  last_command_ok_ = rc == FOXY_BATTERY_OK ? 1.0 : 0.0;

  if (rc != FOXY_BATTERY_OK) {
    RCLCPP_WARN(
      rclcpp::get_logger("foxy_batteryHardwareInterface"),
      "foxy_battery command failed: %s", foxy_battery_strerror(rc));
  }
}



} // namespace foxy_hardware_interface


PLUGINLIB_EXPORT_CLASS(
  foxy_hardware_interface::BatteryClientInterface,
  hardware_interface::SystemInterface)
