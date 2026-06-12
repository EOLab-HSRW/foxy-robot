#include "foxy_controllers/battery_state_broadcaster.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace foxy_controllers
{
namespace
{
constexpr double kCommandAsserted = 1.0;
constexpr double kCommandIdle = 0.0;

inline double quiet_nan()
{
  return std::numeric_limits<double>::quiet_NaN();
}
}  // namespace

controller_interface::CallbackReturn BatteryStateBroadcaster::on_init()
{
  try {
    auto_declare<std::string>("battery_name", battery_name_);
    auto_declare<std::string>("frame_id", frame_id_);
    auto_declare<std::string>("topic_name", topic_name_);
    auto_declare<double>("publish_rate", publish_rate_);
    auto_declare<bool>("allow_shutdown", allow_shutdown_);
    auto_declare<bool>("charging_current_positive", charging_current_positive_);
    auto_declare<double>("current_deadband_ma", current_deadband_ma_);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during parameter declaration: %s", exception.what());
    return controller_interface::CallbackReturn::ERROR;
  }


  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
BatteryStateBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = state_interface_names();
  return config;
}

controller_interface::InterfaceConfiguration
BatteryStateBroadcaster::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = command_interface_names();
  return config;
}

controller_interface::CallbackReturn BatteryStateBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)

{
  battery_name_ = get_node()->get_parameter("battery_name").as_string();
  frame_id_ = get_node()->get_parameter("frame_id").as_string();
  topic_name_ = get_node()->get_parameter("topic_name").as_string();
  publish_rate_ = get_node()->get_parameter("publish_rate").as_double();
  allow_shutdown_ = get_node()->get_parameter("allow_shutdown").as_bool();
  charging_current_positive_ = get_node()->get_parameter("charging_current_positive").as_bool();
  current_deadband_ma_ = get_node()->get_parameter("current_deadband_ma").as_double();

  if (battery_name_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'battery_name' must not be empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (publish_rate_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'publish_rate' must be > 0.0.");
    return controller_interface::CallbackReturn::ERROR;
  }

  publish_period_ns_ = static_cast<int64_t>(1e9 / publish_rate_);
  if (publish_period_ns_ <= 0) {
    publish_period_ns_ = 1;
  }

  current_deadband_ma_ = std::max(0.0, current_deadband_ma_);

  auto publisher = get_node()->create_publisher<sensor_msgs::msg::BatteryState>(

    topic_name_, rclcpp::SystemDefaultsQoS());
  realtime_publisher_ =

    std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::BatteryState>>(publisher);

  ping_service_ = get_node()->create_service<Trigger>(
    "~/ping",
    [this](
      const std::shared_ptr<Trigger::Request> request,
      std::shared_ptr<Trigger::Response> response) {
      queue_command(CommandRequest::Ping, request, response);
    });

  refresh_service_ = get_node()->create_service<Trigger>(
    "~/refresh",
    [this](
      const std::shared_ptr<Trigger::Request> request,
      std::shared_ptr<Trigger::Response> response) {
      queue_command(CommandRequest::Refresh, request, response);
    });

  version_service_ = get_node()->create_service<Trigger>(

    "~/version",
    [this](
      const std::shared_ptr<Trigger::Request> request,
      std::shared_ptr<Trigger::Response> response) {
      queue_command(CommandRequest::Version, request, response);
    });

  shutdown_service_ = get_node()->create_service<Trigger>(
    "~/shutdown",
    [this](
      const std::shared_ptr<Trigger::Request> request,
      std::shared_ptr<Trigger::Response> response) {
      queue_command(CommandRequest::Shutdown, request, response);
    });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BatteryStateBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  last_publish_time_ns_ = 0;
  ping_requested_.store(false);

  refresh_requested_.store(false);
  version_requested_.store(false);
  shutdown_requested_.store(false);
  set_all_command_values(kCommandIdle);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BatteryStateBroadcaster::on_deactivate(

  const rclcpp_lifecycle::State & /*previous_state*/)
{
  ping_requested_.store(false);
  refresh_requested_.store(false);
  version_requested_.store(false);
  shutdown_requested_.store(false);

  set_all_command_values(kCommandIdle);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BatteryStateBroadcaster::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  realtime_publisher_.reset();
  ping_service_.reset();
  refresh_service_.reset();
  version_service_.reset();
  shutdown_service_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type BatteryStateBroadcaster::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & /*period*/)
{
  // Default all one-shot command interfaces to zero. If a service queued a command, this update
  // raises exactly one command for one controller-manager cycle; the hardware plugin detects the
  // 0 -> 1 edge in its write() method.
  set_all_command_values(kCommandIdle);

  if (ping_requested_.exchange(false)) {
    set_command_value("ping", kCommandAsserted);

  }
  if (refresh_requested_.exchange(false)) {
    set_command_value("get", kCommandAsserted);
  }
  if (version_requested_.exchange(false)) {
    set_command_value("version", kCommandAsserted);
  }
  if (shutdown_requested_.exchange(false)) {
    set_command_value("shutdown", kCommandAsserted);
  }

  const int64_t now_ns = time.nanoseconds();
  if (
    last_publish_time_ns_ > 0 &&
    now_ns >= last_publish_time_ns_ &&
    (now_ns - last_publish_time_ns_) < publish_period_ns_)
  {
    return controller_interface::return_type::OK;
  }

  last_publish_time_ns_ = now_ns;


  if (realtime_publisher_ && realtime_publisher_->trylock()) {
    fill_battery_state_message(realtime_publisher_->msg_, time);
    realtime_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}


std::vector<std::string> BatteryStateBroadcaster::state_interface_names() const
{
  return {
    battery_name_ + "/online",
    battery_name_ + "/stale",
    battery_name_ + "/percent",
    battery_name_ + "/voltage_mv",
    battery_name_ + "/current_ma",
    battery_name_ + "/temperature_mc",
    battery_name_ + "/time_to_empty_min",
    battery_name_ + "/cycle_count",
    battery_name_ + "/usb_out_1_mv",
    battery_name_ + "/usb_out_2_mv",
    battery_name_ + "/charger_voltage_mv",

    battery_name_ + "/age_ms",
    battery_name_ + "/packet_count",
    battery_name_ + "/last_read_result",
    battery_name_ + "/last_command_result",
    battery_name_ + "/last_command_ok",
    battery_name_ + "/protocol_version"};
}

std::vector<std::string> BatteryStateBroadcaster::command_interface_names() const
{
  return {
    battery_name_ + "/ping",
    battery_name_ + "/get",
    battery_name_ + "/version",
    battery_name_ + "/shutdown"};
}

double BatteryStateBroadcaster::get_state_value(
  const std::string & interface_name,
  double fallback) const

{
  const std::string full_name = battery_name_ + "/" + interface_name;

  const auto it = std::find_if(

    state_interfaces_.cbegin(), state_interfaces_.cend(),
    [&full_name](const auto & interface) {return interface.get_name() == full_name;});

  return it == state_interfaces_.cend() ? fallback : it->get_value();
}

bool BatteryStateBroadcaster::set_command_value(
  const std::string & interface_name,
  double value)

{
  const std::string full_name = battery_name_ + "/" + interface_name;

  const auto it = std::find_if(

    command_interfaces_.begin(), command_interfaces_.end(),
    [&full_name](const auto & interface) {return interface.get_name() == full_name;});

  if (it == command_interfaces_.end()) {

    return false;
  }

  it->set_value(value);
  return true;
}

void BatteryStateBroadcaster::set_all_command_values(double value)
{
  for (auto & interface : command_interfaces_) {
    interface.set_value(value);

  }
}

void BatteryStateBroadcaster::queue_command(
  CommandRequest request,
  const std::shared_ptr<Trigger::Request> /*request_msg*/,
  std::shared_ptr<Trigger::Response> response_msg)
{
  switch (request) {
    case CommandRequest::Ping:
      ping_requested_.store(true);
      response_msg->success = true;
      response_msg->message = "queued batteryd PING command";
      return;

    case CommandRequest::Refresh:

      refresh_requested_.store(true);
      response_msg->success = true;
      response_msg->message = "queued batteryd GET refresh command";
      return;

    case CommandRequest::Version:
      version_requested_.store(true);
      response_msg->success = true;
      response_msg->message = "queued batteryd VERSION command; read protocol_version state after update";
      return;

    case CommandRequest::Shutdown:
      if (!allow_shutdown_) {
        response_msg->success = false;
        response_msg->message = "batteryd SHUTDOWN service is disabled; set allow_shutdown=true to enable it";
        return;
      }
      shutdown_requested_.store(true);
      response_msg->success = true;
      response_msg->message = "queued batteryd SHUTDOWN command";
      return;
  }

  response_msg->success = false;
  response_msg->message = "unknown command request";
}

uint8_t BatteryStateBroadcaster::compute_power_supply_status(
  double online,
  double percent,
  double current_ma) const
{
  using sensor_msgs::msg::BatteryState;

  if (online <= 0.5) {
    return BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  }

  if (percent >= 99.5) {
    return BatteryState::POWER_SUPPLY_STATUS_FULL;
  }

  const double signed_current_ma = charging_current_positive_ ? current_ma : -current_ma;
  if (signed_current_ma > current_deadband_ma_) {

    return BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  }

  if (signed_current_ma < -current_deadband_ma_) {
    return BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  }

  return BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
}

uint8_t BatteryStateBroadcaster::compute_power_supply_health(

  double stale,
  double last_read_result) const

{
  using sensor_msgs::msg::BatteryState;

  if (stale > 0.5 || last_read_result != 0.0) {
    return BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  }

  return BatteryState::POWER_SUPPLY_HEALTH_GOOD;
}

void BatteryStateBroadcaster::fill_battery_state_message(
  sensor_msgs::msg::BatteryState & message,
  const rclcpp::Time & stamp) const
{
  using sensor_msgs::msg::BatteryState;


  const double online = get_state_value("online", 0.0);
  const double stale = get_state_value("stale", 1.0);
  const double percent = get_state_value("percent", -1.0);
  const double voltage_mv = get_state_value("voltage_mv", 0.0);
  const double current_ma = get_state_value("current_ma", 0.0);

  const double temperature_mc = get_state_value("temperature_mc", 0.0);
  const double time_to_empty_min = get_state_value("time_to_empty_min", -1.0);
  const double cycle_count = get_state_value("cycle_count", -1.0);
  const double last_read_result = get_state_value("last_read_result", -1.0);

  message.header.stamp = stamp;
  message.header.frame_id = frame_id_;

  message.voltage = voltage_mv > 0.0 ? voltage_mv / 1000.0 : quiet_nan();
  message.temperature = temperature_mc != 0.0 ? temperature_mc / 1000.0 : quiet_nan();
  message.current = current_ma / 1000.0;
  message.charge = quiet_nan();
  message.capacity = quiet_nan();
  message.design_capacity = quiet_nan();
  message.percentage = percent >= 0.0 ? percent / 100.0 : quiet_nan();

  message.power_supply_status = compute_power_supply_status(online, percent, current_ma);
  message.power_supply_health = compute_power_supply_health(stale, last_read_result);
  message.power_supply_technology = BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;

  message.present = online > 0.5;
  message.cell_voltage.clear();
  message.cell_temperature.clear();
  message.location = battery_name_;
  message.serial_number.clear();

  // sensor_msgs/BatteryState does not have dedicated time-to-empty or cycle-count fields.
  // Keep those values as ros2_control state interfaces for controllers that need them.
  (void)time_to_empty_min;
  (void)cycle_count;
}

}  // namespace foxy_controlers

PLUGINLIB_EXPORT_CLASS(
  foxy_controllers::BatteryStateBroadcaster,
  controller_interface::ControllerInterface)


