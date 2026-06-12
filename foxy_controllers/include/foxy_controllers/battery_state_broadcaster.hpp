#ifndef FOXY_CONTROLLERS__BATTERY_STATE_BROADCASTER_HPP_
#define FOXY_CONTROLLERS__BATTERY_STATE_BROADCASTER_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace foxy_controllers
{

class BatteryStateBroadcaster : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;


  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;


private:
  using Trigger = std_srvs::srv::Trigger;

  enum class CommandRequest
  {
    Ping,
    Refresh,
    Version,
    Shutdown
  };

  std::vector<std::string> state_interface_names() const;
  std::vector<std::string> command_interface_names() const;

  double get_state_value(const std::string & interface_name, double fallback) const;
  bool set_command_value(const std::string & interface_name, double value);
  void set_all_command_values(double value);

  void queue_command(

    CommandRequest request,
    const std::shared_ptr<Trigger::Request> request_msg,
    std::shared_ptr<Trigger::Response> response_msg);


  uint8_t compute_power_supply_status(double online, double percent, double current_ma) const;
  uint8_t compute_power_supply_health(double stale, double last_read_result) const;

  void fill_battery_state_message(
    sensor_msgs::msg::BatteryState & message,
    const rclcpp::Time & stamp) const;

  std::string battery_name_{"battery"};

  std::string frame_id_{"battery"};
  std::string topic_name_{"~/battery_state"};

  double publish_rate_{1.0};
  int64_t publish_period_ns_{1000000000};
  int64_t last_publish_time_ns_{0};

  bool allow_shutdown_{false};
  bool charging_current_positive_{true};
  double current_deadband_ma_{50.0};

  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::BatteryState>>
    realtime_publisher_;


  rclcpp::Service<Trigger>::SharedPtr ping_service_;
  rclcpp::Service<Trigger>::SharedPtr refresh_service_;
  rclcpp::Service<Trigger>::SharedPtr version_service_;
  rclcpp::Service<Trigger>::SharedPtr shutdown_service_;

  std::atomic_bool ping_requested_{false};

  std::atomic_bool refresh_requested_{false};
  std::atomic_bool version_requested_{false};
  std::atomic_bool shutdown_requested_{false};
};

}  // namespace foxy_controllers

#endif  // FOXY_CONTROLLERS__BATTERY_STATE_BROADCASTER_HPP_
