#include "foxy_hardware_interface/imu_sensor_hardware.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/rclcpp.hpp"

namespace foxy_hardware_interface
{

namespace
{

constexpr char kLoggerName[] = "FoxyImuSensorHardware";

constexpr std::array<const char *, 10> kRequiredStateInterfaces{
  {
    "orientation.x",
    "orientation.y",
    "orientation.z",
    "orientation.w",
    "angular_velocity.x",
    "angular_velocity.y",
    "angular_velocity.z",

    "linear_acceleration.x",
    "linear_acceleration.y",
    "linear_acceleration.z",
  }
};

}  // namespace

FoxyImuSensorHardware::~FoxyImuSensorHardware()
{

  close_driver();
}

hardware_interface::CallbackReturn FoxyImuSensorHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{

  const auto base_result =
    hardware_interface::SensorInterface::on_init(info);

  if (base_result != hardware_interface::CallbackReturn::SUCCESS) {
    return base_result;
  }


  if (info_.sensors.size() != 1U) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Expected exactly one <sensor> element, got %zu",
      info_.sensors.size());


    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & sensor = info_.sensors.front();

  sensor_name_ = sensor.name;

  if (sensor_name_.empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "The <sensor> name must not be empty");

    return hardware_interface::CallbackReturn::ERROR;

  }

  if (!sensor.command_interfaces.empty()) {
    RCLCPP_ERROR(

      rclcpp::get_logger(kLoggerName),
      "Sensor '%s' must not define command interfaces",
      sensor_name_.c_str());

    return hardware_interface::CallbackReturn::ERROR;
  }

  if (
    sensor.state_interfaces.size() !=
    kRequiredStateInterfaces.size())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Sensor '%s' must define exactly %zu state interfaces; got %zu",
      sensor_name_.c_str(),
      kRequiredStateInterfaces.size(),
      sensor.state_interfaces.size());

    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const char * required_name : kRequiredStateInterfaces) {
    const auto interface_it = std::find_if(
      sensor.state_interfaces.cbegin(),

      sensor.state_interfaces.cend(),
      [required_name](
        const hardware_interface::InterfaceInfo & state_interface)
      {
        return state_interface.name == required_name;

      });

    if (interface_it == sensor.state_interfaces.cend()) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Sensor '%s' is missing state interface '%s'",
        sensor_name_.c_str(),
        required_name);

      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  const auto resource_it =
    info_.hardware_parameters.find("resource_name");

  if (resource_it != info_.hardware_parameters.end()) {
    resource_name_ = resource_it->second;
  }


  if (resource_name_.empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Hardware parameter 'resource_name' must not be empty");

    return hardware_interface::CallbackReturn::ERROR;
  }

  close_driver();

  reset_values();

  return hardware_interface::CallbackReturn::SUCCESS;

}

hardware_interface::CallbackReturn
FoxyImuSensorHardware::on_configure(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  close_driver();

  reset_values();

  robot_ = robot_init();
  robot_initialized_ = true;

  if (!robot_ok(&robot_)) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "robot_init() failed with error %d",

      robot_.error);

    close_driver();

    return hardware_interface::CallbackReturn::ERROR;
  }

  imu_ = imu_init_name(
    &robot_,
    resource_name_.c_str());

  if (imu_.ctx == nullptr) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "imu_init_name('%s') failed",
      resource_name_.c_str());


    close_driver();

    return hardware_interface::CallbackReturn::ERROR;
  }

  imu_initialized_ = true;

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Configured sensor '%s' using foxy-drivers resource '%s'",
    sensor_name_.c_str(),
    resource_name_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
FoxyImuSensorHardware::on_cleanup(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  close_driver();
  reset_values();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
FoxyImuSensorHardware::on_shutdown(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  close_driver();
  reset_values();


  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
FoxyImuSensorHardware::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  if (
    !robot_initialized_ ||
    !imu_initialized_ ||
    imu_.ctx == nullptr)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Cannot activate sensor '%s': "
      "the IMU resource is not configured",
      sensor_name_.c_str());

    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
FoxyImuSensorHardware::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{

  /*

   * In ROS 2 Control Humble, Sensor::read() may be called while the
   * hardware component is INACTIVE or ACTIVE.
   *
   * Therefore, do not close the IMU here. The resource is released
   * by on_cleanup(), on_shutdown(), on_error(), or the destructor.
   */
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
FoxyImuSensorHardware::on_error(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  close_driver();
  reset_values();

  /*
   * Returning SUCCESS allows the Humble hardware wrapper to move the
   * component back to UNCONFIGURED after cleanup.

   */
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
FoxyImuSensorHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.reserve(kRequiredStateInterfaces.size());

  state_interfaces.emplace_back(
    sensor_name_,
    "orientation.x",
    &orientation_[0]);

  state_interfaces.emplace_back(
    sensor_name_,
    "orientation.y",
    &orientation_[1]);

  state_interfaces.emplace_back(
    sensor_name_,
    "orientation.z",
    &orientation_[2]);

  state_interfaces.emplace_back(
    sensor_name_,
    "orientation.w",
    &orientation_[3]);

  state_interfaces.emplace_back(
    sensor_name_,
    "angular_velocity.x",
    &angular_velocity_[0]);

  state_interfaces.emplace_back(
    sensor_name_,
    "angular_velocity.y",
    &angular_velocity_[1]);

  state_interfaces.emplace_back(
    sensor_name_,
    "angular_velocity.z",
    &angular_velocity_[2]);

  state_interfaces.emplace_back(
    sensor_name_,
    "linear_acceleration.x",
    &linear_acceleration_[0]);

  state_interfaces.emplace_back(
    sensor_name_,
    "linear_acceleration.y",
    &linear_acceleration_[1]);

  state_interfaces.emplace_back(
    sensor_name_,
    "linear_acceleration.z",
    &linear_acceleration_[2]);


  return state_interfaces;
}

hardware_interface::return_type FoxyImuSensorHardware::read(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & /* period */)
{
  if (
    !robot_initialized_ ||
    !imu_initialized_ ||
    imu_.ctx == nullptr)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Cannot read sensor '%s': "
      "the IMU resource is not configured",
      sensor_name_.c_str());

    return hardware_interface::return_type::ERROR;
  }

  /*
   * foxy-drivers returns:
   *
   *   accel_ms2[]  -> metres per second squared
   *   gyro_rads[]  -> radians per second

   *
   * sensor_msgs/msg/Imu requires:
   *
   *   linear_acceleration -> metres per second squared
   *   angular_velocity    -> radians per second
   */
  const imu_sample_t sample = imu_read(imu_);

  for (std::size_t axis = 0; axis < 3U; ++axis) {
    angular_velocity_[axis] =
      static_cast<double>(sample.gyro_rads[axis]);

    linear_acceleration_[axis] =
      static_cast<double>(sample.accel_ms2[axis]);
  }

  /*
   * foxy-drivers does not provide an orientation estimate.
   *
   * Keep a valid identity quaternion as a placeholder. The
   * imu_sensor_broadcaster configuration must mark orientation as
   * unavailable using orientation_covariance[0] == -1.
   */
  orientation_[0] = 0.0;
  orientation_[1] = 0.0;
  orientation_[2] = 0.0;
  orientation_[3] = 1.0;

  return hardware_interface::return_type::OK;
}

void FoxyImuSensorHardware::reset_values() noexcept
{
  orientation_ = {
    {0.0, 0.0, 0.0, 1.0}
  };

  angular_velocity_.fill(0.0);
  linear_acceleration_.fill(0.0);
}

void FoxyImuSensorHardware::close_driver() noexcept
{
  if (imu_initialized_) {
    imu_deinit(
      &robot_,
      &imu_);


    imu_initialized_ = false;
  }

  imu_ = imu_t{};

  if (robot_initialized_) {
    robot_deinit(&robot_);
    robot_initialized_ = false;
  }

  robot_ = robot_t{};
}

}  // namespace foxy_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  foxy_hardware_interface::FoxyImuSensorHardware,
  hardware_interface::SensorInterface)
